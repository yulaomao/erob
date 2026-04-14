from __future__ import annotations

import ctypes
import json
import os
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any


class ControllerError(RuntimeError):
    pass


class _AxisMetadataStruct(ctypes.Structure):
    _fields_ = [
        ("logical_axis_id", ctypes.c_int),
        ("joint_name", ctypes.c_char * 64),
        ("min_angle_deg", ctypes.c_double),
        ("max_angle_deg", ctypes.c_double),
        ("max_velocity_deg_s", ctypes.c_double),
        ("max_accel_deg_s2", ctypes.c_double),
        ("max_decel_deg_s2", ctypes.c_double),
        ("configured_serial", ctypes.c_char * 64),
        ("configured_eep_man", ctypes.c_uint32),
        ("configured_eep_id", ctypes.c_uint32),
        ("configured_eep_rev", ctypes.c_uint32),
    ]


class _AxisStateStruct(ctypes.Structure):
    _fields_ = [
        ("online", ctypes.c_int),
        ("enabled", ctypes.c_int),
        ("fault", ctypes.c_int),
        ("statusword", ctypes.c_uint16),
        ("controlword", ctypes.c_uint16),
        ("motion_mode", ctypes.c_int),
        ("cia402_state", ctypes.c_int),
        ("position_mode_state", ctypes.c_int),
        ("follow_mode_state", ctypes.c_int),
        ("actual_position_count", ctypes.c_int32),
        ("actual_velocity_count_s", ctypes.c_int32),
        ("actual_torque", ctypes.c_int16),
        ("actual_angle_deg", ctypes.c_double),
        ("actual_velocity_deg_s", ctypes.c_double),
        ("target_angle_deg", ctypes.c_double),
        ("position_error_deg", ctypes.c_double),
        ("target_reached", ctypes.c_int),
        ("near_positive_limit", ctypes.c_int),
        ("near_negative_limit", ctypes.c_int),
        ("al_status_code", ctypes.c_int),
        ("last_error_code", ctypes.c_int),
        ("motion_mode_name", ctypes.c_char * 32),
        ("cia402_state_name", ctypes.c_char * 64),
        ("position_mode_name", ctypes.c_char * 64),
        ("follow_mode_name", ctypes.c_char * 64),
    ]


@dataclass(slots=True)
class AxisMetadata:
    logical_axis_id: int
    joint_name: str
    min_angle_deg: float
    max_angle_deg: float
    max_velocity_deg_s: float
    max_accel_deg_s2: float
    max_decel_deg_s2: float
    configured_serial: str
    configured_eep_man: int
    configured_eep_id: int
    configured_eep_rev: int


@dataclass(slots=True)
class AxisState:
    online: bool
    enabled: bool
    fault: bool
    statusword: int
    controlword: int
    motion_mode: int
    cia402_state: int
    position_mode_state: int
    follow_mode_state: int
    actual_position_count: int
    actual_velocity_count_s: int
    actual_torque: int
    actual_angle_deg: float
    actual_velocity_deg_s: float
    target_angle_deg: float
    position_error_deg: float
    target_reached: bool
    near_positive_limit: bool
    near_negative_limit: bool
    al_status_code: int
    last_error_code: int
    motion_mode_name: str
    cia402_state_name: str
    position_mode_name: str
    follow_mode_name: str


def _decode_c_string(raw: bytes | None) -> str:
    if not raw:
        return ""
    return raw.decode("utf-8", errors="ignore").rstrip("\x00")


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _resolve_library_path(library_path: str | None) -> Path:
    if library_path:
        candidate = Path(library_path).expanduser().resolve()
        if candidate.exists():
            return candidate
        raise FileNotFoundError(f"未找到共享库: {candidate}")

    env_library_path = os.getenv("EROB_C_API_LIB")
    if env_library_path:
        candidate = Path(env_library_path).expanduser().resolve()
        if candidate.exists():
            return candidate
        raise FileNotFoundError(f"EROB_C_API_LIB 指向的共享库不存在: {candidate}")

    env_path = Path.cwd()
    suffix = {
        "linux": ".so",
        "darwin": ".dylib",
        "win32": ".dll",
    }
    extension = suffix.get(__import__("sys").platform, ".so")
    library_name = f"liberob_c_api{extension}" if extension != ".dll" else "erob_c_api.dll"
    workspace = _repo_root()
    candidates = [
        workspace / "build" / "demo" / library_name,
        workspace / "build" / library_name,
        workspace / "install" / "bin" / library_name,
        env_path / library_name,
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate

    matches = list((workspace / "build").rglob(library_name))
    if matches:
        return matches[0]

    raise FileNotFoundError(
        "未找到 erob_c_api 共享库，请先执行 cmake 构建，或通过 library_path / EROB_C_API_LIB 指定路径。"
    )


class ErobController:
    def __init__(
        self,
        config_path: str | None = None,
        library_path: str | None = None,
    ) -> None:
        self._library_path = _resolve_library_path(library_path)
        self._lib = ctypes.CDLL(str(self._library_path))
        self._configure_api()
        workspace = _repo_root()
        resolved_config = Path(config_path) if config_path else workspace / "config" / "erob_arm.yaml"
        self._config_path = str(resolved_config)
        self._handle = self._lib.erob_controller_create(self._config_path.encode("utf-8"))
        if not self._handle:
            raise ControllerError("创建控制器失败")
        self._lock = threading.Lock()
        self._cached_states: list[AxisState] = []
        self._cached_preferred_adapter = ""
        self._cached_degraded = False
        self._metadata = self._read_metadata()

    def close(self) -> None:
        if self._handle:
            with self._lock:
                self._lib.erob_controller_shutdown(self._handle)
                self._lib.erob_controller_destroy(self._handle)
                self._handle = None

    def __enter__(self) -> ErobController:
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass

    @property
    def metadata(self) -> list[AxisMetadata]:
        return list(self._metadata)

    @property
    def library_path(self) -> Path:
        return self._library_path

    @property
    def config_path(self) -> str:
        return self._config_path

    def initialize(self) -> None:
        with self._lock:
            self._require_bool(self._lib.erob_controller_initialize(self._handle), "initialize")

    def shutdown(self) -> None:
        with self._lock:
            self._require_bool(self._lib.erob_controller_shutdown(self._handle), "shutdown")

    def enable_axis(self, axis_id: int) -> None:
        with self._lock:
            self._require_bool(self._lib.erob_controller_enable_axis(self._handle, axis_id), "enable_axis")

    def disable_axis(self, axis_id: int) -> None:
        with self._lock:
            self._require_bool(self._lib.erob_controller_disable_axis(self._handle, axis_id), "disable_axis")

    def reset_fault(self, axis_id: int) -> None:
        with self._lock:
            self._require_bool(self._lib.erob_controller_reset_fault(self._handle, axis_id), "reset_fault")

    def enable_all(self) -> None:
        with self._lock:
            self._require_bool(self._lib.erob_controller_enable_all(self._handle), "enable_all")

    def disable_all(self) -> None:
        with self._lock:
            self._require_bool(self._lib.erob_controller_disable_all(self._handle), "disable_all")

    def quick_stop_axis(self, axis_id: int) -> None:
        with self._lock:
            self._require_bool(self._lib.erob_controller_quick_stop_axis(self._handle, axis_id), "quick_stop_axis")

    def quick_stop_all(self) -> None:
        with self._lock:
            self._require_bool(self._lib.erob_controller_quick_stop_all(self._handle), "quick_stop_all")

    def move_to(self, axis_id: int, angle_deg: float, velocity_deg_s: float) -> None:
        with self._lock:
            self._require_bool(
                self._lib.erob_controller_move_to(self._handle, axis_id, angle_deg, velocity_deg_s),
                "move_to",
            )

    def start_follow(self, axis_id: int) -> None:
        with self._lock:
            self._require_bool(self._lib.erob_controller_start_follow(self._handle, axis_id), "start_follow")

    def update_follow_target(self, axis_id: int, angle_deg: float) -> None:
        with self._lock:
            self._require_bool(
                self._lib.erob_controller_update_follow_target(self._handle, axis_id, angle_deg),
                "update_follow_target",
            )

    def stop_follow(self, axis_id: int) -> None:
        with self._lock:
            self._require_bool(self._lib.erob_controller_stop_follow(self._handle, axis_id), "stop_follow")

    def set_preferred_adapter(self, adapter_name: str) -> None:
        with self._lock:
            self._lib.erob_controller_set_preferred_adapter(self._handle, adapter_name.encode("utf-8"))

    def get_preferred_adapter(self) -> str:
        with self._lock:
            self._cached_preferred_adapter = _decode_c_string(
                self._lib.erob_controller_get_preferred_adapter(self._handle)
            )
            return self._cached_preferred_adapter

    def try_get_preferred_adapter(self) -> str:
        if not self._lock.acquire(blocking=False):
            return self._cached_preferred_adapter
        try:
            self._cached_preferred_adapter = _decode_c_string(
                self._lib.erob_controller_get_preferred_adapter(self._handle)
            )
            return self._cached_preferred_adapter
        finally:
            self._lock.release()

    def is_degraded(self) -> bool:
        with self._lock:
            self._cached_degraded = bool(self._lib.erob_controller_is_degraded(self._handle))
            return self._cached_degraded

    def try_is_degraded(self) -> bool:
        if not self._lock.acquire(blocking=False):
            return self._cached_degraded
        try:
            self._cached_degraded = bool(self._lib.erob_controller_is_degraded(self._handle))
            return self._cached_degraded
        finally:
            self._lock.release()

    def scan_adapters(self) -> list[dict[str, Any]]:
        with self._lock:
            return self._json_call(self._lib.erob_controller_scan_adapters_json)

    def scan_motors(self) -> list[dict[str, Any]]:
        with self._lock:
            return self._json_call(self._lib.erob_controller_scan_motors_json)

    def rescan_current(self) -> list[dict[str, Any]]:
        with self._lock:
            return self._json_call(self._lib.erob_controller_rescan_current_json)

    def rescan_all(self) -> list[dict[str, Any]]:
        with self._lock:
            return self._json_call(self._lib.erob_controller_rescan_all_json)

    def get_discovered_motors(self) -> list[dict[str, Any]]:
        with self._lock:
            return self._json_call(self._lib.erob_controller_get_discovered_motors_json)

    def get_binding_reports(self) -> list[dict[str, Any]]:
        with self._lock:
            return self._json_call(self._lib.erob_controller_get_binding_reports_json)

    def get_axis_state(self, axis_id: int) -> AxisState:
        with self._lock:
            return self._read_axis_state(axis_id)

    def try_get_all_axis_states(self) -> list[AxisState]:
        if not self._lock.acquire(blocking=False):
            return list(self._cached_states)
        try:
            states = [self._read_axis_state(axis_id) for axis_id in range(len(self._metadata))]
            self._cached_states = states
            return list(states)
        finally:
            self._lock.release()

    def last_error(self) -> str:
        if not self._handle:
            return ""
        return _decode_c_string(self._lib.erob_controller_last_error(self._handle))

    def _configure_api(self) -> None:
        self._lib.erob_controller_create.argtypes = [ctypes.c_char_p]
        self._lib.erob_controller_create.restype = ctypes.c_void_p
        self._lib.erob_controller_destroy.argtypes = [ctypes.c_void_p]
        self._lib.erob_controller_destroy.restype = None

        bool_calls = [
            "erob_controller_initialize",
            "erob_controller_shutdown",
            "erob_controller_enable_all",
            "erob_controller_disable_all",
            "erob_controller_quick_stop_all",
            "erob_controller_is_degraded",
        ]
        for name in bool_calls:
            getattr(self._lib, name).argtypes = [ctypes.c_void_p]
            getattr(self._lib, name).restype = ctypes.c_int

        one_axis_calls = [
            "erob_controller_enable_axis",
            "erob_controller_disable_axis",
            "erob_controller_reset_fault",
            "erob_controller_quick_stop_axis",
            "erob_controller_start_follow",
            "erob_controller_stop_follow",
        ]
        for name in one_axis_calls:
            getattr(self._lib, name).argtypes = [ctypes.c_void_p, ctypes.c_int]
            getattr(self._lib, name).restype = ctypes.c_int

        self._lib.erob_controller_move_to.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_double,
            ctypes.c_double,
        ]
        self._lib.erob_controller_move_to.restype = ctypes.c_int

        self._lib.erob_controller_update_follow_target.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_double,
        ]
        self._lib.erob_controller_update_follow_target.restype = ctypes.c_int

        self._lib.erob_controller_axis_count.argtypes = [ctypes.c_void_p]
        self._lib.erob_controller_axis_count.restype = ctypes.c_int

        self._lib.erob_controller_get_axis_metadata.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.POINTER(_AxisMetadataStruct),
        ]
        self._lib.erob_controller_get_axis_metadata.restype = ctypes.c_int

        self._lib.erob_controller_get_axis_state.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.POINTER(_AxisStateStruct),
        ]
        self._lib.erob_controller_get_axis_state.restype = ctypes.c_int

        string_calls = [
            "erob_controller_last_error",
            "erob_controller_scan_adapters_json",
            "erob_controller_scan_motors_json",
            "erob_controller_rescan_current_json",
            "erob_controller_rescan_all_json",
            "erob_controller_get_discovered_motors_json",
            "erob_controller_get_binding_reports_json",
            "erob_controller_get_preferred_adapter",
        ]
        for name in string_calls:
            getattr(self._lib, name).argtypes = [ctypes.c_void_p]
            getattr(self._lib, name).restype = ctypes.c_char_p

        self._lib.erob_controller_set_preferred_adapter.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
        self._lib.erob_controller_set_preferred_adapter.restype = None

    def _read_metadata(self) -> list[AxisMetadata]:
        axis_count = int(self._lib.erob_controller_axis_count(self._handle))
        metadata: list[AxisMetadata] = []
        for axis_id in range(axis_count):
            raw = _AxisMetadataStruct()
            ok = self._lib.erob_controller_get_axis_metadata(self._handle, axis_id, ctypes.byref(raw))
            if not ok:
                raise ControllerError(self.last_error() or f"读取轴 {axis_id} 配置失败")
            metadata.append(
                AxisMetadata(
                    logical_axis_id=raw.logical_axis_id,
                    joint_name=_decode_c_string(raw.joint_name),
                    min_angle_deg=raw.min_angle_deg,
                    max_angle_deg=raw.max_angle_deg,
                    max_velocity_deg_s=raw.max_velocity_deg_s,
                    max_accel_deg_s2=raw.max_accel_deg_s2,
                    max_decel_deg_s2=raw.max_decel_deg_s2,
                    configured_serial=_decode_c_string(raw.configured_serial),
                    configured_eep_man=int(raw.configured_eep_man),
                    configured_eep_id=int(raw.configured_eep_id),
                    configured_eep_rev=int(raw.configured_eep_rev),
                )
            )
        return metadata

    def _read_axis_state(self, axis_id: int) -> AxisState:
        raw = _AxisStateStruct()
        ok = self._lib.erob_controller_get_axis_state(self._handle, axis_id, ctypes.byref(raw))
        if not ok:
            raise ControllerError(self.last_error() or f"读取轴 {axis_id} 状态失败")
        return AxisState(
            online=bool(raw.online),
            enabled=bool(raw.enabled),
            fault=bool(raw.fault),
            statusword=int(raw.statusword),
            controlword=int(raw.controlword),
            motion_mode=int(raw.motion_mode),
            cia402_state=int(raw.cia402_state),
            position_mode_state=int(raw.position_mode_state),
            follow_mode_state=int(raw.follow_mode_state),
            actual_position_count=int(raw.actual_position_count),
            actual_velocity_count_s=int(raw.actual_velocity_count_s),
            actual_torque=int(raw.actual_torque),
            actual_angle_deg=float(raw.actual_angle_deg),
            actual_velocity_deg_s=float(raw.actual_velocity_deg_s),
            target_angle_deg=float(raw.target_angle_deg),
            position_error_deg=float(raw.position_error_deg),
            target_reached=bool(raw.target_reached),
            near_positive_limit=bool(raw.near_positive_limit),
            near_negative_limit=bool(raw.near_negative_limit),
            al_status_code=int(raw.al_status_code),
            last_error_code=int(raw.last_error_code),
            motion_mode_name=_decode_c_string(raw.motion_mode_name),
            cia402_state_name=_decode_c_string(raw.cia402_state_name),
            position_mode_name=_decode_c_string(raw.position_mode_name),
            follow_mode_name=_decode_c_string(raw.follow_mode_name),
        )

    def _json_call(self, function: Any) -> list[dict[str, Any]]:
        payload = _decode_c_string(function(self._handle))
        if not payload:
            return []
        try:
            return json.loads(payload)
        except json.JSONDecodeError as error:
            raise ControllerError(f"JSON 解析失败: {error}: {payload}") from error

    def _require_bool(self, ok: int, action: str) -> None:
        if ok:
            return
        error_message = self.last_error() or f"{action} 失败"
        raise ControllerError(error_message)