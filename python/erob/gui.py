from __future__ import annotations

import importlib
import sys
from concurrent.futures import Future, ThreadPoolExecutor
from typing import Callable

try:
    _qt_core = importlib.import_module("PyQt5.QtCore")
    _qt_gui = importlib.import_module("PyQt5.QtGui")
    _qt_widgets = importlib.import_module("PyQt5.QtWidgets")
except ImportError:
    _qt_core = importlib.import_module("PyQt6.QtCore")
    _qt_gui = importlib.import_module("PyQt6.QtGui")
    _qt_widgets = importlib.import_module("PyQt6.QtWidgets")

Qt = _qt_core.Qt
QObject = _qt_core.QObject
QTimer = _qt_core.QTimer
pyqtSignal = _qt_core.pyqtSignal
QAbstractItemView = _qt_widgets.QAbstractItemView
QApplication = _qt_widgets.QApplication
QComboBox = _qt_widgets.QComboBox
QDial = _qt_widgets.QDial
QDoubleSpinBox = _qt_widgets.QDoubleSpinBox
QFrame = _qt_widgets.QFrame
QGridLayout = _qt_widgets.QGridLayout
QGroupBox = _qt_widgets.QGroupBox
QHBoxLayout = _qt_widgets.QHBoxLayout
QLabel = _qt_widgets.QLabel
QLineEdit = _qt_widgets.QLineEdit
QListWidget = _qt_widgets.QListWidget
QListWidgetItem = _qt_widgets.QListWidgetItem
QMainWindow = _qt_widgets.QMainWindow
QMessageBox = _qt_widgets.QMessageBox
QPlainTextEdit = _qt_widgets.QPlainTextEdit
QPushButton = _qt_widgets.QPushButton
QSizePolicy = _qt_widgets.QSizePolicy
QSplitter = _qt_widgets.QSplitter
QTabWidget = _qt_widgets.QTabWidget
QTableWidget = _qt_widgets.QTableWidget
QTableWidgetItem = _qt_widgets.QTableWidgetItem
QVBoxLayout = _qt_widgets.QVBoxLayout
QWidget = _qt_widgets.QWidget
QFont = _qt_gui.QFont
QColor = _qt_gui.QColor

if hasattr(Qt, "AlignmentFlag"):
    _ALIGN_HCENTER = Qt.AlignmentFlag.AlignHCenter
    _FRAME_STYLED_PANEL = QFrame.Shape.StyledPanel
    _POLICY_EXPANDING = QSizePolicy.Policy.Expanding
    _POLICY_PREFERRED = QSizePolicy.Policy.Preferred
    _USER_ROLE = Qt.ItemDataRole.UserRole
    _ITEM_IS_EDITABLE = Qt.ItemFlag.ItemIsEditable
    _SELECTION_EXTENDED = QAbstractItemView.SelectionMode.ExtendedSelection
    _ORIENTATION_HORIZONTAL = Qt.Orientation.Horizontal
    _ORIENTATION_VERTICAL = Qt.Orientation.Vertical
else:
    _ALIGN_HCENTER = Qt.AlignHCenter
    _FRAME_STYLED_PANEL = QFrame.StyledPanel
    _POLICY_EXPANDING = QSizePolicy.Expanding
    _POLICY_PREFERRED = QSizePolicy.Preferred
    _USER_ROLE = Qt.UserRole
    _ITEM_IS_EDITABLE = Qt.ItemIsEditable
    _SELECTION_EXTENDED = QAbstractItemView.ExtendedSelection
    _ORIENTATION_HORIZONTAL = Qt.Horizontal
    _ORIENTATION_VERTICAL = Qt.Vertical

from .sdk import AxisMetadata, AxisState, ControllerError, ErobController


FOLLOW_ACTIVE_STATES = {"EnteringCsvMode", "Following", "Stopping"}


class UiSignals(QObject):
    command_finished = pyqtSignal(str, bool, str, object)


class StatusChip(QLabel):
    def __init__(self, text: str, parent: QWidget | None = None) -> None:
        super().__init__(text, parent)
        self.setObjectName("StatusChip")

    def set_color(self, background: str, foreground: str = "#f7f1e8") -> None:
        self.setStyleSheet(
            f"background:{background};color:{foreground};border-radius:12px;padding:4px 10px;font-weight:600;"
        )


class MainWindow(QMainWindow):
    def __init__(self, controller: ErobController) -> None:
        super().__init__()
        self.controller = controller
        self.axis_metadata = controller.metadata
        self.axis_states: list[AxisState] = []
        self._follow_active_axes: set[int] = set()
        self._dial_scale = 10
        self._pending_follow_target = 0.0
        self.executor = ThreadPoolExecutor(max_workers=1)
        self.signals = UiSignals()
        self.signals.command_finished.connect(self._handle_command_finished)
        self.binding_reports: dict[int, dict] = {}
        self.discovery_rows: list[dict] = []
        self.adapter_rows: list[dict] = []
        self._follow_timer = QTimer(self)
        self._follow_timer.setSingleShot(True)
        self._follow_timer.setInterval(50)
        self._follow_timer.timeout.connect(self._emit_follow_target)
        self._build_ui()
        self._refresh_static_views()
        self._refresh_timer = QTimer(self)
        self._refresh_timer.setInterval(150)
        self._refresh_timer.timeout.connect(self.refresh_states)
        self._refresh_timer.start()

    def closeEvent(self, event) -> None:
        self.executor.shutdown(wait=False, cancel_futures=True)
        self.controller.close()
        super().closeEvent(event)

    def _build_ui(self) -> None:
        self.setWindowTitle("eRob Python Control Console")
        self.resize(1500, 980)
        font = QFont("Noto Sans CJK SC", 10)
        QApplication.instance().setFont(font)

        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(18, 18, 18, 18)
        root.setSpacing(14)

        header = QFrame()
        header.setObjectName("HeaderPanel")
        header_layout = QGridLayout(header)
        header_layout.setHorizontalSpacing(12)
        header_layout.setVerticalSpacing(10)
        header_layout.addWidget(QLabel("配置文件"), 0, 0)
        self.config_edit = QLineEdit(self.controller.config_path)
        self.config_edit.setReadOnly(True)
        header_layout.addWidget(self.config_edit, 0, 1, 1, 3)
        header_layout.addWidget(QLabel("共享库"), 1, 0)
        self.library_edit = QLineEdit(str(self.controller.library_path))
        self.library_edit.setReadOnly(True)
        header_layout.addWidget(self.library_edit, 1, 1, 1, 3)
        header_layout.addWidget(QLabel("适配器"), 2, 0)
        self.adapter_combo = QComboBox()
        self.adapter_combo.setEditable(False)
        header_layout.addWidget(self.adapter_combo, 2, 1)
        self.preferred_label = QLabel("preferred: -")
        self.preferred_label.setObjectName("HeaderNote")
        header_layout.addWidget(self.preferred_label, 2, 2)
        self.degraded_label = QLabel("总线状态: 未初始化")
        self.degraded_label.setObjectName("HeaderStatus")
        header_layout.addWidget(self.degraded_label, 2, 3)

        button_row = QHBoxLayout()
        for text, handler in [
            ("扫描网卡", self.scan_adapters),
            ("全网重扫电机", self.rescan_all),
            ("当前网卡重扫", self.rescan_current),
            ("初始化", self.initialize_controller),
            ("关闭", self.shutdown_controller),
            ("全使能", lambda: self.submit("enable-all", self.controller.enable_all)),
            ("全失能", lambda: self.submit("disable-all", self.controller.disable_all)),
            ("总急停", lambda: self.submit("quick-stop-all", self.controller.quick_stop_all)),
        ]:
            button = QPushButton(text)
            if text == "总急停":
                button.setObjectName("DangerButton")
            button.clicked.connect(handler)
            button_row.addWidget(button)
        button_row.addStretch(1)
        header_layout.addLayout(button_row, 3, 0, 1, 4)
        root.addWidget(header)

        main_splitter = QSplitter(_ORIENTATION_HORIZONTAL)
        main_splitter.setChildrenCollapsible(False)

        # Left side: Control Panel
        control_panel = self._build_control_panel()
        main_splitter.addWidget(control_panel)

        # Right side: Vertical Splitter (Status Table and Tabs)
        right_splitter = QSplitter(_ORIENTATION_VERTICAL)
        right_splitter.setChildrenCollapsible(False)

        status_panel = self._build_status_panel()
        right_splitter.addWidget(status_panel)

        # Bottom part of right side: Tab Widget
        tabs = QTabWidget()
        
        self.adapter_table = QTableWidget(0, 4)
        self.adapter_table.setHorizontalHeaderLabels(["名称", "描述", "扫描", "从站数"])
        self.adapter_table.horizontalHeader().setStretchLastSection(True)
        tabs.addTab(self.adapter_table, "网卡扫描")

        self.motor_table = QTableWidget(0, 5)
        self.motor_table.setHorizontalHeaderLabels(["适配器", "从站", "名称", "序列号", "eRob"])
        self.motor_table.horizontalHeader().setStretchLastSection(True)
        tabs.addTab(self.motor_table, "发现的电机")

        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        tabs.addTab(self.log_view, "事件日志")

        right_splitter.addWidget(tabs)
        right_splitter.setStretchFactor(0, 3)
        right_splitter.setStretchFactor(1, 2)

        main_splitter.addWidget(right_splitter)
        main_splitter.setStretchFactor(0, 2)
        main_splitter.setStretchFactor(1, 5)

        root.addWidget(main_splitter, 1)

        self.setCentralWidget(central)
        self.setStyleSheet(_STYLE_SHEET)

    def _build_control_panel(self) -> QWidget:
        panel = QFrame()
        panel.setObjectName("ControlPanel")
        panel.setFrameShape(_FRAME_STYLED_PANEL)
        panel.setSizePolicy(_POLICY_EXPANDING, _POLICY_PREFERRED)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(14)

        title = QLabel("共享控制面板")
        title.setObjectName("AxisTitle")
        layout.addWidget(title)

        self.selection_chip = StatusChip("未选择目标轴")
        self.selection_chip.set_color("#6b5e54")
        layout.addWidget(self.selection_chip)

        selection_box = QGroupBox("控制目标选择")
        selection_layout = QVBoxLayout(selection_box)
        selection_layout.addWidget(QLabel("可多选。所有动作都会同时作用于选中的目标轴。"))
        self.selection_detail_label = QLabel("已选: -")
        self.selection_detail_label.setObjectName("InfoLine")
        self.binding_detail_label = QLabel("绑定: -")
        self.binding_detail_label.setWordWrap(True)
        self.binding_detail_label.setObjectName("InfoLine")

        self.target_list = QListWidget()
        self.target_list.setSelectionMode(_SELECTION_EXTENDED)
        self.target_list.itemSelectionChanged.connect(self._update_selection_summary)
        for metadata in self.axis_metadata:
            item = QListWidgetItem(f"Axis {metadata.logical_axis_id} | {metadata.joint_name}")
            item.setData(_USER_ROLE, metadata.logical_axis_id)
            self.target_list.addItem(item)
        if self.target_list.count() > 0:
            self.target_list.item(0).setSelected(True)
        selection_layout.addWidget(self.target_list)

        selection_buttons = QHBoxLayout()
        select_all_button = QPushButton("全选")
        select_all_button.clicked.connect(self._select_all_axes)
        clear_button = QPushButton("清空")
        clear_button.clicked.connect(self._clear_selected_axes)
        selection_buttons.addWidget(select_all_button)
        selection_buttons.addWidget(clear_button)
        selection_layout.addLayout(selection_buttons)
        selection_layout.addWidget(self.selection_detail_label)
        selection_layout.addWidget(self.binding_detail_label)
        layout.addWidget(selection_box)

        motion_box = QGroupBox("点位运动")
        motion_layout = QGridLayout(motion_box)
        motion_layout.addWidget(QLabel("目标角度"), 0, 0)
        min_angle = min(metadata.min_angle_deg for metadata in self.axis_metadata)
        max_angle = max(metadata.max_angle_deg for metadata in self.axis_metadata)
        max_velocity = max(metadata.max_velocity_deg_s for metadata in self.axis_metadata)
        self.target_spin = QDoubleSpinBox()
        self.target_spin.setRange(min_angle, max_angle)
        self.target_spin.setDecimals(1)
        self.target_spin.setSingleStep(1.0)
        motion_layout.addWidget(self.target_spin, 0, 1)
        motion_layout.addWidget(QLabel("速度"), 1, 0)
        self.velocity_spin = QDoubleSpinBox()
        self.velocity_spin.setRange(1.0, max(1.0, max_velocity))
        self.velocity_spin.setValue(min(60.0, max_velocity))
        self.velocity_spin.setDecimals(1)
        self.velocity_spin.setSingleStep(5.0)
        motion_layout.addWidget(self.velocity_spin, 1, 1)
        move_button = QPushButton("对已选轴执行 moveTo")
        move_button.clicked.connect(self._move_selected_axes)
        motion_layout.addWidget(move_button, 2, 0, 1, 2)
        layout.addWidget(motion_box)

        follow_box = QGroupBox("随动轮盘")
        follow_layout = QVBoxLayout(follow_box)
        self.follow_target_label = QLabel("目标: 0.0°")
        self.follow_target_label.setObjectName("DialLabel")
        follow_layout.addWidget(self.follow_target_label, alignment=_ALIGN_HCENTER)
        self.follow_dial = QDial()
        self.follow_dial.setNotchesVisible(True)
        self.follow_dial.setWrapping(False)
        self.follow_dial.setRange(int(min_angle * self._dial_scale), int(max_angle * self._dial_scale))
        self.follow_dial.valueChanged.connect(self._handle_dial_change)
        follow_layout.addWidget(self.follow_dial, alignment=_ALIGN_HCENTER)
        follow_buttons = QHBoxLayout()
        start_follow_button = QPushButton("启动随动")
        start_follow_button.clicked.connect(self._start_follow_selected)
        stop_follow_button = QPushButton("停止随动")
        stop_follow_button.clicked.connect(self._stop_follow_selected)
        follow_buttons.addWidget(start_follow_button)
        follow_buttons.addWidget(stop_follow_button)
        follow_layout.addLayout(follow_buttons)
        self.follow_hint_label = QLabel("轮盘目标会发送到已选且已经进入 follow 模式的轴。")
        self.follow_hint_label.setObjectName("InfoLine")
        self.follow_hint_label.setWordWrap(True)
        follow_layout.addWidget(self.follow_hint_label)
        layout.addWidget(follow_box)

        action_box = QGroupBox("基础动作")
        action_layout = QGridLayout(action_box)
        enable_button = QPushButton("使能已选轴")
        enable_button.clicked.connect(lambda: self._submit_selected("enable-selected", self.controller.enable_axis))
        disable_button = QPushButton("失能已选轴")
        disable_button.clicked.connect(lambda: self._submit_selected("disable-selected", self.controller.disable_axis))
        reset_button = QPushButton("清故障")
        reset_button.clicked.connect(lambda: self._submit_selected("reset-selected", self.controller.reset_fault))
        quick_stop_button = QPushButton("急停已选轴")
        quick_stop_button.setObjectName("DangerButton")
        quick_stop_button.clicked.connect(lambda: self._submit_selected("quick-stop-selected", self.controller.quick_stop_axis))
        action_layout.addWidget(enable_button, 0, 0)
        action_layout.addWidget(disable_button, 0, 1)
        action_layout.addWidget(reset_button, 1, 0)
        action_layout.addWidget(quick_stop_button, 1, 1)
        layout.addWidget(action_box)
        layout.addStretch(1)
        return panel

    def _build_status_panel(self) -> QWidget:
        panel = self._wrap_panel("轴状态总览", QWidget())
        layout = panel.layout()
        self.axis_table = QTableWidget(len(self.axis_metadata), 11)
        self.axis_table.setHorizontalHeaderLabels(
            ["选中", "轴", "关节", "在线", "使能", "故障", "当前角度", "目标角度", "速度", "模式", "绑定"]
        )
        self.axis_table.horizontalHeader().setStretchLastSection(True)
        self.axis_table.verticalHeader().setVisible(False)
        self.axis_table.setMinimumHeight(320)
        layout.addWidget(self.axis_table)
        return panel

    def _wrap_panel(self, title: str, widget: QWidget) -> QGroupBox:
        box = QGroupBox(title)
        layout = QVBoxLayout(box)
        layout.addWidget(widget)
        return box

    def submit(self, label: str, func: Callable[[], object]) -> None:
        future = self.executor.submit(self._safe_call, func)
        future.add_done_callback(lambda done, task_label=label: self._forward_result(task_label, done))

    def _safe_call(self, func: Callable[[], object]) -> object:
        return func()

    def _forward_result(self, label: str, future: Future) -> None:
        try:
            result = future.result()
            self.signals.command_finished.emit(label, True, "", result)
        except Exception as error:
            self.signals.command_finished.emit(label, False, str(error), None)

    def _handle_command_finished(self, label: str, ok: bool, message: str, result: object) -> None:
        if ok:
            if label != "follow-target-selected":
                self._append_log(f"[{label}] 完成")
            if label == "scan-adapters":
                self.adapter_rows = list(result or [])
                self._populate_adapter_table(self.adapter_rows)
            elif label in {"rescan-all", "rescan-current"}:
                self.discovery_rows = list(result or [])
                self._populate_motor_table(self.discovery_rows)
                self._refresh_binding_reports()
            elif label in {"initialize", "shutdown"}:
                self._refresh_static_views()
        else:
            self._append_log(f"[{label}] 失败: {message}")
            if label != "follow-target-selected":
                QMessageBox.critical(self, "控制命令失败", message)
        self.refresh_states()

    def initialize_controller(self) -> None:
        adapter = self.adapter_combo.currentText().strip()

        def task() -> None:
            if adapter:
                self.controller.set_preferred_adapter(adapter)
            self.controller.initialize()

        self.submit("initialize", task)

    def shutdown_controller(self) -> None:
        self.submit("shutdown", self.controller.shutdown)

    def scan_adapters(self) -> None:
        self.submit("scan-adapters", self.controller.scan_adapters)

    def rescan_all(self) -> None:
        self.submit("rescan-all", self.controller.rescan_all)

    def rescan_current(self) -> None:
        self.submit("rescan-current", self.controller.rescan_current)

    def refresh_states(self) -> None:
        try:
            states = self.controller.try_get_all_axis_states()
        except ControllerError as error:
            self._append_log(f"[state-refresh] {error}")
            return
        if not states:
            return
        self.axis_states = states
        self._follow_active_axes = {
            axis_id for axis_id, state in enumerate(states) if state.follow_mode_name in FOLLOW_ACTIVE_STATES
        }
        if not self.binding_reports:
            self._refresh_binding_reports()
        self._populate_axis_table(states)
        self._sync_follow_dial_from_state()
        self._update_selection_summary()
        self._update_header_status(states)

    def _refresh_binding_reports(self) -> None:
        try:
            reports = self.controller.get_binding_reports()
            self.binding_reports = {int(report.get("logical_axis_id", -1)): report for report in reports}
        except ControllerError as error:
            self._append_log(f"[binding-refresh] {error}")

    def _refresh_static_views(self) -> None:
        try:
            self.discovery_rows = self.controller.get_discovered_motors()
            self.binding_reports = {
                int(report.get("logical_axis_id", -1)): report
                for report in self.controller.get_binding_reports()
            }
        except ControllerError:
            self.discovery_rows = []
            self.binding_reports = {}
        self._populate_motor_table(self.discovery_rows)
        self.preferred_label.setText(f"preferred: {self.controller.get_preferred_adapter() or '-'}")
        self._update_selection_summary()

    def _populate_adapter_table(self, rows: list[dict]) -> None:
        self.adapter_table.setRowCount(len(rows))
        self.adapter_combo.blockSignals(True)
        self.adapter_combo.clear()
        for row_index, row in enumerate(rows):
            values = [
                row.get("name", ""),
                row.get("description", ""),
                "yes" if row.get("scan_success") else "no",
                str(row.get("discovered_slave_count", 0)),
            ]
            for column_index, value in enumerate(values):
                item = QTableWidgetItem(str(value))
                item.setFlags(item.flags() & ~_ITEM_IS_EDITABLE)
                self.adapter_table.setItem(row_index, column_index, item)
            name = row.get("name", "")
            if name:
                self.adapter_combo.addItem(name)
        preferred = self.controller.get_preferred_adapter()
        if preferred:
            index = self.adapter_combo.findText(preferred)
            if index >= 0:
                self.adapter_combo.setCurrentIndex(index)
        self.adapter_combo.blockSignals(False)

    def _populate_motor_table(self, rows: list[dict]) -> None:
        self.motor_table.setRowCount(len(rows))
        for row_index, row in enumerate(rows):
            values = [
                row.get("adapter_name", ""),
                row.get("slave_index", ""),
                row.get("name", ""),
                row.get("serial_number", ""),
                "yes" if row.get("is_erob_motor") else "no",
            ]
            for column_index, value in enumerate(values):
                item = QTableWidgetItem(str(value))
                item.setFlags(item.flags() & ~_ITEM_IS_EDITABLE)
                self.motor_table.setItem(row_index, column_index, item)

    def _populate_axis_table(self, states: list[AxisState]) -> None:
        selected_axes = set(self.selected_axis_ids(silent=True))
        self.axis_table.setRowCount(len(self.axis_metadata))
        for row_index, metadata in enumerate(self.axis_metadata):
            state = states[row_index] if row_index < len(states) else None
            report = self.binding_reports.get(row_index, {})
            values = [
                "yes" if row_index in selected_axes else "",
                str(metadata.logical_axis_id),
                metadata.joint_name,
                "在线" if state and state.online else "离线",
                "使能" if state and state.enabled else "未使能",
                "故障" if state and state.fault else "正常",
                f"{state.actual_angle_deg:.1f}°" if state else "-",
                f"{state.target_angle_deg:.1f}°" if state else "-",
                f"{state.actual_velocity_deg_s:.1f}°/s" if state else "-",
                (
                    f"{state.motion_mode_name}/{state.follow_mode_name}/{state.position_mode_name}"
                    if state else "-"
                ),
                report.get("detail", metadata.configured_serial or "-"),
            ]
            for column_index, value in enumerate(values):
                item = QTableWidgetItem(str(value))
                item.setFlags(item.flags() & ~_ITEM_IS_EDITABLE)
                if row_index in selected_axes:
                    item.setBackground(QColor("#efe1c7"))
                if column_index == 5 and state is not None:
                    item.setForeground(QColor("#bb3e03") if state.fault else QColor("#2b9348"))
                self.axis_table.setItem(row_index, column_index, item)

    def _update_header_status(self, states: list[AxisState]) -> None:
        degraded = self.controller.try_is_degraded()
        faults = sum(1 for state in states if state.fault)
        enabled = sum(1 for state in states if state.enabled)
        self.preferred_label.setText(f"preferred: {self.controller.try_get_preferred_adapter() or '-'}")
        self.degraded_label.setText(
            f"总线状态: {'degraded' if degraded else 'ready'} | enabled {enabled}/{len(states)} | fault {faults}"
        )
        self.degraded_label.setStyleSheet(
            f"color:{'#c44536' if degraded or faults else '#1c7c54'};font-weight:700;"
        )

    def selected_axis_ids(self, silent: bool = False) -> list[int]:
        axis_ids = [item.data(_USER_ROLE) for item in self.target_list.selectedItems()]
        if axis_ids or silent:
            return axis_ids
        self._append_log("[selection] 请先选择至少一个控制目标")
        QMessageBox.information(self, "未选择目标", "请先选择至少一个控制目标轴。")
        return []

    def _selected_metadata(self) -> list[AxisMetadata]:
        selected = set(self.selected_axis_ids(silent=True))
        return [metadata for metadata in self.axis_metadata if metadata.logical_axis_id in selected]

    def _update_selection_summary(self) -> None:
        selected_metadata = self._selected_metadata()
        if not selected_metadata:
            self.selection_chip.setText("未选择目标轴")
            self.selection_chip.set_color("#6b5e54")
            self.selection_detail_label.setText("已选: -")
            self.binding_detail_label.setText("绑定: -")
            return

        names = ", ".join(metadata.joint_name for metadata in selected_metadata)
        self.selection_chip.setText(f"已选 {len(selected_metadata)} 轴")
        self.selection_chip.set_color("#9c6644")
        self.selection_detail_label.setText(f"已选: {names}")

        binding_lines = []
        for metadata in selected_metadata:
            report = self.binding_reports.get(metadata.logical_axis_id, {})
            detail = report.get("detail", metadata.configured_serial or "未绑定")
            binding_lines.append(f"{metadata.joint_name}: {detail}")
        self.binding_detail_label.setText("绑定: " + " | ".join(binding_lines))

    def _select_all_axes(self) -> None:
        for index in range(self.target_list.count()):
            self.target_list.item(index).setSelected(True)

    def _clear_selected_axes(self) -> None:
        self.target_list.clearSelection()

    def _run_axis_batch(self, axis_ids: list[int], operation: Callable[[int], None]) -> list[int]:
        errors = []
        for axis_id in axis_ids:
            try:
                operation(axis_id)
            except Exception as error:
                errors.append(f"axis {axis_id}: {error}")
        if errors:
            raise ControllerError("；".join(errors))
        return axis_ids

    def _submit_selected(self, label: str, operation: Callable[[int], None]) -> None:
        axis_ids = self.selected_axis_ids()
        if not axis_ids:
            return
        self.submit(label, lambda axes=axis_ids, op=operation: self._run_axis_batch(axes, op))

    def _move_selected_axes(self) -> None:
        axis_ids = self.selected_axis_ids()
        if not axis_ids:
            return
        angle = self.target_spin.value()
        velocity = self.velocity_spin.value()
        self.submit(
            "move-selected",
            lambda axes=axis_ids, target=angle, speed=velocity: self._run_axis_batch(
                axes,
                lambda axis_id: self.controller.move_to(axis_id, target, speed),
            ),
        )

    def _start_follow_selected(self) -> None:
        self._submit_selected("start-follow-selected", self.controller.start_follow)

    def _stop_follow_selected(self) -> None:
        self._submit_selected("stop-follow-selected", self.controller.stop_follow)

    def _selected_follow_axes(self) -> list[int]:
        selected_axes = self.selected_axis_ids(silent=True)
        return [axis_id for axis_id in selected_axes if axis_id in self._follow_active_axes]

    def _handle_dial_change(self, raw_value: int) -> None:
        self._pending_follow_target = raw_value / self._dial_scale
        self.follow_target_label.setText(f"目标: {self._pending_follow_target:.1f}°")
        if self._selected_follow_axes():
            self._follow_timer.start()

    def _emit_follow_target(self) -> None:
        axis_ids = self._selected_follow_axes()
        if not axis_ids:
            return
        angle = self._pending_follow_target
        self.submit(
            "follow-target-selected",
            lambda axes=axis_ids, target=angle: self._run_axis_batch(
                axes,
                lambda axis_id: self.controller.update_follow_target(axis_id, target),
            ),
        )

    def _sync_follow_dial_from_state(self) -> None:
        if self._follow_timer.isActive():
            return
        selected_follow_axes = self._selected_follow_axes()
        if not selected_follow_axes or not self.axis_states:
            return
        axis_id = selected_follow_axes[0]
        if axis_id >= len(self.axis_states):
            return
        target_angle = self.axis_states[axis_id].target_angle_deg
        self.follow_dial.blockSignals(True)
        self.follow_dial.setValue(int(target_angle * self._dial_scale))
        self.follow_dial.blockSignals(False)
        self.follow_target_label.setText(f"目标: {target_angle:.1f}°")

    def _append_log(self, message: str) -> None:
        self.log_view.appendPlainText(message)


_STYLE_SHEET = """
QMainWindow {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
        stop:0 #f3ede2, stop:0.45 #e9dfcf, stop:1 #d9d0c3);
}
QGroupBox, QFrame#HeaderPanel, QFrame#ControlPanel {
    background: rgba(252, 249, 243, 0.92);
    border: 1px solid rgba(101, 78, 57, 0.22);
    border-radius: 18px;
}
QGroupBox {
    margin-top: 12px;
    font-weight: 700;
    color: #3d2d21;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 14px;
    padding: 0 4px;
}
QPushButton {
    background: #264653;
    color: #f7f1e8;
    border: none;
    border-radius: 12px;
    padding: 10px 14px;
    font-weight: 700;
}
QPushButton:hover {
    background: #2f5f70;
}
QPushButton#DangerButton {
    background: #a63c06;
}
QPushButton#DangerButton:hover {
    background: #c2511d;
}
QLineEdit, QComboBox, QDoubleSpinBox, QPlainTextEdit, QTableWidget, QListWidget, QTabWidget {
    background: rgba(255, 252, 246, 0.96);
    border: 1px solid rgba(61, 45, 33, 0.18);
    border-radius: 10px;
    padding: 8px;
    selection-background-color: #d68c45;
}
QTabBar::tab {
    background: #e9dfcf;
    border: 1px solid rgba(101, 78, 57, 0.22);
    border-bottom: none;
    border-top-left-radius: 8px;
    border-top-right-radius: 8px;
    padding: 6px 12px;
    margin-right: 2px;
    color: #5a4a3c;
    font-weight: 600;
}
QTabBar::tab:selected {
    background: rgba(255, 252, 246, 0.96);
    color: #3d2d21;
    font-weight: 700;
}
QTabWidget::pane {
    border: 1px solid rgba(61, 45, 33, 0.18);
    border-radius: 10px;
    background: rgba(255, 252, 246, 0.96);
    top: -1px;
}
QHeaderView::section {
    background: #d9c9ad;
    color: #3d2d21;
    padding: 6px;
    border: none;
}
QSplitter::handle {
    background: rgba(156, 102, 68, 0.16);
}
QLabel#AxisTitle {
    font-size: 18px;
    font-weight: 800;
    color: #3d2d21;
}
QLabel#InfoLine, QLabel#DialLabel, QLabel#HeaderNote, QLabel#HeaderStatus {
    color: #5a4a3c;
    font-weight: 600;
}
QDial {
    background: transparent;
}
"""


def main() -> int:
    app = QApplication(sys.argv)
    controller = ErobController()
    window = MainWindow(controller)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())