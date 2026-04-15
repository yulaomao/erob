from __future__ import annotations

import importlib
import sys
from concurrent.futures import Future, ThreadPoolExecutor
from typing import Any
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
QPointF = _qt_core.QPointF
QRectF = _qt_core.QRectF
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
QScrollArea = _qt_widgets.QScrollArea
QSizePolicy = _qt_widgets.QSizePolicy
QSplitter = _qt_widgets.QSplitter
QTabWidget = _qt_widgets.QTabWidget
QTableWidget = _qt_widgets.QTableWidget
QTableWidgetItem = _qt_widgets.QTableWidgetItem
QVBoxLayout = _qt_widgets.QVBoxLayout
QWidget = _qt_widgets.QWidget
QColor = _qt_gui.QColor
QFont = _qt_gui.QFont
QPainter = _qt_gui.QPainter
QPainterPath = _qt_gui.QPainterPath
QPen = _qt_gui.QPen

if hasattr(Qt, "AlignmentFlag"):
    _ALIGN_HCENTER = Qt.AlignmentFlag.AlignHCenter
    _FRAME_STYLED_PANEL = QFrame.Shape.StyledPanel
    _POLICY_EXPANDING = QSizePolicy.Policy.Expanding
    _POLICY_PREFERRED = QSizePolicy.Policy.Preferred
    _SCROLLBAR_ALWAYS_OFF = Qt.ScrollBarPolicy.ScrollBarAlwaysOff
    _USER_ROLE = Qt.ItemDataRole.UserRole
    _ITEM_IS_EDITABLE = Qt.ItemFlag.ItemIsEditable
    _ITEM_IS_SELECTABLE = Qt.ItemFlag.ItemIsSelectable
    _ITEM_IS_ENABLED = Qt.ItemFlag.ItemIsEnabled
    _SELECTION_EXTENDED = QAbstractItemView.SelectionMode.ExtendedSelection
    _ORIENTATION_HORIZONTAL = Qt.Orientation.Horizontal
    _ORIENTATION_VERTICAL = Qt.Orientation.Vertical
else:
    _ALIGN_HCENTER = Qt.AlignHCenter
    _FRAME_STYLED_PANEL = QFrame.StyledPanel
    _POLICY_EXPANDING = QSizePolicy.Expanding
    _POLICY_PREFERRED = QSizePolicy.Preferred
    _SCROLLBAR_ALWAYS_OFF = Qt.ScrollBarAlwaysOff
    _USER_ROLE = Qt.UserRole
    _ITEM_IS_EDITABLE = Qt.ItemIsEditable
    _ITEM_IS_SELECTABLE = Qt.ItemIsSelectable
    _ITEM_IS_ENABLED = Qt.ItemIsEnabled
    _SELECTION_EXTENDED = QAbstractItemView.ExtendedSelection
    _ORIENTATION_HORIZONTAL = Qt.Horizontal
    _ORIENTATION_VERTICAL = Qt.Vertical

from .follow_test import FollowTestResult, run_follow_performance_test
from .sdk import AxisMetadata, AxisState, ControllerError, ErobController, MoveCommandStatus


FOLLOW_ACTIVE_STATES = {"EnteringCsvMode", "Following", "Stopping"}
MOVE_STATUS_OK = {MoveCommandStatus.COMPLETED, MoveCommandStatus.ISSUED}
POSITION_MOVING_STATES = {"SendingSetpoint", "Moving", "Timeout"}


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


class FollowTestPlotWidget(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._result: FollowTestResult | None = None
        self.setMinimumHeight(380)

    def clear_result(self) -> None:
        self._result = None
        self.update()

    def set_result(self, result: FollowTestResult) -> None:
        self._result = result
        self.update()

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        try:
            if hasattr(QPainter, "RenderHint"):
                painter.setRenderHint(QPainter.RenderHint.Antialiasing)
            else:
                painter.setRenderHint(QPainter.Antialiasing)
            painter.fillRect(self.rect(), QColor("#fffaf1"))

            result = self._result
            if result is None or not result.samples:
                painter.setPen(QColor("#7f6d5b"))
                alignment = Qt.AlignmentFlag.AlignCenter if hasattr(Qt, "AlignmentFlag") else Qt.AlignCenter
                painter.drawText(self.rect(), alignment, "运行一次随动性能测试后，这里会显示目标/实际/误差曲线")
                return

            outer = self.rect().adjusted(18, 18, -18, -18)
            top_rect = QRectF(outer.left(), outer.top() + 26, outer.width(), outer.height() * 0.58)
            bottom_rect = QRectF(outer.left(), top_rect.bottom() + 40, outer.width(), outer.height() * 0.26)

            self._draw_panel(
                painter,
                top_rect,
                "目标 / 实际角度",
                [sample.target_angle_deg for sample in result.samples],
                [sample.actual_angle_deg for sample in result.samples],
                result,
                QColor("#bc6c25"),
                QColor("#1d3557"),
            )
            self._draw_panel(
                painter,
                bottom_rect,
                "位置误差",
                [sample.position_error_deg for sample in result.samples],
                None,
                result,
                QColor("#c1121f"),
                None,
                zero_line=True,
            )
            self._draw_legend(painter, outer)
        finally:
            painter.end()

    def _draw_panel(
        self,
        painter: QPainter,
        rect: QRectF,
        title: str,
        primary_values: list[float],
        secondary_values: list[float] | None,
        result: FollowTestResult,
        primary_color: QColor,
        secondary_color: QColor | None,
        zero_line: bool = False,
    ) -> None:
        painter.setPen(QPen(QColor("#d4c5ad"), 1))
        painter.drawRoundedRect(rect, 12, 12)
        painter.setPen(QColor("#5a4a3c"))
        painter.drawText(QPointF(rect.left() + 12, rect.top() - 6), title)

        all_values = list(primary_values)
        if secondary_values:
            all_values.extend(secondary_values)
        if zero_line:
            all_values.append(0.0)
        min_value = min(all_values)
        max_value = max(all_values)
        if abs(max_value - min_value) < 1e-6:
            max_value += 1.0
            min_value -= 1.0

        plot_rect = rect.adjusted(12, 12, -12, -12)
        duration_s = max(result.samples[-1].elapsed_s, 1e-6)

        painter.setPen(QPen(QColor("#efe3d2"), 1))
        for fraction in (0.25, 0.5, 0.75):
            y_pos = plot_rect.top() + plot_rect.height() * fraction
            painter.drawLine(QPointF(plot_rect.left(), y_pos), QPointF(plot_rect.right(), y_pos))

        if zero_line and min_value <= 0.0 <= max_value:
            zero_y = self._map_value(0.0, min_value, max_value, plot_rect)
            painter.setPen(QPen(QColor("#c9b79c"), 1))
            painter.drawLine(QPointF(plot_rect.left(), zero_y), QPointF(plot_rect.right(), zero_y))

        self._draw_series(painter, plot_rect, result, primary_values, min_value, max_value, primary_color)
        if secondary_values and secondary_color is not None:
            self._draw_series(painter, plot_rect, result, secondary_values, min_value, max_value, secondary_color)

        painter.setPen(QColor("#7f6d5b"))
        left_alignment = Qt.AlignmentFlag.AlignLeft if hasattr(Qt, "AlignmentFlag") else Qt.AlignLeft
        alignment = Qt.AlignmentFlag.AlignRight if hasattr(Qt, "AlignmentFlag") else Qt.AlignRight
        painter.drawText(QRectF(plot_rect.left(), plot_rect.bottom() + 4, 90, 18), left_alignment, "0.0 s")
        painter.drawText(
            QRectF(plot_rect.right() - 90, plot_rect.bottom() + 4, 90, 18),
            alignment,
            f"{duration_s:.1f} s",
        )
        painter.drawText(QPointF(plot_rect.left(), plot_rect.top() + 10), f"{max_value:.1f}°")
        painter.drawText(QPointF(plot_rect.left(), plot_rect.bottom() - 2), f"{min_value:.1f}°")

    def _draw_series(
        self,
        painter: QPainter,
        plot_rect: QRectF,
        result: FollowTestResult,
        values: list[float],
        min_value: float,
        max_value: float,
        color: QColor,
    ) -> None:
        if not values:
            return
        duration_s = max(result.samples[-1].elapsed_s, 1e-6)
        path = QPainterPath()
        for index, sample in enumerate(result.samples):
            x_ratio = sample.elapsed_s / duration_s
            x_pos = plot_rect.left() + plot_rect.width() * x_ratio
            y_pos = self._map_value(values[index], min_value, max_value, plot_rect)
            point = QPointF(x_pos, y_pos)
            if index == 0:
                path.moveTo(point)
            else:
                path.lineTo(point)
        painter.setPen(QPen(color, 2.0))
        painter.drawPath(path)

    def _map_value(self, value: float, min_value: float, max_value: float, plot_rect: QRectF) -> float:
        ratio = (value - min_value) / max(max_value - min_value, 1e-6)
        return plot_rect.bottom() - ratio * plot_rect.height()

    def _draw_legend(self, painter: QPainter, rect: QRectF) -> None:
        legend_items = [
            (QColor("#bc6c25"), "目标角度"),
            (QColor("#1d3557"), "实际角度"),
            (QColor("#c1121f"), "位置误差"),
        ]
        x_pos = rect.left()
        y_pos = rect.top()
        for color, label in legend_items:
            painter.setPen(QPen(color, 3))
            painter.drawLine(QPointF(x_pos, y_pos), QPointF(x_pos + 18, y_pos))
            painter.setPen(QColor("#5a4a3c"))
            painter.drawText(QPointF(x_pos + 24, y_pos + 5), label)
            x_pos += 112


class MainWindow(QMainWindow):
    def __init__(self, controller: ErobController) -> None:
        super().__init__()
        self.controller = controller
        self.axis_metadata = controller.metadata
        self.axis_metadata_by_id = {
            metadata.logical_axis_id: metadata for metadata in self.axis_metadata
        }
        self.axis_states: list[AxisState] = []
        self.available_axis_ids: set[int] = set()
        self._follow_active_axes: set[int] = set()
        self._dial_scale = 10
        self._pending_follow_target = 0.0
        self.executor = ThreadPoolExecutor(max_workers=4)
        self.signals = UiSignals()
        self.signals.command_finished.connect(self._handle_command_finished)
        self.binding_reports: dict[int, dict] = {}
        self.discovery_rows: list[dict] = []
        self.adapter_rows: list[dict] = []
        self._follow_test_running = False
        self._follow_test_result: FollowTestResult | None = None
        self._follow_timer = QTimer(self)
        self._follow_timer.setSingleShot(True)
        self._follow_timer.setInterval(20)
        self._follow_timer.timeout.connect(self._emit_follow_target)
        self._build_ui()
        self._reload_axis_metadata()
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
        self.resize(1480, 940)
        font = QFont("Noto Sans CJK SC", 10)
        QApplication.instance().setFont(font)

        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(20, 20, 20, 20)
        root.setSpacing(16)

        body = QHBoxLayout()
        body.setSpacing(16)

        sidebar_scroll = QScrollArea()
        sidebar_scroll.setWidgetResizable(True)
        sidebar_scroll.setHorizontalScrollBarPolicy(_SCROLLBAR_ALWAYS_OFF)
        sidebar_scroll.setObjectName("SidebarScroll")
        sidebar = self._build_control_panel()
        sidebar_scroll.setWidget(sidebar)
        sidebar_scroll.setMinimumWidth(480)
        sidebar_scroll.setMaximumWidth(520)
        body.addWidget(sidebar_scroll)

        workspace_tabs = QTabWidget()
        workspace_tabs.addTab(self._build_overview_tab(), "运行总览")
        workspace_tabs.addTab(self._build_follow_test_tab(), "随动测试")
        workspace_tabs.addTab(self._build_discovery_tab(), "设备发现")
        workspace_tabs.addTab(self._build_log_tab(), "事件日志")
        workspace_tabs.setMinimumWidth(720)
        workspace_tabs.setMaximumWidth(920)
        body.addWidget(workspace_tabs)
        body.addStretch(1)

        root.addLayout(body, 1)

        self.target_list.itemSelectionChanged.connect(self._update_selection_summary)

        self.setCentralWidget(central)
        self.setStyleSheet(_STYLE_SHEET)

    def _build_control_panel(self) -> QWidget:
        panel = QFrame()
        panel.setObjectName("SidebarPanel")
        panel.setFrameShape(_FRAME_STYLED_PANEL)
        panel.setSizePolicy(_POLICY_EXPANDING, _POLICY_PREFERRED)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(14)

        title = QLabel("eRob 控制台")
        title.setObjectName("AxisTitle")
        layout.addWidget(title)

        subtitle = QLabel("左侧只保留操作，右侧集中展示运行与设备信息。")
        subtitle.setObjectName("InfoLine")
        subtitle.setWordWrap(True)
        layout.addWidget(subtitle)

        self.selection_chip = StatusChip("未选择目标轴")
        self.selection_chip.set_color("#6b5e54")
        layout.addWidget(self.selection_chip)

        layout.addWidget(self._build_system_panel())

        selection_box = QGroupBox("控制目标")
        selection_layout = QVBoxLayout(selection_box)
        selection_layout.addWidget(QLabel("扫描并绑定成功后，轴会自动出现在这里。批量动作只作用于当前选中的轴。"))
        self.target_list = QListWidget()
        self.target_list.setSelectionMode(_SELECTION_EXTENDED)
        selection_layout.addWidget(self.target_list)

        selection_buttons = QHBoxLayout()
        select_all_button = QPushButton("全选")
        select_all_button.clicked.connect(self._select_all_axes)
        clear_button = QPushButton("清空")
        clear_button.clicked.connect(self._clear_selected_axes)
        selection_buttons.addWidget(select_all_button)
        selection_buttons.addWidget(clear_button)
        selection_layout.addLayout(selection_buttons)
        layout.addWidget(selection_box)

        motion_box = QGroupBox("点位运动")
        motion_layout = QGridLayout(motion_box)
        motion_layout.addWidget(QLabel("目标角度"), 0, 0)
        min_angle, max_angle, max_velocity = self._current_axis_limits()
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

        follow_test_box = QGroupBox("随动性能测试")
        follow_test_layout = QVBoxLayout(follow_test_box)
        self.follow_test_status_label = QLabel("状态: 未运行")
        self.follow_test_status_label.setObjectName("InfoLine")
        follow_test_layout.addWidget(self.follow_test_status_label)
        follow_test_description = QLabel(
            "对当前单轴执行一段 60 Hz 下发的分段余弦目标序列，包含多次跨零反向和大跨度折返；可先收小摆幅，再逐步放大挑战强度。"
        )
        follow_test_description.setObjectName("InfoLine")
        follow_test_description.setWordWrap(True)
        follow_test_layout.addWidget(follow_test_description)
        follow_test_tuning_layout = QGridLayout()
        follow_test_tuning_layout.addWidget(QLabel("摆幅系数"), 0, 0)
        self.follow_test_amplitude_spin = QDoubleSpinBox()
        self.follow_test_amplitude_spin.setRange(20.0, 100.0)
        self.follow_test_amplitude_spin.setDecimals(0)
        self.follow_test_amplitude_spin.setSingleStep(5.0)
        self.follow_test_amplitude_spin.setSuffix(" %")
        self.follow_test_amplitude_spin.setValue(60.0)
        self.follow_test_amplitude_spin.valueChanged.connect(self._handle_follow_test_amplitude_change)
        follow_test_tuning_layout.addWidget(self.follow_test_amplitude_spin, 0, 1)
        follow_test_tuning_layout.addWidget(QLabel("建议先从 40% 到 60% 开始，再逐步加大。"), 1, 0, 1, 2)
        follow_test_layout.addLayout(follow_test_tuning_layout)
        follow_test_buttons = QHBoxLayout()
        run_follow_test_button = QPushButton("启动随动测试")
        run_follow_test_button.clicked.connect(self._start_follow_performance_test)
        clear_follow_test_button = QPushButton("清空结果")
        clear_follow_test_button.clicked.connect(self._clear_follow_test_result)
        follow_test_buttons.addWidget(run_follow_test_button)
        follow_test_buttons.addWidget(clear_follow_test_button)
        follow_test_layout.addLayout(follow_test_buttons)
        layout.addWidget(follow_test_box)

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

    def _build_system_panel(self) -> QWidget:
        panel = self._wrap_panel("系统连接", QWidget())
        layout = panel.layout()
        grid = QGridLayout()
        grid.addWidget(QLabel("配置文件"), 0, 0)
        self.config_edit = QLineEdit(self.controller.config_path)
        self.config_edit.setReadOnly(True)
        grid.addWidget(self.config_edit, 0, 1)
        grid.addWidget(QLabel("共享库"), 1, 0)
        self.library_edit = QLineEdit(str(self.controller.library_path))
        self.library_edit.setReadOnly(True)
        grid.addWidget(self.library_edit, 1, 1)
        layout.addLayout(grid)

        self.preferred_label = QLabel("最近扫描适配器: -")
        self.preferred_label.setObjectName("HeaderNote")
        self.degraded_label = QLabel("总线状态: 未连接")
        self.degraded_label.setObjectName("HeaderStatus")
        self.degraded_label.setWordWrap(True)
        layout.addWidget(self.preferred_label)
        layout.addWidget(self.degraded_label)

        action_grid = QGridLayout()
        actions = [
            ("扫描设备", self.scan_devices),
            ("关闭", self.shutdown_controller),
            ("全使能", lambda: self.submit("enable-all", self.controller.enable_all)),
            ("全失能", lambda: self.submit("disable-all", self.controller.disable_all)),
            ("总急停", lambda: self.submit("quick-stop-all", self.controller.quick_stop_all)),
        ]
        for index, (text, handler) in enumerate(actions):
            button = QPushButton(text)
            if text == "总急停":
                button.setObjectName("DangerButton")
            button.clicked.connect(handler)
            action_grid.addWidget(button, index // 2, index % 2)
        layout.addLayout(action_grid)
        return panel

    def _build_overview_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(14)

        summary = QFrame()
        summary.setObjectName("OverviewPanel")
        summary.setFrameShape(_FRAME_STYLED_PANEL)
        summary_layout = QGridLayout(summary)
        summary_layout.setHorizontalSpacing(16)
        summary_layout.setVerticalSpacing(10)

        self.selection_detail_label = QLabel("已选: -")
        self.selection_detail_label.setObjectName("InfoLine")
        self.binding_detail_label = QLabel("绑定: -")
        self.binding_detail_label.setWordWrap(True)
        self.binding_detail_label.setObjectName("InfoLine")
        self.axis_health_label = QLabel("轴状态: -")
        self.axis_health_label.setObjectName("InfoLine")
        self.follow_status_label = QLabel("随动状态: -")
        self.follow_status_label.setObjectName("InfoLine")

        summary_layout.addWidget(QLabel("当前选中"), 0, 0)
        summary_layout.addWidget(self.selection_detail_label, 0, 1)
        summary_layout.addWidget(QLabel("绑定摘要"), 1, 0)
        summary_layout.addWidget(self.binding_detail_label, 1, 1)
        summary_layout.addWidget(QLabel("运行状态"), 2, 0)
        summary_layout.addWidget(self.axis_health_label, 2, 1)
        summary_layout.addWidget(QLabel("随动目标"), 3, 0)
        summary_layout.addWidget(self.follow_status_label, 3, 1)
        layout.addWidget(summary)

        panel = self._wrap_panel("轴状态总览", QWidget())
        panel_layout = panel.layout()
        self.axis_table = QTableWidget(len(self.axis_metadata), 11)
        self.axis_table.setHorizontalHeaderLabels(
            [
                "选中",
                "轴",
                "关节",
                "在线",
                "使能",
                "故障",
                "当前角度",
                "目标角度",
                "速度",
                "模式",
                "绑定",
            ]
        )
        self.axis_table.horizontalHeader().setStretchLastSection(True)
        self.axis_table.verticalHeader().setVisible(False)
        self.axis_table.setAlternatingRowColors(True)
        panel_layout.addWidget(self.axis_table)
        layout.addWidget(panel, 1)
        return page

    def _build_discovery_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(14)

        self.motor_table = QTableWidget(0, 5)
        self.motor_table.setHorizontalHeaderLabels(["适配器", "从站", "名称", "序列号", "eRob"])
        self.motor_table.horizontalHeader().setStretchLastSection(True)
        layout.addWidget(self._wrap_panel("扫描到的设备", self.motor_table), 1)
        return page

    def _build_follow_test_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(14)

        summary = QFrame()
        summary.setObjectName("OverviewPanel")
        summary_layout = QGridLayout(summary)
        summary_layout.setHorizontalSpacing(16)
        summary_layout.setVerticalSpacing(10)

        self.follow_test_axis_label = QLabel("测试轴: -")
        self.follow_test_axis_label.setObjectName("InfoLine")
        self.follow_test_profile_label = QLabel("序列特征: 60 Hz 分段余弦、多次反向、大跨度折返 | 摆幅 60%")
        self.follow_test_profile_label.setObjectName("InfoLine")
        self.follow_test_metrics_label = QLabel("误差指标: -")
        self.follow_test_metrics_label.setObjectName("InfoLine")
        self.follow_test_metrics_label.setWordWrap(True)
        self.follow_test_dynamic_label = QLabel("动态指标: -")
        self.follow_test_dynamic_label.setObjectName("InfoLine")
        self.follow_test_dynamic_label.setWordWrap(True)

        summary_layout.addWidget(QLabel("测试对象"), 0, 0)
        summary_layout.addWidget(self.follow_test_axis_label, 0, 1)
        summary_layout.addWidget(QLabel("目标序列"), 1, 0)
        summary_layout.addWidget(self.follow_test_profile_label, 1, 1)
        summary_layout.addWidget(QLabel("误差表现"), 2, 0)
        summary_layout.addWidget(self.follow_test_metrics_label, 2, 1)
        summary_layout.addWidget(QLabel("动态表现"), 3, 0)
        summary_layout.addWidget(self.follow_test_dynamic_label, 3, 1)
        layout.addWidget(summary)

        self.follow_test_plot = FollowTestPlotWidget()
        layout.addWidget(self._wrap_panel("曲线结果", self.follow_test_plot), 1)
        return page

    def _build_log_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(14)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        layout.addWidget(self._wrap_panel("事件日志", self.log_view), 1)
        return page

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
            if label == "scan-devices":
                self.discovery_rows = list(result or [])
                self._refresh_static_views()
                self._log_scan_snapshot()
            elif label == "shutdown":
                self._refresh_static_views()
            elif label == "follow-performance-test":
                self._follow_test_running = False
                self._apply_follow_test_result(result)
        else:
            if label == "follow-performance-test":
                self._follow_test_running = False
                self.follow_test_status_label.setText(f"状态: 失败 | {message}")
            self._append_log(f"[{label}] 失败: {message}")
            if label == "scan-devices":
                self._refresh_static_views()
                if self.discovery_rows:
                    self._log_scan_snapshot()
            if label != "follow-target-selected":
                QMessageBox.critical(self, "控制命令失败", message)
        self.refresh_states()

    def shutdown_controller(self) -> None:
        self.submit("shutdown", self.controller.shutdown)

    def scan_devices(self) -> None:
        self.submit("scan-devices", self.controller.scan_devices)

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
        self._update_selection_summary()
        self._update_overview_summary(states)
        self._update_header_status(states)

    def _refresh_binding_reports(self) -> None:
        try:
            reports = self.controller.get_binding_reports()
            self.binding_reports = {int(report.get("logical_axis_id", -1)): report for report in reports}
            self._refresh_target_list_availability()
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
        self._reload_axis_metadata()
        self.preferred_label.setText(f"最近扫描适配器: {self.controller.get_preferred_adapter() or '-'}")
        self._refresh_target_list_availability()
        self._update_selection_summary()

    def _reload_axis_metadata(self) -> None:
        self.axis_metadata = self.controller.refresh_metadata()
        self.axis_metadata_by_id = {
            metadata.logical_axis_id: metadata for metadata in self.axis_metadata
        }
        self.axis_table.setRowCount(len(self.axis_metadata))
        min_angle, max_angle, max_velocity = self._current_axis_limits()
        self.target_spin.setRange(min_angle, max_angle)
        self.follow_dial.setRange(int(min_angle * self._dial_scale), int(max_angle * self._dial_scale))
        self.velocity_spin.setRange(1.0, max(1.0, max_velocity))
        self.velocity_spin.setValue(min(self.velocity_spin.value(), max_velocity))

    def _log_scan_snapshot(self) -> None:
        self._append_log(f"[scan] discovered motors: {len(self.discovery_rows)}")
        for row in self.discovery_rows:
            self._append_log(
                "[scan] motor "
                f"adapter={row.get('adapter_name', '-')}, "
                f"slave={row.get('slave_index', '-')}, "
                f"name={row.get('name', '-')}, "
                f"serial={row.get('serial_number', '-')}, "
                f"erob={row.get('is_erob_motor', False)}"
            )

        self._append_log(f"[scan] axis metadata count: {len(self.axis_metadata)}")
        for metadata in self.axis_metadata:
            self._append_log(
                f"[scan] axis metadata id={metadata.logical_axis_id}, joint={metadata.joint_name}, "
                f"range=({metadata.min_angle_deg:.1f},{metadata.max_angle_deg:.1f}), "
                f"max_vel={metadata.max_velocity_deg_s:.1f}"
            )

        self._append_log(f"[scan] binding report count: {len(self.binding_reports)}")
        for axis_id in sorted(self.binding_reports):
            report = self.binding_reports[axis_id]
            self._append_log(
                f"[scan] binding axis={axis_id}, bound={report.get('bound', False)}, "
                f"joint={report.get('joint_name', '-')}, detail={report.get('detail', '-')}, "
                f"adapter={report.get('discovered_adapter', '-')}, serial={report.get('discovered_serial', '-')}"
            )

        self._append_log(
            f"[scan] target list count: {self.target_list.count()} | available axis ids: {sorted(self.available_axis_ids)}"
        )
        for index in range(self.target_list.count()):
            item = self.target_list.item(index)
            self._append_log(f"[scan] target item {index}: {item.text()}")

    def _current_axis_limits(self) -> tuple[float, float, float]:
        if not self.axis_metadata:
            return (-130.0, 130.0, 180.0)
        return (
            min(metadata.min_angle_deg for metadata in self.axis_metadata),
            max(metadata.max_angle_deg for metadata in self.axis_metadata),
            max(metadata.max_velocity_deg_s for metadata in self.axis_metadata),
        )

    def _refresh_target_list_availability(self) -> None:
        previously_selected = set(self.selected_axis_ids(silent=True))
        available_axis_ids = {
            int(report.get("logical_axis_id", -1))
            for report in self.binding_reports.values()
            if report.get("bound")
        }
        self.available_axis_ids = {axis_id for axis_id in available_axis_ids if axis_id >= 0}

        self.target_list.blockSignals(True)
        self.target_list.clear()
        for axis_id in sorted(self.available_axis_ids):
            metadata = self.axis_metadata_by_id.get(axis_id)
            if metadata is None:
                continue
            item = QListWidgetItem(f"Axis {axis_id} | {metadata.joint_name}")
            item.setData(_USER_ROLE, axis_id)
            item.setFlags(item.flags() | _ITEM_IS_SELECTABLE | _ITEM_IS_ENABLED)
            self.target_list.addItem(item)
            if axis_id in previously_selected:
                item.setSelected(True)

        if self.target_list.count() > 0 and not self.selected_axis_ids(silent=True):
            self.target_list.item(0).setSelected(True)
        self.target_list.blockSignals(False)

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
            axis_id = metadata.logical_axis_id
            report = self.binding_reports.get(axis_id, {})
            values = [
                "yes" if axis_id in selected_axes else "",
                str(axis_id),
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
                if axis_id in selected_axes:
                    item.setBackground(QColor("#efe1c7"))
                if column_index == 5 and state is not None:
                    item.setForeground(QColor("#bb3e03") if state.fault else QColor("#2b9348"))
                self.axis_table.setItem(row_index, column_index, item)

    def _update_overview_summary(self, states: list[AxisState]) -> None:
        online_count = sum(1 for state in states if state.online)
        enabled_count = sum(1 for state in states if state.enabled)
        fault_count = sum(1 for state in states if state.fault)
        follow_axes = self._selected_follow_axes()
        self.axis_health_label.setText(
            f"在线 {online_count}/{len(states)} | 使能 {enabled_count}/{len(states)} | 故障 {fault_count}"
        )
        if follow_axes:
            axis_names = [
                metadata.joint_name
                for metadata in self.axis_metadata
                if metadata.logical_axis_id in set(follow_axes)
            ]
            self.follow_status_label.setText(
                f"已进入随动: {', '.join(axis_names)} | 当前目标 {self._pending_follow_target:.1f}°"
            )
        else:
            self.follow_status_label.setText("随动状态: 当前没有已选轴处于 follow 模式")

    def _update_header_status(self, states: list[AxisState]) -> None:
        degraded = self.controller.try_is_degraded()
        faults = sum(1 for state in states if state.fault)
        enabled = sum(1 for state in states if state.enabled)
        self.preferred_label.setText(f"最近扫描适配器: {self.controller.try_get_preferred_adapter() or '-'}")
        self.degraded_label.setText(
            f"总线状态: {'degraded' if degraded else 'ready'} | enabled {enabled}/{len(states)} | fault {faults}"
        )
        self.degraded_label.setStyleSheet(
            f"color:{'#c44536' if degraded or faults else '#1c7c54'};font-weight:700;"
        )

    def selected_axis_ids(self, silent: bool = False) -> list[int]:
        axis_ids = [
            item.data(_USER_ROLE)
            for item in self.target_list.selectedItems()
            if int(item.data(_USER_ROLE)) in self.available_axis_ids
        ]
        if axis_ids or silent:
            return axis_ids
        if not self.available_axis_ids:
            self._append_log("[selection] 当前没有扫描并绑定成功的可用轴")
            QMessageBox.information(self, "无可用轴", "请先扫描设备；系统会自动初始化，待轴成功绑定后才能选择控制目标。")
            return []
        self._append_log("[selection] 请先选择至少一个控制目标")
        QMessageBox.information(self, "未选择目标", "请先选择至少一个控制目标轴。")
        return []

    def _selected_metadata(self) -> list[AxisMetadata]:
        selected = set(self.selected_axis_ids(silent=True))
        return [metadata for metadata in self.axis_metadata if metadata.logical_axis_id in selected]

    def _update_selection_summary(self) -> None:
        selected_metadata = self._selected_metadata()
        if not selected_metadata:
            if self.available_axis_ids:
                self.selection_chip.setText("未选择目标轴")
                self.selection_chip.set_color("#6b5e54")
            else:
                self.selection_chip.setText("无可用轴")
                self.selection_chip.set_color("#7f5539")
            self.selection_detail_label.setText("已选: -")
            self.binding_detail_label.setText("绑定: 扫描后会把成功绑定的轴加入列表")
            if self.axis_states:
                self._populate_axis_table(self.axis_states)
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
        if self.axis_states:
            self._populate_axis_table(self.axis_states)

    def _select_all_axes(self) -> None:
        for index in range(self.target_list.count()):
            self.target_list.item(index).setSelected(True)

    def _clear_selected_axes(self) -> None:
        self.target_list.clearSelection()

    def _run_axis_batch(self, axis_ids: list[int], operation: Callable[[int], None]) -> list[int]:
        errors = []
        for axis_id in axis_ids:
            try:
                result = operation(axis_id)
                if isinstance(result, MoveCommandStatus) and result not in MOVE_STATUS_OK:
                    errors.append(f"axis {axis_id}: {result.name.lower()}")
            except Exception as error:
                errors.append(f"axis {axis_id}: {error}")
        if errors:
            raise ControllerError("；".join(errors))
        return axis_ids

    def _run_axis_batch_parallel(self, axis_ids: list[int], operation: Callable[[int], None]) -> list[int]:
        def run_single(axis_id: int) -> str | None:
            try:
                result = operation(axis_id)
                if isinstance(result, MoveCommandStatus) and result not in MOVE_STATUS_OK:
                    return f"axis {axis_id}: {result.name.lower()}"
                return None
            except Exception as error:
                return f"axis {axis_id}: {error}"

        max_workers = max(1, min(len(axis_ids), 4))
        with ThreadPoolExecutor(max_workers=max_workers) as executor:
            errors = [message for message in executor.map(run_single, axis_ids) if message]
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
        busy_axes = []
        for axis_id in axis_ids:
            if axis_id < len(self.axis_states):
                state = self.axis_states[axis_id]
                if (
                    state.follow_mode_name in FOLLOW_ACTIVE_STATES
                    and abs(state.actual_velocity_deg_s) > 0.5
                ):
                    busy_axes.append(f"axis {axis_id}")
        if busy_axes:
            message = "以下轴仍处于随动且速度未降下，不能执行 moveTo: " + ", ".join(busy_axes)
            self._append_log(f"[move-selected] {message}")
            QMessageBox.information(self, "随动未停止", message)
            return
        angle = self.target_spin.value()
        velocity = self.velocity_spin.value()
        self.submit(
            "move-selected",
            lambda axes=axis_ids, target=angle, speed=velocity: self._run_axis_batch_parallel(
                axes,
                lambda axis_id: self.controller.move_to(axis_id, target, speed),
            ),
        )

    def _start_follow_selected(self) -> None:
        selected_axes = self.selected_axis_ids()
        if not selected_axes:
            return
        busy_axes = []
        for axis_id in selected_axes:
            if axis_id < len(self.axis_states):
                state = self.axis_states[axis_id]
                if (
                    state.position_mode_name in POSITION_MOVING_STATES
                    and not state.target_reached
                    and abs(state.actual_velocity_deg_s) > 0.5
                ):
                    busy_axes.append(f"axis {axis_id}")
        if busy_axes:
            message = "以下轴仍在位置运动中，需到位或速度接近 0 后才能启动随动: " + ", ".join(busy_axes)
            self._append_log(f"[start-follow-selected] {message}")
            QMessageBox.information(self, "位置运动未结束", message)
            return
        if self.axis_states:
            axis_id = selected_axes[0]
            self._prime_follow_dial_from_axis(axis_id)
        self.submit(
            "start-follow-selected",
            lambda axes=selected_axes: self._run_axis_batch_parallel(axes, self.controller.start_follow),
        )

    def _stop_follow_selected(self) -> None:
        axis_ids = self.selected_axis_ids()
        if not axis_ids:
            return
        self.submit(
            "stop-follow-selected",
            lambda axes=axis_ids: self._run_axis_batch_parallel(axes, self.controller.stop_follow),
        )

    def _selected_follow_axes(self) -> list[int]:
        selected_axes = self.selected_axis_ids(silent=True)
        return [axis_id for axis_id in selected_axes if axis_id in self._follow_active_axes]

    def _handle_dial_change(self, raw_value: int) -> None:
        self._pending_follow_target = raw_value / self._dial_scale
        self.follow_target_label.setText(f"目标: {self._pending_follow_target:.1f}°")
        if self._selected_follow_axes():
            self._follow_timer.start()

    def _handle_follow_test_amplitude_change(self, value: float) -> None:
        if self._follow_test_running:
            return
        self.follow_test_profile_label.setText(
            f"序列特征: 60 Hz 分段余弦、多次反向、大跨度折返 | 摆幅 {value:.0f}%"
        )

    def _start_follow_performance_test(self) -> None:
        if self._follow_test_running:
            QMessageBox.information(self, "测试进行中", "当前已有一项随动性能测试正在运行。")
            return
        axis_ids = self.selected_axis_ids()
        if not axis_ids:
            return
        if len(axis_ids) != 1:
            QMessageBox.information(self, "请选择单轴", "随动性能测试一次只支持一根轴，请先只选择一个控制目标。")
            return

        axis_id = axis_ids[0]
        if axis_id >= len(self.axis_states):
            QMessageBox.information(self, "状态未就绪", "当前轴状态尚未刷新完成，请稍后再试。")
            return
        state = self.axis_states[axis_id]
        if not state.online or not state.enabled or state.fault:
            QMessageBox.information(self, "轴状态不满足", "测试前需要轴在线、已使能且无故障。")
            return
        if state.follow_mode_name in FOLLOW_ACTIVE_STATES:
            QMessageBox.information(self, "随动仍在运行", "请先停止当前随动，再启动性能测试。")
            return

        metadata = self.axis_metadata_by_id.get(axis_id)
        if metadata is None:
            QMessageBox.information(self, "缺少轴配置", f"未找到 axis {axis_id} 的配置元数据。")
            return

        amplitude_scale = self.follow_test_amplitude_spin.value() / 100.0
        self._follow_test_running = True
        self.follow_test_status_label.setText(
            f"状态: 正在测试 Axis {axis_id} | 60 Hz 分段余弦目标序列 | 摆幅 {amplitude_scale * 100:.0f}%"
        )
        self.submit(
            "follow-performance-test",
            lambda target_axis_id=axis_id, target_metadata=metadata, target_amplitude=amplitude_scale: run_follow_performance_test(
                self.controller,
                target_axis_id,
                target_metadata,
                amplitude_scale=target_amplitude,
            ),
        )

    def _apply_follow_test_result(self, result: object) -> None:
        if not isinstance(result, FollowTestResult):
            return
        self._follow_test_result = result
        self.follow_test_plot.set_result(result)
        self.follow_test_status_label.setText(
            f"状态: 完成 | Axis {result.axis_id} {result.axis_name} | 样本 {len(result.samples)}"
        )
        self.follow_test_axis_label.setText(
            f"测试轴: Axis {result.axis_id} | {result.axis_name} | 用时 {result.duration_s:.2f} s"
        )
        self.follow_test_profile_label.setText(
            "序列特征: "
            f"60 Hz 分段余弦、多次反向、大跨度折返 | 摆幅 {result.amplitude_scale * 100:.0f}%"
        )
        self.follow_test_metrics_label.setText(
            "误差指标: "
            f"max {result.max_abs_error_deg:.2f}° | mean {result.mean_abs_error_deg:.2f}° | "
            f"RMS {result.rms_error_deg:.2f}° | P95 {result.p95_abs_error_deg:.2f}°"
        )
        self.follow_test_dynamic_label.setText(
            "动态指标: "
            f"目标跨度 {result.target_span_deg:.1f}° | 反向次数 {result.reversal_count} | "
            f"峰值速度 {result.peak_velocity_deg_s:.1f}°/s"
        )
        self._append_log(
            "[follow-performance-test] "
            f"axis={result.axis_id}, samples={len(result.samples)}, max_err={result.max_abs_error_deg:.2f}deg, "
            f"rms_err={result.rms_error_deg:.2f}deg, reversals={result.reversal_count}, "
            f"peak_vel={result.peak_velocity_deg_s:.1f}deg/s, amplitude={result.amplitude_scale * 100:.0f}%"
        )

    def _clear_follow_test_result(self) -> None:
        if self._follow_test_running:
            QMessageBox.information(self, "测试进行中", "当前测试尚未结束，暂不能清空结果。")
            return
        self._follow_test_result = None
        self.follow_test_plot.clear_result()
        self.follow_test_status_label.setText("状态: 未运行")
        self.follow_test_axis_label.setText("测试轴: -")
        self.follow_test_profile_label.setText(
            f"序列特征: 60 Hz 分段余弦、多次反向、大跨度折返 | 摆幅 {self.follow_test_amplitude_spin.value():.0f}%"
        )
        self.follow_test_metrics_label.setText("误差指标: -")
        self.follow_test_dynamic_label.setText("动态指标: -")

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

    def _prime_follow_dial_from_axis(self, axis_id: int) -> None:
        if not self.axis_states or axis_id >= len(self.axis_states):
            return
        target_angle = self.axis_states[axis_id].actual_angle_deg
        self._pending_follow_target = target_angle
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