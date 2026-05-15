#!/usr/bin/env python3

import sys
from typing import Optional

from PyQt5 import QtCore, QtWidgets
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from auv_core_helper.action import SetKCL
from auv_core_helper.msg import LatLong, SpiralPathData, SerpentinePathData, CircularPathData


class KclActionClient(Node):
    def __init__(self) -> None:
        super().__init__("kcl_client_gui")
        self._client = ActionClient(self, SetKCL, "/set_kcl_state")
        self._goal_handle = None

    def is_ready(self) -> bool:
        if hasattr(self._client, "server_is_ready"):
            return self._client.server_is_ready()
        return self._client.wait_for_server(timeout_sec=0.0)

    def send_goal(self, goal: SetKCL.Goal, feedback_cb) -> None:
        if not self._client.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("SetKCL action server not available.")
        future = self._client.send_goal_async(goal, feedback_callback=feedback_cb)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        self._goal_handle = future.result()
        if not self._goal_handle or not self._goal_handle.accepted:
            raise RuntimeError("SetKCL goal was rejected.")

    def wait_result(self, timeout_sec: float = 0.1) -> Optional[SetKCL.Result]:
        if not self._goal_handle:
            return None
        result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        if not result_future.done():
            return None
        return result_future.result().result


class KclGui(QtWidgets.QWidget):
    def __init__(self, client: KclActionClient) -> None:
        super().__init__()
        self._client = client
        self._setup_ui()

        self._spin_timer = QtCore.QTimer(self)
        self._spin_timer.timeout.connect(self._spin_once)
        self._spin_timer.start(50)

        self._result_timer = QtCore.QTimer(self)
        self._result_timer.timeout.connect(self._check_result)
        self._result_timer.start(200)

    def _setup_ui(self) -> None:
        self.setWindowTitle("KCL SetKCL Client")
        self.resize(720, 640)

        self._stack = QtWidgets.QStackedWidget()

        # Page 1: mode selection
        mode_page = QtWidgets.QWidget()
        mode_layout = QtWidgets.QVBoxLayout(mode_page)
        mode_layout.addWidget(QtWidgets.QLabel("Select desired state"))
        self._state_combo = QtWidgets.QComboBox()
        self._state_combo.addItems([
            "PATH_FOLLOWING",
            "TRAJECTORY_FOLLOWING",
            "WAYPOINT_NAVIGATION",
            "HOLD",
            "IDLE",
        ])
        self._state_combo.currentTextChanged.connect(self._apply_mode_ui)
        mode_layout.addWidget(self._state_combo)
        self._server_label = QtWidgets.QLabel("Waiting for SetKCL action server...")
        mode_layout.addWidget(self._server_label)
        mode_layout.addStretch(1)
        self._next_btn = QtWidgets.QPushButton("Next")
        self._next_btn.clicked.connect(self._go_to_form)
        mode_layout.addWidget(self._next_btn)

        # Page 2: form
        form_page = QtWidgets.QWidget()
        form_layout = QtWidgets.QVBoxLayout(form_page)

        self._pose_tab = self._build_pose_tab()
        self._trajectory_tab = self._build_trajectory_tab()
        self._path_tab = self._build_path_tab()

        self._mode_stack = QtWidgets.QStackedWidget()
        self._pose_tab_idx = self._mode_stack.addWidget(self._pose_tab)
        self._trajectory_tab_idx = self._mode_stack.addWidget(self._trajectory_tab)
        self._path_tab_idx = self._mode_stack.addWidget(self._path_tab)
        form_layout.addWidget(self._mode_stack)

        btn_row = QtWidgets.QHBoxLayout()
        back_btn = QtWidgets.QPushButton("Back")
        back_btn.clicked.connect(lambda: self._stack.setCurrentIndex(0))
        self._send_btn = QtWidgets.QPushButton("Send")
        self._send_btn.clicked.connect(self._send_goal)
        btn_row.addWidget(back_btn)
        btn_row.addStretch(1)
        btn_row.addWidget(self._send_btn)
        form_layout.addLayout(btn_row)

        self._status_label = QtWidgets.QLabel("Status: idle")
        self._progress = QtWidgets.QProgressBar()
        self._progress.setRange(0, 100)
        self._progress.setValue(0)
        form_layout.addWidget(self._status_label)
        form_layout.addWidget(self._progress)

        self._stack.addWidget(mode_page)
        self._stack.addWidget(form_page)

        main_layout = QtWidgets.QVBoxLayout(self)
        main_layout.addWidget(self._stack)
        self._apply_mode_ui(self._state_combo.currentText())

        self._server_timer = QtCore.QTimer(self)
        self._server_timer.timeout.connect(self._update_server_status)
        self._server_timer.start(300)
        self._update_server_status()

    def _build_pose_tab(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QFormLayout(widget)

        self._lat = QtWidgets.QDoubleSpinBox()
        self._lat.setRange(-90.0, 90.0)
        self._lat.setDecimals(8)
        self._lat.setValue(0.0)

        self._lon = QtWidgets.QDoubleSpinBox()
        self._lon.setRange(-180.0, 180.0)
        self._lon.setDecimals(8)
        self._lon.setValue(0.0)

        self._depth = QtWidgets.QDoubleSpinBox()
        self._depth.setRange(-1000.0, 1000.0)
        self._depth.setDecimals(3)
        self._depth.setValue(0.0)

        self._lat_label = QtWidgets.QLabel("Latitude")
        self._lon_label = QtWidgets.QLabel("Longitude")
        self._depth_label = QtWidgets.QLabel("Depth")
        layout.addRow(self._lat_label, self._lat)
        layout.addRow(self._lon_label, self._lon)
        layout.addRow(self._depth_label, self._depth)
        return widget

    def _build_trajectory_tab(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QFormLayout(widget)

        self._traj_x = QtWidgets.QDoubleSpinBox()
        self._traj_x.setRange(-100000.0, 100000.0)
        self._traj_x.setDecimals(3)

        self._traj_y = QtWidgets.QDoubleSpinBox()
        self._traj_y.setRange(-100000.0, 100000.0)
        self._traj_y.setDecimals(3)

        self._traj_z = QtWidgets.QDoubleSpinBox()
        self._traj_z.setRange(-100000.0, 100000.0)
        self._traj_z.setDecimals(3)

        self._traj_time = QtWidgets.QDoubleSpinBox()
        self._traj_time.setRange(0.0, 36000.0)
        self._traj_time.setDecimals(2)
        self._traj_time.setValue(0.0)

        layout.addRow("X (local)", self._traj_x)
        layout.addRow("Y (local)", self._traj_y)
        layout.addRow("Z (local)", self._traj_z)
        layout.addRow("Trajectory Time (s)", self._traj_time)
        return widget

    def _build_path_tab(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)

        form = QtWidgets.QFormLayout()
        self._path_mode = QtWidgets.QComboBox()
        self._path_mode.addItems(["Serpentine2D", "Spiral2D", "Circular2D"])
        self._path_mode.currentTextChanged.connect(self._apply_path_mode_ui)
        self._resume_path = QtWidgets.QCheckBox()

        form.addRow("Path Mode", self._path_mode)
        form.addRow("Resume Path", self._resume_path)
        layout.addLayout(form)

        self._path_tabs = QtWidgets.QTabWidget()
        self._spiral_tab = self._build_spiral_tab()
        self._serpentine_tab = self._build_serpentine_tab()
        self._circular_tab = self._build_circular_tab()
        self._spiral_tab_idx = self._path_tabs.addTab(self._spiral_tab, "Spiral")
        self._serpentine_tab_idx = self._path_tabs.addTab(self._serpentine_tab, "Serpentine")
        self._circular_tab_idx = self._path_tabs.addTab(self._circular_tab, "Circular")
        layout.addWidget(self._path_tabs)

        self._apply_path_mode_ui(self._path_mode.currentText())

        return widget

    def _build_spiral_tab(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QFormLayout(widget)
        self._spiral_diameter = QtWidgets.QDoubleSpinBox()
        self._spiral_diameter.setRange(0.0, 10000.0)
        self._spiral_diameter.setDecimals(3)
        self._spiral_increment = QtWidgets.QDoubleSpinBox()
        self._spiral_increment.setRange(0.0, 10000.0)
        self._spiral_increment.setDecimals(3)
        layout.addRow("Spiral Diameter", self._spiral_diameter)
        layout.addRow("Spiral Increment", self._spiral_increment)
        return widget

    def _build_serpentine_tab(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QFormLayout(widget)

        self._serp_origin_lat = QtWidgets.QDoubleSpinBox()
        self._serp_origin_lat.setRange(-90.0, 90.0)
        self._serp_origin_lat.setDecimals(8)
        self._serp_origin_lon = QtWidgets.QDoubleSpinBox()
        self._serp_origin_lon.setRange(-180.0, 180.0)
        self._serp_origin_lon.setDecimals(8)

        self._serp_fl_lat = QtWidgets.QDoubleSpinBox()
        self._serp_fl_lat.setRange(-90.0, 90.0)
        self._serp_fl_lat.setDecimals(8)
        self._serp_fl_lon = QtWidgets.QDoubleSpinBox()
        self._serp_fl_lon.setRange(-180.0, 180.0)
        self._serp_fl_lon.setDecimals(8)

        self._serp_fr_lat = QtWidgets.QDoubleSpinBox()
        self._serp_fr_lat.setRange(-90.0, 90.0)
        self._serp_fr_lat.setDecimals(8)
        self._serp_fr_lon = QtWidgets.QDoubleSpinBox()
        self._serp_fr_lon.setRange(-180.0, 180.0)
        self._serp_fr_lon.setDecimals(8)

        self._serp_right_lat = QtWidgets.QDoubleSpinBox()
        self._serp_right_lat.setRange(-90.0, 90.0)
        self._serp_right_lat.setDecimals(8)
        self._serp_right_lon = QtWidgets.QDoubleSpinBox()
        self._serp_right_lon.setRange(-180.0, 180.0)
        self._serp_right_lon.setDecimals(8)

        layout.addRow("Origin Lat", self._serp_origin_lat)
        layout.addRow("Origin Lon", self._serp_origin_lon)
        layout.addRow("Front Left Lat", self._serp_fl_lat)
        layout.addRow("Front Left Lon", self._serp_fl_lon)
        layout.addRow("Front Right Lat", self._serp_fr_lat)
        layout.addRow("Front Right Lon", self._serp_fr_lon)
        layout.addRow("Right Lat", self._serp_right_lat)
        layout.addRow("Right Lon", self._serp_right_lon)
        return widget

    def _build_circular_tab(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QFormLayout(widget)

        self._circ_diameter = QtWidgets.QDoubleSpinBox()
        self._circ_diameter.setRange(0.0, 10000.0)
        self._circ_diameter.setDecimals(3)

        self._circ_center_lat = QtWidgets.QDoubleSpinBox()
        self._circ_center_lat.setRange(-90.0, 90.0)
        self._circ_center_lat.setDecimals(8)

        self._circ_center_lon = QtWidgets.QDoubleSpinBox()
        self._circ_center_lon.setRange(-180.0, 180.0)
        self._circ_center_lon.setDecimals(8)

        self._circ_clockwise = QtWidgets.QCheckBox()
        self._circ_clockwise.setChecked(True)

        layout.addRow("Circular Diameter", self._circ_diameter)
        layout.addRow("Center Lat", self._circ_center_lat)
        layout.addRow("Center Lon", self._circ_center_lon)
        layout.addRow("Clockwise", self._circ_clockwise)
        return widget

    def _go_to_form(self) -> None:
        self._apply_mode_ui(self._state_combo.currentText())
        self._stack.setCurrentIndex(1)

    def _set_path_tab_visible(self, index: int, visible: bool) -> None:
        try:
            self._path_tabs.setTabVisible(index, visible)
        except AttributeError:
            self._path_tabs.setTabEnabled(index, visible)

    def _apply_mode_ui(self, mode: str) -> None:
        if mode == "TRAJECTORY_FOLLOWING":
            self._mode_stack.setCurrentIndex(self._trajectory_tab_idx)
            return

        if mode == "PATH_FOLLOWING":
            self._mode_stack.setCurrentIndex(self._path_tab_idx)
            self._apply_path_mode_ui(self._path_mode.currentText())
            return

        if mode == "WAYPOINT_NAVIGATION":
            self._mode_stack.setCurrentIndex(self._pose_tab_idx)
            return

        # HOLD / IDLE fallback: show an empty page to reduce confusion.
        self._mode_stack.setCurrentIndex(self._pose_tab_idx)

    def _apply_path_mode_ui(self, mode: str) -> None:
        if mode == "Spiral2D":
            self._set_path_tab_visible(self._spiral_tab_idx, True)
            self._set_path_tab_visible(self._serpentine_tab_idx, False)
            self._set_path_tab_visible(self._circular_tab_idx, False)
            self._path_tabs.setCurrentIndex(self._spiral_tab_idx)
            return

        if mode == "Circular2D":
            self._set_path_tab_visible(self._spiral_tab_idx, False)
            self._set_path_tab_visible(self._serpentine_tab_idx, False)
            self._set_path_tab_visible(self._circular_tab_idx, True)
            self._path_tabs.setCurrentIndex(self._circular_tab_idx)
            return

        # Serpentine2D default
        self._set_path_tab_visible(self._spiral_tab_idx, False)
        self._set_path_tab_visible(self._serpentine_tab_idx, True)
        self._set_path_tab_visible(self._circular_tab_idx, False)
        self._path_tabs.setCurrentIndex(self._serpentine_tab_idx)

    def _update_server_status(self) -> None:
        ready = self._client.is_ready()
        if ready:
            self._server_label.setText("SetKCL action server: ready")
        else:
            self._server_label.setText("Waiting for SetKCL action server...")
        self._next_btn.setEnabled(ready)
        self._send_btn.setEnabled(ready)

    def _send_goal(self) -> None:
        goal = SetKCL.Goal()
        mode = self._state_combo.currentText()
        goal.desired_state = mode
        if mode == "TRAJECTORY_FOLLOWING":
            goal.position = LatLong(latitude=self._traj_x.value(), longitude=self._traj_y.value())
            goal.depth = float(self._traj_z.value())
            goal.trajectory_time = float(self._traj_time.value())
        else:
            goal.position = LatLong(latitude=self._lat.value(), longitude=self._lon.value())
            goal.depth = float(self._depth.value())
            goal.trajectory_time = float(self._traj_time.value())

        goal.path_mode = self._path_mode.currentText()
        goal.resume_path = bool(self._resume_path.isChecked())

        goal.spiral_data = SpiralPathData(
            spiral_diameter=float(self._spiral_diameter.value()),
            spiral_increment=float(self._spiral_increment.value()),
        )
        goal.serpentine_data = SerpentinePathData(
            origin=LatLong(latitude=float(self._serp_origin_lat.value()), longitude=float(self._serp_origin_lon.value())),
            front_left=LatLong(latitude=float(self._serp_fl_lat.value()), longitude=float(self._serp_fl_lon.value())),
            front_right=LatLong(latitude=float(self._serp_fr_lat.value()), longitude=float(self._serp_fr_lon.value())),
            right=LatLong(latitude=float(self._serp_right_lat.value()), longitude=float(self._serp_right_lon.value())),
        )
        goal.circular_data = CircularPathData(
            circular_diameter=float(self._circ_diameter.value()),
            center_point=LatLong(latitude=float(self._circ_center_lat.value()), longitude=float(self._circ_center_lon.value())),
            clockwise=bool(self._circ_clockwise.isChecked()),
        )

        try:
            self._client.send_goal(goal, self._handle_feedback)
            self._status_label.setText("Status: goal sent")
            self._progress.setValue(0)
        except RuntimeError as exc:
            self._status_label.setText(f"Status: {exc}")

    def _handle_feedback(self, feedback_msg: SetKCL.Feedback) -> None:
        feedback = feedback_msg.feedback
        self._progress.setValue(int(feedback.action_progress))
        self._status_label.setText(
            f"Status: state={feedback.actual_state} progress={feedback.action_progress:.1f}%"
        )

    def _check_result(self) -> None:
        result = self._client.wait_result()
        if result is None:
            return
        self._status_label.setText(f"Result: success={result.success} message={result.message}")

    def _spin_once(self) -> None:
        rclpy.spin_once(self._client, timeout_sec=0.01)


def main() -> None:
    rclpy.init()
    client = KclActionClient()
    app = QtWidgets.QApplication(sys.argv)
    gui = KclGui(client)
    gui.show()
    try:
        sys.exit(app.exec_())
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
