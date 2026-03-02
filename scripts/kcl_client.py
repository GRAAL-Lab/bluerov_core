#!/usr/bin/env python3

import sys
from typing import Any, Callable

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from auv_core_helper.action import SetKCL
from auv_core_helper.msg import LatLong
from auv_core_helper.msg import SpiralPathData
from auv_core_helper.msg import SerpentinePathData
from auv_core_helper.msg import CircularPathData


def _parse_bool(value: str) -> bool:
    lowered = value.strip().lower()
    if lowered in ("1", "true", "t", "yes", "y"):
        return True
    if lowered in ("0", "false", "f", "no", "n"):
        return False
    raise ValueError(f"Invalid boolean value: '{value}'")


def _prompt_value(label: str, current: Any, cast: Callable[[str], Any]) -> Any:
    prompt = f"{label} [{current}]: "
    raw = input(prompt).strip()
    if not raw:
        return current
    return cast(raw)


class KclClient(Node):
    def __init__(self) -> None:
        super().__init__("kcl_client")

        self.declare_parameter("interactive", False)
        self.declare_parameter("desired_state", "PATH_FOLLOWING")
        self.declare_parameter("trajectory_time", 0.0)

        self.declare_parameter("position.latitude", 0.0)
        self.declare_parameter("position.longitude", 0.0)
        self.declare_parameter("depth", 0.0)

        self.declare_parameter("path_mode", "Serpentine2D")
        self.declare_parameter("resume_path", False)

        self.declare_parameter("spiral.spiral_diameter", 0.0)
        self.declare_parameter("spiral.spiral_increment", 0.0)

        self.declare_parameter("serpentine.origin.latitude", 0.0)
        self.declare_parameter("serpentine.origin.longitude", 0.0)
        self.declare_parameter("serpentine.front_left.latitude", 0.0)
        self.declare_parameter("serpentine.front_left.longitude", 0.0)
        self.declare_parameter("serpentine.front_right.latitude", 0.0)
        self.declare_parameter("serpentine.front_right.longitude", 0.0)
        self.declare_parameter("serpentine.right.latitude", 0.0)
        self.declare_parameter("serpentine.right.longitude", 0.0)

        self.declare_parameter("circular.circular_diameter", 0.0)
        self.declare_parameter("circular.center_point.latitude", 0.0)
        self.declare_parameter("circular.center_point.longitude", 0.0)
        self.declare_parameter("circular.clockwise", True)

        self._client = ActionClient(self, SetKCL, "/set_kcl_state")

    def _get_param(self, name: str) -> Any:
        return self.get_parameter(name).value

    def _interactive_update(self) -> None:
        self.get_logger().info("Interactive mode: press Enter to keep defaults.")
        self.set_parameters([
            rclpy.parameter.Parameter(
                "desired_state",
                rclpy.Parameter.Type.STRING,
                _prompt_value("desired_state", self._get_param("desired_state"), str),
            ),
            rclpy.parameter.Parameter(
                "trajectory_time",
                rclpy.Parameter.Type.DOUBLE,
                _prompt_value("trajectory_time", self._get_param("trajectory_time"), float),
            ),
            rclpy.parameter.Parameter(
                "position.latitude",
                rclpy.Parameter.Type.DOUBLE,
                _prompt_value("position.latitude", self._get_param("position.latitude"), float),
            ),
            rclpy.parameter.Parameter(
                "position.longitude",
                rclpy.Parameter.Type.DOUBLE,
                _prompt_value("position.longitude", self._get_param("position.longitude"), float),
            ),
            rclpy.parameter.Parameter(
                "depth",
                rclpy.Parameter.Type.DOUBLE,
                _prompt_value("depth", self._get_param("depth"), float),
            ),
            rclpy.parameter.Parameter(
                "path_mode",
                rclpy.Parameter.Type.STRING,
                _prompt_value("path_mode", self._get_param("path_mode"), str),
            ),
            rclpy.parameter.Parameter(
                "resume_path",
                rclpy.Parameter.Type.BOOL,
                _prompt_value("resume_path", self._get_param("resume_path"), _parse_bool),
            ),
            rclpy.parameter.Parameter(
                "spiral.spiral_diameter",
                rclpy.Parameter.Type.DOUBLE,
                _prompt_value(
                    "spiral.spiral_diameter",
                    self._get_param("spiral.spiral_diameter"),
                    float,
                ),
            ),
            rclpy.parameter.Parameter(
                "spiral.spiral_increment",
                rclpy.Parameter.Type.DOUBLE,
                _prompt_value(
                    "spiral.spiral_increment",
                    self._get_param("spiral.spiral_increment"),
                    float,
                ),
            ),
            rclpy.parameter.Parameter(
                "serpentine.origin.latitude",
                rclpy.Parameter.Type.DOUBLE,
                _prompt_value(
                    "serpentine.origin.latitude",
                    self._get_param("serpentine.origin.latitude"),
                    float,
                ),
            ),
            rclpy.parameter.Parameter(
                "serpentine.origin.longitude",
                rclpy.Parameter.Type.DOUBLE,
                _prompt_value(
                    "serpentine.origin.longitude",
                    self._get_param("serpentine.origin.longitude"),
                    float,
                ),
            ),
            rclpy.parameter.Parameter(
                "serpentine.front_left.latitude",
                rclpy.Parameter.Type.DOUBLE,
                _prompt_value(
                    "serpentine.front_left.latitude",
                    self._get_param("serpentine.front_left.latitude"),
                    float,
                ),
            ),
            rclpy.parameter.Parameter(
                "serpentine.front_left.longitude",
                rclpy.Parameter.Type.DOUBLE,
                _prompt_value(
                    "serpentine.front_left.longitude",
                    self._get_param("serpentine.front_left.longitude"),
                    float,
                ),
            ),
            rclpy.parameter.Parameter(
                "serpentine.front_right.latitude",
                rclpy.Parameter.Type.DOUBLE,
                _prompt_value(
                    "serpentine.front_right.latitude",
                    self._get_param("serpentine.front_right.latitude"),
                    float,
                ),
            ),
            rclpy.parameter.Parameter(
                "serpentine.front_right.longitude",
                rclpy.Parameter.Type.DOUBLE,
                _prompt_value(
                    "serpentine.front_right.longitude",
                    self._get_param("serpentine.front_right.longitude"),
                    float,
                ),
            ),
            rclpy.parameter.Parameter(
                "serpentine.right.latitude",
                rclpy.Parameter.Type.DOUBLE,
                _prompt_value(
                    "serpentine.right.latitude",
                    self._get_param("serpentine.right.latitude"),
                    float,
                ),
            ),
            rclpy.parameter.Parameter(
                "serpentine.right.longitude",
                rclpy.Parameter.Type.DOUBLE,
                _prompt_value(
                    "serpentine.right.longitude",
                    self._get_param("serpentine.right.longitude"),
                    float,
                ),
            ),
            rclpy.parameter.Parameter(
                "circular.circular_diameter",
                rclpy.Parameter.Type.DOUBLE,
                _prompt_value(
                    "circular.circular_diameter",
                    self._get_param("circular.circular_diameter"),
                    float,
                ),
            ),
            rclpy.parameter.Parameter(
                "circular.center_point.latitude",
                rclpy.Parameter.Type.DOUBLE,
                _prompt_value(
                    "circular.center_point.latitude",
                    self._get_param("circular.center_point.latitude"),
                    float,
                ),
            ),
            rclpy.parameter.Parameter(
                "circular.center_point.longitude",
                rclpy.Parameter.Type.DOUBLE,
                _prompt_value(
                    "circular.center_point.longitude",
                    self._get_param("circular.center_point.longitude"),
                    float,
                ),
            ),
            rclpy.parameter.Parameter(
                "circular.clockwise",
                rclpy.Parameter.Type.BOOL,
                _prompt_value(
                    "circular.clockwise",
                    self._get_param("circular.clockwise"),
                    _parse_bool,
                ),
            ),
        ])

    def send_goal(self) -> None:
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("SetKCL action server not available.")
            return

        if self._get_param("interactive"):
            self._interactive_update()

        goal = SetKCL.Goal()
        goal.desired_state = self._get_param("desired_state")
        goal.position = LatLong(
            latitude=self._get_param("position.latitude"),
            longitude=self._get_param("position.longitude"),
        )
        goal.depth = float(self._get_param("depth"))
        goal.trajectory_time = float(self._get_param("trajectory_time"))

        goal.path_mode = self._get_param("path_mode")
        goal.resume_path = bool(self._get_param("resume_path"))

        goal.spiral_data = SpiralPathData(
            spiral_diameter=float(self._get_param("spiral.spiral_diameter")),
            spiral_increment=float(self._get_param("spiral.spiral_increment")),
        )

        goal.serpentine_data = SerpentinePathData(
            origin=LatLong(
                latitude=float(self._get_param("serpentine.origin.latitude")),
                longitude=float(self._get_param("serpentine.origin.longitude")),
            ),
            front_left=LatLong(
                latitude=float(self._get_param("serpentine.front_left.latitude")),
                longitude=float(self._get_param("serpentine.front_left.longitude")),
            ),
            front_right=LatLong(
                latitude=float(self._get_param("serpentine.front_right.latitude")),
                longitude=float(self._get_param("serpentine.front_right.longitude")),
            ),
            right=LatLong(
                latitude=float(self._get_param("serpentine.right.latitude")),
                longitude=float(self._get_param("serpentine.right.longitude")),
            ),
        )

        goal.circular_data = CircularPathData(
            circular_diameter=float(self._get_param("circular.circular_diameter")),
            center_point=LatLong(
                latitude=float(self._get_param("circular.center_point.latitude")),
                longitude=float(self._get_param("circular.center_point.longitude")),
            ),
            clockwise=bool(self._get_param("circular.clockwise")),
        )

        self.get_logger().info(f"Sending SetKCL goal: {goal.desired_state}")
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("SetKCL goal was rejected.")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(f"Result: success={result.success} message='{result.message}'")


def main(argv: list[str] | None = None) -> None:
    rclpy.init(args=argv)
    node = KclClient()
    try:
        node.send_goal()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
