#!/usr/bin/env python3
"""Convert geographic targets to Nav2 NavigateToPose goals."""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from tf2_ros import Buffer, TransformException, TransformListener

from gps_nav2_goal.geo import geodetic_to_enu_offset, rotate_enu_to_map


class GpsNav2GoalNode(Node):
    """Bridge a WGS84 geographic goal into the local Nav2 map frame."""

    def __init__(self) -> None:
        super().__init__('gps_nav2_goal_node')

        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('goal_topic', '/gps_goal')
        self.declare_parameter('debug_goal_topic', '/gps_goal/map_pose')
        self.declare_parameter('navigate_to_pose_action', '/navigate_to_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        self.declare_parameter('anchor_mode', 'first_fix')
        self.declare_parameter('anchor_latitude', 0.0)
        self.declare_parameter('anchor_longitude', 0.0)
        self.declare_parameter('anchor_altitude', 0.0)
        self.declare_parameter('anchor_map_x', 0.0)
        self.declare_parameter('anchor_map_y', 0.0)
        self.declare_parameter('map_yaw_deg', 0.0)

        self.declare_parameter('goal_orientation_mode', 'face_goal')
        self.declare_parameter('fixed_goal_yaw_deg', 0.0)
        self.declare_parameter('use_goal_altitude', False)
        self.declare_parameter('minimum_fix_status', int(NavSatStatus.STATUS_FIX))
        self.declare_parameter('maximum_goal_distance_m', 5000.0)
        self.declare_parameter('tf_timeout_sec', 0.2)
        self.declare_parameter('action_retry_period_sec', 0.5)

        self._map_frame = str(self.get_parameter('map_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._anchor_mode = str(self.get_parameter('anchor_mode').value).lower()
        self._map_yaw_rad = math.radians(float(self.get_parameter('map_yaw_deg').value))

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._action_name = str(self.get_parameter('navigate_to_pose_action').value)
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            self._action_name,
        )

        self._latest_fix: Optional[NavSatFix] = None
        self._pending_geo_goal: Optional[GeoPointStamped] = None
        self._goal_in_progress = False

        self._anchor_lat: Optional[float] = None
        self._anchor_lon: Optional[float] = None
        self._anchor_alt: float = 0.0
        self._anchor_map_x: float = 0.0
        self._anchor_map_y: float = 0.0

        if self._anchor_mode == 'fixed':
            self._load_fixed_anchor()
        elif self._anchor_mode != 'first_fix':
            raise ValueError("anchor_mode must be 'first_fix' or 'fixed'")

        gps_topic = str(self.get_parameter('gps_topic').value)
        goal_topic = str(self.get_parameter('goal_topic').value)
        debug_topic = str(self.get_parameter('debug_goal_topic').value)

        self.create_subscription(
            NavSatFix,
            gps_topic,
            self._gps_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(GeoPointStamped, goal_topic, self._goal_callback, 10)
        self._debug_goal_publisher = self.create_publisher(PoseStamped, debug_topic, 10)
        self._retry_timer = self.create_timer(
            float(self.get_parameter('action_retry_period_sec').value),
            self._process_pending_goal,
        )

        self.get_logger().info(
            f'Ready: GPS={gps_topic}, geographic goal={goal_topic}, '
            f'Nav2 action={self._action_name}, anchor_mode={self._anchor_mode}'
        )

    def _load_fixed_anchor(self) -> None:
        self._anchor_lat = float(self.get_parameter('anchor_latitude').value)
        self._anchor_lon = float(self.get_parameter('anchor_longitude').value)
        self._anchor_alt = float(self.get_parameter('anchor_altitude').value)
        self._anchor_map_x = float(self.get_parameter('anchor_map_x').value)
        self._anchor_map_y = float(self.get_parameter('anchor_map_y').value)

        if not self._valid_lat_lon(self._anchor_lat, self._anchor_lon):
            raise ValueError('Fixed anchor latitude/longitude are invalid')

        self.get_logger().info(
            'Loaded fixed geographic anchor: '
            f'({self._anchor_lat:.8f}, {self._anchor_lon:.8f}) -> '
            f'map({self._anchor_map_x:.3f}, {self._anchor_map_y:.3f})'
        )

    def _gps_callback(self, msg: NavSatFix) -> None:
        if not self._fix_is_usable(msg):
            return

        self._latest_fix = msg
        if self._anchor_mode == 'first_fix' and self._anchor_lat is None:
            self._try_initialize_anchor_from_fix(msg)

    def _try_initialize_anchor_from_fix(self, fix: NavSatFix) -> None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(
                    seconds=float(self.get_parameter('tf_timeout_sec').value)
                ),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'Cannot initialize GPS anchor yet: TF {self._map_frame} -> '
                f'{self._base_frame} unavailable ({exc})',
                throttle_duration_sec=2.0,
            )
            return

        self._anchor_lat = fix.latitude
        self._anchor_lon = fix.longitude
        self._anchor_alt = fix.altitude if math.isfinite(fix.altitude) else 0.0
        self._anchor_map_x = transform.transform.translation.x
        self._anchor_map_y = transform.transform.translation.y
        self.get_logger().info(
            'Initialized anchor from first valid fix: '
            f'({self._anchor_lat:.8f}, {self._anchor_lon:.8f}) -> '
            f'map({self._anchor_map_x:.3f}, {self._anchor_map_y:.3f})'
        )

    def _goal_callback(self, msg: GeoPointStamped) -> None:
        if not self._valid_lat_lon(msg.position.latitude, msg.position.longitude):
            self.get_logger().error('Rejected geographic goal: invalid latitude/longitude')
            return

        self._pending_geo_goal = msg
        self.get_logger().info(
            f'Received geographic goal: lat={msg.position.latitude:.8f}, '
            f'lon={msg.position.longitude:.8f}'
        )
        self._process_pending_goal()

    def _process_pending_goal(self) -> None:
        if self._pending_geo_goal is None or self._goal_in_progress:
            return
        if self._anchor_lat is None or self._anchor_lon is None:
            self.get_logger().warn(
                'Goal waiting: geographic anchor is not initialized',
                throttle_duration_sec=2.0,
            )
            return
        if not self._action_client.server_is_ready():
            self.get_logger().warn(
                'Goal waiting: Nav2 NavigateToPose action server is not ready',
                throttle_duration_sec=2.0,
            )
            return

        geo_goal = self._pending_geo_goal
        map_goal = self._convert_goal(geo_goal)
        if map_goal is None:
            self._pending_geo_goal = None
            return

        self._pending_geo_goal = None
        self._debug_goal_publisher.publish(map_goal)

        action_goal = NavigateToPose.Goal()
        action_goal.pose = map_goal
        self._goal_in_progress = True
        future = self._action_client.send_goal_async(
            action_goal,
            feedback_callback=self._feedback_callback,
        )
        future.add_done_callback(self._goal_response_callback)

    def _convert_goal(self, msg: GeoPointStamped) -> Optional[PoseStamped]:
        goal_altitude = msg.position.altitude
        if not math.isfinite(goal_altitude):
            goal_altitude = self._anchor_alt

        offset = geodetic_to_enu_offset(
            latitude=msg.position.latitude,
            longitude=msg.position.longitude,
            altitude=goal_altitude,
            origin_latitude=self._anchor_lat,
            origin_longitude=self._anchor_lon,
            origin_altitude=self._anchor_alt,
        )
        distance = math.hypot(offset.east, offset.north)
        maximum_distance = float(self.get_parameter('maximum_goal_distance_m').value)
        if maximum_distance > 0.0 and distance > maximum_distance:
            self.get_logger().error(
                f'Rejected goal: {distance:.1f} m from anchor exceeds '
                f'maximum_goal_distance_m={maximum_distance:.1f}'
            )
            return None

        dx_map, dy_map = rotate_enu_to_map(
            offset.east,
            offset.north,
            self._map_yaw_rad,
        )
        goal_x = self._anchor_map_x + dx_map
        goal_y = self._anchor_map_y + dy_map
        goal_yaw = self._compute_goal_yaw(goal_x, goal_y)

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self._map_frame
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y
        pose.pose.position.z = (
            offset.up if bool(self.get_parameter('use_goal_altitude').value) else 0.0
        )
        pose.pose.orientation.z = math.sin(goal_yaw * 0.5)
        pose.pose.orientation.w = math.cos(goal_yaw * 0.5)

        self.get_logger().info(
            f'Converted goal: ENU({offset.east:.2f}, {offset.north:.2f}) m -> '
            f'{self._map_frame}({goal_x:.2f}, {goal_y:.2f}), '
            f'yaw={math.degrees(goal_yaw):.1f} deg'
        )
        return pose

    def _compute_goal_yaw(self, goal_x: float, goal_y: float) -> float:
        mode = str(self.get_parameter('goal_orientation_mode').value).lower()
        if mode == 'fixed':
            return math.radians(float(self.get_parameter('fixed_goal_yaw_deg').value))
        if mode != 'face_goal':
            self.get_logger().warn(
                f"Unknown goal_orientation_mode='{mode}', using face_goal",
                throttle_duration_sec=5.0,
            )

        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                rclpy.time.Time(),
            )
            rover_x = transform.transform.translation.x
            rover_y = transform.transform.translation.y
            return math.atan2(goal_y - rover_y, goal_x - rover_x)
        except TransformException:
            return math.radians(float(self.get_parameter('fixed_goal_yaw_deg').value))

    def _goal_response_callback(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._goal_in_progress = False
            self.get_logger().error(f'Failed to send Nav2 goal: {exc}')
            return

        if not goal_handle.accepted:
            self._goal_in_progress = False
            self.get_logger().error('Nav2 rejected the goal')
            return

        self.get_logger().info('Nav2 accepted the geographic goal')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            f'Nav2 distance remaining: {feedback.distance_remaining:.2f} m'
        )

    def _result_callback(self, future) -> None:
        self._goal_in_progress = False
        try:
            wrapped_result = future.result()
            status = wrapped_result.status
            result = wrapped_result.result
            error_code = getattr(result, 'error_code', 0)
            error_msg = getattr(result, 'error_msg', '')
            self.get_logger().info(
                f'Nav2 goal finished: status={status}, error_code={error_code}, '
                f'error_msg="{error_msg}"'
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Could not read Nav2 result: {exc}')

    def _fix_is_usable(self, msg: NavSatFix) -> bool:
        minimum_status = int(self.get_parameter('minimum_fix_status').value)
        if msg.status.status < minimum_status:
            self.get_logger().warn(
                f'Ignoring GPS fix with status={msg.status.status}',
                throttle_duration_sec=2.0,
            )
            return False
        if not self._valid_lat_lon(msg.latitude, msg.longitude):
            return False
        return True

    @staticmethod
    def _valid_lat_lon(latitude: float, longitude: float) -> bool:
        return (
            math.isfinite(latitude)
            and math.isfinite(longitude)
            and -90.0 <= latitude <= 90.0
            and -180.0 <= longitude <= 180.0
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GpsNav2GoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
