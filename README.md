# gps_nav2_goal

ROS 2 Jazzy package that converts a geographic WGS84 target into a local Nav2
`NavigateToPose` goal.

## Data flow

```text
Satellite map / UI
  -> /gps_goal                 geographic_msgs/msg/GeoPointStamped

mavlink_sensor_bridge
  -> /gps/fix                  sensor_msgs/msg/NavSatFix

TF map -> base_link
  -> gps_nav2_goal
  -> /gps_goal/map_pose        geometry_msgs/msg/PoseStamped (debug)
  -> /navigate_to_pose         nav2_msgs/action/NavigateToPose
```

## Why an anchor is required

A GPS coordinate has no direct relation to Nav2's metric `map` frame. This node
uses one known correspondence:

```text
(anchor latitude, longitude) <-> (anchor map x, map y)
```

It then converts the geographic target into an east/north metric offset and
rotates that offset into the `map` frame.

## Anchor modes

### `first_fix` (recommended for a local outdoor session)

At startup, the first valid `/gps/fix` is associated with the rover's current
TF pose in `map`. Start localization and Nav2 first, place the rover at the map
origin/aligned starting pose, then launch this node.

### `fixed` (repeatable georeferenced map)

Configure the exact WGS84 coordinate and corresponding Nav2 map coordinate:

```yaml
anchor_mode: fixed
anchor_latitude: 48.123456
anchor_longitude: 2.123456
anchor_altitude: 100.0
anchor_map_x: 0.0
anchor_map_y: 0.0
```

## Map orientation

`map_yaw_deg` rotates the local East/North frame into Nav2's map frame.

- `0`: map +X points east, map +Y points north.
- `90`: east points toward map +Y.
- `-90`: east points toward map -Y.

This angle must match the orientation of the map used by Nav2.

## Installation

```bash
cd ~/ros2_ws/src
# Copy or clone gps_nav2_goal here

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select gps_nav2_goal
source install/setup.bash
```

## Launch

```bash
ros2 launch gps_nav2_goal gps_nav2_goal.launch.py \
  params_file:=$(ros2 pkg prefix gps_nav2_goal)/share/gps_nav2_goal/config/gps_nav2_goal.yaml
```

Or run directly:

```bash
ros2 run gps_nav2_goal gps_nav2_goal_node \
  --ros-args --params-file ~/ros2_ws/src/gps_nav2_goal/config/gps_nav2_goal.yaml
```

## Publish a geographic goal

Replace the example coordinates with the position selected on the satellite map:

```bash
ros2 topic pub --once /gps_goal geographic_msgs/msg/GeoPointStamped "{
  header: {frame_id: 'wgs84'},
  position: {
    latitude: 48.123500,
    longitude: 2.123700,
    altitude: 100.0
  }
}"
```

The node sends the converted pose to Nav2 through the `/navigate_to_pose` action.
It also publishes the converted pose on `/gps_goal/map_pose` for inspection in
RViz.

## Required startup order

1. Start the rover TF and localization (`map -> odom -> base_link`).
2. Start Nav2 and verify `/navigate_to_pose` exists.
3. Start `mavlink_sensor_bridge` and verify `/gps/fix` has a valid fix.
4. Start `gps_nav2_goal`.
5. Publish a `/gps_goal` target.

## Checks

```bash
ros2 topic echo /gps/fix
ros2 topic echo /gps_goal/map_pose
ros2 action list | grep navigate_to_pose
ros2 run tf2_ros tf2_echo map base_link
```

## Important limitations

- This package converts coordinates; it does not create a traversable satellite
  occupancy grid for Nav2.
- Nav2 still needs a valid global costmap/map covering the target.
- Standard GPS error can be several metres. RTK GPS is recommended for precise
  outdoor navigation.
- The local tangent-plane approximation is designed for nearby goals. The
  default safety limit is 5 km from the anchor.
- Set `map_yaw_deg` correctly or the goal will be rotated around the anchor.

## License

MIT
