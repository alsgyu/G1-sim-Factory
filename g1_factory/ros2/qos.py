"""Single source of truth for topic names, message types and QoS of the stack.

docs/ROS2_INTERFACE.md is generated from measurements of these exact profiles.
Remap topic names (``--ros-args -r``) to attach Isaac Sim or a real G1; keep QoS
compatible: a RELIABLE subscriber cannot receive from a BEST_EFFORT publisher.
"""
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def _qos(depth, reliable=True, latched=False):
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=depth,
                      reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL if latched else DurabilityPolicy.VOLATILE)


# Large, periodic sensor streams: latest data matters more than every sample.
SENSOR = _qos(5, reliable=False)          # == rclpy qos_profile_sensor_data
IMU = _qos(50, reliable=False)            # 200 Hz; deeper queue for estimators
CLOCK = _qos(1, reliable=False)           # matches rclpy TimeSource's /clock subscription
STATE = _qos(10)                          # odom, joint_states, cmd_vel, status
TF = _qos(100)                            # tf2_ros TransformBroadcaster default
TF_STATIC = _qos(1, latched=True)         # tf2_ros StaticTransformBroadcaster default
LATCHED = _qos(1, latched=True)           # /map, /vln/subgoals: late joiners get the last one
COMMAND = _qos(10)                        # /goal_pose, /vln/instruction
EVENTS = _qos(50)                         # /vln/feedback, /nav/status
EVIDENCE = _qos(2)                        # /vln/inspection_image (event images, reliable)
PATH = _qos(1)                            # /plan: only the newest plan is valid

# name: (message type, QoS) - message types are given as "pkg/msg/Type" strings.
TOPICS = {
    "/clock": ("rosgraph_msgs/msg/Clock", CLOCK),
    "/camera/camera/color/image_raw": ("sensor_msgs/msg/Image", SENSOR),
    "/camera/camera/color/camera_info": ("sensor_msgs/msg/CameraInfo", SENSOR),
    "/camera/camera/aligned_depth_to_color/image_raw": ("sensor_msgs/msg/Image", SENSOR),
    "/camera/camera/aligned_depth_to_color/camera_info": ("sensor_msgs/msg/CameraInfo", SENSOR),
    "/livox/lidar": ("sensor_msgs/msg/PointCloud2", SENSOR),
    "/livox/imu": ("sensor_msgs/msg/Imu", IMU),
    "/odom": ("nav_msgs/msg/Odometry", STATE),
    "/joint_states": ("sensor_msgs/msg/JointState", STATE),
    "/tf": ("tf2_msgs/msg/TFMessage", TF),
    "/tf_static": ("tf2_msgs/msg/TFMessage", TF_STATIC),
    "/map": ("nav_msgs/msg/OccupancyGrid", LATCHED),
    "/sim/status": ("std_msgs/msg/String", STATE),
    "/cmd_vel": ("geometry_msgs/msg/Twist", STATE),
    "/vln/instruction": ("std_msgs/msg/String", COMMAND),
    "/vln/feedback": ("std_msgs/msg/String", EVENTS),
    "/vln/subgoals": ("geometry_msgs/msg/PoseArray", LATCHED),
    "/vln/inspection_image": ("sensor_msgs/msg/CompressedImage", EVIDENCE),
    "/goal_pose": ("geometry_msgs/msg/PoseStamped", COMMAND),
    "/plan": ("nav_msgs/msg/Path", PATH),
    "/nav/status": ("std_msgs/msg/String", EVENTS),
    "/vln/nav_request": ("std_msgs/msg/String", COMMAND),
    "/vln/nav_status": ("std_msgs/msg/String", EVENTS),
    "/vln/decision_image": ("sensor_msgs/msg/CompressedImage", EVIDENCE),
}


def describe(qos: QoSProfile) -> str:
    return (f"{qos.reliability.name}, {qos.durability.name}, {qos.history.name}({qos.depth})")
