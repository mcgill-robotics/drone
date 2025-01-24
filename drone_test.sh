source drone_ws/install/setup.bash

ros2 topic pub -t 1 /enqueue_command custom_msgs/msg/Action "{action_type: 2, takeoff_height: 10.}"

ros2 topic pub -t 1 /enqueue_command custom_msgs/msg/Action "{action_type: 1, n: 10., e: 10., d: -10., vn: 1., ve: 1., vd: 1., vtol_config: 1}"

ros2 topic pub -t 1 /enqueue_command custom_msgs/msg/Action "{action_type: 1, n: -200., e: 200., d: -10., vn: 1., ve: 1., vd: 1., vtol_config: 1}"

ros2 topic pub -t 1 /enqueue_command custom_msgs/msg/Action "{action_type: 1, n: 0., e: 0., d: -10., vn: 1., ve: 1., vd: 1., vtol_config: 1}"

ros2 topic pub -t 1 /enqueue_command custom_msgs/msg/Action "{action_type: 1, n: 0., e: 0., d: -10., vn: 1., ve: 1., vd: 1., vtol_config: 1}"

ros2 topic pub -t 1 /enqueue_command custom_msgs/msg/Action "{action_type: 3}"

