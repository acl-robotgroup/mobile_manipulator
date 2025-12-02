#!/bin/bash

# ref: https://nvidia-isaac-ros.github.io/concepts/visual_global_localization/tutorials/tutorial_map_creation.html

ros2 run isaac_mapping_ros rosbag_to_mapping_data \
    --sensor_data_bag_file camera \
    --camera_topic_config $(ros2 pkg prefix --share mobile_manipulator_perception)/config/rosbag_to_mapping_data.yaml \
    --pose_bag_file pose \
    --pose_topic_name /odom \
    --expected_pose_child_frame_name base_footprint \
    --output_folder_path map/raw \

# for some resaon this vairable is requied
ISAAC_SIM_WS=""

ros2 run isaac_ros_visual_mapping create_cuvgl_map.py \
    --map_folder=map \
    --binary_folder_path $(ros2 pkg prefix isaac_ros_visual_mapping)/bin/visual_mapping \
    --config_folder_path $(ros2 pkg prefix --share isaac_ros_visual_mapping)/configs/isaac \
    --model_dir $(ros2 pkg prefix --share isaac_ros_visual_mapping)/models

# will build model cache if not exists, so modify permissions to be writable
sudo chmod 777 $(ros2 pkg prefix --share isaac_ros_visual_mapping)/models/aliked_lightglue
