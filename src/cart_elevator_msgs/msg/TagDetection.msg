# Single AprilTag detection from the Limelight, republished into ROS by nt_bridge.
# Maps directly onto Limelight NT4 outputs: tid / ta / tx / ty / targetpose_robotspace.
# If no tag is currently visible, tag_id is -1 (publishers may also simply not publish).

std_msgs/Header header

int32 tag_id              # Limelight 'tid'. -1 = no detection.
float64 area              # 'ta' — fraction of image filled by tag (0..1). Proxy for distance.
float64 tx_deg            # 'tx' — horizontal angle from optical axis to tag center.
float64 ty_deg            # 'ty' — vertical angle.

bool has_pose             # true if pose_in_robot_frame is populated.
geometry_msgs/Pose pose_in_robot_frame  # 'targetpose_robotspace' — tag pose relative to robot.
