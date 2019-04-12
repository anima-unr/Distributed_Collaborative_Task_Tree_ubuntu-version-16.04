// BAXTER READ ME //

need to do following in each terminal:
 export ROS_MASTER_URI=http://prada:11311
 export ROS_IP=:10.68.###.##      (for your IP through VPN)
 source ~/onr_ws/devel/setup.bach (or your workspace for this code)


then run the following:

/----------------- REPLACED BY jb_moveit.launch --------------------->
roslaunch kinect2_bridge kinect2_bridge.launch publish_tf:=true depth_registration:=true // for baxter
roslaunch vision_manip_pipeline baxter_base_trans.launch

///// cd /etc/ros/; roslaunch openni_head.launch      //FOR PR2 (on the pr2)


// instead of moveit new_test.launch
rosrun baxter_interface joint_trajectory_action_server.py
roslaunch baxter_moveit_config demo_baxter.launch right_electric_gripper:=true left_electric_gripper:=true  // launched rviz too, load rviz config here


roslaunch vision_manip_pipeline vision_manip_low_traffic.launch


/----
// New things for ortho proj //
rosrun vision_manip_pipeline orthoProj
rosrun vision_manip_pipeline orthographic_tf_pub.py /// IF launching freenect-dave dont need this!!
/----
/----------------------------------------------------------------------------->


/----- REAPLACED BY jb_servers.launch ---------/
roslaunch darknet_ros darknet_ros.launch

rosrun vision_manip_pipeline jb_yolo_obj_det_server.py

rosrun vision_manip_pipeline jb_conv_coord_server //NOT THE .py here, we want c version!

rosrun vision_manip_pipeline jb_get_grasp_server.py

rosrun vision_manip_pipeline jb_pub_workspace_corners_server.py

rosrun vision_manip_pipeline jb_vision_manip_pipeline.py <object>
/------------------------------------------------------------------/


rosrun unr_object_manipulation pick_place_service_visionManip.py -r test.txt

roslaunch table_setting_demo multi_robot_task_demo_visionManip_baxter.launch

roslaunch remote_mutex table_setting_mutex_baxter.launch

roslaunch unr_object_manipulation peer_connection_visionManip_baxter.launch
