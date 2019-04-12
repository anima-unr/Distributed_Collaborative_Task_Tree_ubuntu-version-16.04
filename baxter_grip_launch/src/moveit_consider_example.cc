#include <ros/ros.h>
#include <tf/transform_listener.h>


#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>



#include <gpd/GraspConfigList.h>
#include <Eigen/Dense>

tf::TransformListener *t;
moveit::planning_interface::MoveGroup *group;
moveit_msgs::DisplayTrajectory display_trajectory;
gpd::GraspConfigList gcl;
ros::Publisher *marker_pub, *pose_pub;

void graspCallback(const gpd::GraspConfigList::ConstPtr& gc)
{
	gcl = *gc;
}

void init_moveit()
{
	ros::NodeHandle nh;
  group = new moveit::planning_interface::MoveGroup("right_arm");
  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.

  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group->getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group->getEndEffectorLink().c_str());

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the 
  // end-effector.

  geometry_msgs::PoseStamped currentPose;
  currentPose = group->getCurrentPose();

  std::cout << "Current pose: " << currentPose << '\n';

}

int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "consider_example");
	ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
	spinner.start();
	t = new tf::TransformListener();

	// init subscribers / publishers
	ros::Subscriber grasps_sub = nh.subscribe("/detect_grasps/clustered_grasps", 1000, graspCallback);
	ros::Publisher mp = nh.advertise<visualization_msgs::Marker>("/visualization_marker",1000);
	ros::Publisher p = nh.advertise<geometry_msgs::PoseStamped>("/pose2",1000);
	marker_pub = &mp;
	pose_pub = &p;

	// init moveit
	init_moveit();

	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

	ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();
	const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("right_arm");

	const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
	planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(kinematic_model));
	planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(kinematic_model, nh, "planning_plugin", "request_adapters"));
	ros::WallDuration sleep_time(10.0);


  ROS_INFO("Ready to run consider pipeline.");

  ros::Rate loop_rate(10.0);
  while( ros::ok() )
  {
  	if( gcl.grasps.size() > 0 ) {
    	geometry_msgs::PoseStamped pose;
    	pose.header = gcl.header;

	    Eigen::Vector3d up(gcl.grasps[0].binormal.x, gcl.grasps[0].binormal.y, gcl.grasps[0].binormal.z);
  	  Eigen::Vector3d forward(gcl.grasps[0].approach.x, gcl.grasps[0].approach.y, gcl.grasps[0].approach.z);
    
    	Eigen::Vector3d m2 = forward;
    	Eigen::Vector3d m0 = up.cross(forward);
    	Eigen::Vector3d m1 = m2.cross(m0);

    	double r0 = m0[0] + m1[1] + m2[2];
    	if( r0 > 0 )
			{
        double r1 = sqrt(r0 + 1.0);
        pose.pose.orientation.w = r1 * 0.5;
        r1 = 0.5 / r1;
        pose.pose.orientation.x = (m1[2] - m2[1]) * r1;
        pose.pose.orientation.y = (m2[0] - m0[2]) * r1;
        pose.pose.orientation.z = (m0[1] - m1[0]) * r1;
    	}
    	else if(( m0[0] >= m1[1]) && (m0[0] >= m2[2]) )
    	{
        double r1 = sqrt(((1 + m0[0]) - m1[1]) - m2[2]);
        double r2 = 0.5 / r1;
        pose.pose.orientation.x = 0.5 * r1;
        pose.pose.orientation.y = (m0[1] + m1[0]) * r2;
        pose.pose.orientation.z = (m0[2] + m2[0]) * r2;
        pose.pose.orientation.w = (m1[2] - m2[1]) * r2;
    	}
    	else if (m1[1] > m2[2])
    	{
        double r1 = sqrt(((1 + m1[1]) - m0[0]) - m2[2]);
        double r2 = 0.5 / r1;
        pose.pose.orientation.x = (m1[0] + m0[1]) * r2;
        pose.pose.orientation.y = 0.5 * r1;
        pose.pose.orientation.z = (m2[1] + m1[2]) * r2;
        pose.pose.orientation.w = (m2[0] - m0[2]) * r2;
    	}
    	else
    	{
        double r1 = sqrt(((1 + m2[2]) - m0[0]) - m1[1]);
        double r2 = 0.5 / r1;
        pose.pose.orientation.x = (m2[0] + m0[2]) * r2;
        pose.pose.orientation.y = (m2[1] + m1[2]) * r2;
        pose.pose.orientation.z = 0.5 * r1;
        pose.pose.orientation.w = (m0[1] - m1[0]) * r2;
    	}

    	//normalize quaternion
    	double mag = sqrt(pose.pose.orientation.x*pose.pose.orientation.x + pose.pose.orientation.y*pose.pose.orientation.y + pose.pose.orientation.z*pose.pose.orientation.z + pose.pose.orientation.w*pose.pose.orientation.w);
    	pose.pose.orientation.x /= mag;
    	pose.pose.orientation.y /= mag;
    	pose.pose.orientation.z /= mag;
    	pose.pose.orientation.w /= mag;

	  	pose.pose.position.x =  gcl.grasps[0].bottom.x;
  		pose.pose.position.y =  gcl.grasps[0].bottom.y;
  		pose.pose.position.z =  gcl.grasps[0].bottom.z;

			bool found_ik = kinematic_state->setFromIK(joint_model_group, pose.pose, 10, 0.1);
			pose_pub->publish(pose);

			geometry_msgs::PoseStamped tp;
			t->transformPose("/base", ros::Time(0), pose, "/world", tp );

			if( found_ik )
			{
				ROS_INFO( "found good grasp");
				planning_interface::MotionPlanRequest req;
				planning_interface::MotionPlanResponse res;
				std::vector<double> tolerance_pose(3, 0.01);
				std::vector<double> tolerance_angle(3, 0.01);
				req.group_name = "right_arm";
				moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("right_gripper", tp, tolerance_pose, tolerance_angle);
				req.goal_constraints.push_back(pose_goal);
				planning_pipeline->generatePlan(planning_scene, req, res);ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
				moveit_msgs::DisplayTrajectory display_trajectory;

				/* Visualize the trajectory */
				ROS_INFO("Visualizing the trajectory");
				moveit_msgs::MotionPlanResponse response;
				res.getMessage(response);

				display_trajectory.trajectory_start = response.trajectory_start;
				display_trajectory.trajectory.push_back(response.trajectory);
				display_publisher.publish(display_trajectory);

				sleep_time.sleep();



		  	group->setPoseTarget(pose);
			}
			else
			{
				ROS_WARN( "no good grasp found");
			}
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}