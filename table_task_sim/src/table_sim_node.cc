#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <table_task_sim/Object.h>
#include <table_task_sim/Robot.h>
#include <table_task_sim/SimState.h>
#include <table_task_sim/PickUpObject.h>
#include <table_task_sim/PlaceObject.h>
#include <geometry_msgs/Pose.h>
#include <yaml-cpp/yaml.h>

// used to make sure that markers for robots/goals/objects do not collide and overwrite each other
#define OBJ_PFX 1000
#define ROB_PFX 2000
#define GOAL_PFX 3000

// defines top speed of robot TODO: define in a file
#define TOP_SPEED 0.1

// filename for yaml config file
table_task_sim::SimState load_state_from_file(std::string filename);

// message object used to store current sim state (published each simulator time_step)
table_task_sim::SimState simstate;

/**
	lookup_object_by_name
		finds index of object in simstate by name

	args:
		objname: name of object to find

	returns:
		index of object if found
		-1 if object not found

**/

int lookup_object_by_name( std::string objname )
{
	for( int i = 0; i < simstate.objects.size(); i++ )
	{
		if ( objname.compare( simstate.objects[i].name ) == 0 )
		{
			//object found
			return i;
		}
	}

	// got through entire object list and did not find object
	return -1;
}

/**
	pick
		implements service PickUpObject.srv
		moves robot to object's location and then sets the robot as holding the object
		fails if object with name in req does not exist
		blocks until object is held (or failure)

	args:
		req: robot id and object to pick up (name as string)
		res: result (0 if successful, 1 if failure)

	returns:
		true: if service was successfully called
		false: never
**/


bool pick(table_task_sim::PickUpObject::Request  &req,
          table_task_sim::PickUpObject::Response &res)
{
  //res.result = table_task_sim::PickUpObject::SUCCESS;
	res.result = 0;

	// find object with the name to use
	int idx = lookup_object_by_name(req.object_name);

	if( idx < 0 )
	{
		// object not found return true, but respond with failure code
		res.result = 1;
		return true;
	}

	// object found 

	// set robot's goal to match object
	simstate.robots[req.robot_id].goal = simstate.objects[idx].pose;
	float dist = 999;
	ros::Rate loop_rate( 10 );

	// wait for robot to reach goal
	do {
		float xdist = simstate.robots[req.robot_id].goal.position.x - simstate.robots[req.robot_id].pose.position.x;
		float ydist = simstate.robots[req.robot_id].goal.position.y - simstate.robots[req.robot_id].pose.position.y;
		dist = hypot(ydist,xdist);
		ROS_INFO ("robot [%d] moving to [%s] dist: %f", req.robot_id, req.object_name.c_str(), dist);
		loop_rate.sleep();
	} while( dist > 0.0001 );

	simstate.robots[req.robot_id].holding = req.object_name;

	res.result = 0;
	return true;
}

/**
	place
		implements service PlaceObject.srv
		moves robot and held object to goal location and then stops holding object
		fails if robot is not holding an object
		blocks until goal is reached (or failure)

	args:
		req: robot id and goal location (Pose)
		res: result (0 if successful, 1 if failure)

	returns:
		true: if service was successfully called
		false: never
**/


bool place(table_task_sim::PlaceObject::Request		&req,
		   table_task_sim::PlaceObject::Response	&res)
{
	if( simstate.robots[req.robot_id].holding.compare("") == 0 )
	{
		// not holding anything
		res.result = 1;
		return true;
	}

	// move to place goal
	simstate.robots[req.robot_id].goal = req.goal;
	float dist = 999;
	ros::Rate loop_rate( 10 );

	// wait for robot to reach goal
	do {
		float xdist = simstate.robots[req.robot_id].goal.position.x - simstate.robots[req.robot_id].pose.position.x;
		float ydist = simstate.robots[req.robot_id].goal.position.y - simstate.robots[req.robot_id].pose.position.y;
		dist = hypot(ydist,xdist);
		loop_rate.sleep();
	} while( dist > 0.0001 );

	// drop object
	simstate.robots[req.robot_id].holding = std::string("");
	
	res.result = 0;
	return true;
}

/**
	publish_markers(mp)
		publishes markers for viewing simulator state in rviz
		creates marker message and adds table surface, robots, goals, and objects
		populates information from simstate variable

	args:
		mp: ros publisher (previously instantiated in main)

	returns:
		void
**/

void publish_markers(ros::Publisher *mp)
{
	visualization_msgs::Marker marker;

	// these values are the same for all
	marker.header.frame_id = "/table";
	marker.header.stamp = ros::Time::now();
	marker.action = visualization_msgs::Marker::ADD;

	// publish table
	marker.id = 0001;
	marker.type = visualization_msgs::Marker::CUBE;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = -0.02;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 2.0;
    marker.scale.y = 1.0;
    marker.scale.z = 0.01;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.1f;
    marker.color.g = 0.1f;
    marker.color.b = 0.1f;
    marker.color.a = 1.0;

	mp->publish(marker);

	// publish object poses
	marker.type = visualization_msgs::Marker::CUBE;
	for( int i = 0; i < simstate.objects.size(); i++ )
	{
		marker.id = OBJ_PFX + i;
		marker.pose = simstate.objects[i].pose;
	    marker.scale = simstate.objects[i].scale;
	    marker.color = simstate.objects[i].color;

		mp->publish(marker);
	}

	// publish robot poses
	marker.type = visualization_msgs::Marker::CYLINDER;
	for( int i = 0; i < simstate.robots.size(); i++ )
	{
		marker.id = ROB_PFX + i;
		marker.pose = simstate.robots[i].pose;
	    marker.scale.x = 0.075;
	    marker.scale.y = 0.075;
	    marker.scale.z = 0.01;
	    marker.color = simstate.robots[i].color;

		mp->publish(marker);		

		marker.id = GOAL_PFX + i;
		marker.pose = simstate.robots[i].goal;
	    marker.scale.x = 0.03;
	    marker.scale.y = 0.03;
	    marker.scale.z = 0.01;
	    marker.color = simstate.robots[i].color;
	    marker.color.a = 0.5;

		mp->publish(marker);		

	}
}

/**
	populate_state()
		function to populate the initial state variables
		TODO: read from file or parameter service (probably file) instead of hard-coded values

	args: none
	returns: void
**/

void populate_state(std::string filename)
{
	YAML::Node config = YAML::LoadFile(filename.c_str());
	if( config.IsNull() )
	{
		ROS_ERROR( "file not found: [%s]", filename.c_str());
	}

	YAML::Node objects = config["objects"];
	for( int i = 0; i < objects.size(); i++ )
	{
		table_task_sim::Object obj;
		obj.name = objects[i]["name"].as<std::string>();
		ROS_INFO( "loading object: [%s]", obj.name.c_str() );
		obj.pose.position.x = objects[i]["location"]["x"].as<double>();
		obj.pose.position.y = objects[i]["location"]["y"].as<double>();
		obj.pose.position.z = 0.0;
		obj.pose.orientation.x = 0.0;
		obj.pose.orientation.y = 0.0;
		obj.pose.orientation.z = 0.0;
		obj.pose.orientation.w = 1.0;
		obj.scale.x = objects[i]["scale"]["x"].as<double>();
		obj.scale.y = objects[i]["scale"]["y"].as<double>();
		obj.scale.z = objects[i]["scale"]["z"].as<double>();
		obj.color.r = objects[i]["color"]["r"].as<double>();
		obj.color.g = objects[i]["color"]["g"].as<double>();
		obj.color.b = objects[i]["color"]["b"].as<double>();
		obj.color.a = objects[i]["color"]["a"].as<double>();
		simstate.objects.push_back(obj);
	}

	YAML::Node robots = config["robots"];
	for( int i = 0; i < robots.size(); i++ )
	{
		table_task_sim::Robot rob;
		rob.pose.position.x = robots[i]["location"]["x"].as<double>();
		rob.pose.position.y = robots[i]["location"]["y"].as<double>();
		rob.pose.position.z = 0.0;
		rob.color.r = robots[i]["color"]["r"].as<double>();
		rob.color.g = robots[i]["color"]["g"].as<double>();
		rob.color.b = robots[i]["color"]["b"].as<double>();
		rob.color.a = robots[i]["color"]["a"].as<double>();
		rob.goal = rob.pose;

		simstate.robots.push_back(rob);
	}
}


int main(int argc, char* argv[] )
{
	ros::init(argc, argv, "table_sim");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	ros::Rate loop_rate(100);
	
	std::string filename;
	if( nh_priv.getParam("filename", filename) ) 
	{
		ROS_INFO( "simulator loading config from filename: [%s]", filename.c_str() );
		// populate data (TODO replace with file I/O)
		populate_state(filename);
	}
	else
	{
		ROS_INFO( "file not found... [%s]", filename.c_str() );
		return 1;
	}

	// declare subscribers
	ros::ServiceServer pick_service = nh.advertiseService("pick_service", pick);
	ros::ServiceServer place_service = nh.advertiseService("place_service", place);

	// declare publishers
	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("markers", 1000);
	ros::Publisher state_pub = nh.advertise<table_task_sim::SimState>("state", 1000);
	
	// async spinner thread
	ros::AsyncSpinner spinner(4); // Use 4 threads
	spinner.start();

	ros::Time last_iter = ros::Time::now();

	/* main control loop */
	while( ros::ok() )
	{
		ros::Time curr_time = ros::Time::now();

		// update end effector positions
		ros::Duration diff = curr_time - last_iter;
		for( int i = 0; i < simstate.robots.size(); i++ )
		{
			// get distance from current pos to goal
			float xdist = simstate.robots[i].goal.position.x - simstate.robots[i].pose.position.x;
			float ydist = simstate.robots[i].goal.position.y - simstate.robots[i].pose.position.y;
			float dist = hypot(ydist,xdist);
			float theta = atan2(ydist, xdist);

			float r = TOP_SPEED * diff.toSec();

			// if dist is less than top speed / dir
			if( dist < r )
			{
				// robot has reached goal in this iter, just set the current pos to the goal
				simstate.robots[i].pose = simstate.robots[i].goal;
			}
			else
			{
				// move direction of travel top speed / duration
				simstate.robots[i].pose.position.x += r * cos(theta);
				simstate.robots[i].pose.position.y += r * sin(theta);
			}

			// if the robot is holding an object, move the object to where the robot is
			if( simstate.robots[i].holding.length() > 0 )
			{
				// find object with the name in holding
				int idx = lookup_object_by_name(simstate.robots[i].holding);
				simstate.objects[idx].pose.position = simstate.robots[i].pose.position;
			} 
		} // for i

		// publish markers
		publish_markers(&marker_pub);

		// publish current object and end effector positions 
		state_pub.publish(simstate);

		last_iter = curr_time;
		loop_rate.sleep();
	} // while ros::ok()

	return 0;
}