#include <tf/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "vision_manip_pipeline/Conv2DTo3D.h"
#include "vision_manip_pipeline/GetObjLoc.h"
#include "vision_manip_pipeline/GetGrasp.h"
#include "vision_manip_pipeline/PubWorkspace.h"

#include <signal.h>

  tf::TransformListener *t;

// ==========================================================

//TODO: double check order of w,x,y,z! -> since rest of code set up with numbers not as map, do this instead!!!!
//geometry_msgs::PoseStamped getPoseTrans(double x, double y, double z, std::map<std::string, double> ori, const std::string old_frame, const std::string new_frame){
geometry_msgs::PoseStamped getPoseTrans(double x, double y, double z, std::vector<double> ori, const std::string old_frame, const std::string new_frame){
  // ros::Time now = ros::Time::now() - ros::Duration(1.0);
  ros::Time now = ros::Time(0);
  // tf::TransformListener t = new tf::TransformListener(ros::Duration(10.0), true);
  //tf::TransformListener t;
  t->waitForTransform(new_frame, old_frame, now, ros::Duration(3.0));

  geometry_msgs::PoseStamped pnt;
  pnt.header.frame_id = old_frame;
  pnt.header.stamp = now;
  pnt.pose.position.x = x;
  pnt.pose.position.y = y;
  pnt.pose.position.z = z;
  pnt.pose.orientation.w = ori[0]; //TODO: double check order of w,x,y,z!
  pnt.pose.orientation.x = ori[1]; //TODO: double check order of w,x,y,z!
  pnt.pose.orientation.y = ori[2]; //TODO: double check order of w,x,y,z!
  pnt.pose.orientation.z = ori[3]; //TODO: double check order of w,x,y,z!

  //geometry_msgs::PoseStamped newPnt = t.transformPose(new_frame, pnt);
  geometry_msgs::PoseStamped newPnt;
  t->transformPose(new_frame, pnt, newPnt);
  
  std::cout << "\nTRANSFORM: " << newPnt << '\n';
  return newPnt;
}

// ==========================================================

geometry_msgs::PointStamped transPoint(double x, double y, double z, const std::string old_frame, const std::string new_frame){
  // ros::Time now = ros::Time::now() + ros::Duration(1.0);
  ros::Time now = ros::Time(0);
  // tf::TransformListener t = new tf::TransformListener(ros::Duration(10.0), true);
  t->waitForTransform(new_frame, old_frame, now, ros::Duration(3.0));

  geometry_msgs::PointStamped pnt;
  pnt.header.frame_id = old_frame;
  pnt.header.stamp = now;
  pnt.point.x = x;
  pnt.point.y = y;
  pnt.point.z = z;

  // geometry_msgs::PoseStamped newPnt = t.transformPose(new_frame, pnt);
  geometry_msgs::PointStamped newPnt;
  t->transformPoint(new_frame, pnt, newPnt);

  std::cout << "\nTRANSFORM: " << newPnt << '\n';
  return newPnt;
}

// ==========================================================

bool moveArm(geometry_msgs::PoseStamped newApp, geometry_msgs::PoseStamped newPnt){

  moveit::planning_interface::MoveGroup group("right_arm");
  printf("Move it TEST0\n");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // group.setPoseReferenceFrame("/torso_lift_link");
  // // We can print the name of the reference frame for this robot.
  // ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  // // We can also print the name of the end-effector link for this group.
  // ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  //------------------
  printf("Move to approach\n");

  geometry_msgs::Pose pose_target;
  pose_target.orientation.x = newApp.pose.orientation.x;
  pose_target.orientation.y = newApp.pose.orientation.y;
  pose_target.orientation.z = newApp.pose.orientation.z;
  pose_target.orientation.w = newApp.pose.orientation.w;
  pose_target.position.x = newApp.pose.position.x;
  pose_target.position.y = newApp.pose.position.y;
  pose_target.position.z = newApp.pose.position.z;

  group.setPoseTarget(pose_target);

  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");

  if( success) {
    /* Sleep to give Rviz time to visualize the plan. */
    printf("\tPlanning...");
    sleep(5.0);
    // printf("\tMoving...");
    // group.move();
    // sleep(5.0);
  }
  else {
    printf("Moving to next grasp!\n");
    return false;
  }

 //------------------
  printf("Move to pick\n");

  // geometry_msgs::Pose pose_target;
  pose_target.orientation.x = newPnt.pose.orientation.x;
  pose_target.orientation.y = newPnt.pose.orientation.y;
  pose_target.orientation.z = newPnt.pose.orientation.z;
  pose_target.orientation.w = newPnt.pose.orientation.w;
  pose_target.position.x = newPnt.pose.position.x;
  pose_target.position.y = newPnt.pose.position.y;
  pose_target.position.z = newPnt.pose.position.z;

  group.setPoseTarget(pose_target);

  // moveit::planning_interface::MoveGroup::Plan my_plan;
  success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");

  if( success) {
    /* Sleep to give Rviz time to visualize the plan. */
    printf("\tPlanning...");
    sleep(5.0);
    // printf("\tMoving...");
    // group.move();
    // sleep(5.0);
  }
  else {
    printf("Moving to next grasp!\n");
  }

  if(success) {
    return true;
  }
  else {
    return false;
  }
      

}

// ==========================================================

void calcOffset(geometry_msgs::PoseStamped newPnt) {

  // publish tf at point, then backtrack along axis to get the position of the wrist roll so gripper is at point

  // might need to first move to an approach point further along axsi

  // then move to fixed wrist roll joint
}

// ==========================================================

bool checkInBounds(geometry_msgs::PointStamped newPnt) {

  // pr2 arm is 100 cm.... 10^2 = x^2 + y^2 .... so if x^2 + y^2 < 0.9 then okay, else return 0? 
  // but first need to convert to r_torso_lift_side_plate_link frame?? - then can just do the mathhhh

  // convert to r_torso_lift_side_plate_link frame
 geometry_msgs::PointStamped transPnt = transPoint(newPnt.point.x, newPnt.point.y, newPnt.point.z, "/test", "/r_torso_lift_side_plate_link");

  // check if in bounds
  double threshold = 0.7;
  double dist = 1.0;

  dist = (transPnt.point.x * transPnt.point.x) + (transPnt.point.y * transPnt.point.y);

  if( dist < threshold ) {
    return true;
  } 
  else {
    return false;
  }

  // return true; //TODO REMOVE THIS!

}

// ==========================================================

std::vector<double> rotationQuat(std::vector<double> approach, std::vector<double> axis,
                                 std::vector<double> binormal){

    double r1, r2;
    std::vector<double> rotation = {0,0,0,1};
    Eigen::Vector3d up(binormal[0], binormal[1], binormal[2]);
    Eigen::Vector3d forward(axis[0], axis[1], axis[2]);

    Eigen::Vector3d m0 = up.cross(forward);
    Eigen::Vector3d m1 = forward.cross(m0);

    double r0 = m0[0] + m0[1] + m0[2];
    if(r0 > 0){
      double r1 = sqrt(r0 + 1.0);
      rotation[3] = r1 * 0.5;
      r1 = 0.5 / r1;
      rotation[0] = (m1[2] - forward[1]) * r1;
      rotation[1] = (forward[0] - m0[2]) * r1;
      rotation[2] = (m0[1] - m1[0]) * r1;
    }
    else if((m0[0] >= m1[1]) && (m0[0] >= forward[2])){
      r1 = sqrt(((1 + m0[0]) - m1[1]) - forward[2]);
      r2 = 0.5 / r1;
      rotation[0] = 0.5 * r1;
      rotation[1] = (m0[1] + m1[0]) * r2;
      rotation[2] = (m0[2] + forward[0]) * r2;
      rotation[3] = (m1[2] - forward[1]) * r2;
    }
    else if (m1[1] > forward[2]){
      r1 = sqrt(((1 + m1[1]) - m0[0]) - forward[2]);
      r2 = 0.5 / r1;
      rotation[0] = (m1[0] + m0[1]) * r2;
      rotation[1] = 0.5 * r1;
      rotation[2] = (forward[1] + m1[2]) * r2;
      rotation[3] = (forward[0] - m0[2]) * r2;
    }
    else{
      r1 = sqrt(((1 + forward[2]) - m0[0]) - m1[1]);
      r2 = 0.5 / r1;
      rotation[0] = (forward[0] + m0[2]) * r2;
      rotation[1] = (forward[1] + m1[2]) * r2;
      rotation[2] = 0.5 * r1;
      rotation[3] = (m0[1] - m1[0]) * r2;
    }

    double mag = sqrt(rotation[0]*rotation[0] + rotation[1]*rotation[1] + rotation[2]*rotation[2] + rotation[3]*rotation[3]);
    rotation[0] = rotation[0] / mag;
    rotation[1] = rotation[1] / mag;
    rotation[2] = rotation[2] / mag;
    rotation[3] = rotation[3] / mag;

    return rotation;
}

// ==========================================================

int main(int argc, char **argv){
    std::string obj_name;
    if(argc == 2){
      obj_name = argv[1];
    }
    else{
      std::cout << "Error: Please call function with an object name, e.g.\n";
      std::cout <<"\trosrun vision_manip_pipeline jb_vision_manip_pipeline <object_name>\n";
      exit(1);
    }
    std::cout << "Requesting: " << obj_name << '\n';

    ros::init(argc, argv, "vision_manip");
/* TODO: FIGURE OUT ROSLAUNCH STUFF!!!
    std::string uuid = roslaunch.rlutil.get_or_generate_uuid(None, False);
    roslaunch.configure_logging(uuid);
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/janelle/onr_ws/src/gpd/launch/jb_tutorial1.launch"])
*/
    // rosservice call with object to find location of in YOLO
        // So get the bounding box of the image and
        // calculate the center of it as a start for the grasping window?
    //TODO: FIX THESE!
    //vision_manip_pipeline::GetObjLoc::Response resp = get_obj_loc_client(obj_name);
    
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(1);
    spinner.start();


    t = new tf::TransformListener();
    ros::Duration(1.0).sleep();
    ros::ServiceClient objLocClient = n.serviceClient<vision_manip_pipeline::GetObjLoc>("get_object_loc");
    vision_manip_pipeline::GetObjLoc objLocSrv;
    
    objLocSrv.request.obj_name = obj_name;
    if(objLocClient.call(objLocSrv)) {
       std::cout << objLocSrv.response << '\n';
    }
    else{
       ROS_ERROR("ERROR: Failed to call objLoc Service!" );
    }
   
    int x = (objLocSrv.response.xmax - objLocSrv.response.xmin)/2 + objLocSrv.response.xmin;
    int y = (objLocSrv.response.ymax - objLocSrv.response.ymin)/2 + objLocSrv.response.ymin;
    std::cout << "x: " << x << ' ' << "y: " << y << '\n';

    // if object not detected, then exit?
    if(x == 0 && y == 0) {
       ROS_INFO("Error: Object not detected, try again!");
       return -1;
    }

    // calculate the transform of the point in the raw image to
    // the grasping window in the depth point cloud!
    //TODO: FIX THESE!
    //vision_manip_pipeline::Conv2DTo3D::Response resp3 = conv_coord_client(x,y);

    // ros::NodeHandle n;
    ros::ServiceClient conv2DTo3DClient = n.serviceClient<vision_manip_pipeline::Conv2DTo3D>("conv_coord");
    vision_manip_pipeline::Conv2DTo3D conv2DTo3DSrv;
    
    conv2DTo3DSrv.request.x = x;
    conv2DTo3DSrv.request.y = y;
    if(conv2DTo3DClient.call(conv2DTo3DSrv)) {
       std::cout << conv2DTo3DSrv.response << '\n';
    }
    else{
       ROS_ERROR("ERROR: Failed to call convCoord Service!" );
    }

    // TODO: JB
    // transform the point from kinect frame to the orthographic frame (static tf projection)
    geometry_msgs::PointStamped newPnt = transPoint(conv2DTo3DSrv.response.newX, conv2DTo3DSrv.response.newY, conv2DTo3DSrv.response.newZ, "/kinect2_ir_optical_frame", "/test");

    //TODO: JB
    // generate the cube from the transformed point instead
    //  first set the param for the workspace based on the response?!?
    double eps = 0.1;
    
    
    
//------------------------------    
// FIXXXXXX    


    // pr2 arm is 100 cm.... 10^2 = x^2 + y^2 .... so if x^2 + y^2 < 0.9 then okay, else return 0? 
    // but first need to convert to r_torso_lift_side_plate_link frame?? - then can just do the mathhhh
    bool inBounds = false;

    if( !checkInBounds(newPnt) ) {
      std::cout << "Error: Grasp outside of reachable arm space, will now return\n";  
      return -1;  
    }

    std::vector<double> cube(6);
    // cube = {newPnt.point.x - eps, newPnt.point.x + eps, newPnt.point.y - eps, newPnt.point.y + eps, newPnt.point.z - 2*eps, newPnt.point.z + 0.5*eps};    
    cube = {newPnt.point.x - eps, newPnt.point.x + eps, newPnt.point.y - eps, newPnt.point.y + eps, newPnt.point.z - 2*eps, 1.25};    
    // cube = {conv2DTo3DSrv.response.newX - eps, conv2DTo3DSrv.response.newX + eps, conv2DTo3DSrv.response.newY- eps, conv2DTo3DSrv.response.newY + eps, conv2DTo3DSrv.response.newZ - eps, conv2DTo3DSrv.response.newZ + eps};    
  
    std::cout << "cube to search for graps: " << cube[0] << ' ' << cube[1] << ' ' << cube[2] << ' ' << cube[3] << ' ' << cube[4] << ' ' << cube[5] << '\n'; 

    ros::param::set("/detect_grasps/workspace", cube);
    ros::param::set("/detect_grasps/workspace_grasps", cube);

    // ros::NodeHandle n;
    ros::ServiceClient pubWorkspaceClient = n.serviceClient<vision_manip_pipeline::PubWorkspace>("pub_workspace_corners");
    vision_manip_pipeline::PubWorkspace pubWorkspaceSrv;
    
    std::vector<double> tempPos(3);
    tempPos = {0,0,0}; 
    std::vector<double> tempOri(4);
    tempOri = {0,0,0,0}; 

    pubWorkspaceSrv.request.pos = tempPos;
    pubWorkspaceSrv.request.pos2 = tempPos;
    pubWorkspaceSrv.request.ori = tempOri;
    if(pubWorkspaceClient.call(pubWorkspaceSrv)) {
       std::cout << pubWorkspaceSrv.response << '\n';
    }
    else{
       ROS_ERROR("ERROR: Failed to call pubWorkspace Service!" );
    }



    // TODO: JB
    // transform the point cloud to the orthographic frame (static tf projection) ->  will probs need to change frames in launch to get cloud in correct frame!!!
    // transPointCloud(cloud, "/local/depth_registered/points", "/local/depth_registered/trans_points"):


/* TODO: FIGURE OUT ROSLAUNCH STUFF!!!
    // relaunch the grasp stuffsssss
    launch.start()
*/
    // path: /home/janelle/onr_ws/src/gpd/launch/jb_tutorial1.launch
    // system("roslaunch vision_manip_pipeline jb_tutorial1.launch &");

    ros::ServiceClient getGraspClient = n.serviceClient<vision_manip_pipeline::GetGrasp>("get_grasp");
    vision_manip_pipeline::GetGrasp getGraspSrv;

    pid_t pid;
    pid = fork();
    if(pid == 0) { // child process
        setpgid(getpid(), getpid());
        system("roslaunch vision_manip_pipeline jb_tutorial1.launch");
    } 
    else {   // parent process
        // sleep(30);
        // printf("Sleep returned\n");

      // // rosservice call to gpd with the calculated grasping window in the
      // // point cloud to get the top grasp
       //TODO: FIX THESE!
       //vision_manip_pipeline::GetGrasp::Response resp2 = get_grasp_client(resp3.newX, resp3.newY, resp3.newZ);

      // ros::NodeHandle n;
      
      getGraspSrv.request.x = conv2DTo3DSrv.response.newX;
      getGraspSrv.request.y = conv2DTo3DSrv.response.newY;
      getGraspSrv.request.y = conv2DTo3DSrv.response.newZ;
      if(getGraspClient.call(getGraspSrv)) {
         std::cout << getGraspSrv.response << '\n';
      }
      else{
         ROS_ERROR("ERROR: Failed to call getGrasp Service!" );
      }

      kill(-pid, SIGKILL); // kill the launch process
      // signal(SIGINT, SIG_IGN); // kill the node it brings up?
      system("rosnode kill /detect_grasps"); 
      printf("killed process group %d\n", pid);
    }

//  TODO: FIGURE OUT ROSLAUNCH STUFF!!!
//     launch.shutdown()

    // TODO: FIGURE OUT HOW TO RETURN 0 instead of NONE upon fail becuase no C equiv???
    // IF GRASP NOT FOUND, return 0?
    //if( resp2 == "None") {

  if( getGraspSrv.response.num_grasps == 0 ){
      std::cout << "Error: No grasp found, will now return\n";
      return -1;
  }


  for(int indx = 0; indx < getGraspSrv.response.num_grasps; indx++) {

      // if( getGraspSrv.response.grasps.grasps[indx].score.data == 0 ){
      //     std::cout << "Error: No grasp found, will now return\n";
      //     return -1;
      // }

    // convert from geometry_msgs/Vector3 to std::vector<float>
    std::vector<double> axis(3);
    axis = {getGraspSrv.response.grasps.grasps[indx].axis.x, getGraspSrv.response.grasps.grasps[indx].axis.y, getGraspSrv.response.grasps.grasps[indx].axis.z}; 
    std::vector<double> binormal(3);
    binormal = {getGraspSrv.response.grasps.grasps[indx].binormal.x, getGraspSrv.response.grasps.grasps[indx].binormal.y, getGraspSrv.response.grasps.grasps[indx].binormal.z}; 
    std::vector<double> approach(3);
    approach = {getGraspSrv.response.grasps.grasps[indx].approach.x, getGraspSrv.response.grasps.grasps[indx].approach.y, getGraspSrv.response.grasps.grasps[indx].approach.z}; 


  // //------------------------------    


    // // convert the top grasp format to a move-it useable format
    // // then use moveit to move the arm of the Baxter/PR2? in rviz/eal world?
    std::vector<double> ori = rotationQuat(axis, binormal, approach);
    std::cout << "ori: " << ori[0] << ',' << ori[1] << ',' << ori[2] << ',' << ori[3] << '\n';

    // visualize the workspace and grasp.......
    std::vector<double> pos(3);

    std::vector<double> base(3);
    std::vector<double> vec(3);
    std::vector<double> ext_approach(3);
    // pos = {getGraspSrv.response.grasps.grasps[indx].surface.x, getGraspSrv.response.grasps.grasps[indx].surface.y, getGraspSrv.response.grasps.grasps[indx].surface.z};

    // TODO: Try to plan to the approach point instead?!?!?!?! -> FIX THIS WITH DAVE MATH!!!!

    base = {getGraspSrv.response.grasps.grasps[indx].bottom.x, getGraspSrv.response.grasps.grasps[indx].bottom.y, getGraspSrv.response.grasps.grasps[indx].bottom.z};
    vec = {getGraspSrv.response.grasps.grasps[indx].approach.x, getGraspSrv.response.grasps.grasps[indx].approach.y, getGraspSrv.response.grasps.grasps[indx].approach.z};
    // vec = getGraspSrv.response.grasps.grasps[indx].approach
    ext_approach = base;
    ext_approach[0] -= 0.08*vec[0];
    ext_approach[1] -= 0.08*vec[1];
    ext_approach[2] -= 0.08*vec[2];

    // pos = {getGraspSrv.response.grasps.grasps[indx].approach.x, getGraspSrv.response.grasps.grasps[indx].approach.y, getGraspSrv.response.grasps.grasps[indx].approach.z};
    pos = ext_approach;
    std::vector<double> tilt(4);
    tilt = {ori[0], ori[1], ori[2], ori[3]}; //TODO: double check order of w,x,y,z!
    std::cout << "pos: " << pos[0] << ',' << pos[1] << ',' << pos[2] << '\n';
    std::cout << "tilt: " << tilt[0] << ',' << tilt[1] << ',' << tilt[2] << ',' << tilt[3] << '\n';
    
    //TODO: FIX THESE!
    //pub_workspace_corners_client(pos,tilt)

    // ros::NodeHandle n;
    // ros::ServiceClient pubWorkspaceClient = n.serviceClient<vision_manip_pipeline::PubWorkspace>("pub_workspace_corners");
    // vision_manip_pipeline::PubWorkspace pubWorkspaceSrv;
    
    pubWorkspaceSrv.request.pos = pos;
    pubWorkspaceSrv.request.pos2 = base;
    pubWorkspaceSrv.request.ori = tilt;
    if(pubWorkspaceClient.call(pubWorkspaceSrv)) {
       std::cout << pubWorkspaceSrv.response << '\n';
    }
    else{
       ROS_ERROR("ERROR: Failed to call pubWorkspace Service!" );
    }

    // TODO: JB
    //  WILL need to transform from other frame here nowwwwwww!!!!!
    // Transform point into correct PR2 frame for motion planning etc...
    // geometry_msgs::PoseStamped newPose = getPoseTrans(pos[0], pos[1], pos[2], ori, "/test", "/torso_lift_link");
    geometry_msgs::PoseStamped newApproach = getPoseTrans(pos[0], pos[1], pos[2], ori, "/test", "/odom_combined");
    geometry_msgs::PoseStamped newPick = getPoseTrans(base[0], base[1], base[2], ori, "/test", "/odom_combined");

    // std::cout << "final pos: " << newPose.pose.position.x << ',' << newPose.pose.position.y << ',' << newPose.pose.position.z << '\n';
    // std::cout << "final ori: " << newPose.pose.orientation.w << ',' << newPose.pose.orientation.x << ',' << newPose.pose.orientation.y << ',' << newPose.pose.orientation.z << '\n';

//------------------------------    
// FIXXXXXX    
    // TODO: use moveit to plan to this position and orientation!
    if ( moveArm(newApproach, newPick) ) {
      break;
    }
// //------------------------------    

  }

return 0;

}


