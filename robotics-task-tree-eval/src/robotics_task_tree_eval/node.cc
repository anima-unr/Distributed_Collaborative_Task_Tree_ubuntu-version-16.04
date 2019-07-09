/*
robotics-task-tree-eval
Copyright (C) 2015  Luke Fraser

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "robotics_task_tree_eval/node.h"
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <stdlib.h>
#include <string>
#include <vector>
#include "robotics_task_tree_msgs/State.h"
#include "log.h"
#include "vision_manip_pipeline/VisionManip.h"
#include "table_setting_demo/pick_and_place.h"

// #include <regex>

namespace task_net {

#define PUB_SUB_QUEUE_SIZE 100
#define STATE_MSG_LEN (sizeof(State))
#define ACTIVATION_THESH 0.1
#define ACTIVATION_FALLOFF 0.999f

int APPLEHACK = 0;

bool RESP_RECEIVED = false;
bool FAILED_PICK = false;

void PeerCheckThread(Node *node);


//---------------

float getSuitability(uint16_t node, uint8_t robot, std::string object, ros::ServiceClient *visManipClient_pntr) {
  //ROS_ERROR("ENTERED SUITABILITY !!!!!!!!!!!!!!!");
  return 0.0;
  // if not a behavior node, object will be 'N/A' -> so set suitability to be 1?!??
  // if( object.compare("N/A") == 0) {
  //   ROS_INFO("Node has no object, so setting suitability to 1...");
  //   return 1; //TODO JB INTEGRATION -- what should this beeeee?!?!!?
  // }

  // for now just strictly hard code objects for each robot.....
  // this param is only used in dummy behavior, so it does not affect THEN AND OR nodes!

  // define for pr2
  //if(robot == 0) {
  //   // Cup
  //   if( node == 3 ){
  //     return 1.0;
  //   }
  //   // Sugar
  //   if( node == 4 ){
  //     return 0.25;
  //   }
  //   // Tea
  //   if( node == 5 ){
  //     return 1.0;
  //   }
  //   // Left_Bread
  //   if( node == 7 ){
  //     return 0.3;
  //   }
  //   // Meat
  //   if( node == 8 ){
  //     return 0.2;
  //   }
  //   // Lettuce
  //   if( node == 11 ){
  //     return 0.0;
  //   }
  //   // Right_Bread
  //   if( node == 12 ){
  //     return 0.0;
  //   }

    // instead call vision manip pipeline....
  //   vision_manip_pipeline::VisionManip visManipSrv;
  //   visManipSrv.request.obj_name = object.c_str();
  //   std::cout << "Trying Object:    " << object.c_str() << '\n';
  //   if(visManipClient_pntr->call(visManipSrv)){
  //     // ROS_INFO("NewX: %f NewY: %f NewZ: %f", (float)srv.response.newX, (float)srv.response.newY, (float)srv.response.newZ);
  //     std::cout << "Object:             " << object.c_str() << '\n';
  //     std::cout << "Approach Pose:      " << visManipSrv.response.approach_pose << '\n';
  //     std::cout << "Pick Pose:          " << visManipSrv.response.pick_pose << '\n';
  //     std::cout << "Score of Top Grasp: " << visManipSrv.response.score << '\n';
  //     std::cout << "Top Valid Grasp:    " << visManipSrv.response.grasp << '\n';
  //     return visManipSrv.response.score.data;
  //   }
  //   else{
  //     ROS_ERROR("Failed to call service vision_manip, setting score to 0 for object: %s!", object.c_str());
  //     return 0.0;
  //   }
  // }

  //define for baxter
  // else if(robot == 1) {
  //   // Cup
  //   if( node == 16 ){
  //     return 0.0;
  //   }
  //   // Sugar
  //   if( node == 18 ){
  //     return 0.75;
  //   }
  //   // Tea
  //   if( node == 19 ){
  //     return 0.0;
  //   }
  //   // Left_Bread
  //   if( node == 21 ){
  //     return 1.0;
  //   }
  //   // Meat
  //   if( node == 23 ){
  //     return 1.0;
  //   }
  //   // Lettuce
  //   if( node == 24 ){
  //     return 1.0;
  //   }
  //   // Right_Bread
  //   if( node == 25 ){
  //     return 1.0;
  //   }

    // instead call vision manip pipeline....
    // vision_manip_pipeline::VisionManip visManipSrv;
    // visManipSrv.request.obj_name = object.c_str();
    // std::cout << "Trying Object:    " << object.c_str() << '\n';
    // if(visManipClient_pntr->call(visManipSrv)){
    //   // ROS_INFO("NewX: %f NewY: %f NewZ: %f", (float)srv.response.newX, (float)srv.response.newY, (float)srv.response.newZ);
    //   std::cout << "Object:             " << object.c_str() << '\n';
    //   std::cout << "Approach Pose:      " << visManipSrv.response.approach_pose << '\n';
    //   std::cout << "Pick Pose:          " << visManipSrv.response.pick_pose << '\n';
    //   std::cout << "Score of Top Grasp: " << visManipSrv.response.score << '\n';
    //   std::cout << "Top Valid Grasp:    " << visManipSrv.response.grasp << '\n';
    //   return visManipSrv.response.score.data;
    // }
    // else{
    //   ROS_ERROR("Failed to call service vision_manip, setting score to 0 for object: %s!", object.c_str());
    //   return 0.0;
    // }
  //}

}

//---------------


Node::Node() {
  state_.active = false;
  state_.done = false;
  thread_running_ = false;
  parent_done_ = false;
}

Node::Node(NodeId_t name, NodeList peers, NodeList children, NodeId_t parent,
    State_t state,
    std::string object,
    bool use_local_callback_queue, boost::posix_time::millisec mtime):
    local_("~") {
  if (use_local_callback_queue) {
    ROS_DEBUG("Local Callback Queues");
    pub_nh_.setCallbackQueue(pub_callback_queue_);
    sub_nh_.setCallbackQueue(sub_callback_queue_);
  }
    ROS_WARN("Node::Node was called!!!!\n");

  // set client for vision manip pipeline
  ros::ServiceClient visManipClient = local_.serviceClient<vision_manip_pipeline::VisionManip>("/vision_manip");
  visManipClient_pntr = &visManipClient;

  // Generate reverse map
  GenerateNodeBitmaskMap();
  name_   = node_dict_[GetBitmask(name.topic)];

  for (NodeListIterator it = peers.begin(); it != peers.end(); ++it) {
    if(strcmp(it->topic.c_str(), "NONE") != 0) {
      peers_.push_back(node_dict_[GetBitmask(it->topic)]);
      // NOTE: THIS IS PROBABLY IN THE WRONG SPOT NOW BUT IT WAS CAUSING ISSUES BELOW!
      ROS_WARN( "PEER OKAY" );
    }
  }

  for (NodeListIterator it = children.begin(); it != children.end(); ++it) {
    if(strcmp(it->topic.c_str(), "NONE") != 0) {
      children_.push_back(node_dict_[GetBitmask(it->topic)]);
      ROS_WARN( "CHILD IS FOUND" );
    }
  }
  parent_ = node_dict_[GetBitmask(parent.topic)];
  // Setup bitmasks
  InitializeBitmask(name_);
  InitializeBitmasks(peers_);
  InitializeBitmasks(children_);
  InitializeBitmask(parent_);

  ROS_WARN( "BITMASKS" );

  state_ = state;
  state_.owner = name_->mask;
  state_.active = false;
  state_.done = false;
  state_.activation_level = 0.0f;
  state_.activation_potential = 0.0f;
  state_.peer_active = false;
  state_.peer_done = false;
  state_.check_peer = false;
  state_.peer_okay = false;
  state_.highest = name_->mask;
  state_.highest_potential = 0.0;
  state_.collision = false;
  state_.peerPlacing = false;
  state_.selfPlacing = false;
  thread_running_ = false;

  // for obj dropped monitoring
  hold_status_.dropped = false;
  hold_status_.pick = false;
  hold_status_.object_name = "N/A";
  hold_status_.issue = "N/A";


  object_ = object;

  // parse out parent type
  int i = 0;
  std::string type;
  while(parent.topic[i] != '_'){
    i++;
  }
  i++;
  type = parent.topic[i];
  state_.parent_type = std::stoi(type, nullptr,10);
  ROS_INFO("PARENT TYPE %d", state_.parent_type);


  ROS_WARN( "BITMASKS" );

  // get suitability of node based on robot
  // NOTE: this param is only used in dummy/place behavior, so it does not affect THEN AND OR nodes!
  state_.suitability = getSuitability(state_.owner.node, state_.owner.robot, object_, visManipClient_pntr);

  // Get bitmask
  // printf("name: %s\n", name_->topic.c_str());
  mask_ = GetBitmask(name_->topic);

  // Setup Publisher/subscribers
  InitializeSubscriber(name_);
  InitializePublishers(children_, &children_pub_list_, "_parent");
  InitializePublishers(peers_, &peer_pub_list_, "_peer");
  InitializePublisher(parent_, &parent_pub_);

  // undo function
  // std::string tempName_ = name_->topic;
  InitializeStatePublisher(name_, &self_pub_, "_state");
  // name_->topic = tempName_;
  // InitializePublisher(name_, &undo_pub_, "_undo");
  // name_->topic = tempName_ + "_state";

  init_dialogue_ = pub_nh_.advertise<dialogue::Issue>("issues", 1000);

  NodeInit(mtime);

  ROS_WARN("END OF NODE CONSTRUCTOR");
}

Node::~Node() {}

void Node::init()
{

}

void Node::undoCallback(ConstControlMessagePtr_t msg) {
  ROS_INFO("[%s]: Node::undoCallback was called!!!!\n\n\n\n\n", name_->topic.c_str());
  boost::unique_lock<boost::mutex> lck(mut);
  DeactivateNode();
}

void Node::dropCallback(std_msgs::String msg) {
  ROS_ERROR("[%s]: Node::dropCallback was called!!!!", name_->topic.c_str());
  boost::unique_lock<boost::mutex> lck(mut);
  // ROS_INFO("Robot ID: %d", state_.owner.robot);

  hold_status_.dropped = true;
  hold_status_.issue = msg.data;
  ROS_INFO("issue: %s", msg.data.c_str());
}

void Node::InitializeBitmask(NodeId_t * node) {
    // ROS_INFO("Node::InitializeBitmask was called!!!!\n");
  node->mask = GetBitmask(node->topic);
}

void Node::InitializeBitmasks(NodeListPtr nodes) {
    // ROS_INFO("Node::InitializeBitmasks was called!!!!\n");
  for (NodeListPtrIterator it = nodes.begin(); it != nodes.end(); ++it) {
    InitializeBitmask(*it);
  }
}

void Node::GenerateNodeBitmaskMap() {
      // ROS_INFO("Node::GenerateNodeBitmaskMap was called!!!!\n");

  std::vector<std::string> nodes;
  if (local_.getParam("NodeList", nodes)) {
    // printf("Generating BitmaskMap\n");
    for (std::vector<std::string>::iterator it = nodes.begin();
        it != nodes.end(); ++it) {
      NodeId_t *nptr = new NodeId_t;
      nptr->topic = *it;
      nptr->mask = GetBitmask(*it);
      nptr->pub = NULL;
      nptr->state =  {nptr->mask, false, false, 0.0f, 0.0f};
      node_dict_[nptr->mask] = nptr;
      // printf("Adding [%s] to Dictionary.\n", nptr->topic.c_str());
    }
  }
}

void Node::Activate() {
    ROS_DEBUG("[%s]: Node::Activate was called!!!!", name_->topic.c_str());

  // TODO JB: have this only spin a new thread if the thread doesn't already exist
  // create peer_check thread if it isn't already running

  if(!peer_check_thread) {
    state_.check_peer = true;
    peer_check_thread  = new boost::thread(&PeerCheckThread, this);
    ROS_DEBUG("\n\tThread was not active, so has been created!\n");
    peer_check_thread->detach();
  }
  // if peer check thread reached the end, then kill it?
  else if(!state_.check_peer){
    ROS_DEBUG("\n\nThread has finished, killing it! [%d] \n\n", thread_running_ );
      if( thread_running_ ) {
        ROS_DEBUG( "killing thread");
        peer_check_thread->interrupt();
        peer_check_thread->join();
      }
      peer_check_thread = NULL;
  }
  // still running so leave alone
  else {
        ROS_DEBUG("\n\tThread was already active\n");
      }
 // if thread is okay, run this??
 if(state_.peer_okay) {

      ROS_INFO("NODE::Activate: peer has made it into the if statement!!! %d %d",state_.selfPlacing,state_.active);
    if (!state_.active && !state_.done) {
      //ActivationPrecondition();
      //ROS_INFO("NODE::here!!! %d",ActivationPrecondition());
    //if (!state_.done) {
      if (ActivationPrecondition()) {
        ROS_INFO("Activating Node: %s", name_->topic.c_str());
        // printf("\t\tNode::Activate Activating Node: %s\n\n", name_->topic.c_str());
        {
          boost::lock_guard<boost::mutex> lock(work_mut);
          state_.active = true;
          ROS_ERROR("State was set to true!");
          // Send activation to peers to avoid race condition
          // this will publish the updated state to say I am now active
          PublishStateToPeers();
        }
        cv.notify_all();
        // TODO JB: kill the thread now
        // peer_check_thread->interrupt();
        // peer_check_thread = NULL;
        }

    }
    else if(state_.active && state_.selfPlacing){
      // ROS_INFO("NODE::here!!!@@ %d",ActivationPrecondition());
      if (ActivationPrecondition()) {
        ROS_INFO("Activating Node: %s", name_->topic.c_str());
        // printf("\t\tNode::Activate Activating Node: %s\n\n", name_->topic.c_str());
        {
          boost::lock_guard<boost::mutex> lock(work_mut);
          state_.active = true;
          //ROS_ERROR("State was set to true!");
          // Send activation to peers to avoid race condition
          // this will publish the updated state to say I am now active
          state_.selfPlacing = false;
          ROS_ERROR("selfPlacing is true and Active is true");
          PublishStateToPeers();

        }
        cv.notify_all();
        // TODO JB: kill the thread now
        // peer_check_thread->interrupt();
        // peer_check_thread = NULL;

        }



    }
    state_.peer_okay = false;
    ROS_DEBUG("NODE::ACTIVATE: check peer set back to false!!!");
  }
   //ROS_ERROR("Peer Not Okay.");

}

bool Node::ActivationPrecondition() {
     ROS_INFO("Node::ActivationPrecondition was called!!!!\n");
     printf("activation precond\n");
  return true;
}

void Node::Deactivate() {
  ROS_INFO("Node::Deactivate was called!!!!\n");
  //ros::Duration(10).sleep();
  // undo current node
  if(state_.active || state_.done) {
    state_.active = false;
    state_.done = false;
    state_.activation_level = 1000.0f;
    state_.activation_potential = 1000.0f;
  }

  DeactivatePeer();

  ROS_WARN("Work thread is being unlocked");
  work_mut.unlock();
  delete work_thread;
  work_thread = new boost::thread(&WorkThread, this);
  ROS_WARN("Work thread was terminated and then restarted");

}

void Node::ActivateNode(NodeId_t node) {
  // ROS_INFO("Node::ActivateNode was called!!!!\n");
}

void Node::DeactivateNode() {
      ROS_INFO("Node::DeactivateNode was called!!!!\n");
      // thread_running_ = true;
      // // pause architecture
      // boost::thread *pause_thread = new boost::thread(&Node::Deactivate, this);
      // // wait until node is deactivated to continue
      // pause_thread->join();
      Deactivate();

      // set peerUndone to false, and temp to false
      return;
}

void Node::DeactivatePeer() {
  boost::shared_ptr<ControlMessage_t> msg(new ControlMessage_t);
  msg->sender = mask_;
  msg->activation_level = state_.activation_level;
  msg->activation_potential = state_.activation_potential;
  msg->done = state_.done;
  msg->active = state_.active;
  msg->parent_type = state_.parent_type;
  msg->collision = state_.collision;
  msg->peerPlacing = state_.peerPlacing;
  msg->peerUndone = true;

  // publish to all peers
  for (PubList::iterator it = peer_pub_list_.begin();
      it != peer_pub_list_.end(); ++it) {
    it->publish(msg);
  }
}

void Node::ReleaseMutexLocs() {
  // don't release the mutex here

  //ROS_WARN("ReleaseMutexLocs in Node.cc\n\n\n\n\n\n\n\n\n");
  return;
}


void Node::DialogueCallback(const dialogue::Resolution::ConstPtr &msg) {
  RESP_RECEIVED = true;
  FAILED_PICK = true;
  ros::param::set("/Collision", true);
  bool coll_test;
  ros::Duration(1).sleep();
  ros::param::get("/Collision", coll_test);
  std::cout << "Collision is set to: " << coll_test << "\n\n\n\n\n\n\n";

  // if(object_ == "Apple"){
  //   APPLEHACK = 1;
  // }

  // if msg true - set node to done (person placed it)
  if(msg->method == "human_pick_and_place" || msg->method == "human_position_obj") {
    state_.active = false;
    state_.done = false;
    state_.activation_level = 0.0f;
    state_.activation_potential = 0.0f;
    state_.peerPlacing = true;
    // ros::param::set("/Collision", true);
    PublishStateToPeers();
    ros::Duration(1).sleep();
    state_.peerPlacing = false;
    state_.peer_active = true;
    ros::param::set("/Collision", false);
  }
  else if( msg->method == "positioning_done") {
    ROS_WARN("Dialogue finished, positioning_done");
    state_.active = false;
    state_.done = true;
    state_.activation_level = 0.0f;
    state_.activation_potential = 0.0f;
  }
  else if( msg->method == "human_handed_object" ) {
    table_setting_demo::pick_and_place msg;
    msg.request.object = object_;
    ros::param::set("/OutOfBounds", true);
    if (ros::service::call("pick_and_place_object", msg)) {
    }
    ROS_WARN("Serivce call made from dialogue, sleeping for 30 seconds");
    ros::Duration(35).sleep();
    state_.active = false;
    state_.done = true;
    state_.activation_level = 0.0f;
    state_.activation_potential = 0.0f;
  }
  else if (msg->method == "robot_pick_and_place") {
    table_setting_demo::pick_and_place msg;
    msg.request.object = object_;
    // ros::param::set("/Collision", true);
    if (ros::service::call("pick_and_place_object", msg)) {
    }
    ROS_WARN("Serivce call made from dialogue, sleeping for 30 seconds");
    ros::Duration(90).sleep();
    state_.active = false;
    state_.done = true;
    state_.activation_level = 0.0f;
    state_.activation_potential = 0.0f;
    ros::param::set("/Collision", false);
  }

  // otherwise undo node (person did not help)
  else {
    DeactivateNode();
  }

  //unlock the mutex becuase work completed???? Either human placed or robot needs to restart so...
  ROS_WARN("Dialogue finished, releasing mutex");
  // ros::param::set("/Collision", false);
  // work_mut.unlock();
  //ReleaseMutexLocs();
  //state_.activation_potential = 0.1f;
}

void Node::Dialogue() {
  // pause moveit
  // if(APPLEHACK == 1) {
  //   hold_status_.dropped = false;
  //   return;
  // } 
  ROS_WARN("NODE::DOIALOGUE got calledddd here");
  ros::param::set("/Collision", true);
  bool coll_test;
  ros::Duration(1).sleep();
  ros::param::get("/Collision", coll_test);
  std::cout << "Dialogue: Collision is set to: " << coll_test << "\n\n\n\n\n\n\n";

  // publish to initialize dalogue
  dialogue::Issue msg;
  msg.issue = hold_status_.issue;
  msg.object = object_;
  msg.robot_id = state_.owner.robot;

  ROS_WARN("NODE::DOIALOGUE is publishing the issue: %s", msg.issue.c_str());
  init_dialogue_.publish(msg);
  ros::spinOnce();

  ros::Subscriber dialogue_resp_sub = pub_nh_.subscribe<dialogue::Resolution>("resolution", 1000, &Node::DialogueCallback, this);
  ROS_INFO("IN dialogue while loop!\n");
  // wait for response
  while(!RESP_RECEIVED) {
    ros::spinOnce();
    // ROS_INFO("IN dialogue while loop!\n");
  }
  RESP_RECEIVED = false;
  hold_status_.dropped = false;
  ReleaseMutexLocs();
  ros::param::set("/Collision", false);
  //state_.activation_potential = 0.1f;
  //state_.peer_active = true;
  //ros::Duration(5).sleep();

}

void Node::Finish() {
      ROS_INFO("Node::Finish was called!!!!\n");

  //Deactivate();
  state_.done = true;
}

State Node::GetState() {
      // ROS_INFO("Node::GetState was called!!!!\n");

  return state_;
}

void Node::SendToParent(const robotics_task_tree_msgs::ControlMessage msg) {
  ROS_INFO("[%s]: Node::SendToParent was called", name_->topic.c_str() );
  ControlMessagePtr msg_temp(new robotics_task_tree_msgs::ControlMessage);
  *msg_temp = msg;
  parent_pub_.publish(msg_temp);
}
void Node::SendToParent(const ControlMessagePtr_t msg) {
  ROS_INFO("[%s]: Node::SendToParent was called", name_->topic.c_str() );
  parent_pub_.publish(msg);
}
void Node::SendToChild(NodeBitmask node,
  const robotics_task_tree_msgs::ControlMessage msg) {
  //ROS_INFO("[%s]: Node::SendToChild was called", name_->topic.c_str() );
  // get publisher for specific node
  ros::Publisher* pub = node_dict_[node]->pub;
  // publish message to the specific child
  ControlMessagePtr msg_temp(new robotics_task_tree_msgs::ControlMessage);
  *msg_temp = msg;
  pub->publish(msg_temp);
}
void Node::SendToChild(NodeBitmask node, const ControlMessagePtr_t msg) {
    // ROS_INFO("Node::SendToChild was called!!!!\n");
  //ROS_INFO("[%s]: Node::SendToChild was called", name_->topic.c_str() );

  node_dict_[node]->pub->publish(msg);
}
void Node::SendToPeer(NodeBitmask node,
  const robotics_task_tree_msgs::ControlMessage msg) {
  ROS_INFO("[%s]: Node::SendToPeer was called", name_->topic.c_str() );

    // ROS_INFO("Node::SendToPeer was called!!!!\n");
  // get publisher for specific node
  ros::Publisher* pub = node_dict_[node]->pub;
  // publish message to the specific child
  ControlMessagePtr msg_temp(new robotics_task_tree_msgs::ControlMessage);
  *msg_temp = msg;
  pub->publish(msg_temp);

}
void Node::SendToPeer(NodeBitmask node, const ControlMessagePtr_t msg) {
  ROS_INFO("[%s]: Node::SendToPeer was called", name_->topic.c_str() );
  node_dict_[node]->pub->publish(msg);
}

void Node::ReceiveFromParent(ConstControlMessagePtr_t msg) {
  //ROS_INFO("[%s]: Node::ReceiveFromParent was called", name_->topic.c_str() );
  // Set activation level from parent
  // TODO(Luke Fraser) Use mutex to avoid race condition setup in publisher
  boost::unique_lock<boost::mutex> lck(mut);
  if( msg->type == 0 )
    state_.activation_level = msg->activation_level;
  if( msg->done != 0 )
  {
    parent_done_ = true;
    ROS_DEBUG( "[%s]: parent state is done", name_->topic.c_str() );
    state_.active = false;
  }
  //state_.done = msg->done;
}

void Node::ReceiveFromChildren(ConstControlMessagePtr_t msg) {
  ROS_DEBUG("Node::ReceiveFromChildren was called!!!!");
  // Determine the child
  NodeId_t *child = node_dict_[msg->sender];
  boost::unique_lock<boost::mutex> lck(mut);
  child->state.activation_level = msg->activation_level;
  child->state.activation_potential = msg->activation_potential;
  child->state.done = msg->done;
  child->state.highest.node = msg->highest.node;
  child->state.highest.type = msg->highest.type;
  child->state.highest.robot = msg->highest.robot;
  child->state.active = msg->active;
}

void Node::ReceiveFromPeers(ConstControlMessagePtr_t msg) {
    // ROS_INFO("Node::ReceiveFromPeers was called!!!!\n");
  // boost::unique_lock<boost::mutex> lck(mut);
  // state_.activation_level = msg->activation_level;
  // state_.done = msg->done;
  boost::unique_lock<boost::mutex> lck(mut);
  // TODO: Modify this to keep track of peer states from a list of peers if the node's parent is an OR node
  //       This will require doing some sort of "or" on the state so that if it was ever 1 it will stay one
  //       across multiple msgs being recived to know that one of the peers was active.....
  // NOTE: This logic isn't quite happening....... like the OR node never gets hit, we see from
  //       printing below that only the place nodes send messages here......?@?@?@?
  // ROS_INFO("\n\n%d\n\n",msg->sender.type);
  // if( msg->sender.type == 5 && msg->sender.parent_type == 1 ) {
  if( msg->parent_type == 1 ) {
    state_.peer_active = msg->active || state_.peer_active;
    state_.peer_done = msg->done || state_.peer_done;
    // ROS_INFO("OR NODE ACTIVE, node: %d set to be %d\n\n", msg->sender.node, state_.peer_active);
    // ROS_INFO("OR NODE DONE, node: %d set to be %d\n\n", msg->sender.node, state_.peer_done);

    // if(msg->active == 1){
    //   ROS_INFO("OR NODE, node %d msg said node was active!!! %d\n\n", msg->sender.node, state_.peer_active);
    // }

  }
  // otherwise not OR so set peer active and done as normal!
  else {
    state_.peer_active = msg->active;
    state_.peer_done = msg->done;
  }
  // undo function
  if(msg->peerUndone) {
    state_.peer_active = false;
    state_.peer_done = false;
  }
  if(msg->collision) {
    hold_status_.dropped = true;
    hold_status_.issue = "collision";
  }
  if(msg->peerPlacing) {
    //ReleaseMutexLocs();
    state_.done = false;
    state_.active = true;
    state_.peer_active = false;
    state_.peer_done = false;
    state_.activation_level = 100;
    state_.selfPlacing = true;

  }
  state_.done = state_.done || state_.peer_done;
  // ROS_INFO("OTHER, set msg based on peer lists!!! %d\n\n", state_.peer_active);
}

// Main Loop of Update Thread. spins once every mtime milliseconds
void UpdateThread(Node *node, boost::posix_time::millisec mtime) {
    ROS_DEBUG("Node::UpdateThread was called!!!!");
    sleep(5);
  while (true) {
    node->Update();
    boost::this_thread::sleep(mtime);
  }
}

// TODO: need to be able to reset node if work fails
// TODO: Need to be able to cancel work as well.
// IDEA: a work master is started to hault work if necessary.
// IDEA: This thread may be able to start the thread then become the work watcher
// IDEA: The work watcher may need to funtion earlier than the work thread is started.
void WorkThread(Node *node) {
      ROS_DEBUG("[%s]: Node::WorkThread was called!!!!", node->name_->topic.c_str() );


  boost::unique_lock<boost::mutex> lock(node->work_mut);
  while (!node->state_.active) {
    node->cv.wait(lock);
  }
  ROS_DEBUG("work thread Initialized");
  // Process Data
  node->working = true;
  node->Work();

  if ( !FAILED_PICK ) // if did not fail pick i.e. if failed_pick == false
  {
    ROS_WARN("Work thread is ending becuase pick worked\n\n\n");
    ROS_WARN("     Object was %s", node->object_.c_str());
    boost::unique_lock<boost::mutex> lck(node->mut);
    node->state_.active = false;
    node->state_.done = true;
    node->working = false;
  }
  else{
    // Hakcity hack hack
    //  even though we are releasing the mutex from a collision object, the work thread is never ended
    //  so the next object is using the work thread from the previous object that had a collision,
    //  which means this section of code is only entered after the robot completes the place from the next object
    //  so for now we are hard coding the state active and done messages here to reflect this.
    //   THIS NEEDS TO GET FIXED by INSERT_NAME!!!!!!!
    FAILED_PICK = false;
    node->state_.active = false;
    node->state_.done = true;
    node->working = false;
    ROS_WARN("Work thread is ending becuase issue was found");


 }
node->PublishDoneParent();
node->PublishStateToPeers();

// int sleepTime = 200 + (75*node->mask_.robot);
// boost::this_thread::sleep(boost::posix_time::millisec(sleepTime));
// ROS_INFO("Sleeping for %d", sleepTime);
ROS_INFO("[%s]: Work Thread has ended", node->name_->topic.c_str() );
}

// TODO JB: implementation for peer thread!
void PeerCheckThread(Node *node) {
  ROS_DEBUG_NAMED("PeerCheck", "Node::PeerCheckThread was called!!!!");
  node->thread_running_ = true;
try{
  // wait for checking to be asked!
  boost::unique_lock<boost::mutex> lockp(node->peer_mut);
   while (!node->state_.check_peer) {

    ROS_DEBUG_NAMED("PeerCheck", "PeerCheckThread is waiting!");
    node->cv.wait(lockp);
  }
  // LOG_INFO("check peer thread Initialized");
  // notify peers I want to start this node
  // by sending status and activation potential to peers
  node->PublishStateToPeers();

  // TODO: In the future maybe make a recieve from peers call here to ensure
  // that this happens right since the timing of the return from the check
  // causing issues for THEN without some hard-coded offset as below?!?!

  // wait for full loop so can recieved data back from peers
  // NOTE: Due to the exact same timing in the THEN case, change the loop time to deal
  //       with latency for the different sets of nodes
  // int buff = 200+(node->state_.owner.robot * 200);
  // ROS_DEBUG_NAMED("PeerCheck", "\n\t\t\tBUFF: %d \tTOTAL TIME: %d", buff, buff);
  // boost::this_thread::sleep(boost::posix_time::millisec(buff));

  // for each peer, check status
  // (might have to change logic to take highest of all peers?!?)
  // for now just assume only 1 peer!!!
  bool oneOkay = true;
  for (NodeListPtr::iterator it = node->peers_.begin();
      it != node->peers_.end(); ++it) {

    // printf("\n\nPeer DATA:\t%s\n\tactive: %d\tdone:%d\n\n", (*it)->topic.c_str(),(*it)->state.active,(*it)->state.done);
    ROS_DEBUG_NAMED("PeerCheck", "\n\nPeer DATA:\t%s\n\tactive: %d\tdone:%d\n\n", (*it)->topic.c_str(),node->state_.peer_active,node->state_.peer_done);
    ROS_DEBUG_NAMED("PeerCheck", "\n\nMe   DATA:\t%s\n\tactive: %d\tdone:%d\n\n", node->name_->topic.c_str(),node->state_.active,node->state_.done);

    // if peer done, then peer_okay = False (since already completed, I can't activate)
    // if((*it)->state.done) {
    if(node->state_.peer_done) {
       ROS_DEBUG_NAMED("PeerCheck", "PeerCheckThread: Case 1!!");
      // node->state_.peer_okay = false;
      oneOkay = oneOkay && false;
    }
    // otherwise if peer active
    // else if ((*it)->state.active) {
    // NOTE: THIS DOESNT WORK IF OR NODE IF OTHER CHILD IS ACTIVE!!! FIX THIS LOGIC HERE!!!!
    else if (node->state_.peer_active ) {
      // if (node->state_.active && ((*it)->state.activation_potential < node->state_.activation_potential)) {
      //   // if I'm already active and my activation potential/level (which one? potential right)
      //   // is > my peer's activation potentional/level, then peer_okay = True
      //   // NOTE: I don't think this will ever happen, becuase I am not active yet so if
      //   // my peer is already active, then it made it through this process and so it wont
      //   // be stopped from doing work already?!?!
      //  printf("\n\nPeerCheckThread: Case 2!!\n\n");
      //  node->state_.peer_okay = true;
      // }
      // //   // otherwise mine < peer, so let peer be set to active, implies peer_okay = False
      // else{
      ROS_DEBUG_NAMED("PeerCheck", "PeerCheckThread: Case 3!!");
      // node->state_.peer_okay = false;
      oneOkay = oneOkay && false;
      // lower my activation level for this node
      ROS_DEBUG_NAMED("PeerCheck", "\tCurr level: %f\n", node->state_.activation_level);
      node->state_.activation_level = ACTIVATION_FALLOFF*node->state_.activation_level;
      node->state_.activation_potential = ACTIVATION_FALLOFF*node->state_.activation_potential;
      //node->state_.activation_level = 0.1;
      //node->state_.activation_potential = 0.1;
      ROS_DEBUG_NAMED("PeerCheck", "\tNew level: %f\n\n", node->state_.activation_level);
      // }

    }
    // otherwise, peer is not active and peer is not done so I can activate, peer_okay = True
    else if (!node->state_.peer_done && !node->state_.peer_active) {
       ROS_DEBUG_NAMED("PeerCheck", "PeerCheckThread: Case 4!!");
      // node->state_.peer_okay = true;
      oneOkay = oneOkay && true;
    }
    else {
      ROS_WARN("\n\nERROR! PeerCheckThread: Undefined case! Please redo logic!\n\n");
      oneOkay = false;
    }
  }
  node->state_.peer_okay = oneOkay;

  //boost::this_thread::sleep(boost::posix_time::millisec(2000));
  ROS_DEBUG_NAMED("PeerCheck", "\nPeercheckthread is at end!!!!\n");
  node->state_.check_peer = false;
  node->thread_running_ = false;
}
catch(...) {
  ROS_WARN("Peer Check THREAD INTERRUPTED\n\n\n\n");
  node->thread_running_ = false;
}
  node->thread_running_ = false;
}

void CheckThread(Node *node) {
      // ROS_INFO("Node::CheckThread was called!!!!\n");

  boost::mutex mut;
  boost::unique_lock<boost::mutex> lock(mut);
  while (!node->state_.active) {
    ROS_DEBUG("Check Thread waiting");
    // ROS_INFO("Check Thread waiting");
    node->cv.wait(lock);
  }
  ROS_DEBUG("Check Work Thread Initialized");
  // ROS_INFO("Check Work Thread Initialized");
  while (node->state_.active) {
    if (!node->CheckWork()) {
      ROS_DEBUG("Deleting Thread! and Restarting");
      // ROS_INFO("Deleting Thread! and Restarting");
      {
        boost::unique_lock<boost::mutex> lock(node->mut);
        node->work_thread->interrupt();
        delete node->work_thread;
        node->UndoWork();
        node->working = false;
        node->state_.active = false;
        node->work_thread = new boost::thread(&WorkThread, node);
        node->check_thread = new boost::thread(&CheckThread, node);
        break;
      }
    }
    boost::this_thread::sleep(boost::posix_time::millisec(10));
  }
}

void Node::RecordToFile() {
      // ROS_INFO("Node::RecordToFile was called!!!!\n");

  boost::posix_time::ptime time_t_epoch(boost::gregorian::date(1970,1,1));
  boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::universal_time() - time_t_epoch;
  double seconds = (double)diff.total_seconds() + (double)diff.fractional_seconds() / 1000000.0;
  record_file  << std::fixed
        << seconds
        << ", "
        << state_.active
        << ", "
        << state_.done
        << ", "
        << state_.activation_level
        << ", "
        << state_.activation_potential
        << ","
        << working
        << ","
        << state_.suitability
        << "\n";
        record_file.flush();
}
void RecordThread(Node *node) {
      // ROS_INFO("Node::RecordThreadc was called!!!!\n");

  // Open Record File
  while (true) {
    node->RecordToFile();
    boost::this_thread::sleep(boost::posix_time::millisec(100));
  }
}
// Initialize node threads and variables
void Node::NodeInit(boost::posix_time::millisec mtime) {
  // ROS_INFO("Node::NodeInit was called!!!!\n");

  // Initialize node threads
  update_thread = new boost::thread(&UpdateThread, this, mtime);
  work_thread   = new boost::thread(&WorkThread, this);
  check_thread  = new boost::thread(&CheckThread, this);
  // peer_check_thread  = new boost::thread(&PeerCheckThread, this);

  // Initialize recording Thread
  std::string filename = "/home/bashira/catkin_ws/src/Distributed_Collaborative_Task_Tree/Data/" + name_->topic + "_Data_.csv";
  ROS_INFO("Creating Data File: %s", filename.c_str());
  record_file.open(filename.c_str());
  record_file.precision(15);
  record_thread = new boost::thread(&RecordThread, this);
}

void Node::ActivationFalloff() {
    // ROS_INFO("Node::ActivationFalloff was called!!!!\n");
  boost::unique_lock<boost::mutex> lck(mut);
  state_.activation_level *= ACTIVATION_FALLOFF;
}
// Main Loop of the Node type Each Node Will have this fucnction called at each
// time step to process node properties. Each node should run in its own thread
void Node::Update() {
  ROS_DEBUG("[%s]: Node::Update was called!!!!", name_->topic.c_str());


  // Check if Done // check parent done status
  if (!IsDone()  ) {

    if( parent_done_ )
    {
      // deactivate

      // set done to true
      //state_.done = true;
    }

    // Check Activation Level
    if (IsActive()) {
      // check in object has been dropped
      if(hold_status_.dropped) {
        // pause architecture
        ROS_ERROR("Hold_status dropped is true, launching dialogue node!!!"); // why is this not getting printed out????
        boost::thread *pause_thread = new boost::thread(&Node::Dialogue, this);
        pause_thread->join();
      }
      // Check Preconditions
      if (Precondition()) {
        if(name_->topic.compare("PLACE_3_0_003_state")==0){
        ROS_ERROR("[%s]: Preconditions Satisfied Safe To Do Work!",
          name_->topic.c_str());
      }
        Activate();
      } else {

      //   if(name_->topic.compare("PLACE_3_0_003_state")==0){
       //   ROS_ERROR("[%s]: Preconditions Not Satisfied, Spreading Activation!",
       //     name_->topic.c_str());
      //  }
        SpreadActivation();
      }
      ActivationFalloff();
    }
    else {
    //    if(name_->topic.compare("PLACE_3_0_003_state")==0){
    //     ROS_ERROR("[%s]: Activation Below Threshold !",
   //     name_->topic.c_str());
   //   }
      ROS_DEBUG_THROTTLE(1, "[%s]: Not Active: %f", name_->topic.c_str(),
        state_.activation_level); }
  }
  // Publish Status
  PublishStatus();
}

void Node::Work() {
    // ROS_INFO("Node::Work was called!!!!\n");
  printf("Doing Work\n");
  boost::this_thread::sleep(boost::posix_time::millisec(1000));
  printf("Done!\n");
}

bool Node::CheckWork() {
    // ROS_INFO("Node::CheckWork was called!!!!\n");
  // LOG_INFO("Checking Work");
  boost::this_thread::sleep(boost::posix_time::millisec(100));
  return true;
}

void Node::UndoWork() {
      // ROS_INFO("Node::UndoWork was called!!!!\n");

  ROS_DEBUG("Undoing Work");
}
// Deprecated function. use ros message data type with struct generality.
std::string StateToString(State state) {
    // ROS_INFO("StateToString was called!!!!\n");

  char buffer[sizeof(State)*8];
  snprintf(buffer, sizeof(buffer), "Owner:%u, Active:%d, Done:%d, Level:%f",
    *reinterpret_cast<uint32_t*>(&state),
    *(reinterpret_cast<uint8_t*>(&state)+sizeof(NodeBitmask)),
    *(reinterpret_cast<uint8_t*>(&state)+sizeof(NodeBitmask)+sizeof(bool)),
    *(reinterpret_cast<float*>(&state)+sizeof(NodeBitmask)+sizeof(bool)*2));
  std::string str = buffer;
  return str;
}

void Node::PublishStatus() {
  ROS_DEBUG("[%s]: Node::PublishStatus was called", name_->topic.c_str());
  robotics_task_tree_msgs::State msg;
  msg.owner.type = state_.owner.type;
  msg.owner.robot = state_.owner.robot;
  msg.owner.node = state_.owner.node;
  msg.active = state_.active;
  msg.done = state_.done;
  msg.activation_level = state_.activation_level;
  msg.activation_potential = state_.activation_potential;
  msg.peer_active = state_.peer_active;
  msg.peer_done = state_.peer_done;
  msg.highest.type = state_.highest.type;
  msg.highest.robot = state_.highest.robot;
  msg.highest.node = state_.highest.node;
  msg.highest_potential = state_.highest_potential;
  msg.parent_type = state_.parent_type;

  //*msg = state_; // for some reason this doesn't work anymore
  //ROS_INFO("[%s]: PublishStatus", name_->topic.c_str() );
  self_pub_.publish(msg);

  // Publish Activation Potential
  PublishActivationPotential();
  PublishStateToPeers();
  PublishStateToChildren();
}

void Node::PublishStateToPeers() {
  ROS_DEBUG("[%s]: Node::PublishStateToPeers was called", name_->topic.c_str());
  boost::shared_ptr<ControlMessage_t> msg(new ControlMessage_t);
  msg->sender = mask_;
  msg->activation_level = state_.activation_level;
  msg->activation_potential = state_.activation_potential;
  msg->done = state_.done;
  msg->active = state_.active;
  msg->parent_type = state_.parent_type;
  msg->collision = state_.collision;
  msg->peerPlacing = state_.peerPlacing;
  msg->peerUndone = false;

  for (PubList::iterator it = peer_pub_list_.begin();
      it != peer_pub_list_.end(); ++it) {
    it->publish(msg);
  }
}

void Node::PublishStateToChildren() {
  boost::shared_ptr<ControlMessage_t> msg(new ControlMessage_t);
  msg->sender = mask_;
  msg->type = 1; // sets to state only control message
  msg->activation_level = state_.activation_level;
  msg->activation_potential = state_.activation_potential;
  msg->done = state_.done;
  msg->active = state_.active;
  msg->parent_type = state_.parent_type;
  msg->collision = state_.collision;
  msg->peerPlacing = state_.peerPlacing;
  msg->peerUndone = false;


  for (PubList::iterator it = children_pub_list_.begin();
      it != children_pub_list_.end(); ++it) {
    it->publish(msg);
  }

}

void Node::PublishActivationPotential() {
  ROS_DEBUG("[%s]: Node::PublishActivationPotential was called", name_->topic.c_str());
  // Update Activation Potential
  UpdateActivationPotential();
  ControlMessagePtr_t msg(new ControlMessage_t);
  msg->sender = mask_;
  msg->activation_level = state_.activation_level;
  msg->activation_potential = state_.activation_potential;
  msg->done = state_.done;
  msg->highest.node = state_.highest.node;
  msg->highest.type = state_.highest.type;
  msg->highest.robot = state_.highest.robot;
  // ROS_INFO("msg->activation_level %f", msg->activation_level);
  // ROS_INFO("msg->activation_potential %f", msg->activation_potential);
  msg->active = state_.active;
  parent_pub_.publish(msg);
}

void Node::UpdateActivationPotential() {
      // ROS_INFO("Node::UpdateActivationPotential was called!!!!\n");

}

void Node::PublishDoneParent() {
  ROS_DEBUG("Node::PublishDoneParent was called!!!!\n");

  ControlMessagePtr_t msg(new ControlMessage_t);
  msg->sender = mask_;
  msg->activation_level = state_.activation_level;
  msg->activation_potential = state_.activation_potential;
  msg->done = state_.done;
  msg->active = state_.active;
  parent_pub_.publish(msg);
  // printf("Publish Status: %d\n", msg->done);
}

bool Node::IsDone() {
      // ROS_INFO("Node::IsDone was called!!!!\n");

  return state_.done;
}
bool Node::IsActive() {
      // ROS_INFO("Node::IsActive was called!!!!\n");

  return state_.activation_level > ACTIVATION_THESH;
}
float Node::ActivationLevel() {
      // ROS_INFO("Node::ActivationLevel was called!!!!\n");

  return state_.activation_level;
}
bool Node::Precondition() {
      // ROS_INFO("Node::Precondition was called!!!!\n");

  // TODO(Luke Fraser) Merge children/peer/name/parent lists to point to the
  // same as dictionary
  bool satisfied = true;
  for (NodeListPtrIterator it = children_.begin();
      it != children_.end(); ++it) {
    satisfied = satisfied && (*it)->state.done;
  }
  if (satisfied)
    return true;
  return false;
}
uint32_t Node::SpreadActivation() {
      // ROS_INFO("Node::SpreadActivation was called!!!!\n");

}

void Node::InitializeSubscriber(NodeId_t *node) {
    // ROS_INFO("Node::InitializeSubscriber was called!!!!\n");
  std::string peer_topic = node->topic + "_peer";
  ROS_INFO("[SUBSCRIBER] - Creating Peer Topic: %s", peer_topic.c_str());
  peer_sub_     = sub_nh_.subscribe(peer_topic,
    PUB_SUB_QUEUE_SIZE,
    &Node::ReceiveFromPeers,
    this);

  ROS_INFO("[SUBSCRIBER] - Creating Child Topic: %s", node->topic.c_str());
  children_sub_ = sub_nh_.subscribe(node->topic,
    PUB_SUB_QUEUE_SIZE,
    &Node::ReceiveFromChildren,
    this);
  std::string parent_topic = node->topic + "_parent";
  ROS_INFO("[SUBSCRIBER] - Creating Parent Topic: %s", parent_topic.c_str());
  parent_sub_ = sub_nh_.subscribe(parent_topic,
    PUB_SUB_QUEUE_SIZE,
    &Node::ReceiveFromParent,
    this);
  //std::string undo_topic = node->topic + "_undo";
  // std::string undo_topic = object_ + "_undo";
  // ROS_INFO("[SUBSCRIBER] - Creating UNDO Topic: %s", undo_topic.c_str());
  // undo_sub_ = sub_nh_.subscribe(undo_topic,
  //   PUB_SUB_QUEUE_SIZE,
  //   &Node::undoCallback,
  //   this);
  // std::string drop_topic = node->topic + "_dropped";
  std::string drop_topic = object_ + "_dropped";
  ROS_INFO("[SUBSCRIBER] - Creating objDropListener Topic: %s", drop_topic.c_str());
  drop_sub_ = sub_nh_.subscribe(drop_topic,
    PUB_SUB_QUEUE_SIZE,
    &Node::dropCallback,
    this);
}
void Node::InitializePublishers(NodeListPtr nodes, PubList *pub,
    const char * topic_addition) {
    // ROS_INFO("Node::InitializePublishers was called!!!!\n");
  for (NodeListPtrIterator it = nodes.begin(); it != nodes.end(); ++it) {
    ros::Publisher * topic = new ros::Publisher;
    *topic =
      pub_nh_.advertise<robotics_task_tree_msgs::ControlMessage>(
        (*it)->topic + topic_addition,
        PUB_SUB_QUEUE_SIZE);

    pub->push_back(*topic);
    node_dict_[(*it)->mask]->pub = topic;
    node_dict_[(*it)->mask]->topic += topic_addition;
    ROS_INFO("[PUBLISHER] - Creating Topic: %s", (*it)->topic.c_str());
  }
}

void Node::InitializePublisher(NodeId_t *node, ros::Publisher *pub,
    const char * topic_addition) {
      // ROS_INFO("Node::InitializePublisher was called!!!!\n");

  node->topic += topic_addition;
  ROS_INFO("[PUBLISHER] - Creating Topic: %s", node->topic.c_str());
  (*pub) =
    pub_nh_.advertise<robotics_task_tree_msgs::ControlMessage>(node->topic,
      PUB_SUB_QUEUE_SIZE);
  node_dict_[node->mask]->pub = pub;
  // node_dict_[node.mask]->topic += topic_addition;
}

void Node::InitializeStatePublisher(NodeId_t *node, ros::Publisher *pub,
  const char * topic_addition) {
      // ROS_INFO("Node::InitializeStatePublisher was called!!!!\n");

  node->topic += topic_addition;
  ROS_INFO("[PUBLISHER] - Creating Topic: %s", node->topic.c_str());
  (*pub) = pub_nh_.advertise<robotics_task_tree_msgs::State>(node->topic,
    PUB_SUB_QUEUE_SIZE);
  node_dict_[node->mask]->pub = pub;
  // node_dict_[node.mask]->topic += topic_addition;
}

NodeBitmask Node::GetBitmask(std::string name) {
    // ROS_INFO("Node::GetBitmask was called!!!!\n");
  // Split underscores
  std::vector<std::string> split_vec;
  boost::algorithm::split(split_vec, name,
    boost::algorithm::is_any_of("_"));
  NodeBitmask mask;
  // node_type
  mask.type  = static_cast<uint8_t>(atoi(split_vec[1].c_str()));
  mask.robot = static_cast<uint8_t>(atoi(split_vec[2].c_str()));
  mask.node  = static_cast<uint16_t>(atoi(split_vec[3].c_str()));
  return mask;
}
NodeId_t Node::GetNodeId(NodeBitmask id) {
    // ROS_INFO("Node::GetNodeId was called!!!!\n");
  return *node_dict_[id];
}

ros::CallbackQueue* Node::GetPubCallbackQueue() {
    // ROS_INFO("Node::GetPubCallbackQueue was called!!!!\n");
  return pub_callback_queue_;
}
ros::CallbackQueue* Node::GetSubCallbackQueue() {
    // ROS_INFO("Node::GetSubCallbackQueue was called!!!!\n");
  return sub_callback_queue_;
}
}  // namespace task_net
