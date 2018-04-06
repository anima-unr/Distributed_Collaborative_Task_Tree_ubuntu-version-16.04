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
#include <stdlib.h>
#include <string>
#include <vector>
#include "robotics_task_tree_msgs/State.h"
#include "log.h"

namespace task_net {

#define PUB_SUB_QUEUE_SIZE 100
#define STATE_MSG_LEN (sizeof(State))
#define ACTIVATION_THESH 0.1
#define ACTIVATION_FALLOFF 0.98f

void PeerCheckThread(Node *node);


Node::Node() {
  state_.active = false;
  state_.done = false;
  thread_running_ = false;
  parent_done_ = false;
}

Node::Node(NodeId_t name, NodeList peers, NodeList children, NodeId_t parent,
    State_t state,
    bool use_local_callback_queue, boost::posix_time::millisec mtime):
    local_("~") {
  if (use_local_callback_queue) {
    ROS_DEBUG("Local Callback Queues");
    pub_nh_.setCallbackQueue(pub_callback_queue_);
    sub_nh_.setCallbackQueue(sub_callback_queue_);
  }

  // Generate reverse map
  GenerateNodeBitmaskMap();
  name_ = node_dict_[GetBitmask(name.topic)];

  for (NodeListIterator it = peers.begin(); it != peers.end(); ++it) {
    if(strcmp(it->topic.c_str(), "NONE") != 0) {
      peers_.push_back(node_dict_[GetBitmask(it->topic)]);
      // NOTE: THIS IS PROBABLY IN THE WRONG SPOT NOW BUT IT WAS CAUSING ISSUES BELOW!
    }
  }

  for (NodeListIterator it = children.begin(); it != children.end(); ++it) {
    children_.push_back(node_dict_[GetBitmask(it->topic)]);
  }
  parent_ = node_dict_[GetBitmask(parent.topic)];
  // Setup bitmasks
  InitializeBitmask(name_);
  InitializeBitmasks(peers_);
  InitializeBitmasks(children_);
  InitializeBitmask(parent_);

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
  thread_running_ = false;

  int i = 0;
  std::string type;
  while(parent.topic[i] != '_'){
    i++;
  }
  i++;
  type = parent.topic[i];
  state_.parent_type = std::stoi(type, nullptr,10);
  ROS_INFO("PARENT TYPE %d", state_.parent_type);

  // Get bitmask
  mask_ = GetBitmask(name_->topic);

  // Setup Publisher/subscribers
  InitializeSubscriber(name_);
  InitializePublishers(children_, &children_pub_list_, "_parent");
  InitializePublishers(peers_, &peer_pub_list_, "_peer");
  InitializePublisher(parent_, &parent_pub_);
  InitializeStatePublisher(name_, &self_pub_, "_state");

  NodeInit(mtime);
}

Node::~Node() {}

void Node::init()
{

}

void Node::InitializeBitmask(NodeId_t * node) {
  node->mask = GetBitmask(node->topic);
}

void Node::InitializeBitmasks(NodeListPtr nodes) {
  for (NodeListPtrIterator it = nodes.begin(); it != nodes.end(); ++it) {
    InitializeBitmask(*it);
  }
}

void Node::GenerateNodeBitmaskMap() {

  std::vector<std::string> nodes;
  if (local_.getParam("NodeList", nodes)) {
    for (std::vector<std::string>::iterator it = nodes.begin();
        it != nodes.end(); ++it) {
      NodeId_t *nptr = new NodeId_t;
      nptr->topic = *it;
      nptr->mask = GetBitmask(*it);
      nptr->pub = NULL;
      nptr->state =  {nptr->mask, false, false, 0.0f, 0.0f};
      node_dict_[nptr->mask] = nptr;
    }
  }
}

void Node::Activate() {
    ROS_DEBUG("[%s]: Node::Activate was called!!!!", name_->topic.c_str());

  // TODO JB: have this only spin a new thread if the thread doesn't already exist
  // create peer_check thread if it isn't already running

  // Create new thread if not active
  if(!peer_check_thread) {
    state_.check_peer = true;
    peer_check_thread  = new boost::thread(&PeerCheckThread, this);
    ROS_DEBUG("\n\tThread was not active, so has been created!\n");
    peer_check_thread->detach();
  }
  // if peer check thread reached the end, then kill it?
  else if(!state_.check_peer){
    ROS_DEBUG("\n\nThread has finished, killing it! [%d] \n\n", thread_running_);
      if( thread_running_ ) {
        ROS_DEBUG("killing thread");
        peer_check_thread->interrupt();
        peer_check_thread->join();
      }
      peer_check_thread = NULL;
  }
  // still running so leave alone
  else {
        ROS_DEBUG("\n\tThread was already active\n");
      }
 // if thread is okay, run this
 if(state_.peer_okay) {

      ROS_DEBUG("NODE::Activate: peer has made it into the if statement!!!");
    if (!state_.active && !state_.done) {

      if (ActivationPrecondition()) {
        ROS_INFO("Activating Node: %s", name_->topic.c_str());
        {
          boost::lock_guard<boost::mutex> lock(work_mut);
          state_.active = true;
          ROS_DEBUG("State was set to true!");
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
    state_.peer_okay = false;
    ROS_DEBUG("NODE::ACTIVATE: check peer set back to false!!!");
  }
}

bool Node::ActivationPrecondition() {
  return true;
}

// Deactivate node
void Node::Deactivate() {
  // if (state_.active && state_.owner == name_.c_str()) {
  //   state_.active = false;
  // }

  // TODO kill all subscribers
}

// Activate node
void Node::ActivateNode(NodeId_t node) {

}

// Deactivate node
void Node::DeactivateNode(NodeId_t node) {

}

// Complete node
void Node::Finish() {
  Deactivate();
  state_.done = true;
}

// Get state of current node
State Node::GetState() {
  return state_;
}

void Node::SendToParent(const robotics_task_tree_msgs::ControlMessage msg) {
  ROS_DEBUG("[%s]: Node::SendToParent was called", name_->topic.c_str() );
  ControlMessagePtr msg_temp(new robotics_task_tree_msgs::ControlMessage);
  *msg_temp = msg;
  parent_pub_.publish(msg_temp);
}
void Node::SendToParent(const ControlMessagePtr_t msg) {
  ROS_DEBUG("[%s]: Node::SendToParent was called", name_->topic.c_str() );
  parent_pub_.publish(msg);
}
void Node::SendToChild(NodeBitmask node,
  const robotics_task_tree_msgs::ControlMessage msg) {
  // get publisher for specific node
  ros::Publisher* pub = node_dict_[node]->pub;
  // publish message to the specific child
  ControlMessagePtr msg_temp(new robotics_task_tree_msgs::ControlMessage);
  *msg_temp = msg;
  pub->publish(msg_temp);
}
void Node::SendToChild(NodeBitmask node, const ControlMessagePtr_t msg) {
  node_dict_[node]->pub->publish(msg);
}
void Node::SendToPeer(NodeBitmask node,
  const robotics_task_tree_msgs::ControlMessage msg) {
  ROS_DEBUG("[%s]: Node::SendToPeer was called", name_->topic.c_str() );

  // get publisher for specific node
  ros::Publisher* pub = node_dict_[node]->pub;
  // publish message to the specific child
  ControlMessagePtr msg_temp(new robotics_task_tree_msgs::ControlMessage);
  *msg_temp = msg;
  pub->publish(msg_temp);

}
void Node::SendToPeer(NodeBitmask node, const ControlMessagePtr_t msg) {
  ROS_DEBUG("[%s]: Node::SendToPeer was called", name_->topic.c_str() );
  node_dict_[node]->pub->publish(msg);
}

void Node::ReceiveFromParent(ConstControlMessagePtr_t msg) {
  // Set activation level from parent
  // TODO(Luke Fraser) Use mutex to avoid race condition setup in publisher
  boost::unique_lock<boost::mutex> lck(mut);
  if( msg->type == 0 )
    state_.activation_level = msg->activation_level;
  if( msg->done != 0 )
  {
    parent_done_ = true;
    ROS_DEBUG("[%s]: parent state is done", name_->topic.c_str() );
    state_.active = false;
  }
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
  //if( msg->parent_type == 1 ) {
    state_.peer_active = msg->active || state_.peer_active;
    state_.peer_done = msg->done || state_.peer_done;
    // ROS_INFO("OR NODE ACTIVE, node: %d set to be %d\n\n", msg->sender.node, state_.peer_active);
    // ROS_INFO("OR NODE DONE, node: %d set to be %d\n\n", msg->sender.node, state_.peer_done);

    // if(msg->active == 1){
    //   ROS_INFO("OR NODE, node %d msg said node was active!!! %d\n\n", msg->sender.node, state_.peer_active);
    // }

  //}
  // otherwise not OR so set peer active and done as normal!
/*  else {
    state_.peer_active = msg->active;
    state_.peer_done = msg->done;
  }*/
  state_.done = state_.done || state_.peer_done;
  // ROS_INFO("OTHER, set msg based on peer lists!!! %d\n\n", state_.peer_active);
}

// Main Loop of Update Thread. spins once every mtime milliseconds
void UpdateThread(Node *node, boost::posix_time::millisec mtime) {
    ROS_DEBUG("Node::UpdateThread was called!!!!");
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
  boost::unique_lock<boost::mutex> lck(node->mut);
  node->state_.active = false;
  node->state_.done = true;
  node->working = false;
  node->PublishDoneParent();
  node->PublishStateToPeers();

  // int sleepTime = 200 + (75*node->mask_.robot);
  // boost::this_thread::sleep(boost::posix_time::millisec(sleepTime));
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
  boost::mutex mut;
  boost::unique_lock<boost::mutex> lock(mut);
  while (!node->state_.active) {
    ROS_DEBUG("Check Thread waiting");
    node->cv.wait(lock);
  }
  ROS_DEBUG("Check Work Thread Initialized");
  while (node->state_.active) {
    if (!node->CheckWork()) {
      ROS_DEBUG("Deleting Thread! and Restarting");
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
        << "\n";
        record_file.flush();
}

void RecordThread(Node *node) {
  // Open Record File
  while (true) {
    node->RecordToFile();
    boost::this_thread::sleep(boost::posix_time::millisec(100));
  }
}
// Initialize node threads and variables
void Node::NodeInit(boost::posix_time::millisec mtime) {

  // Initialize node threads
  update_thread = new boost::thread(&UpdateThread, this, mtime);
  work_thread   = new boost::thread(&WorkThread, this);
  check_thread  = new boost::thread(&CheckThread, this);
  // peer_check_thread  = new boost::thread(&PeerCheckThread, this);

  // Initialize recording Thread
  std::string filename = "/home/janelle/onr_ws/src/Distributed_Collaborative_Task_Tree/Data/" + name_->topic + "_Data_.csv";
  ROS_INFO("Creating Data File: %s", filename.c_str());
  record_file.open(filename.c_str());
  record_file.precision(15);
  record_thread = new boost::thread(&RecordThread, this);
}

void Node::ActivationFalloff() {
  boost::unique_lock<boost::mutex> lck(mut);
  state_.activation_level *= ACTIVATION_FALLOFF;
}

// Main Loop of the Node type Each Node Will have this fucnction called at each
// time step to process node properties. Each node should run in its own thread
void Node::Update() {
  ROS_DEBUG("[%s]: Node::Update was called!!!!", name_->topic.c_str());

  // Check if Done // check parent done status
  if (!IsDone()  ) {
    // Check Activation Level
    if (IsActive()) {
      // Check Preconditions
      if (Precondition()) {
        ROS_DEBUG("[%s]: Preconditions Satisfied Safe To Do Work!",
          name_->topic.c_str());
        Activate();
      } else {
          ROS_DEBUG("[%s]: Preconditions Not Satisfied, Spreading Activation!",
            name_->topic.c_str());
        // Spread activation to other nodes in tree
        SpreadActivation();
      }
      // Lower activation level of current node
      ActivationFalloff();
    }
    else {
      ROS_DEBUG_THROTTLE(1, "[%s]: Not Active: %f", name_->topic.c_str(),
        state_.activation_level); }
  }
  // Publish Status
  PublishStatus();
}

void Node::Work() {
  printf("Doing Work\n");
  boost::this_thread::sleep(boost::posix_time::millisec(1000));
  printf("Done!\n");
}

bool Node::CheckWork() {
  boost::this_thread::sleep(boost::posix_time::millisec(100));
  return true;
}

void Node::UndoWork() {
  ROS_DEBUG("Undoing Work");
}

// Deprecated function. use ros message data type with struct generality.
std::string StateToString(State state) {
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
  msg->active = state_.active;
  parent_pub_.publish(msg);
}

void Node::UpdateActivationPotential() {

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
}

bool Node::IsDone() {
  return state_.done;
}

bool Node::IsActive() {
  return state_.activation_level > ACTIVATION_THESH;
}

float Node::ActivationLevel() {
  return state_.activation_level;
}
bool Node::Precondition() {
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

}

void Node::InitializeSubscriber(NodeId_t *node) {
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
}
void Node::InitializePublishers(NodeListPtr nodes, PubList *pub,
    const char * topic_addition) {
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

  node->topic += topic_addition;
  ROS_INFO("[PUBLISHER] - Creating Topic: %s", node->topic.c_str());
  (*pub) = pub_nh_.advertise<robotics_task_tree_msgs::State>(node->topic,
    PUB_SUB_QUEUE_SIZE);
  node_dict_[node->mask]->pub = pub;
  // node_dict_[node.mask]->topic += topic_addition;
}

NodeBitmask Node::GetBitmask(std::string name) {
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
  return *node_dict_[id];
}

ros::CallbackQueue* Node::GetPubCallbackQueue() {
  return pub_callback_queue_;
}
ros::CallbackQueue* Node::GetSubCallbackQueue() {
  return sub_callback_queue_;
}

}  // namespace task_net
