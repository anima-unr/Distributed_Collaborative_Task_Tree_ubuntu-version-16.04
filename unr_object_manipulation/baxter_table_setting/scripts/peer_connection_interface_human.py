#!/usr/bin/env python

import zmq
import rospy
import threading
import Queue
from robotics_task_tree_msgs.msg import *
import pickle

def enum(*sequential, **named):
    enums = dict(zip(sequential, range(len(sequential))), **named)
    reverse = dict((value, key) for key, value in enums.iteritems())
    enums['reverse_mapping'] = reverse
    return type('Enum', (), enums)

def SubThread(sub, cb, event):
    count = 0
    timeout = False
    while event.is_set():
        try:
            x = sub.recv_multipart()
            if len(x) == 2:
                #print 'properly formatted msg: [%s]'%(x[0])
                address = x[0]
                msg = x[1]
                cb(msg, address)
                timeout = False
                count += 1
            else:
                print len(x)
        except zmq.ZMQError as e:
            #print e
            if e.errno == zmq.ETERM:
                break
            elif e.errno == zmq.EAGAIN:
                timeout = True
            else:
                print 'weird error'
                raise

def PubThread(pub, queue, event, debug):
    print 'pub'
    rate = rospy.Rate(1000)
    while event.is_set():
        if not queue.empty():
            #if msg.startswith('PLACE') or msg.startswith('AND') or msg.startswith('OR') or msg.startswith('THEN'):
            #    if debug != 0:
            #        print 'warn: seems like an improperly formatted message'
            #    else:
            #        sys.stdout.write('=')
            #else:
            #    ros_pubs[topic].publish(pickle.loads(msg))
            pub.send_multipart(queue.get())
        else:
            if debug != 0:
                print 'pub queue empty, not publishing'
        rate.sleep()


class NodePeerConnectionInterface:
    def __init__(self, server_params, running_event, debug):
        print(server_params)
        print(debug)
        self.server_params = server_params
        self.debug = debug
        self.context = zmq.Context()
        self.running_event = running_event
        self.pub_queue = Queue.Queue()
        self.ros_pubs = dict()
        self.ros_subs = dict()
        self.subs = dict()
        self.buf = ''
        self.pub = self.CreateZMQPub()


    def InitializeSubscriber(self, name):
        #print 'setting up subscriber:2'
        if self.debug != 0 :
            print 'setting up subscriber: %s'%(name)
        # Setup peer listener
        topic = '%s%s'%(name, '_peer')
        #print(topic)
        self.ros_subs[topic] = rospy.Subscriber(topic, ControlMessage, self.ReceiveFromPeer, queue_size=5, callback_args=topic)

    def InitializePublisher(self, name):
        #print 'setting up publisher:3'
        if self.debug != 0 :
            print 'setting up publisher: %s'%(name)
        # setup peer publisher
        topic = '%s%s'%(name, '_peer')
        #print(topic)
        self.ros_pubs[topic] = rospy.Publisher(topic, ControlMessage, queue_size=5)

        # setup zeromq subscriber
        self.subs[topic] = self.CreateZMQSub(topic, self.SendToPeer)

    def ReceiveFromPeer(self, msg, topic):
        #print 'setting up subscriber:4'
        # Publish to server
        if self.debug != 0:
            print 'Received from peer topic: %s'%(topic)
        else:
            sys.stdout.write('.')
        #self.pub.send_multipart([topic, pickle.dumps(msg)])
        self.pub_queue.put([topic, pickle.dumps(msg)])


    def SendToPeer(self, msg, topic):
        #print 'setting up subscriber:5'
        if self.debug != 0:
            print 'send to peer topic: %s'%(topic)
        else:
            sys.stdout.write('+')
        self.ros_pubs[topic].publish(pickle.loads(msg))

    def CreateZMQPub(self):
        #print('hello~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
        print(self.server_params.pub_port)
        print 'setting up subscriber:6'
        publisher = self.context.socket(zmq.PUB)
        publisher.bind("tcp://*:%s"%(self.server_params.pub_port))

        pub_thread = threading.Thread(target=PubThread, args=[publisher, self.pub_queue, self.running_event, self.debug])
        pub_thread.start()

        return publisher

    def CreateZMQSub(self, topic, cb):
        #print 'setting up subscriber:7'
        subscriber = self.context.socket(zmq.SUB)
        print(self.server_params.address)
        a = subscriber.connect("tcp://%s:%s"%(self.server_params.address, self.server_params.sub_port))
        print(a)
        if a==1:
            print('trueeeeeeeeeeeee')
        subscriber.setsockopt(zmq.SUBSCRIBE, topic)
        subscriber.setsockopt(zmq.RCVTIMEO, 1000)

        # create threads for subscribing/publishing
        sub_thread = threading.Thread(target=SubThread, args=[subscriber, cb, self.running_event])
        sub_thread.start()

        return subscriber

    def AddNode(self, name):
        self.InitializeSubscriber(name)
        self.InitializePublisher(name)
    def AddOutputNode(self, name):
        self.InitializeSubscriber(name)
    def AddInputNode(self, name):
        self.InitializePublisher(name)



ROBOT = enum('PR2', 'BAXTER')
ROBOT_DICT = {'PR2' : 0, 'BAXTER' : 1}
def main():
    # Initialize Node
    rospy.init_node('interface')
    class ServerParam:
        def __init__(self):
            pass
    server = ServerParam
    server.sub_port = rospy.get_param('~sub_port', '5565')
    server.pub_port = rospy.get_param('~pub_port', '5566')
    server.address = '134.197.43.10 '
    #server.address = 'localhost'
    running_event = threading.Event()
    running_event.set()
    print(server.address)
    print 'Creating Nodes'

    node_list = rospy.get_param('~NodeList', None)
    task_file = rospy.get_param('~Nodes', None)
    robot = rospy.get_param('~robot', None)
    robot = ROBOT_DICT[robot]
    debug = rospy.get_param('~debug',0)
    print 'debug'

    interface = NodePeerConnectionInterface(server, running_event, debug)

    for node in node_list:
        if task_file[node]['mask']['robot'] != robot:
            interface.AddOutputNode(node)
        else:
            interface.AddInputNode(node)

    print 'Spinning'
    #rospy.spin()
    rate = rospy.Rate(10)
    n = 0
    while not rospy.is_shutdown():
        sys.stdout.write('\n')
        sys.stdout.flush()
        rate.sleep()

    print 'shutting Down node'
    if rospy.is_shutdown():
        print 'Clearing event'
        running_event.clear()

if __name__ == '__main__':
    main()
