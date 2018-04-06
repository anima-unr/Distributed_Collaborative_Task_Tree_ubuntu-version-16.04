#!/usr/bin/env python
import roslib; roslib.load_manifest('laser_assembler'); roslib.load_manifest('pr2_msgs')
import rospy; from laser_assembler.srv import *
from sensor_msgs.msg import *
from pr2_msgs.srv import *
from std_msgs.msg import *
from pr2_msgs.msg import *


def scanner():
	pub = rospy.Publisher('/tilt_cloud', PointCloud2, queue_size=10)
	rospy.init_node("test_client")

	rospy.wait_for_service("pr2/laser_tilt_controller/set_periodic_cmd")
	set_cmd = rospy.ServiceProxy('pr2/laser_tilt_controller/set_periodic_cmd', SetPeriodicCmd)
	set_cmd(PeriodicCmd(Header(), 'linear', 3, 10, 0))

	rospy.wait_for_service("assemble_scans")


	rate = rospy.Rate(10)
	rospy.sleep(3)
	while not rospy.is_shutdown():
		try:
			assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
			time = rospy.Time().now()
			then = time - rospy.Duration(3)
			resp = assemble_scans(then, time)
			print 'searching from: %f to %f' % (then.to_sec() , time.to_sec())
			print "Got cloud with %u points" % len(resp.cloud.data)
		
			pub.publish(resp.cloud)

		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		rate.sleep()

if __name__ == '__main__':
    try:
        scanner()
    except rospy.ROSInterruptException:
        pass
