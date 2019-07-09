#!/usr/bin/env python
'''
 * chatter.py
 * Copyright (c) 2018, Michael Simmons, David Feil-Seifer
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Nevada, Reno nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
'''
import time
import rospy
from dialogue.msg import Issue
from dialogue.msg import Resolution
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient
from std_srvs.srv import Empty
'''
	getYesOrNo
		gets a yes or no response from the keyboard
	args:
		none
	returns:
		true: if input = "yes", "y", or the return key
		false: if response = "no", or "n"
'''
def getYesOrNo():
  print("PLEASE SAY YES OR NO NOW!")

   # for speech stuff
   # rospy.wait_for_service('recognizer/startSpeechPub')
   # try:
   #     startSpeechPub = rospy.ServiceProxy('recognizer/startSpeechPub', Empty)
   #     # test = std_msgsEmpty()
   #     startSpeechPub()
   # except rospy.ServiceException, e:
   #     print "Service call failed: %s"%e

   # flag = True
   # while(flag):
   #      msg = rospy.wait_for_message('recognizer/output', String)
   #      print("Msg: %s", msg)
   #      if msg.data == "":
   #          pass
   #      elif msg.data == "yes":
   #          return True
   #      elif msg.data == "no":
   #          return False
   #      else:
   #          sound_handle.say( "Invalid response! Try again!")

   # for terminal
  done = False
  while not done:
    response = raw_input("(Y/n)?\n").lower()
    if response == "yes" or response =="y" or response =="":
      return True
    elif response== "no" or response =="n":
      return False
    else:
      sound_handle.say( "Invalid response! Try again!")

# '''
# dialogue
# 	Responds to an issue topic. Through the keyboard a human and robot communicate about
# 	how to solve a problem the robot is facing. When a solution has been found a resolution
# 	msg is published.

# args:
# 	data: Issue.msg
# 	pub: resolution publishing topic
# returns:
# 	void
# '''
sound_handle=SoundClient()
def printAndSpeak(line):
	sound_handle.say(line)
	print line

def dialogue(data, pub):
  issue=data.issueprint "GOT TO DIALOGUE!!!!!!!"
	obj= data.object
	msg= Resolution()
	msg.object=obj
	msg.robot_id=data.robot_id
	msg.method=None
	while(msg.method==None):
		print("Issue is: %s", issue)
		if issue == "collision":
      printAndSpeak("It appears that you are going to grab the %s." % obj)
    	rospy.sleep(4)
    	printAndSpeak("Should I grab the %s?" %(obj))
    	rospy.sleep(3)
        if getYesOrNo():
          msg.method="robot_pick_and_place"
          printAndSpeak("Alright! I will pick the object!" % (obj))
        else:
          msg.method = "human_pick_and_place"
          printAndSpeak("Okay. Then please pick and place the %s. Thank you." % (obj))
	pub.publish(msg)
# '''
# chatter
# 	initializes the resolution publisher, chatter node, and issues subscriber.
# args:
# 	none
# returns:
# 	none
# '''
def chatter():

  pub = rospy.Publisher('resolution', Resolution, queue_size=10)
  rospy.init_node('chatter', anonymous=True)
  rospy.Subscriber('issues', Issue, dialogue, pub, queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
#initializes the dialogue
if __name__ == '__main__':
  chatter()
