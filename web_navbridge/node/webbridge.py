#!/usr/bin/env python
import roslib; roslib.load_manifest('web_navbridge')
import rospy
import actionlib
from move_base_msgs.msg import *
from actionlib_msgs.msg import *
from web_navbridge.srv import *

"""
    Bridge node for web clients. This will assumes that there is just one client all the time.

    1. Keep watching rosbridge client list to see which client comes in and out.
"""

class WebNavbridge(object):
    srv_status_name = '/web_navbridge/status'
    srv_status     = None

    srv_command_name = '/web_navbridge/command'
    srv_command    = None

    action_bridge = {} 
    pub_bridge = {}
    action_name = '/move_base'
    action = None

    last_goal = None
    
    def __init__(self):

        self.status = "Ready"
        self.initilized = False
        self.isDisconnected = False

        # action client for navigation
        self.action = actionlib.SimpleActionClient(self.action_name,MoveBaseAction)
        self.action_result = rospy.Subscriber(self.action_name + '/result',MoveBaseActionResult,self.processResult)
        self.action.wait_for_server()

        # status for front-end
        srv_status = rospy.Service(self.srv_status_name,Status,self.processStatusService)
        srv_command = rospy.Service(self.srv_command_name,Command,self.processCommandService)
        
    def processStatusService(self,srv):
        print "Status : " + self.status
        return StatusResponse(self.status) 

    def processCommandService(self,srv):
        print "Command : " + str(srv.command)
        if srv.command == "send_goal":
            goal = MoveBaseGoal()             
            goal.target_pose = srv.pose
            self.last_goal = srv.pose
            self.action.send_goal(goal)
        elif srv.command == "cancel_goal":
            self.action.cancel_goal()
        elif srv.command == "last_goal":
            goal = MoveBaseGoal()             
            goal.target_pose = self.last_goal
            self.action.send_goal(goal)
        else:
            print "Error"

        return CommandResponse("OK","OK")
    def processResult(self,msg):
        if self.isDisconnected:
            self.status = "Disconnected"
            self.isDisconnected = False
        else:
            self.status = "Ready"
            self.last_goal = None

    def spin(self):
        rospy.spin()



if __name__ == '__main__':
    rospy.init_node('web_navbridge')
    
    c = WebNavbridge()
    rospy.loginfo('Initialized')
    c.spin()
    rospy.loginfo('Bye bye')
