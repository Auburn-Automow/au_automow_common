#!/usr/bin/env python

import roslib; roslib.load_manifest('automow_controller')

import rospy
import tf
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose2D, PoseStamped, Twist
from std_msgs.msg import Int32
from automow_pose import automow_pose

import math

path_has_orientation = False

def wrapToPi(angle):
    """
    Wrap a given angle in radians to the range -pi to pi.

    @param angle : The angle to be wrapped
    @param type angle : float
    @return : Wrapped angle
    @rtype : float
    """
    return np.mod(angle+np.pi,2.0*np.pi)-np.pi

class AutomowController:
    def __init__(self):
        # Parameters
        self.TARGET_TURN = rospy.get_param('turn_speed',0.4)
        
        # Subscribers
        self.sub_pose = rospy.Subscriber('/ekf/pose', PoseStamped, 
                self.pose_cb)
        self.sub_global_path = rospy.Subscriber('/au_controller/global_path', Path,
                self.global_path_cb)
        self.sub_goal = rospy.Subscriber('/au_controller/goal', PoseStamped,
                self.goal_cb)
        self.sub_system_update = rospy.Subscriber('/au_controller/state', Int32,
                self.system_update_cb)
        
        # Publishers
        self.pub_cmdvel = rospy.Publisher('cmd_vel', Twist)

        # Class variables
        self.robot_pose = automow_pose() 
        self.local_goal_pose = automow_pose()
        self.global_goal_pose = automow_pose()
        self.system_state = 0

    def global_path_cb(self, msg):
        return

    def goal_cb(self,msg):
        self.global_goal_pose.x = msg.pose.position.x
        self.global_goal_pose.y = msg.pose.position.y
        self.global_goal_pose.z = msg.pose.position.z
        self.global_goal_pose.qx = msg.pose.orientation.x
        self.global_goal_pose.qy = msg.pose.orientation.y
        self.global_goal_pose.qz = msg.pose.orientation.z
        self.global_goal_pose.qw = msg.pose.orientation.w
        self.global_goal_pose.update_alpha()
        return

    def pose_cb(self,msg):
        self.robot_pose.x = msg.pose.position.x
        self.robot_pose.y = msg.pose.position.y
        self.robot_pose.z = msg.pose.position.z
        self.robot_pose.qx = msg.pose.orientation.x
        self.robot_pose.qy = msg.pose.orientation.y
        self.robot_pose.qz = msg.pose.orientation.z
        self.robot_pose.qw = msg.pose.orientation.w
        self.robot_pose.update_alpha()
        return

    def system_update_cb(self,msg):
        self.system_state = msg.data
        return

    def turn_toward_location(self,target_pose, tolerance):
        """ Turn towards a desired location within a set tolerance """
        desired_direction = math.atan2(target_pose.y - self.robot_pose.y, \
                target_pose.x - self.robot_pose.x)
        current_direction = self.robot_pose.alpha

        diff_direction = wrapToPi(desired_direction - current_direction)

        if(diff_direction > tolerance):
            self.setTurn(self.TARGET_TURN)
        elif(diff_direction < -tolerance):
            self.setTurn(-self.TARGET_TURN)
        else:
            self.setTurn(0)
            return 1
        return 0

    def turn_on_wheel(self, target_pose, tolerance, wheel):
        """ Turn towards a desired direction, on one wheel """
        # TODO: Finish this
        desired_direction = math.atan2(target_pose.y - self.robot_pose.y, \
                target_pose.x - self.robot_pose.x)
        current_direction = self.robot_pose.alpha

        diff_direction = wrapToPi(desired_direction - current_direction)


    def setTurn(self,speed):
        msg = Twist()
        msg.angular.z = speed
        self.pub_cmdvel.publish(msg)

    def within_dist_of_goal(self,dist):
        """ Returns True if the robot is less than dist of the goal """
        x_dist = self.global_goal_pose.x - self.robot_pose.x
        y_dist = self.global_goal_pose.y - self.robot_pose.y
        dist_to_goal = math.sqrt(x_dist*x_dist + y_dist*y_dist)
        return(dist_to_goal < dist)


def main():
    rospy.init_node('automow_controller')
    controller = AutomowController()
    
    if controller.system_state == 1:
        controller.turn_toward_location(controller.global_goal_pose,0.1)

    rospy.spin()


if __name__=='__main__':
    main()
