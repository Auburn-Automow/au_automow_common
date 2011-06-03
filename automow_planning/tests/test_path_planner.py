#!/usr/bin/env python
import sys
import os
sys.path.append("../nodes")
import threading
import time

from path_planner import PathPlanner

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def main():
    """docstring for main"""
    pp = PathPlanner(True)
    
    pp.meters_per_cell = 0.4
    pp.field_frame_id = "odom_combined"
    pp.goal_timeout = 15.0
    pp.pick_furthest = False
    pp.file_name = os.path.join(os.getcwd(),"survey.csv")
    pp.field_shape_publish_rate = 1.0
    
    pp_thread = threading.Thread(target=pp.Init())
    pp_thread.start()
    
    time.sleep(1.0)
    
    data = MoveBaseGoal()
    data.target_pose.pose.position.x = 1
    data.target_pose.pose.position.y = 1
    
    data.target_pose.pose.position.x *= pp.meters_per_cell
    data.target_pose.pose.position.x += pp.offset[0]
    
    data.target_pose.pose.position.y *= pp.meters_per_cell
    data.target_pose.pose.position.y = pp.offset[1] - data.target_pose.pose.position.y
    
    pp.odomCallback(data)
    
    curr_dest = pp.current_destination
    while curr_dest != None:
        if rospy.is_shutdown():
            return
        if curr_dest == -1:
            continue
        data = curr_dest
        
        time.sleep(0.1)
        
        data.target_pose.pose.position.x *= pp.meters_per_cell
        data.target_pose.pose.position.x += pp.offset[0]
        
        data.target_pose.pose.position.y *= pp.meters_per_cell
        data.target_pose.pose.position.y = pp.offset[1] - data.target_pose.pose.position.y
        
        pp.odomCallback(data)
        curr_dest = pp.current_destination


if __name__ == '__main__':
    main()