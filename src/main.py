#! /usr/bin/python3

import rospy                  # Import the Python library for ROS
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped         # Import the Twist Message from the geometry_msgs package
import csv 
from scipy.spatial import distance
from math import atan2
from tf.transformations import quaternion_from_euler
import os
import roslib

START_TRESHOLD = 0.15
roslib.packages.get_pkg_dir('final_project')
PATH_NAME = roslib.packages.get_pkg_dir('final_project') + '/paths/path3.csv'
GOAL_MODE = 2
# 0: the goal quaternion is ignored
# 1: the goal quaternion is computed based on the previous goal and the current goal
# 2: the goal quaternion is computed based on the current goal and the next goal


class robot_pose:
    def __init__(self):
        self.pose = PoseWithCovarianceStamped()
    
    def localization_callback(self, data):
        self.pose = data
    
    def get_pose(self):
        return (self.pose.pose.pose.position.x, self.pose.pose.pose.position.y)

def wait_user_input():
  while(1):
    print("Press k to start:")
    key = input()
    if key == 'k':
      break

def read_csv():
    path = []
    with open(os.path.abspath(PATH_NAME)) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=' ')
        for line_count, row in enumerate(csv_reader):
            if line_count == 1:
                goal = (float(row[0]),float(row[1]))
            else:
                path.append((float(row[0]),float(row[1])))
    path.append(goal)
    return path

def goal_is_reached(goal, pose):
    
    if distance.euclidean(goal,pose) < START_TRESHOLD:
        return True
    else:
        return False

def get_goal_quaternion(curr_x, curr_y, next_x, next_y):
    diff_x = next_x - curr_x
    diff_y = next_y - curr_y
    goal_angle = atan2(diff_y, diff_x)

    return quaternion_from_euler(0, 0, goal_angle)


def main():
    rospy.init_node('main')    

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    pose_listener = robot_pose()
    rospy.Subscriber("amcl_pose",PoseWithCovarianceStamped,pose_listener.localization_callback)
    path = read_csv()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.orientation.w = 1.0

    index = 0

    # SETUP PHASE
    while not goal_is_reached((path[index][0],path[index][1]), pose_listener.get_pose()):   # make sure of that the turtlebot arrived at starting point
        pass
    print(f"Starting position reached")
    index = 1   # the first point after the starting point
    service = "/move_base/clear_costmaps"
    rospy.wait_for_service(service)
    clear_cost_map_service = rospy.ServiceProxy(service, Empty)
    clear_cost_map_service()    # service to clear the costmap
    wait_user_input()           # wait input for start the time trial
    start_time = rospy.get_time()

    # AUTONOMOUS NAVIGATION PHASE
    while not rospy.is_shutdown():
        
        if index == len(path):
            print("End position reached")
            break
        
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = path[index][0]   # x coordinate of index point
        goal.target_pose.pose.position.y = path[index][1]   # y coordinate of index point

        if GOAL_MODE == 2:
            if index <= len(path)-2:
                quaternion = get_goal_quaternion(path[index][0], path[index][1], path[index+1][0], path[index+1][1])
            else:
                quaternion = get_goal_quaternion(path[index-1][0], path[index-1][1], path[index][0], path[index][1])
        elif GOAL_MODE == 1:
            quaternion = get_goal_quaternion(path[index-1][0], path[index-1][1], path[index][0], path[index][1])
        
        if GOAL_MODE != 0:
            goal.target_pose.pose.orientation.x = quaternion[0]
            goal.target_pose.pose.orientation.y = quaternion[1]
            goal.target_pose.pose.orientation.z = quaternion[2]
            goal.target_pose.pose.orientation.w = quaternion[3]

        client.send_goal(goal)
                
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            if client.get_result():
                print(f"Goal {index} reached in time: {round((rospy.get_time()-start_time),3)}") 
                index+=1
            else:
                print("Goal not reached")
        

    
    
    


if __name__ == "__main__": 
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation session ended.")

