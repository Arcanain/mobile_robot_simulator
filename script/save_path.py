#! /usr/bin/env python3
import math
import pandas as pd

# import for ros function
import rospy
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Header
from std_msgs.msg import Header
from nav_msgs.msg import Odometry

class Save_Path():
    def __init__(self):
        rospy.init_node("save_path", anonymous=True)
        self.savepath_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.path_dict = {}
    
    def odom_callback(self, odom):
        print(odom.pose.pose.position.x)
        current_point = [odom.pose.pose.position.x,
                         odom.pose.pose.position.y,
                         odom.pose.pose.position.z,
                         odom.pose.pose.orientation.x,
                         odom.pose.pose.orientation.y,
                         odom.pose.pose.orientation.z,
                         odom.pose.pose.orientation.w,
                         ]
        self.path_dict[len(self.path_dict)] = current_point

    def save_csv(self):
        # Save CSV path file
        cols = ["x", "y", "z", "w0", "w1", "w2", "w3"]
        df = pd.DataFrame.from_dict(self.path_dict, orient='index',columns=cols)
        df.to_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_0731.csv", index=False)

if __name__ == '__main__':
    print('Save Path is Started...')
    save_path = Save_Path()

    while not rospy.is_shutdown():
        pass
    save_path.save_csv()
    print("Save Path finished!")