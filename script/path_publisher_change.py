#! /usr/bin/env python3
import pandas as pd

# import for ros function
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import Path
from std_msgs.msg import Int8
from std_msgs.msg import Int32

class Path_Publisher():
    def __init__(self):
        rospy.init_node("path_publisher_change", anonymous=True)

        # get pose data from csv file
        #self.csv_data = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_1004.csv")
        self.csv_data = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_1005.csv")
        #self.csv_data = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_estimated_pose.csv")
        #self.csv_data = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_estimater_kakuninsoukou.csv")
        #self.csv_data = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_estimater_lawn_stack_2_tsukuba_hime.csv")

        # initialize publisher
        self.path_pub = rospy.Publisher("/path", Path, queue_size = 50)
        self.path_num_pub = rospy.Publisher("/path_num", Int32, queue_size = 10)
        self.select_path_sub = rospy.Subscriber("/select_path_num", Int8, self.select_csv_file)

    def select_csv_file(self, msg):
        select_num = msg.data
        print(select_num)
        if select_num == 1:
            self.csv_data = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_1005.csv")
        elif select_num == 2:
            self.csv_data = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_1004.csv")
    
    def crate_path(self):
        pose_list = self.get_poses_from_csvdata()
        # creat path data
        self.path = Path()
        self.path.header.stamp = rospy.Time.now()
        self.path.header.frame_id = "map"
        self.path.poses = pose_list

    def get_poses_from_csvdata(self):
        poses_list = []
        for index in range(len(self.csv_data)):
            temp_pose = PoseStamped()
            temp_pose.header.seq = index
            temp_pose.header.stamp = rospy.Time.now()
            temp_pose.header.frame_id = "map"
            temp_pose.pose.position.x = self.csv_data["x"][index]
            temp_pose.pose.position.y = self.csv_data["y"][index]
            temp_pose.pose.position.z = self.csv_data["z"][index]
            temp_pose.pose.orientation.x = self.csv_data["w0"][index]
            temp_pose.pose.orientation.y = self.csv_data["w1"][index]
            temp_pose.pose.orientation.z = self.csv_data["w2"][index]
            temp_pose.pose.orientation.w = self.csv_data["w3"][index]

            poses_list.append(temp_pose)

        return poses_list

    def publish_path(self):
        self.path_pub.publish(self.path)
        self.path_num_pub.publish(len(self.csv_data))

if __name__ == '__main__':
    print('Path Publisher is Started...')
    path_publisher = Path_Publisher()
    
    r = rospy.Rate(50) # 50hz
    while not rospy.is_shutdown():
        path_publisher.crate_path()
        path_publisher.publish_path()
        r.sleep()
    print("Path Publisher finished!")