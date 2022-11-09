#! /usr/bin/env python3
import pandas as pd

# import for ros function
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import Path
from std_msgs.msg import Int8
from std_msgs.msg import Int32
from std_msgs.msg import Bool

class Path_Publisher():
    def __init__(self):
        rospy.init_node("path_publisher_1110", anonymous=True)

        # get pose data from csv file
        self.csv_data1 = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_1005.csv")
        self.csv_data2 = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_1004.csv")
        self.csv_data3 = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_1005.csv")
        self.csv_data4 = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_1004.csv")
        self.csv_data5 = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_1005.csv")
        self.csv_data6 = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_1004.csv")
        self.csv_data7 = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_1005.csv")
        self.csv_data8 = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_1004.csv")

        # init csv data
        self.csv_data = self.csv_data1
        self.csv_file_number = 1

        # initialize publisher
        self.path_pub = rospy.Publisher("/path", Path, queue_size = 10)
        self.path_num_pub = rospy.Publisher("/path_num", Int32, queue_size = 10)

        self.write_start_sub = rospy.Subscriber("/write_start_flg", Bool, self.write_start_callback)
        
        self.write_finish_pub = rospy.Publisher("/write_finish_flg", Bool, queue_size = 10)

        self.csv_path_number_sub = rospy.Subscriber("/csv_path_number", Int8, self.csv_path_number_callback)

    def csv_path_number_callback(self, msg):
        self.csv_file_number = msg.data
        #print(self.csv_file_number)

    def write_start_callback(self, msg):
        write_start_flag = msg.data
        print(write_start_flag)
        print(self.csv_file_number)

        self.write_finish_flag = Bool()
        self.write_finish_flag.data = False

        """
        if write_start_flag:
            if self.csv_file_number == 1:
                self.csv_data = self.csv_data1
            elif self.csv_file_number == 2:
                self.csv_data = self.csv_data2
            elif self.csv_file_number == 3:
                self.csv_data = self.csv_data1
            elif self.csv_file_number == 4:
                self.csv_data = self.csv_data2
            elif self.csv_file_number == 5:
                self.csv_data = self.csv_data1
            elif self.csv_file_number == 6:
                self.csv_data = self.csv_data2
            elif self.csv_file_number == 7:
                self.csv_data = self.csv_data1
            elif self.csv_file_number == 8:
                self.csv_data = self.csv_data2
        """
        if write_start_flag:
            if self.csv_file_number == 1:
                self.csv_data = self.csv_data1
            elif self.csv_file_number == 2:
                self.csv_data = self.csv_data2
            self.write_finish_flag.data = True
        
        self.write_finish_pub.publish(self.write_finish_flag)

    def crate_path(self):
        pose_list = self.get_poses_from_csvdata(self.csv_data)

        # creat path data
        self.path = Path()
        self.path.header.stamp = rospy.Time.now()
        self.path.header.frame_id = "map"
        self.path.poses = pose_list

    def get_poses_from_csvdata(self, csv_data):
        poses_list = []
        for index in range(len(csv_data)):
            temp_pose = PoseStamped()
            temp_pose.header.seq = index
            temp_pose.header.stamp = rospy.Time.now()
            temp_pose.header.frame_id = "map"
            temp_pose.pose.position.x = csv_data["x"][index]
            temp_pose.pose.position.y = csv_data["y"][index]
            temp_pose.pose.position.z = csv_data["z"][index]
            temp_pose.pose.orientation.x = csv_data["w0"][index]
            temp_pose.pose.orientation.y = csv_data["w1"][index]
            temp_pose.pose.orientation.z = csv_data["w2"][index]
            temp_pose.pose.orientation.w = csv_data["w3"][index]

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