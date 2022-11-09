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
        """
        self.csv_data1 = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_1005.csv")
        self.csv_data2 = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_1004.csv")
        self.csv_data3 = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_1005.csv")
        self.csv_data4 = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_1004.csv")
        self.csv_data5 = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_1005.csv")
        self.csv_data6 = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_1004.csv")
        self.csv_data7 = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_1005.csv")
        self.csv_data8 = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_1004.csv")
        """

        #self.csv_data1 = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_1106_4.csv")
        #self.csv_data1 = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_1106_5.csv")
        #self.csv_data1 = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/divide_path1_1106.csv")
        self.csv_data1 = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/divide_path2_1106.csv")
        self.csv_data2 = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_1106_2.csv")
        self.csv_data3 = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_1106_1.csv")
        self.csv_data4 = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_1106_2.csv")
        self.csv_data5 = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_1106_1.csv")
        self.csv_data6 = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_1106_2.csv")
        self.csv_data7 = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_1106_1.csv")
        self.csv_data8 = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_1106_2.csv")

        #self.csv_data1 = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/test_cose1.csv")
        #self.csv_data2 = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/test_cose2.csv")

        #self.csv_data = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_estimated_pose.csv")
        #self.csv_data = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_estimater_kakuninsoukou.csv")
        #self.csv_data = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_estimater_lawn_stack_2_tsukuba_hime.csv")

        # initialize publisher
        self.path1_pub = rospy.Publisher("/path1", Path, queue_size = 10)
        self.path2_pub = rospy.Publisher("/path2", Path, queue_size = 10)
        self.path3_pub = rospy.Publisher("/path3", Path, queue_size = 10)
        self.path4_pub = rospy.Publisher("/path4", Path, queue_size = 10)
        self.path5_pub = rospy.Publisher("/path5", Path, queue_size = 10)
        self.path6_pub = rospy.Publisher("/path6", Path, queue_size = 10)
        self.path7_pub = rospy.Publisher("/path7", Path, queue_size = 10)
        self.path8_pub = rospy.Publisher("/path8", Path, queue_size = 10)

        self.path1_num_pub = rospy.Publisher("/path1_num", Int32, queue_size = 10)
        self.path2_num_pub = rospy.Publisher("/path2_num", Int32, queue_size = 10)
        self.path3_num_pub = rospy.Publisher("/path3_num", Int32, queue_size = 10)
        self.path4_num_pub = rospy.Publisher("/path4_num", Int32, queue_size = 10)
        self.path5_num_pub = rospy.Publisher("/path5_num", Int32, queue_size = 10)
        self.path6_num_pub = rospy.Publisher("/path6_num", Int32, queue_size = 10)
        self.path7_num_pub = rospy.Publisher("/path7_num", Int32, queue_size = 10)
        self.path8_num_pub = rospy.Publisher("/path8_num", Int32, queue_size = 10)

    def crate_path(self):
        pose_list1 = self.get_poses_from_csvdata(self.csv_data1)
        pose_list2 = self.get_poses_from_csvdata(self.csv_data2)
        pose_list3 = self.get_poses_from_csvdata(self.csv_data3)
        pose_list4 = self.get_poses_from_csvdata(self.csv_data4)
        pose_list5 = self.get_poses_from_csvdata(self.csv_data5)
        pose_list6 = self.get_poses_from_csvdata(self.csv_data6)
        pose_list7 = self.get_poses_from_csvdata(self.csv_data7)
        pose_list8 = self.get_poses_from_csvdata(self.csv_data8)

        # creat path data
        # path1
        self.path1 = Path()
        self.path1.header.stamp = rospy.Time.now()
        self.path1.header.frame_id = "map"
        self.path1.poses = pose_list1

        # path2
        self.path2 = Path()
        self.path2.header.stamp = rospy.Time.now()
        self.path2.header.frame_id = "map"
        self.path2.poses = pose_list2

        # path3
        self.path3 = Path()
        self.path3.header.stamp = rospy.Time.now()
        self.path3.header.frame_id = "map"
        self.path3.poses = pose_list3

        # path4
        self.path4 = Path()
        self.path4.header.stamp = rospy.Time.now()
        self.path4.header.frame_id = "map"
        self.path4.poses = pose_list4

        # path5
        self.path5 = Path()
        self.path5.header.stamp = rospy.Time.now()
        self.path5.header.frame_id = "map"
        self.path5.poses = pose_list5

        # path6
        self.path6 = Path()
        self.path6.header.stamp = rospy.Time.now()
        self.path6.header.frame_id = "map"
        self.path6.poses = pose_list6

        # path7
        self.path7 = Path()
        self.path7.header.stamp = rospy.Time.now()
        self.path7.header.frame_id = "map"
        self.path7.poses = pose_list7

        # path8
        self.path8 = Path()
        self.path8.header.stamp = rospy.Time.now()
        self.path8.header.frame_id = "map"
        self.path8.poses = pose_list8

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
        self.path1_pub.publish(self.path1)
        self.path1_num_pub.publish(len(self.csv_data1))

        self.path2_pub.publish(self.path2)
        self.path2_num_pub.publish(len(self.csv_data2))

        self.path3_pub.publish(self.path3)
        self.path3_num_pub.publish(len(self.csv_data3))

        self.path4_pub.publish(self.path4)
        self.path4_num_pub.publish(len(self.csv_data4))

        self.path5_pub.publish(self.path5)
        self.path5_num_pub.publish(len(self.csv_data5))

        self.path6_pub.publish(self.path6)
        self.path6_num_pub.publish(len(self.csv_data6))

        self.path7_pub.publish(self.path7)
        self.path7_num_pub.publish(len(self.csv_data7))

        self.path8_pub.publish(self.path8)
        self.path8_num_pub.publish(len(self.csv_data8))

if __name__ == '__main__':
    print('Path Publisher is Started...')
    path_publisher = Path_Publisher()
    
    r = rospy.Rate(50) # 50hz
    while not rospy.is_shutdown():
        path_publisher.crate_path()
        path_publisher.publish_path()
        r.sleep()
    print("Path Publisher finished!")