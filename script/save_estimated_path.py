#! /usr/bin/env python3
import math
import pandas as pd

# import for ros function
import rospy
from nav_msgs.msg import Odometry

class Save_Path():
    def __init__(self):
        rospy.init_node("save_estimated_path", anonymous=True)
        self.savepath_sub = rospy.Subscriber("/estimated_pose", Odometry, self.odom_callback)

        # pre odometry
        self.pre_x = 0.0 #[m]
        self.pre_y = 0.0 #[m]
        self.dist_thread = 0.1 #[m]

        # path dict for csvfile
        self.path_dict = {}
        
        # path index count
        self.path_index_count = 0

    def odom_callback(self, odom):
        dist = math.sqrt( (odom.pose.pose.position.x - self.pre_x)**2 + (odom.pose.pose.position.y - self.pre_y)**2 )
        if dist >= self.dist_thread:
            current_point = [odom.pose.pose.position.x,
                             odom.pose.pose.position.y,
                             odom.pose.pose.position.z,
                             odom.pose.pose.orientation.x,
                             odom.pose.pose.orientation.y,
                             odom.pose.pose.orientation.z,
                             1.0,
                            ]
            self.path_dict[len(self.path_dict)] = current_point

            # save pre_position
            self.pre_x = odom.pose.pose.position.x
            self.pre_y = odom.pose.pose.position.y

            # add path index
            self.path_index_count = self.path_index_count + 1
            print("PathIndex:", self.path_index_count)
        else:
            pass
    """
    def odom_callback(self, odom):
        current_point = [odom.pose.pose.position.x,
                         odom.pose.pose.position.y,
                         odom.pose.pose.position.z,
                         odom.pose.pose.orientation.x,
                         odom.pose.pose.orientation.y,
                         odom.pose.pose.orientation.z,
                         odom.pose.pose.orientation.w,
                         ]
        self.path_dict[len(self.path_dict)] = current_point
    """
    def save_csv(self):
        # Save CSV path file
        cols = ["x", "y", "z", "w0", "w1", "w2", "w3"]
        df = pd.DataFrame.from_dict(self.path_dict, orient='index',columns=cols)
        df.to_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_1106_8.csv", index=False)

if __name__ == '__main__':
    print('Save Path is Started...')
    save_path = Save_Path()

    while not rospy.is_shutdown():
        pass
    save_path.save_csv()
    print("Save Path finished!")