#! /usr/bin/env python3
import math
import pandas as pd

# import for ros function
import rospy
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Header
from nav_msgs.msg import Odometry

#############################
# Simple Odometry simulator #
#############################
class Odometry_Publisher():

    ##################
    # Initialization #
    ##################
    def __init__(self):

        rospy.init_node('odom_publisher', anonymous=True)
        self.r = rospy.Rate(50)  # 50hz

        #Initialize odometry header
        self.odom_header = Header()
        self.odom_header.seq = 0
        self.odom_header.stamp = rospy.Time.now()
        self.odom_header.frame_id = "map"

        # Initialize pose info
        self.sim_pose = Pose()
        self.sim_pose.position.x = 0.0
        self.sim_pose.position.y = 0.0
        self.sim_pose.position.z = 0.0
        self.sim_pose.orientation.x = 0.0
        self.sim_pose.orientation.y = 0.0
        self.sim_pose.orientation.z = 0.0

        # initialize twist info
        self.sim_twist = Twist()

        # Initialize odometry info
        self.sim_odom = Odometry()
        self.sim_odom.header = self.odom_header
        self.sim_odom.child_frame_id = "base_link"
        self.sim_odom.pose.pose = self.sim_pose
        self.sim_odom.twist.twist = self.sim_twist

        #initialize publisher
        self.emu_odom_pub = rospy.Publisher("/odom", Odometry, queue_size=50)

        #initialize TF
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.map_broadcaster  = tf.TransformBroadcaster()

        self.cmdvel_sub = rospy.Subscriber("/cmd_vel", Twist, self.cb_get_cmdvel_subscriber)
        self.cmdvel_linear_x  = 0.0
        self.cmdvel_linear_y  = 0.0
        self.cmdvel_angular_z = 0.0

        # time setting : https://lilaboc.work/archives/16899313.html
        self.current_time = rospy.get_time()
        self.last_time = rospy.get_time()

        # path saving dict
        self.path_dict = {}

    #############################################
    # Update odometry form User request cmd_vel #
    #############################################
    def update_odom(self):
        #Update Vehicle Pose
        self.current_time = rospy.get_time()
        sampletime = self.current_time - self.last_time
    
        e = tf.transformations.euler_from_quaternion((self.sim_pose.orientation.x, self.sim_pose.orientation.y, self.sim_pose.orientation.z, self.sim_pose.orientation.w))
        yaw_euler = e[2]

        self.sim_pose.position.x = self.sim_pose.position.x + self.cmdvel_linear_x*sampletime*math.cos(yaw_euler)
        self.sim_pose.position.y = self.sim_pose.position.y + self.cmdvel_linear_x*sampletime*math.sin(yaw_euler)
        updated_yaw = e[2] + self.cmdvel_angular_z*sampletime

        updated_quaternion = tf.transformations.quaternion_from_euler(0, 0, updated_yaw)
        self.sim_pose.orientation.x = updated_quaternion[0]
        self.sim_pose.orientation.y = updated_quaternion[1]
        self.sim_pose.orientation.z = updated_quaternion[2]
        self.sim_pose.orientation.w = updated_quaternion[3]

        #update timestamp
        self.odom_header.seq =self.odom_header.seq + 1
        self.odom_header.stamp = rospy.Time.now()
        self.sim_odom.header = self.odom_header
        self.sim_odom.pose.pose = self.sim_pose
        self.sim_odom.twist.twist = self.sim_twist
        self.emu_odom_pub.publish(self.sim_odom)

        #update TF
        odom_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        self.map_broadcaster.sendTransform(
            (0.0, 0.0, 0.0),
            odom_quat,
            rospy.Time.now(),
            "odom",
            "map"
        )

        self.odom_broadcaster.sendTransform(
            (self.sim_odom.pose.pose.position.x, self.sim_odom.pose.pose.position.y, self.sim_odom.pose.pose.position.z),
            updated_quaternion,
            rospy.Time.now(),
            "base_link",
            "odom"
        )

        self.last_time = self.current_time

        #save path plan
        current_path = [0, self.sim_pose.position.x, self.sim_pose.position.y, self.sim_pose.position.z,
                        updated_quaternion[0], updated_quaternion[1], updated_quaternion[2], updated_quaternion[3],
                        self.sim_twist.linear.x, self.sim_twist.linear.y, self.sim_twist.linear.z, 
                        0, 0, self.sim_twist.angular.z]
        self.path_dict[len(self.path_dict)] = current_path
        
    ############
    # save csv #
    ############
    def save_csv(self):
        # Save CSV path file
        cols = ["time", "x", "y", "z", "w0", "w1", "w2", "w3", "vx", "vy", "vz", "roll", "pitch", "yaw"]
        df = pd.DataFrame.from_dict(self.path_dict, orient='index',columns=cols)
        df.to_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data.csv", index=False)
        
    #####################
    # cmdvel subscriber #
    #####################
    def cb_get_cmdvel_subscriber(self, msg):
        self.cmdvel_linear_x  = msg.linear.x
        self.cmdvel_linear_y  = msg.linear.x
        self.cmdvel_angular_z = msg.angular.z
        pass

if __name__ == '__main__':
    print('Simple Odometry Simulator is Started...')
    odometry = Odometry_Publisher()
    r = odometry.r # sampling[Hz]
    while not rospy.is_shutdown():
        odometry.update_odom()
        r.sleep()    
    odometry.save_csv()
    print("finish")
    pass