/***********************
 * 2022 11/05
************************/
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <math.h>
#include <vector>
#include <algorithm>

#include <Eigen/Core>
#include <Eigen/Geometry>

class Pure_Pursuit
{
    private:
        ros::NodeHandle nh;
        ros::Publisher cmd_vel_pub;
        ros::Publisher targetwp_num_pub;
        ros::Publisher select_path_num_pub;
        ros::Publisher trajectory_path_pub;
        ros::Subscriber odom_sub;
        ros::Subscriber joy_sub;
        ros::Subscriber imu_sub;
        ros::Subscriber related_yaw_sub;

        ros::Subscriber path1_sub;
        ros::Subscriber path1_num_sub;
        ros::Subscriber path2_sub;
        ros::Subscriber path2_num_sub;
        ros::Subscriber path3_sub;
        ros::Subscriber path3_num_sub;
        ros::Subscriber path4_sub;
        ros::Subscriber path4_num_sub;
        ros::Subscriber path5_sub;
        ros::Subscriber path5_num_sub;
        ros::Subscriber path6_sub;
        ros::Subscriber path6_num_sub;
        ros::Subscriber path7_sub;
        ros::Subscriber path7_num_sub;
        ros::Subscriber path8_sub;
        ros::Subscriber path8_num_sub;

        float target_LookahedDist; // Lookahed distance for Pure Pursuit[m]

        bool path1_first_flg     = false;
        bool path1_num_first_flg = false;
        bool path2_first_flg     = false;
        bool path2_num_first_flg = false;
        bool path3_first_flg     = false;
        bool path3_num_first_flg = false;
        bool path4_first_flg     = false;
        bool path4_num_first_flg = false;
        bool path5_first_flg     = false;
        bool path5_num_first_flg = false;
        bool path6_first_flg     = false;
        bool path6_num_first_flg = false;
        bool path7_first_flg     = false;
        bool path7_num_first_flg = false;
        bool path8_first_flg     = false;
        bool path8_num_first_flg = false;

        bool path1_goal_flg = false;
        bool path2_goal_flg = false;
        bool path3_goal_flg = false;
        bool path4_goal_flg = false;
        bool path5_goal_flg = false;
        bool path6_goal_flg = false;
        bool path7_goal_flg = false;
        bool path8_goal_flg = false;

        bool path1_reset_index_flg = false;
        bool path2_reset_index_flg = false;
        bool path3_reset_index_flg = false;
        bool path4_reset_index_flg = false;
        bool path5_reset_index_flg = false;
        bool path6_reset_index_flg = false;
        bool path7_reset_index_flg = false;
        bool path8_reset_index_flg = false;

        bool odom_first_flg = false;
        bool imu_first_flg = false;
        bool position_search_flg = false;

        bool check_stop_joy_flg1 = false;
        bool check_stop_joy_flg2 = false;
        bool check_stop_joy_flg3 = false;
        bool check_stop_joy_flg4 = false;
        bool check_stop_flg_reset = false;

        bool select_path_change = false;

        int path_num = 0;
        int path1_num = 0;
        int path2_num = 0;
        int path3_num = 0;
        int path4_num = 0;
        int path5_num = 0;
        int path6_num = 0;
        int path7_num = 0;
        int path8_num = 0;

        int last_index = 0;
        int last_index_dummy = 0;
        int pre_last_index = 0;

        int chnage_path_count = 0;

        nav_msgs::Path trajectory_path;

        // for path_callback
        std::vector<float> path_x;
        std::vector<float> path_y;
        std::vector<float> path_st;

        std::vector<float> path1_x;
        std::vector<float> path1_y;
        std::vector<float> path1_st;

        std::vector<float> path2_x;
        std::vector<float> path2_y;
        std::vector<float> path2_st;

        std::vector<float> path3_x;
        std::vector<float> path3_y;
        std::vector<float> path3_st;

        std::vector<float> path4_x;
        std::vector<float> path4_y;
        std::vector<float> path4_st;

        std::vector<float> path5_x;
        std::vector<float> path5_y;
        std::vector<float> path5_st;

        std::vector<float> path6_x;
        std::vector<float> path6_y;
        std::vector<float> path6_st;

        std::vector<float> path7_x;
        std::vector<float> path7_y;
        std::vector<float> path7_st;

        std::vector<float> path8_x;
        std::vector<float> path8_y;
        std::vector<float> path8_st;

        // for odom_callback
        float current_x;
        float current_y;
        float current_yaw_euler;
        float current_vel;

        // for publish targetwp_num
        std_msgs::Int32 targetwp_num;

        // for publish select_path_num
        std_msgs::Int8 select_path_num;

        // for publish cmd_vel
        geometry_msgs::Twist cmd_vel;
        float goal_th = 0.5; //[m]
        float yaw_rate = 0.0; //[rad/s]

        // cauvature parameter
        float minCurvature = 0.0;
        float maxCurvature = 0.5;
        float minVelocity = 0.1;
        float maxVelocity = 0.3;

    public:
        Pure_Pursuit();
        ~Pure_Pursuit();
        // path callback
        void path1_callback(const nav_msgs::Path &path_msg);
        void path1_num_callback(const std_msgs::Int32 &path_num_msg);
        void path2_callback(const nav_msgs::Path &path_msg);
        void path2_num_callback(const std_msgs::Int32 &path_num_msg);
        void path3_callback(const nav_msgs::Path &path_msg);
        void path3_num_callback(const std_msgs::Int32 &path_num_msg);
        void path4_callback(const nav_msgs::Path &path_msg);
        void path4_num_callback(const std_msgs::Int32 &path_num_msg);
        void path5_callback(const nav_msgs::Path &path_msg);
        void path5_num_callback(const std_msgs::Int32 &path_num_msg);
        void path6_callback(const nav_msgs::Path &path_msg);
        void path6_num_callback(const std_msgs::Int32 &path_num_msg);
        void path7_callback(const nav_msgs::Path &path_msg);
        void path7_num_callback(const std_msgs::Int32 &path_num_msg);
        void path8_callback(const nav_msgs::Path &path_msg);
        void path8_num_callback(const std_msgs::Int32 &path_num_msg);

        // odom callback
        void odom_callback(const nav_msgs::Odometry &odom_msg);
        // joy
        void joy_callback(const sensor_msgs::Joy& joy);
        // imu callback
        void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg);
        // related yaw
        void related_yaw_callback(const std_msgs::Float64 &related_yaw_msg);

        // publish cmd_vel
        void update_cmd_vel();

        void crate_goalLine_point(float a, float b, float c, float d, 
                                  float *right_goal_x, float *right_goal_y, 
                                  float *left_goal_x, float *left_goal_y);

        bool checkLineAndCircleIntersection(float start_x, float start_y, 
                                            float end_x, float end_y, 
                                            float p_x, float p_y);
        
        bool calcurate_command_value(std::vector<float> path_x, std::vector<float> path_y, std::vector<float> path_st, int path_num, int path_type);
        void change_trajectory_path();
};

Pure_Pursuit::Pure_Pursuit()
{
    //cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_automatic", 50);
    targetwp_num_pub = nh.advertise<std_msgs::Int32>("/targetwp_num", 10);
    select_path_num_pub = nh.advertise<std_msgs::Int8>("/select_path_num", 10);
    select_path_num.data = 1;
    trajectory_path_pub = nh.advertise<nav_msgs::Path>("/path", 10);
    // init path
    trajectory_path.header.frame_id = "map";
    trajectory_path.header.stamp = ros::Time::now();
    trajectory_path.header.seq = 0;

    path1_sub = nh.subscribe("/path1", 10, &Pure_Pursuit::path1_callback, this);
    path1_num_sub = nh.subscribe("/path1_num", 10, &Pure_Pursuit::path1_num_callback, this);

    path2_sub = nh.subscribe("/path2", 10, &Pure_Pursuit::path2_callback, this);
    path2_num_sub = nh.subscribe("/path2_num", 10, &Pure_Pursuit::path2_num_callback, this);

    path3_sub = nh.subscribe("/path3", 10, &Pure_Pursuit::path3_callback, this);
    path3_num_sub = nh.subscribe("/path3_num", 10, &Pure_Pursuit::path3_num_callback, this);

    path4_sub = nh.subscribe("/path4", 10, &Pure_Pursuit::path4_callback, this);
    path4_num_sub = nh.subscribe("/path4_num", 10, &Pure_Pursuit::path4_num_callback, this);

    path5_sub = nh.subscribe("/path5", 10, &Pure_Pursuit::path5_callback, this);
    path5_num_sub = nh.subscribe("/path5_num", 10, &Pure_Pursuit::path5_num_callback, this);

    path6_sub = nh.subscribe("/path6", 10, &Pure_Pursuit::path6_callback, this);
    path6_num_sub = nh.subscribe("/path6_num", 10, &Pure_Pursuit::path6_num_callback, this);

    path7_sub = nh.subscribe("/path7", 10, &Pure_Pursuit::path7_callback, this);
    path7_num_sub = nh.subscribe("/path7_num", 10, &Pure_Pursuit::path7_num_callback, this);

    path8_sub = nh.subscribe("/path8", 10, &Pure_Pursuit::path8_callback, this);
    path8_num_sub = nh.subscribe("/path8_num", 10, &Pure_Pursuit::path8_num_callback, this);

    odom_sub = nh.subscribe("/estimated_pose", 10, &Pure_Pursuit::odom_callback, this);
    joy_sub = nh.subscribe("/joy", 10, &Pure_Pursuit::joy_callback, this);
    imu_sub = nh.subscribe("/imu/data", 10, &Pure_Pursuit::imu_callback, this);
    related_yaw_sub = nh.subscribe("/wit/related_yaw", 10, &Pure_Pursuit::related_yaw_callback, this);
}

Pure_Pursuit::~Pure_Pursuit()
{
    std::cout << "Finish Pure Pursuit!" << std::endl;
}

void Pure_Pursuit::path1_callback(const nav_msgs::Path &path_msg)
{
    if (!path1_first_flg && path1_num != 0) {
        std::cout << "first path call" << std::endl;
        
        float last_x = 0.0;
        float last_y = 0.0;
        float st = 0.0;
        float last_st = 0.0;
        for (int index = 0; index < path1_num; index++) {
            path1_x.emplace_back(path_msg.poses[index].pose.position.x);
            path1_y.emplace_back(path_msg.poses[index].pose.position.y);
            
            st = std::sqrt(std::pow((path1_x[index] - last_x), 2.0) + std::pow((path1_y[index] - last_y), 2.0));
            path1_st.emplace_back(last_st + st);
            
            last_x = path1_x[index];
            last_y = path1_y[index];
            last_st = path1_st[index];
        }

        path1_first_flg = true;
    }
}

void Pure_Pursuit::path2_callback(const nav_msgs::Path &path_msg)
{
    if (!path2_first_flg && path2_num != 0) {
        std::cout << "second path call" << std::endl;
        
        float last_x = 0.0;
        float last_y = 0.0;
        float st = 0.0;
        float last_st = 0.0;
        for (int index = 0; index < path2_num; index++) {
            path2_x.emplace_back(path_msg.poses[index].pose.position.x);
            path2_y.emplace_back(path_msg.poses[index].pose.position.y);
            
            st = std::sqrt(std::pow((path2_x[index] - last_x), 2.0) + std::pow((path2_y[index] - last_y), 2.0));
            path2_st.emplace_back(last_st + st);
            
            last_x = path2_x[index];
            last_y = path2_y[index];
            last_st = path2_st[index];
        }

        path2_first_flg = true;
    }
}

void Pure_Pursuit::path3_callback(const nav_msgs::Path &path_msg)
{
    if (!path3_first_flg && path3_num != 0) {
        std::cout << "third path call" << std::endl;
        
        float last_x = 0.0;
        float last_y = 0.0;
        float st = 0.0;
        float last_st = 0.0;
        for (int index = 0; index < path3_num; index++) {
            path3_x.emplace_back(path_msg.poses[index].pose.position.x);
            path3_y.emplace_back(path_msg.poses[index].pose.position.y);
            
            st = std::sqrt(std::pow((path3_x[index] - last_x), 2.0) + std::pow((path3_y[index] - last_y), 2.0));
            path3_st.emplace_back(last_st + st);
            
            last_x = path3_x[index];
            last_y = path3_y[index];
            last_st = path3_st[index];
        }

        path3_first_flg = true;
    }
}

void Pure_Pursuit::path4_callback(const nav_msgs::Path &path_msg)
{
    if (!path4_first_flg && path4_num != 0) {
        std::cout << "fourth path call" << std::endl;
        
        float last_x = 0.0;
        float last_y = 0.0;
        float st = 0.0;
        float last_st = 0.0;
        for (int index = 0; index < path4_num; index++) {
            path4_x.emplace_back(path_msg.poses[index].pose.position.x);
            path4_y.emplace_back(path_msg.poses[index].pose.position.y);
            
            st = std::sqrt(std::pow((path4_x[index] - last_x), 2.0) + std::pow((path4_y[index] - last_y), 2.0));
            path4_st.emplace_back(last_st + st);
            
            last_x = path4_x[index];
            last_y = path4_y[index];
            last_st = path4_st[index];
        }

        path4_first_flg = true;
    }
}

void Pure_Pursuit::path5_callback(const nav_msgs::Path &path_msg)
{
    if (!path5_first_flg && path5_num != 0) {
        std::cout << "fifth path call" << std::endl;
        
        float last_x = 0.0;
        float last_y = 0.0;
        float st = 0.0;
        float last_st = 0.0;
        for (int index = 0; index < path5_num; index++) {
            path5_x.emplace_back(path_msg.poses[index].pose.position.x);
            path5_y.emplace_back(path_msg.poses[index].pose.position.y);
            
            st = std::sqrt(std::pow((path5_x[index] - last_x), 2.0) + std::pow((path5_y[index] - last_y), 2.0));
            path5_st.emplace_back(last_st + st);
            
            last_x = path5_x[index];
            last_y = path5_y[index];
            last_st = path5_st[index];
        }

        path5_first_flg = true;
    }
}

void Pure_Pursuit::path6_callback(const nav_msgs::Path &path_msg)
{
    if (!path6_first_flg && path6_num != 0) {
        std::cout << "sixth path call" << std::endl;
        
        float last_x = 0.0;
        float last_y = 0.0;
        float st = 0.0;
        float last_st = 0.0;
        for (int index = 0; index < path6_num; index++) {
            path6_x.emplace_back(path_msg.poses[index].pose.position.x);
            path6_y.emplace_back(path_msg.poses[index].pose.position.y);
            
            st = std::sqrt(std::pow((path6_x[index] - last_x), 2.0) + std::pow((path6_y[index] - last_y), 2.0));
            path6_st.emplace_back(last_st + st);
            
            last_x = path6_x[index];
            last_y = path6_y[index];
            last_st = path6_st[index];
        }

        path6_first_flg = true;
    }
}

void Pure_Pursuit::path7_callback(const nav_msgs::Path &path_msg)
{
    if (!path7_first_flg && path7_num != 0) {
        std::cout << "seventh path call" << std::endl;
        
        float last_x = 0.0;
        float last_y = 0.0;
        float st = 0.0;
        float last_st = 0.0;
        for (int index = 0; index < path7_num; index++) {
            path7_x.emplace_back(path_msg.poses[index].pose.position.x);
            path7_y.emplace_back(path_msg.poses[index].pose.position.y);
            
            st = std::sqrt(std::pow((path7_x[index] - last_x), 2.0) + std::pow((path7_y[index] - last_y), 2.0));
            path7_st.emplace_back(last_st + st);
            
            last_x = path7_x[index];
            last_y = path7_y[index];
            last_st = path7_st[index];
        }

        path7_first_flg = true;
    }
}

void Pure_Pursuit::path8_callback(const nav_msgs::Path &path_msg)
{
    if (!path8_first_flg && path8_num != 0) {
        std::cout << "eighth path call" << std::endl;
        
        float last_x = 0.0;
        float last_y = 0.0;
        float st = 0.0;
        float last_st = 0.0;
        for (int index = 0; index < path8_num; index++) {
            path8_x.emplace_back(path_msg.poses[index].pose.position.x);
            path8_y.emplace_back(path_msg.poses[index].pose.position.y);
            
            st = std::sqrt(std::pow((path8_x[index] - last_x), 2.0) + std::pow((path8_y[index] - last_y), 2.0));
            path8_st.emplace_back(last_st + st);
            
            last_x = path8_x[index];
            last_y = path8_y[index];
            last_st = path8_st[index];
        }

        path8_first_flg = true;
    }
}

void Pure_Pursuit::path1_num_callback(const std_msgs::Int32 &path_num_msg)
{
    if (!path1_num_first_flg) {
        path1_num = path_num_msg.data;
        path1_num_first_flg = true;
    }
}

void Pure_Pursuit::path2_num_callback(const std_msgs::Int32 &path_num_msg)
{
    if (!path2_num_first_flg) {
        path2_num = path_num_msg.data;
        path2_num_first_flg = true;
    }
}

void Pure_Pursuit::path3_num_callback(const std_msgs::Int32 &path_num_msg)
{
    if (!path3_num_first_flg) {
        path3_num = path_num_msg.data;
        path3_num_first_flg = true;
    }
}

void Pure_Pursuit::path4_num_callback(const std_msgs::Int32 &path_num_msg)
{
    if (!path4_num_first_flg) {
        path4_num = path_num_msg.data;
        path4_num_first_flg = true;
    }
}

void Pure_Pursuit::path5_num_callback(const std_msgs::Int32 &path_num_msg)
{
    if (!path5_num_first_flg) {
        path5_num = path_num_msg.data;
        path5_num_first_flg = true;
    }
}

void Pure_Pursuit::path6_num_callback(const std_msgs::Int32 &path_num_msg)
{
    if (!path6_num_first_flg) {
        path6_num = path_num_msg.data;
        path6_num_first_flg = true;
    }
}

void Pure_Pursuit::path7_num_callback(const std_msgs::Int32 &path_num_msg)
{
    if (!path7_num_first_flg) {
        path7_num = path_num_msg.data;
        path7_num_first_flg = true;
    }
}

void Pure_Pursuit::path8_num_callback(const std_msgs::Int32 &path_num_msg)
{
    if (!path8_num_first_flg) {
        path8_num = path_num_msg.data;
        path8_num_first_flg = true;
    }
}

void Pure_Pursuit::odom_callback(const nav_msgs::Odometry &odom_msg)
{
    current_x = odom_msg.pose.pose.position.x;
    current_y = odom_msg.pose.pose.position.y;

    /*
    double roll, pitch, yaw;
    tf::Quaternion odom_quat(
        odom_msg.pose.pose.orientation.x,
        odom_msg.pose.pose.orientation.y,
        odom_msg.pose.pose.orientation.z,
        odom_msg.pose.pose.orientation.w
    ); 
    tf::Matrix3x3 m(odom_quat);
    m.getRPY(roll, pitch, yaw);

    current_yaw_euler = (float)yaw;
    //std::cout << current_yaw_euler << std::endl;
    */

    odom_first_flg = true;
}

void Pure_Pursuit::joy_callback(const sensor_msgs::Joy& msg)
{
    if (msg.axes.size() != 8) {
        return;
    }

    // batu
    if (msg.buttons[0] == 1){
        check_stop_joy_flg1 = true;
    }
    // maru
    if (msg.buttons[1] == 1){
        check_stop_joy_flg2 = true;
    }
    // sannkaku
    if (msg.buttons[2] == 1){
        check_stop_joy_flg3 = true;
    }
    // sikaku
    if (msg.buttons[3] == 1){
        check_stop_joy_flg4 = true;
    }
}

void Pure_Pursuit::imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double roll, pitch, yaw;
    tf::Quaternion odom_quat(
        imu_msg->orientation.x,
        imu_msg->orientation.y,
        imu_msg->orientation.z,
        imu_msg->orientation.w
    ); 
    tf::Matrix3x3 m(odom_quat);
    m.getRPY(roll, pitch, yaw);

    //current_yaw_euler = (float)yaw + 0.28f + M_PI;
    current_yaw_euler = (float)yaw + M_PI;
    if (current_yaw_euler > M_PI) {
        current_yaw_euler = current_yaw_euler - 2.0 * M_PI;
    } else if (current_yaw_euler < -M_PI) {
        current_yaw_euler = current_yaw_euler + 2.0 * M_PI;
    }
    //std::cout << current_yaw_euler << std::endl;
    
    imu_first_flg = true;
}

void Pure_Pursuit::related_yaw_callback(const std_msgs::Float64 &related_yaw_msg)
{
    //std::cout << related_yaw_msg.data << std::endl;
    //current_yaw_euler = related_yaw_msg.data;
    //imu_first_flg = true;
}

void Pure_Pursuit::update_cmd_vel()
{
    /***********************************************************************
     * change trajectory path
    **********************************************************************/
    change_trajectory_path();

    /***********************************************************************
     * publish trajectory path
    **********************************************************************/
    for (int index = 0; index < path_num; index++) {
        geometry_msgs::PoseStamped point;
        point.header.frame_id = "map";
        point.header.stamp = ros::Time::now();
        point.pose.position.x = path_x[index];
        point.pose.position.y = path_y[index];
        point.pose.position.z = 0.0;
        point.pose.orientation.x = 0;
        point.pose.orientation.y = 0;
        point.pose.orientation.z = 0;
        point.pose.orientation.w = 1.0;
        trajectory_path.poses.push_back(point);
    }
    trajectory_path_pub.publish(trajectory_path);

    /***********************************************************************
     * trajectory path following
     **********************************************************************/
    if (path7_goal_flg == true && path8_first_flg == true && path8_num != 0) {
        path8_goal_flg = calcurate_command_value(path_x, path_y, path_st, path_num, 8);
    } else if (path6_goal_flg == true && path7_first_flg == true && path7_num != 0) {
        path7_goal_flg = calcurate_command_value(path_x, path_y, path_st, path_num, 7);
    } else if (path5_goal_flg == true && path6_first_flg == true && path6_num != 0) {
        path6_goal_flg = calcurate_command_value(path_x, path_y, path_st, path_num, 6);
    } else if (path4_goal_flg == true && path5_first_flg == true && path5_num != 0) {
        path5_goal_flg = calcurate_command_value(path_x, path_y, path_st, path_num, 5);
    } else if (path3_goal_flg == true && path4_first_flg == true && path4_num != 0) {
        path4_goal_flg = calcurate_command_value(path_x, path_y, path_st, path_num, 4);
    } else if (path2_goal_flg == true && path3_first_flg == true && path3_num != 0) {
        path3_goal_flg = calcurate_command_value(path_x, path_y, path_st, path_num, 3);
    } else if (path1_goal_flg == true && path2_first_flg == true && path2_num != 0) {
        path2_goal_flg = calcurate_command_value(path_x, path_y, path_st, path_num, 2);
    } else if (path1_first_flg == true && path1_num != 0) {
        path1_goal_flg = calcurate_command_value(path_x, path_y, path_st, path_num, 1);
    }
}

void Pure_Pursuit::change_trajectory_path()
{
    if (path1_first_flg == true && path1_num != 0) {
        path_x = path1_x;
        path_y = path1_y;
        path_st = path1_st;
        path_num = path1_num;
    } 
    if (path1_goal_flg == true && path2_first_flg == true && path2_num != 0) {
        // path clear
        path_x.clear();
        path_y.clear();
        path_st.clear();

        // index flg reset
        if (!path1_reset_index_flg) {
            last_index = 0;
            last_index_dummy = 0;
            pre_last_index = 0;

            path1_reset_index_flg = true;
        }

        // check stop flag reset
        if (!check_stop_flg_reset) {
            check_stop_joy_flg1 = false;
            check_stop_joy_flg2 = false;
            check_stop_joy_flg3 = false;
            check_stop_joy_flg4 = false;

            check_stop_flg_reset = true;
        }

        path_x = path2_x;
        path_y = path2_y;
        path_st = path2_st;
        path_num = path2_num;
    }
    if (path2_goal_flg == true && path3_first_flg == true && path3_num != 0) {
        // path clear
        path_x.clear();
        path_y.clear();
        path_st.clear();

        // index flg reset
        if (!path2_reset_index_flg) {
            last_index = 0;
            last_index_dummy = 0;
            pre_last_index = 0;

            path2_reset_index_flg = true;
        }

        // check stop flag reset
        if (!check_stop_flg_reset) {
            check_stop_joy_flg1 = false;
            check_stop_joy_flg2 = false;
            check_stop_joy_flg3 = false;
            check_stop_joy_flg4 = false;

            check_stop_flg_reset = true;
        }

        path_x = path3_x;
        path_y = path3_y;
        path_st = path3_st;
        path_num = path3_num;
    }
    if (path3_goal_flg == true && path4_first_flg == true && path4_num != 0) {
        // path clear
        path_x.clear();
        path_y.clear();
        path_st.clear();

        // index flg reset
        if (!path3_reset_index_flg) {
            last_index = 0;
            last_index_dummy = 0;
            pre_last_index = 0;

            path3_reset_index_flg = true;
        }

        // check stop flag reset
        if (!check_stop_flg_reset) {
            check_stop_joy_flg1 = false;
            check_stop_joy_flg2 = false;
            check_stop_joy_flg3 = false;
            check_stop_joy_flg4 = false;

            check_stop_flg_reset = true;
        }

        path_x = path4_x;
        path_y = path4_y;
        path_st = path4_st;
        path_num = path4_num;
    }
    if (path4_goal_flg == true && path5_first_flg == true && path5_num != 0) {
        // path clear
        path_x.clear();
        path_y.clear();
        path_st.clear();

        // index flg reset
        if (!path4_reset_index_flg) {
            last_index = 0;
            last_index_dummy = 0;
            pre_last_index = 0;

            path4_reset_index_flg = true;
        }

        // check stop flag reset
        if (!check_stop_flg_reset) {
            check_stop_joy_flg1 = false;
            check_stop_joy_flg2 = false;
            check_stop_joy_flg3 = false;
            check_stop_joy_flg4 = false;

            check_stop_flg_reset = true;
        }

        path_x = path5_x;
        path_y = path5_y;
        path_st = path5_st;
        path_num = path5_num;
    }
    if (path5_goal_flg == true && path6_first_flg == true && path6_num != 0) {
        // path clear
        path_x.clear();
        path_y.clear();
        path_st.clear();

        // index flg reset
        if (!path5_reset_index_flg) {
            last_index = 0;
            last_index_dummy = 0;
            pre_last_index = 0;

            path5_reset_index_flg = true;
        }

        // check stop flag reset
        if (!check_stop_flg_reset) {
            check_stop_joy_flg1 = false;
            check_stop_joy_flg2 = false;
            check_stop_joy_flg3 = false;
            check_stop_joy_flg4 = false;

            check_stop_flg_reset = true;
        }

        path_x = path6_x;
        path_y = path6_y;
        path_st = path6_st;
        path_num = path6_num;
    }
    if (path6_goal_flg == true && path7_first_flg == true && path7_num != 0) {
        // path clear
        path_x.clear();
        path_y.clear();
        path_st.clear();

        // index flg reset
        if (!path6_reset_index_flg) {
            last_index = 0;
            last_index_dummy = 0;
            pre_last_index = 0;

            path6_reset_index_flg = true;
        }

        // check stop flag reset
        if (!check_stop_flg_reset) {
            check_stop_joy_flg1 = false;
            check_stop_joy_flg2 = false;
            check_stop_joy_flg3 = false;
            check_stop_joy_flg4 = false;

            check_stop_flg_reset = true;
        }

        path_x = path7_x;
        path_y = path7_y;
        path_st = path7_st;
        path_num = path7_num;
    }
    if (path7_goal_flg == true && path8_first_flg == true && path8_num != 0) {
        // path clear
        path_x.clear();
        path_y.clear();
        path_st.clear();

        // index flg reset
        if (!path7_reset_index_flg) {
            last_index = 0;
            last_index_dummy = 0;
            pre_last_index = 0;

            path7_reset_index_flg = true;
        }

        // check stop flag reset
        if (!check_stop_flg_reset) {
            check_stop_joy_flg1 = false;
            check_stop_joy_flg2 = false;
            check_stop_joy_flg3 = false;
            check_stop_joy_flg4 = false;

            check_stop_flg_reset = true;
        }

        path_x = path8_x;
        path_y = path8_y;
        path_st = path8_st;
        path_num = path8_num;
    }
}

bool Pure_Pursuit::calcurate_command_value(std::vector<float> path_x, std::vector<float> path_y, std::vector<float> path_st, int path_num, int path_type)
{
    /***********************************************************************
     * find target index path
    **********************************************************************/
    // calculate path from current position distance
    std::vector<float> dist_from_current_pos;
    for (int index = 0; index < path_num; index++) {
        const float dist = std::abs(std::sqrt(std::pow((path_x[index] - current_x), 2.0) + std::pow((path_y[index] - current_y), 2.0)));
        dist_from_current_pos.emplace_back(dist);
    }
    
    // calculate min dist index
    std::vector<float>::iterator iter = std::min_element(dist_from_current_pos.begin(), dist_from_current_pos.end());
    size_t min_index = std::distance(dist_from_current_pos.begin(), iter);
    last_index_dummy = static_cast<int>(min_index);
    std::cout << last_index_dummy << std::endl;
    
    if (abs(last_index_dummy - pre_last_index) > 10) {
        std::cout << "YES" << std::endl;
        last_index_dummy = pre_last_index;
    }

    float look_ahead_filter = 0.1 * current_vel + 0.3;
    target_LookahedDist = 0.0;
    // look ahead distanceの更新
    while (look_ahead_filter > target_LookahedDist) {
        const double d_x = path_x[last_index_dummy + 1] - path_x[last_index_dummy];
        const double d_y = path_y[last_index_dummy + 1] - path_y[last_index_dummy];
        target_LookahedDist += std::sqrt(d_x * d_x + d_y * d_y);
        //last_index_dummy += 1;
    }

    last_index = last_index_dummy;
    pre_last_index = last_index;

    // publish target waypoint number
    targetwp_num.data = last_index;
    targetwp_num_pub.publish(targetwp_num);
    
    // publish select path number
    if (select_path_change) {
        chnage_path_count++;
        select_path_num.data = select_path_num.data + 1;
    }
    select_path_num_pub.publish(select_path_num);

    // check goal
    /*
    bool goal_flag = checkLineAndCircleIntersection(path_x[path_num - 2], path_y[path_num - 2], 
                                                    path_x[path_num - 1], path_y[path_num - 1], 
                                                    current_x, current_y);

    bool path_goal_flg = false;
    if (goal_flag) {
        std::cout << "Goal!" << std::endl;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub.publish(cmd_vel);

        path_goal_flg = true;

        if (chnage_path_count == 0) {
            select_path_change = true;
        } else {
            select_path_change = false;
        }

        return path_goal_flg;
    }
    */

    /***********************************************************************
     * check goal
    **********************************************************************/
    float goal_x = path_x[path_num - 1];
    float goal_y = path_y[path_num - 1];
    const float goal_dist = std::abs(std::sqrt(std::pow((goal_x - current_x), 2.0) + std::pow((goal_y - current_y), 2.0)));

    bool path_goal_flg = false;
    if (goal_dist < goal_th) {
        std::cout << "Goal!" << std::endl;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub.publish(cmd_vel);

        path_goal_flg = true;

        if (chnage_path_count == 0) {
            select_path_change = true;
        } else {
            select_path_change = false;
        }

        return path_goal_flg;
    }

    /***********************************************************************
     * check 1[m] stop path
    **********************************************************************/
    if (path_type == 1) {
        // first stop position
        if (!check_stop_joy_flg1) {
            float check_stop_x = path_x[path_num - 1 * 155];
            float check_stop_y = path_y[path_num - 1 * 155];
            const float check_stop_dist = std::abs(std::sqrt(std::pow((check_stop_x - current_x), 2.0) + std::pow((check_stop_y - current_y), 2.0)));
            if (check_stop_dist < goal_th) {
                std::cout << "First Check Stop!" << std::endl;
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                cmd_vel_pub.publish(cmd_vel);
                return path_goal_flg;
            }
        }
    }

    if (path_type == 2) {
        // first stop position
        if (!check_stop_joy_flg1) {
            float check_stop_x = path_x[path_num - 1 * 20];
            float check_stop_y = path_y[path_num - 1 * 20];
            const float check_stop_dist = std::abs(std::sqrt(std::pow((check_stop_x - current_x), 2.0) + std::pow((check_stop_y - current_y), 2.0)));
            if (check_stop_dist < goal_th) {
                std::cout << "First Check Stop!" << std::endl;
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                cmd_vel_pub.publish(cmd_vel);
                return path_goal_flg;
            }
        }

        // second stop position
        if (!check_stop_joy_flg2) {
            float check_stop_x = path_x[path_num - 1 * 10];
            float check_stop_y = path_y[path_num - 1 * 10];
            const float check_stop_dist = std::abs(std::sqrt(std::pow((check_stop_x - current_x), 2.0) + std::pow((check_stop_y - current_y), 2.0)));
            if (check_stop_dist < goal_th) {
                std::cout << "Second Check Stop!" << std::endl;
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                cmd_vel_pub.publish(cmd_vel);
                return path_goal_flg;
            }
        }
    }

    /***********************************************************************
     * calculate target commnad value
    **********************************************************************/
    int target_lookahed_index = 0;
    float dist_sp_from_nearest = 0.0;
    float target_lookahed_x = path_x[last_index];
    float target_lookahed_y = path_y[last_index];
    for (int index = last_index; index < last_index + 10; index++) {
        dist_sp_from_nearest = path_st[index] - path_st[last_index];
        if (dist_sp_from_nearest > target_LookahedDist) {
            target_lookahed_x = path_x[index];
            target_lookahed_y = path_y[index];
            //std::cout << index << std::endl;
            target_lookahed_index = index;
            break;
        }
    }

    if (abs(last_index_dummy - pre_last_index) > 10) {
        //std::cout << "YES" << std::endl;
        target_lookahed_x = path_x[pre_last_index + 3];
        target_lookahed_y = path_y[pre_last_index + 3];
    }

    // calculate target yaw rate
    float target_yaw = std::atan2(target_lookahed_y - current_y, target_lookahed_x - current_x);
    float yaw_diff = target_yaw - current_yaw_euler;
    //std::cout << current_yaw_euler << std::endl;

    // float/double型の割り算の余りを求める方法【浮動小数点数の剰余】
    // https://marycore.jp/prog/c-lang/modulo-floating-point-number/
    /*
    if (yaw_diff > M_PI) {
        yaw_diff = fmod(yaw_diff, M_PI);
    } else if (yaw_diff < -M_PI) {
        yaw_diff = fmod(yaw_diff, -M_PI);
    }
    */
    if (yaw_diff > M_PI) {
        yaw_diff = -M_PI + fmod(yaw_diff, M_PI);
    } else if (yaw_diff < -M_PI) {
        yaw_diff = M_PI + fmod(yaw_diff, -M_PI);
    }
    std::cout << yaw_diff << std::endl;

    // calcurate curvature and linear_velocity
    float dx = target_lookahed_x - current_x;
    float dy = target_lookahed_y - current_y;
    float distance = sqrt(pow(dx, 2) + pow(dy, 2));
    float curvature = abs(yaw_diff / distance);

    curvature = std::max(minCurvature, std::min(curvature, maxCurvature));
    curvature = curvature / maxCurvature;
    //change velocity according to curvature (asteroid)
    float target_speed = (maxVelocity-minVelocity) * pow(sin(acos(std::cbrt(curvature))), 3) + minVelocity; //[m/s]   
    current_vel = target_speed;

    float alpha = dist_sp_from_nearest / target_speed;
    if (alpha != 0) {
        yaw_rate = std::abs(yaw_diff) / alpha;
    } else {
        yaw_rate = 0.0;
    }

    // check vehicle orientation and target yaw
    if (std::abs(target_yaw - current_yaw_euler) < M_PI) {
        if (target_yaw < current_yaw_euler) {
            yaw_rate = yaw_rate * (-1.0);
        }
    } else if (std::abs(target_yaw - current_yaw_euler) > M_PI) {
        if (target_yaw > current_yaw_euler) {
            yaw_rate = yaw_rate * (-1.0);
        }
    }

    // publish cmd_vel
    cmd_vel.linear.x = target_speed;
    cmd_vel.angular.z = yaw_rate;
    cmd_vel_pub.publish(cmd_vel);

    return path_goal_flg;
}

void Pure_Pursuit::crate_goalLine_point(float a, float b, float c, float d, 
                                        float *right_goal_x, float *right_goal_y, 
                                        float *left_goal_x, float *left_goal_y)
{
    float tilt = (d - b) / (c - a);
    float slice = b - a * ( (d - b) / (c - a) );

    *right_goal_x = a + 3.0f;
    *left_goal_x = a - 3.0f;

    *right_goal_y = - tilt * (*right_goal_x) + slice;
    *left_goal_y = - tilt * (*left_goal_x) + slice;
}

bool Pure_Pursuit::checkLineAndCircleIntersection(float start_x, float start_y, 
                                                  float end_x, float end_y, 
                                                  float p_x, float p_y)
{
    // crealte goal line intersection point 
    float a_x;
    float a_y;
    float b_x;
    float b_y;
    crate_goalLine_point(start_x, start_y, 
                         end_x, end_y, 
                         &b_x, &b_y, &a_x, &a_y);

    double radius = 0.3;

    double a = b_x - a_x;
    double b = b_y - a_y;
    double a2 = a * a;
    double b2 = b * b;
    double r2 = a2 + b2;
    double tt = -(a * (a_x - p_x) + b * (a_y - p_y));

    double shortest_distance = 0.0;
    if (tt < 0) {
        shortest_distance = sqrt((a_x - p_x) * (a_x - p_x) + (a_y - p_y) * (a_y - p_y));
    } else if (tt > r2) {
        shortest_distance = sqrt((b_x - p_x) * (b_x - p_x) + (b_y - p_y) * (b_y - p_y));
    }

    double f1 = a * (a_y - p_y) - b * (a_x - p_x);
    shortest_distance = sqrt((f1 * f1) / r2);

    //std::cout << shortest_distance << std::endl;

    return shortest_distance < radius;
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "pure_pursuit_estimated_1105");
    
    Pure_Pursuit pure_pursuit;
    ros::Rate loop_rate(50);
	while(ros::ok()){
		ros::spinOnce();
        pure_pursuit.update_cmd_vel();
		loop_rate.sleep();
	}

    return 0;
}