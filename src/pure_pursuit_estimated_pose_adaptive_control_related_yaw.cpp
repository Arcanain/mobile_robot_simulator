#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <math.h>
#include <vector>
#include <algorithm>

class Pure_Pursuit
{
    private:
        ros::NodeHandle nh;
        ros::Publisher cmd_vel_pub;
        ros::Publisher targetwp_num_pub;
        ros::Subscriber path_sub;
        ros::Subscriber path_num_sub;
        ros::Subscriber odom_sub;
        ros::Subscriber imu_sub;
        ros::Subscriber related_yaw_sub;

        float target_LookahedDist; // Lookahed distance for Pure Pursuit[m]

        bool path_first_flg = false;
        bool path_num_first_flg = false;
        bool odom_first_flg = false;
        bool imu_first_flg = false;
        bool position_search_flg = false;
        int path_num = 0;
        int last_index = 0;

        // for path_callback
        std::vector<float> path_x;
        std::vector<float> path_y;
        std::vector<float> path_st;

        // for odom_callback
        float current_x;
        float current_y;
        float current_yaw_euler;
        float current_vel;

        // for publish targetwp_num
        std_msgs::Int32 targetwp_num;

        // for publish cmd_vel
        geometry_msgs::Twist cmd_vel;
        float goal_th = 0.1; //[m]
        float yaw_rate = 0.0; //[rad/s]

        // cauvature parameter
        float minCurvature = 0.1;
        float maxCurvature = 3.0;
        float minVelocity = 0.1;
        float maxVelocity = 0.3;

    public:
        Pure_Pursuit();
        ~Pure_Pursuit();
        // path callback
        void path_callback(const nav_msgs::Path &path_msg);
        void path_num_callback(const std_msgs::Int32 &path_num_msg);
        // odom callback
        void odom_callback(const nav_msgs::Odometry &odom_msg);
        // imu callback
        void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg);
        // related yaw
        void related_yaw_callback(const std_msgs::Float64 &related_yaw_msg);
        // publish cmd_vel
        void update_cmd_vel();
};

Pure_Pursuit::Pure_Pursuit()
{
    //cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_automatic", 50);
    targetwp_num_pub = nh.advertise<std_msgs::Int32>("/targetwp_num", 10);

    path_sub = nh.subscribe("/path", 10, &Pure_Pursuit::path_callback, this);
    path_num_sub = nh.subscribe("/path_num", 10, &Pure_Pursuit::path_num_callback, this);
    odom_sub = nh.subscribe("/estimated_pose", 10, &Pure_Pursuit::odom_callback, this);
    imu_sub = nh.subscribe("/imu/data", 10, &Pure_Pursuit::imu_callback, this);
    related_yaw_sub = nh.subscribe("/wit/related_yaw", 10, &Pure_Pursuit::related_yaw_callback, this);
}

Pure_Pursuit::~Pure_Pursuit()
{
    std::cout << "Finish Pure Pursuit!" << std::endl;
}

void Pure_Pursuit::path_callback(const nav_msgs::Path &path_msg)
{
    if (!path_first_flg && path_num != 0) {
        std::cout << "first path call" << std::endl;
        
        float last_x = 0.0;
        float last_y = 0.0;
        float st = 0.0;
        float last_st = 0.0;
        for (int index = 0; index < path_num; index++) {
            path_x.emplace_back(path_msg.poses[index].pose.position.x);
            path_y.emplace_back(path_msg.poses[index].pose.position.y);
            
            st = std::sqrt(std::pow((path_x[index] - last_x), 2.0) + std::pow((path_y[index] - last_y), 2.0));
            path_st.emplace_back(last_st + st);
            
            last_x = path_x[index];
            last_y = path_y[index];
            last_st = path_st[index];
        }

        path_first_flg = true;
    }
}

void Pure_Pursuit::path_num_callback(const std_msgs::Int32 &path_num_msg)
{
    if (!path_num_first_flg) {
        path_num = path_num_msg.data;
        path_num_first_flg = true;
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
    */

    odom_first_flg = true;
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

    //current_yaw_euler = (float)yaw;

    //current_yaw_euler = fmod((float)yaw, 2.0f * M_PI);
    
    //imu_first_flg = true;
}

void Pure_Pursuit::related_yaw_callback(const std_msgs::Float64 &related_yaw_msg)
{
    std::cout << related_yaw_msg.data << std::endl;
    current_yaw_euler = related_yaw_msg.data;
    imu_first_flg = true;
}

void Pure_Pursuit::update_cmd_vel()
{
    if (path_first_flg == true && odom_first_flg == true && imu_first_flg == true && path_num != 0) {
        // calculate path from current position distance
        std::vector<float> dist_from_current_pos;
        for (int index = 0; index < path_num; index++) {
            const float dist = std::abs(std::sqrt(std::pow((path_x[index] - current_x), 2.0) + std::pow((path_y[index] - current_y), 2.0)));
            dist_from_current_pos.emplace_back(dist);
        }
        
        // calculate min dist index
        std::vector<float>::iterator iter = std::min_element(dist_from_current_pos.begin(), dist_from_current_pos.end());
        size_t min_index = std::distance(dist_from_current_pos.begin(), iter);
        //last_index = static_cast<int>(min_index);
        
        float look_ahead_filter = 0.1 * current_vel + 0.3;
        target_LookahedDist = 0.0;
        // look ahead distanceの更新
        while (look_ahead_filter > target_LookahedDist) {
            const double d_x = path_x[min_index + 1] - path_x[min_index];
            const double d_y = path_y[min_index + 1] - path_y[min_index];
            target_LookahedDist += std::sqrt(d_x * d_x + d_y * d_y);
            min_index += 1;
        }
        
        //target_LookahedDist = 0.5;

        last_index = static_cast<int>(min_index);
        //std::cout << target_LookahedDist << std::endl;
        //std::cout << last_index << std::endl;

        // publish target waypoint number
        targetwp_num.data = last_index;
        targetwp_num_pub.publish(targetwp_num);

        // check goal
        float goal_x = path_x[path_num - 1];
        float goal_y = path_y[path_num - 1];
        const float goal_dist = std::abs(std::sqrt(std::pow((goal_x - current_x), 2.0) + std::pow((goal_y - current_y), 2.0)));
        if (goal_dist < goal_th) {
            std::cout << "Goal!" << std::endl;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_vel_pub.publish(cmd_vel);
            return;
        }

        // calculate target point
        float dist_sp_from_nearest = 0.0;
        float target_lookahed_x = path_x[last_index];
        float target_lookahed_y = path_y[last_index];
        for (int index = last_index; index < path_num; index++) {
            dist_sp_from_nearest = path_st[index] - path_st[last_index];
            if (dist_sp_from_nearest > target_LookahedDist) {
                target_lookahed_x = path_x[index];
                target_lookahed_y = path_y[index];
                break;
            }
        }

        // calculate target yaw rate
        float target_yaw = std::atan2(target_lookahed_y - current_y, target_lookahed_x - current_x);
        float yaw_diff = target_yaw - current_yaw_euler;
        //std::cout << target_yaw << std::endl;
        //std::cout << current_yaw_euler << std::endl;

        // float/double型の割り算の余りを求める方法【浮動小数点数の剰余】
        // https://marycore.jp/prog/c-lang/modulo-floating-point-number/
        if (yaw_diff > M_PI) {
            yaw_diff = fmod(yaw_diff, M_PI);
        } else if (yaw_diff < -M_PI) {
            yaw_diff = fmod(yaw_diff, -M_PI);
        }

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
        return;
    }
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "pure_pursuit_estimated_pose_adaptive_control_related_yaw");
    
    Pure_Pursuit pure_pursuit;
    ros::Rate loop_rate(50);
	while(ros::ok()){
		ros::spinOnce();
        pure_pursuit.update_cmd_vel();
		loop_rate.sleep();
	}

    return 0;
}