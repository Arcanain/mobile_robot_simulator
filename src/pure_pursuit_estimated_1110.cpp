/* Data   : 2022/11/12
 * Author : Ramune6110
 * ********************************************************************************
 * SYSTEM
 * MCU            | Raspberry Pi 4
 * SENSOR
 * Accel/Gyro/Mag | Witmotion
 * Encoder        | 6.5inch In-Wheel-Motor
 * GNSS           | ublox zed-f9p
 * ********************************************************************************
 */
/**********************************************************************
 * Include
**********************************************************************/
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <math.h>
#include <vector>
#include <algorithm>

#include <Eigen/Core>
#include <Eigen/Geometry>

/**********************************************************************
 * Define
**********************************************************************/
//#define STACK_JUDGE

class Pure_Pursuit
{
    private:
        // Publish and Subscriber
        ros::NodeHandle nh;
        ros::Publisher cmd_vel_pub;
        ros::Publisher targetwp_num_pub;
        ros::Publisher csv_path_number_pub;
        ros::Subscriber path_sub;
        ros::Subscriber path_num_sub;
        ros::Subscriber odom_sub;
        ros::Subscriber joy_sub;
        ros::Subscriber imu_sub;
        ros::Subscriber related_yaw_sub;

        ros::Publisher write_start_pub;
        ros::Subscriber write_finish_sub;

        // Lookahed distance for Pure Pursuit[m]
        float target_LookahedDist;

        // various flag
        bool path_first_flg = false;
        bool path_num_first_flg = false;
        bool odom_first_flg = false;
        bool imu_first_flg = false;
        bool position_search_flg = false;
        bool check_stop_joy_flg = false;
        bool select_path_change = false;
        bool stack_flg = false;

        bool goal_flg = false;
        bool path_reset_index_flg = false;
        bool count_flag = false;

        // path, path_num, path_index
        int path_num = 0;
        int last_index = 0;
        int last_index_dummy = 0;
        int pre_last_index = 0;
        int chnage_path_count = 0;

        int pre_csv_path_number = 1;
        
        int goal_count = 0;
        int csv_number_count = 0;

        // for stack judgement
        float stack_count = 0.0;
        float stack_pre_x = 0.0;
        float stack_pre_y = 0.0;

        // for path_callback
        std::vector<float> path_x;
        std::vector<float> path_y;
        std::vector<float> path_st;

        // for odom_callback
        float current_x;
        float current_y;
        float current_yaw_euler;
        float current_vel;
        float pre_current_x;
        float pre_current_y;

        // for publish targetwp_num
        std_msgs::Int32 targetwp_num;

        // for publish csv_path_number
        std_msgs::Int8 csv_path_number;
        std_msgs::Bool write_start_flg;

        // for publish cmd_vel
        geometry_msgs::Twist cmd_vel;
        float goal_th = 0.5; //[m]
        float yaw_rate = 0.0; //[rad/s]

        // cauvature parameter
        float minCurvature = 0.0;
        float maxCurvature = 0.5;
        float minVelocity = 0.1;
        float maxVelocity = 0.3;

        // for subscribe imu_data
        float accl_x;
        float accl_y;
        float accl_z;
        float anglvel_x;
        float anglvel_y;
        float anglvel_z;

    public:
        // constract
        Pure_Pursuit();
        // destract
        ~Pure_Pursuit();
        // path callback
        void path_callback(const nav_msgs::Path &path_msg);
        // path number callback
        void path_num_callback(const std_msgs::Int32 &path_num_msg);
        // odom callback
        void odom_callback(const nav_msgs::Odometry &odom_msg);
        // joy
        void joy_callback(const sensor_msgs::Joy& joy);
        // imu callback
        void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg);
        // related yaw
        void related_yaw_callback(const std_msgs::Float64 &related_yaw_msg);
        // csv file write finish callback
        void write_finish_callback(const std_msgs::Bool& msg);
        // publish cmd_vel
        void update_cmd_vel();

        void crate_goalLine_point(float a, float b, float c, float d, 
                                  float *right_goal_x, float *right_goal_y, 
                                  float *left_goal_x, float *left_goal_y);

        bool checkLineAndCircleIntersection(float start_x, float start_y, 
                                            float end_x, float end_y, 
                                            float p_x, float p_y);

        void stack_judgement();

        bool autonomous_start_flg = false;
};

Pure_Pursuit::Pure_Pursuit()
{
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_automatic", 50);
    targetwp_num_pub = nh.advertise<std_msgs::Int32>("/targetwp_num", 10);
    csv_path_number_pub = nh.advertise<std_msgs::Int8>("/csv_path_number", 10);
    csv_path_number.data = 1;

    path_sub = nh.subscribe("/path", 10, &Pure_Pursuit::path_callback, this);
    path_num_sub = nh.subscribe("/path_num", 10, &Pure_Pursuit::path_num_callback, this);
    odom_sub = nh.subscribe("/estimated_pose", 10, &Pure_Pursuit::odom_callback, this);
    joy_sub = nh.subscribe("/joy", 10, &Pure_Pursuit::joy_callback, this);
    imu_sub = nh.subscribe("/imu/data", 10, &Pure_Pursuit::imu_callback, this);
    related_yaw_sub = nh.subscribe("/wit/related_yaw", 10, &Pure_Pursuit::related_yaw_callback, this);

    write_start_pub = nh.advertise<std_msgs::Bool>("/write_start_flg", 10);
    write_finish_sub = nh.subscribe("/write_finish_flg", 10, &Pure_Pursuit::write_finish_callback, this);
    write_start_flg.data = false;
}

Pure_Pursuit::~Pure_Pursuit()
{
    std::cout << "Finish Pure Pursuit!" << std::endl;
}

void Pure_Pursuit::path_callback(const nav_msgs::Path &path_msg)
{
    if (!path_reset_index_flg) {
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
        check_stop_joy_flg = true;
    }
    // maru
    if (msg.buttons[1] == 1){
        autonomous_start_flg = true;
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
    
    // accl and angular_vel
    accl_x = imu_msg->linear_acceleration.x;
    accl_y = imu_msg->linear_acceleration.y;
    accl_z = imu_msg->linear_acceleration.z;
    anglvel_x = imu_msg->angular_velocity.x;
    anglvel_y = imu_msg->angular_velocity.y;
    anglvel_z = imu_msg->angular_velocity.z;
    
    //std::cout << accl_x << std::endl;
    //std::cout << anglvel_z << std::endl;

    imu_first_flg = true;
}

void Pure_Pursuit::related_yaw_callback(const std_msgs::Float64 &related_yaw_msg)
{
    //std::cout << related_yaw_msg.data << std::endl;
    //current_yaw_euler = related_yaw_msg.data;
    //imu_first_flg = true;
}

void Pure_Pursuit::write_finish_callback(const std_msgs::Bool& msg)
{
    if (msg.data == true) {
        write_start_flg.data = false;
        goal_flg = false;
        path_reset_index_flg = false;
        path_first_flg = false;
        path_num_first_flg = false;
    } else {
        goal_flg = false;
    }
}

void Pure_Pursuit::update_cmd_vel()
{
    if (path_first_flg == true && odom_first_flg == true && imu_first_flg == true && path_num != 0) {
        /**********************************************************************
         * path reset
        **********************************************************************/
        if (!path_reset_index_flg) {
            // path clear
            path_x.clear();
            path_y.clear();
            path_st.clear();
            
            std::cout << "path reset" << std::endl;

            last_index = 0;
            last_index_dummy = 0;
            pre_last_index = 0;
            
            path_x = path_x;
            path_y = path_y;
            path_st = path_st;
            path_num = path_num;
            
            path_reset_index_flg = true;

            write_start_flg.data = false;

            goal_flg = false;
            
            // stop cmd_vel
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_vel_pub.publish(cmd_vel);

            ros::Duration(1).sleep(); // 1秒間停止
        }

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
        //std::cout << last_index_dummy << std::endl;
        
        if (abs(last_index_dummy - pre_last_index) > 10) {
            //std::cout << "YES" << std::endl;
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
        
        /**********************************************************************
         * check goal
        **********************************************************************/
        float goal_x = path_x[path_num - 1];
        float goal_y = path_y[path_num - 1];
        float goal_dist = std::abs(std::sqrt(std::pow((goal_x - current_x), 2.0) + std::pow((goal_y - current_y), 2.0)));

        //std::cout << goal_dist << std::endl;

        // goal judgement
        if (goal_dist < goal_th) {
            std::cout << "Goal!" << std::endl;

            goal_flg = true;

            goal_count = goal_count + 1;
            if (goal_count >= 2) {
                goal_count = 0;
                goal_flg = false;
            }
        }

        // stop goal flag
        if (goal_flg) {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_vel_pub.publish(cmd_vel);

            ros::Duration(1).sleep(); // 1秒間停止
        } 
        
        // move up csv file
        if (goal_flg == true && write_start_flg.data == false) {

            ros::Duration(1).sleep(); // 1秒間停止

            count_flag = true;

            csv_number_count = csv_number_count + 1;
            if (csv_number_count >= 2) {
                csv_number_count = 0;
                count_flag = false;
            } 

            if (count_flag) {
                csv_path_number.data = csv_path_number.data + 1;
                pre_csv_path_number = csv_path_number.data;
            }

            csv_path_number_pub.publish(csv_path_number);

            write_start_flg.data = true;

            write_start_pub.publish(write_start_flg);
        } 

        //write_start_pub.publish(write_start_flg);
                                                                
        // check goal
        /*
        bool goal_flag = checkLineAndCircleIntersection(path_x[path_num - 2], path_y[path_num - 2], 
                                                        path_x[path_num - 1], path_y[path_num - 1], 
                                                        current_x, current_y);
        if (goal_flag) {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_vel_pub.publish(cmd_vel);

            if (chnage_path_count == 0) {
                select_path_change = true;
            } else {
                select_path_change = false;
            }

            return;
        }

        bool test_flag = true;
        if (goal_flag == true && test_flag == true) {
            test_flag = false;
        }
        */

        /**********************************************************************
         * check 1[m] stop path
        **********************************************************************/
        if (csv_path_number.data == 2) {
            if (!check_stop_joy_flg) {
                float check_stop_x = path_x[path_num - 1 * 10];
                float check_stop_y = path_y[path_num - 1 * 10];
                const float check_stop_dist = std::abs(std::sqrt(std::pow((check_stop_x - current_x), 2.0) + std::pow((check_stop_y - current_y), 2.0)));
                if (check_stop_dist < goal_th) {
                    std::cout << "Check Stop!" << std::endl;
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.0;
                    cmd_vel_pub.publish(cmd_vel);
                    return;
                }
            }
        }
        
        /*
        bool check_stop_flag = checkLineAndCircleIntersection(path_x[path_num - 1 * 6], path_y[path_num - 1 * 6], 
                                                              path_x[path_num - 1 * 5], path_y[path_num - 1 * 5], 
                                                              current_x, current_y);
        //std::cout << check_stop_flag << std::endl;

        if (check_stop_flag) {
            std::cout << "Check Stop!" << std::endl;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_vel_pub.publish(cmd_vel);
            return;
        }
        */

        /**********************************************************************
         * calculate target commnad value
        **********************************************************************/
        // calculate target point
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

        // calculate target yaw rate
        float target_yaw = std::atan2(target_lookahed_y - current_y, target_lookahed_x - current_x);
        float yaw_diff = target_yaw - current_yaw_euler;
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
        float target_speed = (maxVelocity - minVelocity) * pow(sin(acos(std::cbrt(curvature))), 3) + minVelocity; //[m/s]   
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

void Pure_Pursuit::stack_judgement()
{
    if (cmd_vel.linear.x != 0 || cmd_vel.angular.z != 0) {
        if (accl_x >= -0.5 && accl_x <= 0.5) {
            stack_flg = true;
        } else {
            stack_flg = false;
        }

        /*
        if (abs(current_x - pre_current_x) <= 0.0001) {
            stack_flg = true;
        } else {
            stack_flg = false;
        }
        */
        //std::cout << current_x - pre_current_x << std::endl;
    }

    if (stack_flg) {
        stack_count = stack_count + 1.0;
        if (stack_count == 500) {
            stack_pre_x = current_x;
            stack_pre_y = current_y;
        }
    } else {
        std::cout << "FALSE" << std::endl;
        stack_count = 0;
    }

    if (stack_count >= 500.0) {
        // publish cmd_vel
        cmd_vel.linear.x = 3.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub.publish(cmd_vel);
    }

    float stack_dist = 0.0;
    if (stack_count >= 500.0) {
        stack_dist = std::abs(std::sqrt(std::pow((current_x - stack_pre_x), 2.0) + std::pow((current_y - stack_pre_y), 2.0)));
        //std::cout << dist << std::endl;
    }
    
    if (stack_dist > 10.0) {
        stack_flg = false;
        stack_count = 0;
    }

    pre_current_x = current_x;
    pre_current_y = current_y;
}

/**
 *******************************************************************************************
 * MAIN Method
 *******************************************************************************************
 */
int main(int argc, char**argv)
{
    ros::init(argc, argv, "pure_pursuit_estimated_1110");
    
    Pure_Pursuit pure_pursuit;
    ros::Rate loop_rate(50);
	while(ros::ok()){
		ros::spinOnce();
        if (pure_pursuit.autonomous_start_flg) {
            pure_pursuit.update_cmd_vel();

            #ifdef STACK_JUDGE
                pure_pursuit.stack_judgement();
            #endif
            
        }
        //pure_pursuit.update_cmd_vel();
		loop_rate.sleep();
	}

    return 0;
}