# mobile_robot_simulator

## Environment
OS : raspbian(buster)  
ROS : noetic  

## display_mobile_robot
display urdf mobile robot  

```
roslaunch mobile_robot_simulator display_mobile_robot.launch 
```

![display_mobile_robot](https://user-images.githubusercontent.com/52307432/182167318-f236d8d8-7f51-444b-8e72-0f9f69232ea3.png)

## odom_publisher

1.publish odometry  
2.keyboard publish /cmd_vel  
3.save odometry data to csvfile  

```
roslaunch mobile_robot_simulator odom_publisher.launch 
```

## display_path
display path  

```
roslaunch mobile_robot_simulator display_path.launch 
```
![display_path](https://user-images.githubusercontent.com/52307432/182168050-873b5ae5-7954-4cb4-bd00-67c43b07aca0.png)

## path_follower
pure pursuit path follower

```
roslaunch mobile_robot_simulator path_follower.launch 
```
![path_follow1](https://user-images.githubusercontent.com/52307432/182187253-4bdcf24b-0b0f-45e6-9cd5-a005cde5e2ea.png)

![path_follow2](https://user-images.githubusercontent.com/52307432/182187282-0fecded3-3b22-4015-b535-5aa34d71e3f9.png)

![path_follow3](https://user-images.githubusercontent.com/52307432/182187301-95ec9489-c267-47ad-95e5-e4f695890f81.png)

### video 
https://user-images.githubusercontent.com/52307432/183284928-a2d0d9b5-49b5-4688-b988-4fa819e89951.mp4
