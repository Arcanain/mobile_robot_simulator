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
![path_follow1](https://user-images.githubusercontent.com/52307432/182168329-f190ba82-16b3-47f0-85ca-231e9f843560.png)

![path_follow2](https://user-images.githubusercontent.com/52307432/182168375-15313e38-5899-4273-9651-89570f47e7d1.png)

![path_follow3](https://user-images.githubusercontent.com/52307432/182168403-7cb4e24b-2b3c-45bf-8bfe-1ac3356c29ec.png)



