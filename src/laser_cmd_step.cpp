
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <unistd.h>


int main(int argc, char** argv){
 	ros::init(argc, argv, "laser_cmd_step");
	ros::NodeHandle n;
	ros::Time current_time;
	
        std_msgs::Float64 laser_angle;
//	ros::Publisher laser_pub = n.advertise<sensor_msgs::LaserScan>("laser", 50);
	int flag = 0;
  	
// 	ros::Publisher laser_cmd_pub = n.advertise<std_msgs::Float64>("/andbot/laser_position_controller/command", 50);
	ros::Publisher laser_cmd_pub = n.advertise<std_msgs::String>("/chatter", 50);



  	float angle = 0.0;
        ros::Rate r1(1);
	std_msgs::String msg;
	std::stringstream ss;

        while(n.ok()) {
		angle = 0.0;
		ss<<angle<<std::endl;
		msg.data=ss.str();
		laser_cmd_pub.publish(msg);
		r1.sleep();
		angle = -1.57;
		ss<<angle<<std::endl;
		msg.data=ss.str();
		laser_cmd_pub.publish(msg);
		r1.sleep();


	}



/*
        ros::Rate r1(50);
        while(n.ok()) {

            angle -= 6.28/720;
            laser_angle.data = angle;
            laser_angle_pub.publish(laser_angle);
            

            r1.sleep();
            ros::spinOnce(); 
            if (angle <= -1.57) break;
        }

        sleep(5); // to make sure the laser joint move to home position

*/

}

