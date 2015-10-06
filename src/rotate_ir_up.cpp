#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <unistd.h>
#include <math.h>

double ir_position;
double distance_scan;

void ir_angle_Callback(const sensor_msgs::JointState& jointstate){
	 ir_position= jointstate.position[2];	
}
void ir_measure_dist_Callback(const sensor_msgs::LaserScan& irscan){
	distance_scan = irscan.ranges[1];	
}
int main(int argc, char** argv){
 	ros::init(argc, argv, "rotating_ir_up");
	ros::NodeHandle n;
	ros::Time current_time;
        std_msgs::Float64 ir_angle;
	double dist[200];
	int flag = -1; //home
	float Kp=8.0;
 	ros::Publisher ir_angle_cmd_pub = n.advertise<std_msgs::Float64>("/andbot/ir_up_velocity_controller/command", 50);
	ros::Subscriber ir_angle_subscriber_;
	ir_angle_subscriber_ = n.subscribe("/andbot/joint_states", 100, ir_angle_Callback);


	sensor_msgs::PointCloud cloud;
	cloud.points.resize(500);
	cloud.header.frame_id = "head_link";

	ros::Publisher ir_measure_pub = n.advertise<sensor_msgs::PointCloud>("ir_up_measure", 50);
	ros::Subscriber ir_measure_dist_subscriber_;
	ir_measure_dist_subscriber_ = n.subscribe("ir_up_dist", 100, ir_measure_dist_Callback);






  	
	unsigned int count=0;
        float angle_cmd = 0;
        ros::Rate r1(100);
	while(n.ok()){
		current_time = ros::Time::now();

//		ROS_INFO("IR_POS %f,FLAG %d",ir_position,flag);
//		ROS_INFO("IR_CMD %f",Kp*(angle_cmd-ir_position));
//		ROS_INFO("COUNTER %d",count);
//		ROS_INFO("TIMER %f",ros::Time::now().toSec());
	if(flag==0) {//back home	
		angle_cmd=-1.57;
		ros::spinOnce();
		ir_angle.data=Kp*(angle_cmd-ir_position);
		ir_angle_cmd_pub.publish(ir_angle);
		if(ir_position<=-1.57*3/4)
			flag=1;
			count=0;
		}
	if(flag==1) {	
		angle_cmd=1.57;
		ros::spinOnce();
		ir_angle.data=Kp*(angle_cmd-ir_position);
		ir_angle_cmd_pub.publish(ir_angle);
		dist[count] = distance_scan;
		cloud.header.stamp = current_time;
		 cloud.points[count].x = distance_scan*cos(ir_position);
		 cloud.points[count].y = distance_scan*sin(ir_position);
		 cloud.points[count].z = -distance_scan*sin(3.14/4);

		ROS_INFO("COUNTER %d ,ANG %f,DIST %f",count,ir_position,dist[count]);
		count++;	

		if(ir_position>=1.57*3/4){
			flag=-1;
		cloud.points.resize(count-1);

                 ir_measure_pub.publish(cloud);

			count=0;
			}

		}

	if(flag==-1) {//back home	
		angle_cmd=-1.57;
		ros::spinOnce();
		ir_angle.data=Kp*(angle_cmd-ir_position);
		ir_angle_cmd_pub.publish(ir_angle);
		if(ir_position<=-1.57*3/4)
			flag=1;

		}
		r1.sleep();

	}

	//	ROS_INFO("COUNT %d",count);
	//	ROS_INFO("ANGLE %f",angle);

}

