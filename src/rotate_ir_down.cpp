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
#include <stdlib.h>
double ir_position;
double distance_scan;

void ir_angle_Callback(const sensor_msgs::JointState& jointstate){
	 ir_position= jointstate.position[1];	
}
void ir_measure_dist_Callback(const sensor_msgs::LaserScan& irscan){
	distance_scan = irscan.ranges[1];	
}
int main(int argc, char** argv){
 	ros::init(argc, argv, "rotating_ir_down");
	ros::NodeHandle n;
	ros::Time current_time;
        std_msgs::Float64 ir_angle;
//	double dist[1000];
//	double* Dist=(double*)malloc(10*sizeof(double));

	int flag = -1; //home
	float Kp=8.0;
	if(argc>1){	
	Kp=atof(argv[1]);
	ROS_INFO("Kp %f",Kp);
	}
	else	
	ROS_INFO("Kp %f",Kp);
 	ros::Publisher ir_angle_cmd_pub = n.advertise<std_msgs::Float64>("/andbot/ir_down_velocity_controller/command", 50);
	ros::Subscriber ir_angle_subscriber_;
	ir_angle_subscriber_ = n.subscribe("/andbot/joint_states", 100, ir_angle_Callback);


	sensor_msgs::PointCloud cloud;
	cloud.points.resize(500);
	cloud.header.frame_id = "ir_down_view";
	cloud.channels.resize(1);
	cloud.channels[0].name = "count";
	cloud.channels[0].values.resize(500);
	ros::Publisher ir_measure_pub = n.advertise<sensor_msgs::PointCloud>("ir_down_measure", 150);
	ros::Subscriber ir_measure_dist_subscriber_;
	ir_measure_dist_subscriber_ = n.subscribe("ir_down_dist", 100, ir_measure_dist_Callback);

  	
	unsigned int count=0;
        float angle_cmd = 0;
        ros::Rate r1(500);
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
		cloud.header.stamp = current_time;
		cloud.points[count].x = distance_scan*cos(ir_position);
		cloud.points[count].y = distance_scan*sin(ir_position);
		cloud.points[count].z = distance_scan*sin(3.14/4);
		cloud.points.resize(count+2);
		ROS_INFO("1 COUNTER %d ,ANG %f,DIST %f,TIMER %f",count,ir_position,distance_scan,ros::Time::now().toSec());
		count++;	

		if(ir_position>=1.57*3/4){
			flag=-1;
                 	ir_measure_pub.publish(cloud);
			count=0;

			}

		}

	if(flag==-1) {//back home	
		angle_cmd=-1.57;
		ros::spinOnce();
		ir_angle.data=Kp*(angle_cmd-ir_position);
		ir_angle_cmd_pub.publish(ir_angle);
		cloud.header.stamp = current_time;
		cloud.points[count].x = distance_scan*cos(ir_position);
		cloud.points[count].y = distance_scan*sin(ir_position);
		cloud.points[count].z = distance_scan*sin(3.14/4);
		cloud.channels[0].values[count] = count;

		cloud.points.resize(count+2);
		cloud.channels[0].values.resize(count+2);
	
		ROS_INFO("-1 COUNTER %d ,ANG %f,DIST %f",count,ir_position,distance_scan);
		count++;	

		if(ir_position<=-1.57*3/4){
			flag=1;
                 	ir_measure_pub.publish(cloud);
			count=0;

			}
		}
		r1.sleep();

	}

	//	ROS_INFO("COUNT %d",count);
	//	ROS_INFO("ANGLE %f",angle);

}

