
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <unistd.h>


int count = 0;
double ir_position;
void irCallback(const sensor_msgs::JointState& jointstate){
	 ir_position= jointstate.position[1];	
}

int main(int argc, char** argv){
 	ros::init(argc, argv, "rotating_ir_down");
	ros::NodeHandle n;
	ros::Time current_time;
        std_msgs::Float64 ir_angle;
	int flag = -1; //home
	float Kp=15.0;

  	
 	ros::Publisher ir_angle_cmd_pub = n.advertise<std_msgs::Float64>("/andbot/ir_down_velocity_controller/command", 50);
	ros::Subscriber ir_angle_subscriber_;
	ir_angle_subscriber_ = n.subscribe("/andbot/joint_states", 100, irCallback);

        float angle_cmd = 0;
        ros::Rate r1(100);
	while(n.ok()){
		ROS_INFO("IR_POS %f,FLAG %d",ir_position,flag);
		ROS_INFO("IR_CMD %f",Kp*(angle_cmd-ir_position));

	if(flag==0) {//back home	
		angle_cmd=-1.57;
		ros::spinOnce();
		ir_angle.data=Kp*(angle_cmd-ir_position);
		ir_angle_cmd_pub.publish(ir_angle);
		r1.sleep();
		if(ir_position<=-1.57*3/4)
			flag=1;
		}
	if(flag==1) {	
		angle_cmd=1.57;
		ros::spinOnce();
		ir_angle.data=Kp*(angle_cmd-ir_position);
		ir_angle_cmd_pub.publish(ir_angle);
		r1.sleep();
		if(ir_position>=1.57*3/4)
			flag=-1;
		}

	if(flag==-1) {//back home	
		angle_cmd=-1.57;
		ros::spinOnce();
		ir_angle.data=Kp*(angle_cmd-ir_position);
		ir_angle_cmd_pub.publish(ir_angle);
		r1.sleep();
		if(ir_position<=-1.57*3/4)
			flag=1;
		}


	}

	//	ROS_INFO("COUNT %d",count);
	//	ROS_INFO("ANGLE %f",angle);

}

