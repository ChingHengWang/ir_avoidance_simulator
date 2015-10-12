#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "ir_up_tf_listener");

  ros::NodeHandle node;

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.waitForTransform("/ir_up_view", "/ir_up",  
                               ros::Time(0), ros::Duration(100.0));
      listener.lookupTransform("/ir_up_view", "/ir_up",  
                               ros::Time(0), transform);
	
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
      tf::Quaternion q;
      q=transform.getRotation();

      ROS_INFO("YAW %f",q.getEuler.yaw());




    rate.sleep();
  }
  return 0;
};
