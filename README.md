#ir_avoidance_simulator

###joint_state_controller/JointStateController
* publish_rate raise to 100Hz can afford more detail motion control and data receive

###rotate_ir_up.cpp and rotate_ir_down.cpp
* using two Callback to catch distance observed by ir sensor and the angle of ir sensor rotate by motor
* using argc and argv to catch the Kp user defined 

		if(argc>1){Kp=atof(argv[1]);}	
* using Pointcloud msg to record the catch data and pub to topic
* initial Pointcloud need to assign name,frame_id, size

		sensor_msgs::PointCloud cloud;

		cloud.points.resize(500);

		cloud.header.frame_id = "ir_down_view";

* be careful to resize PointCloud memory every time get a new data
	
		cloud.points.resize(count+2);	

* every 1.57 angle arrive , publish the PointCloud to topic

		if(ir_position>=1.57*3/4)
 		
		flag=-1;ir_measure_pub.publish(cloud);count=0;

* control loop frequency is 500 , interval is 0.002s, but angle detect and distance interval is 0.01s (100Hz)
 

#Need To Do Next

### ir_up ir_down observe data need to change frame_id
* change observe frame_id from ir_up_view to ir_up
* the Transformation calculation using ROS service funciton

###costmap_common_params.yaml
* observation data is still not plot on costmap!


