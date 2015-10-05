        float angle = 0;
        ros::Rate r1(1);
	while(n.ok()){
	if(flag==0) {//back home	
        	angle -=6.26/180;
		ir_angle.data=angle;
		ir_angle_cmd_pub.publish(ir_angle);
		ros::spinOnce(); 
		r1.sleep();
		if(angle<=-1.57)
			flag=1;
		}
	if(flag==1){
	       	angle +=6.26/360;
		ir_angle.data=angle;
		ir_angle_cmd_pub.publish(ir_angle);
		ros::spinOnce(); 
		r1.sleep();
		if(angle>=1.57)
			flag=-1;
		}
	}

