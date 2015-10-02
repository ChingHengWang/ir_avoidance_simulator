#ir_avoidance_simulator
* andbot.xacro 
* meshes/link.STL 

#First File: ir_avoidance_simulator/urdf/andbot.xacro

#### it's urdf xacro file, which describe the mechanical geometry parameters 
* add arm_link1 and arm_link2 and put on left_arm and and right_arm dividely  
* add the geometry of Head ,Base , Arm and simple body shape
* describe distance between each joint and joint's position and orientation
* put the lidarlite on the head_link ,head_link rotate relative to the body_link
* put 12 sonar links on the base  
* weight issue sill need to update


#Second File: ir_avoidance_simulator/meshes
* it's folder contain link's STL files , the STL files are used for urdf "mesh" Tag 

#Demo Script
* script file path : ir_avoidance_simulator/run_ir_avoidance_simulator
* run gazebo and rviz demo for this urdf 
    
        $ ./run_ir_avoidance_simulator
