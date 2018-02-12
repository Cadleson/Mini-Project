#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

bool action = false;
bool driveMode = false;
int vecLen = 640;
float displace = 0.0;
float ctrLsr = 0.0;
float lftLsr = 0.0;
float rgtLsr = 0.0;


void updateValue(const sensor_msgs::LaserScan msg){
    ctrLsr = msg.ranges[vecLen/2];
    lftLsr = msg.ranges[315];
    rgtLsr = msg.ranges[325];
    displace = (lftLsr + rgtLsr)/2;
    ROS_INFO("Left Laser: [%f]", lftLsr);
    ROS_INFO("Right Laser: [%f]", rgtLsr);
}

bool enabler(std_srvs::SetBool::Request  &req,
    	     std_srvs::SetBool::Response &res){
    action = req.data;
    driveMode = false;
    	     res.success = true;
    if(action){
	res.message = "Behavior On";
    }else{
	res.message = "Behavior Off";
    }     
    
    //ROS_INFO("Enable: [%d]", req.data);
    return true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "seeker_node"); 
  ros::NodeHandle nh; 			
  ros::Subscriber sub = nh.subscribe("/scan", 1000, updateValue);
  ros::ServiceServer enable = nh.advertiseService("/enable", enabler);
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100); 
  ros::Publisher pubDis = nh.advertise<geometry_msgs::Vector3>("/displacement",1);
  ros::Rate loop_rate(5);

  while(ros::ok()){
    geometry_msgs::Twist geoMsg;
    geometry_msgs::Vector3 disMsg;
    disMsg.x = displace;
	if(action){
	    if((isnan(lftLsr) || lftLsr==0.0) && !driveMode){ //reconnaisance       
		geoMsg.angular.z = 0.3;
	    }else{ //seek out
		driveMode = true;
		if(isnan(lftLsr)){
		    geoMsg.angular.z = -0.1;
		    geoMsg.linear.x = 0.2;
		}else if(isnan(rgtLsr)){
		    geoMsg.angular.z = 0.1;	
		    geoMsg.linear.x = 0.2;
		}else{
		    geoMsg.linear.x = 0.2;
		}	
	    }
	}
    ROS_INFO("drive Mode: %d",driveMode);
    pubDis.publish(disMsg);
    pub.publish(geoMsg);
    ros::spinOnce();	
    loop_rate.sleep();
  }
	    
  return 0;
}



