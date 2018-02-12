#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

/*NOTE: When Gazebo world is reset (ctrl-r) the camera topic 
seems to dissapear along with /scan. As a result the turtlebot
idles because it's not receiving any data*/

class Seeker{
    private:
	ros::ServiceServer m_enable; 
	ros::Publisher	   m_pub_veloc;
	ros::Publisher     m_pub_dsplc;
	ros::Subscriber    m_sub;
	ros::NodeHandle    m_nh;

	bool  m_action, m_drive_mode;
	float m_displace, m_ctrLsr, m_lftLsr, m_rgtLsr;
	int   margin;

    public: Seeker(){
	m_sub        = m_nh.subscribe("/scan", 100, &Seeker::updateValues, this);
	m_pub_veloc  = m_nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);
	m_pub_dsplc  = m_nh.advertise<geometry_msgs::Vector3>("/displacement",1);
	m_enable     = m_nh.advertiseService("/enable", &Seeker::enableService, this);

	m_action     = false;
	m_drive_mode = false;
	m_displace   = 0.0; 
	m_ctrLsr     = 0.0; 
	m_lftLsr     = 0.0; 
	m_rgtLsr     = 0.0;

	const int margin = 5;  
    }

    void updateValues(const sensor_msgs::LaserScan msg){
	int vecLen = msg.ranges.size();
	int halfVec = vecLen/2;
	m_ctrLsr = msg.ranges[halfVec];
	m_lftLsr = msg.ranges[(halfVec) - margin];
	m_rgtLsr = msg.ranges[(halfVec) + margin];
	m_displace = (m_lftLsr + m_rgtLsr)/2;
	//ROS_INFO("Left Laser: [%f]", m_lftLsr);
	//ROS_INFO("Right Laser: [%f]", m_rgtLsr);
    }

    bool enableService(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res){
	m_action = req.data;
	m_drive_mode = false;
	res.success = true;

	if(m_action){
	    res.message = "Behavior On";
	}else{
	    res.message = "Behavior Off";
	}     
    	//ROS_INFO("Enable: [%d]", req.data);
    	return true;
    }

    //Getters & Setters
    float getLftLsr(){ return m_lftLsr; }
    float getRgtLsr(){ return m_rgtLsr; }
    float getDisplace(){ return m_displace; }
    bool  getAction(){ return m_action; }
    bool  getDriveMode(){ return m_drive_mode; }
    ros::Publisher getVelPub(){ return m_pub_veloc; }
    ros::Publisher getDplPub(){ return m_pub_dsplc; }
    void  setDriveMode(bool value){ m_drive_mode = value; }
    
};

int main(int argc, char **argv){
   ros::init(argc, argv, "seeker_node"); 
   Seeker seeker;
   ros::Publisher velPub = seeker.getVelPub();
   ros::Publisher dplPub = seeker.getDplPub();

   ros::Rate loop_rate(5);

   while(ros::ok()){
	geometry_msgs::Twist   geoMsg;
	geometry_msgs::Vector3 disMsg;
	disMsg.x = seeker.getDisplace();

	bool  action = seeker.getAction();
	bool  driveMode = seeker.getDriveMode();
	float lftLsr = seeker.getLftLsr();
	float rgtLsr = seeker.getRgtLsr();
	
	if(action){
	    if((isnan(lftLsr) || lftLsr==0.0) && !driveMode){ //reconnaisance       
		geoMsg.angular.z = 0.3;
	    }else{ //seek out
		seeker.setDriveMode(true);
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
    //ROS_INFO("action: %d", action);
    dplPub.publish(disMsg);
    velPub.publish(geoMsg);
    ros::spinOnce();	
    loop_rate.sleep();
  }
	    
  return 1;
}


