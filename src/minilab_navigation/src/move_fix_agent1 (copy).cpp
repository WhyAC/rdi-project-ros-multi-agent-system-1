#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

geometry_msgs::Pose2D current_pose_leader;
ros::Time tk0_1;
ros::Publisher pub_pose2d;
float px_leader,py_leader,vx_leader;
void odomCallbackLeader(const nav_msgs::OdometryConstPtr& msg)
{
    // linear position
    current_pose_leader.x = msg->pose.pose.position.x;
    current_pose_leader.y = msg->pose.pose.position.y;
    tk0_1 = msg->header.stamp;
    px_leader=current_pose_leader.x;
    py_leader=current_pose_leader.y;
    vx_leader = msg->twist.twist.linear.x;
   
    // quaternion to RPY conversion
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // angular position
    current_pose_leader.theta = yaw;
    pub_pose2d.publish(current_pose_leader);
}
geometry_msgs::Pose2D current_pose_agent1;
ros::Publisher pub_pose2dAgent1;
ros::Time tk1_1;
float px_agent1,py_agent1,vx_agent1;
void odomCallbackagent1(const nav_msgs::OdometryConstPtr& msg)
{
    // linear position
    current_pose_agent1.x = msg->pose.pose.position.x;
    current_pose_agent1.y = msg->pose.pose.position.y;
    tk1_1 = msg->header.stamp;
    px_agent1=current_pose_agent1.x;
    py_agent1=current_pose_agent1.y;
    vx_agent1 = msg->twist.twist.linear.x;
    // quaternion to RPY conversion
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // angular position
    current_pose_leader.theta = yaw;
    pub_pose2d.publish(current_pose_agent1);
}

int main(int argc, char **argv)
{
    const double PI = 3.14159265358979323846;
    int sign(double v); //Sign Function Declaration    
    double u_1=0.0,u_2=0.0,z_3=0.0,x_2=1.0,x_2d=1.0;
    double a_1,b_1,s_1,phi_1;
    double a_2,b_2,s_2,phi_2=0.0;
    double e_1,e_2,e_3,e_4=0.0;
    double zeta_4=0.0;
    double d_1=0.0,d_2=0.0;
    double time_sw;
    double alpha_1=20,beta_1=20;
    double alpha_2=10,beta_2=10;
    double u_1d=2,u_2d=1;
    
   
    ROS_INFO("start");

    ros::init(argc, argv, "move_fix_agent1");
    ros::NodeHandle n;
    
    ros::Subscriber sub_odometryAgent1 = n.subscribe("/agent1/odom", 10, odomCallbackagent1);
    ros::Publisher movement_pubAgent1 = n.advertise<geometry_msgs::Twist>("/agent1/cmd_vel",10); 
    //for sensors the value after , should be higher to get a more accurate result (queued)
    pub_pose2dAgent1 = n.advertise<geometry_msgs::Pose2D>("/agent1/pose2d", 10);
    ros::Rate rate(10); //the larger the value, the "smoother" , try value of 1 to see "jerk" movement
    ros::Subscriber sub_odometry = n.subscribe("/leader/odom", 10, odomCallbackLeader);
    //for sensors the value after , should be higher to get a more accurate result (queued)
    pub_pose2d = n.advertise<geometry_msgs::Pose2D>("/leader/pose2d", 10);

         
     
    ROS_INFO("Fixed-Time Tracking Control");
    while(ros::ok())
    {

	geometry_msgs::Twist move;
	ros::Time t = ros::Time::now();

/*		//Observer to estimate leader's states            
		dz1hat0_1=-theta*theta*z0;
	    z1hat0_1 +=dvhat0_1*delta_t.toSec();
	    dz2hat0_1=vhat0_1-2*theta*z0;
	    z2hat0_1 +=dpxhat0_1*delta_t.toSec();
		dz3hat0_1=-theta*theta*z0;
	    z3hat0_1 +=dvhat0_1*delta_t.toSec();
	    dz4hat0_1=vhat0_1-2*theta*z0;
	    z4hat0_1 +=dpxhat0_1*delta_t.toSec();
        
		//Observer to estimate agent1's states
        z1=exp(-2*theta*(t.toSec()-tk1_1.toSec()))*(pxhat1_1-px_agent1);
        dvhat1_1=-theta*theta*z1;
	    vhat1_1 +=dvhat1_1*delta_t.toSec();
	    dpxhat1_1=vhat1_1-2*theta*z1;
	    pxhat1_1 +=dpxhat1_1*delta_t.toSec();
*/	       
	
	// calculate switching time
	time_sw=(2/sqrt(alpha_2))+(2/sqrt(beta_2))+((2*sqrt(2))/sqrt(alpha_1))+((2*sqrt(2))/sqrt(beta_1));
                
	//calculate sliding surface
	a_1=((abs(e_2))^2)*sign(e_2)+(alpha_1*e_1)+(beta_1*((abs(e_1))^3)*sign(e_1));
    s_1=e_2+sqrt(abs(a_1)*sign(a_2));
 
    a_2=((abs(zeta_4))^2)*sign(zeta_4)+(alpha_1*e_3)+(beta_1*((abs(e_3))^3)*sign(e_3));
    s_2=zeta_4+sqrt(abs(a_2)*sign(a_2));

	//calculate sliding mode controller
    b_1=(alpha_2*s_1)+(beta_2*pow(abs(s_1),3)*sign(s_1));
    phi_1= ((alpha_1+(3*beta_1*pow(e_1,2.0)+(2*d_1))/2)*sign(s_1)-sqrt(abs(b_1)*sign(b_1)));

    b_2=(alpha_2*s_2)+(beta_2*pow(abs(s_2),3)*sign(s_2));
    phi_2= ((alpha_1+(3*beta_1*pow(e_3,2.0)+(2*d_2))/2)*sign(s_2)-sqrt(abs(b_2)*sign(b_2)));
	
	// calculating u_1
	u_1=u_1d+phi_1;

        // calculating u_2
	while(t.toSec() < time_sw);
	u_2=1;

        while(t.toSec() >= time_sw);
	u_2=u_2d-((e_4*u_1d)/x_2)+(phi_2/x_2d);
        
	//velocity controls
        move.linear.x = u_2+z_3*u_1; 
        move.angular.z = u_1;
        movement_pubAgent1.publish(move);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

//Function of SIGN
int sign(double v)
{
	if (v<0) return -1;
	if (v>0) return 1;
	return 0;
}
