#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

int sign(double v); //Sign Function Declaration

//Callback function of Agent1
geometry_msgs::Pose2D current_pose_agent1;
ros::Time tk0_1;
ros::Publisher pub_pose2d;
float px_agent1,py_agent1,vx_agent1;
void odomCallbackAgent1(const nav_msgs::OdometryConstPtr& msg)
{
    // linear position
    current_pose_agent1.x = msg->pose.pose.position.x;
    current_pose_agent1.y = msg->pose.pose.position.y;
    tk0_1 = msg->header.stamp;
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
    current_pose_agent1.theta = yaw;
    pub_pose2d.publish(current_pose_agent1);
}

//Callback function of agent2
geometry_msgs::Pose2D current_pose_agent2;
ros::Publisher pub_pose2dAgent2;
ros::Time tk1_1;
float px_agent2,py_agent2,vx_agent2;
void odomCallbackagent2(const nav_msgs::OdometryConstPtr& msg)
{
    // linear position
    current_pose_agent2.x = msg->pose.pose.position.x;
    current_pose_agent2.y = msg->pose.pose.position.y;
    tk1_1 = msg->header.stamp;
    px_agent2=current_pose_agent2.x;
    py_agent2=current_pose_agent2.y;
    vx_agent2 = msg->twist.twist.linear.x;
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
    current_pose_agent2.theta = yaw;
    pub_pose2d.publish(current_pose_agent2);
}



int main(int argc, char **argv)
{
    const double PI = 3.14159265358979323846;
    
    double u_1=0.0,u_2=0.0,z_3=1.0;
    double a_1,b_1,c_1,s_1,phi_1;
    double a_2,b_2,c_2,s_2,phi_2;
    double e_1,e_2,e_3,e_4;
    double zeta_4=0.0;
    double d_1=0.0,d_2=0.0;
    double time_sw;//By experience, feasible time is more than 20s
    double alpha_1=0.1,beta_1=0.1;
    double alpha_2=0.05,beta_2=0.05;
    double u_1d=0.0,u_2d=0.0;
    ros::Duration delta_t;
    ros::Time t_last,t_first;
	float x_2 = px_agent2;
	float x_2d = px_agent1;
	float vxa = vx_agent2;
	float vxl = vx_agent1;
	double u_1_integral=0.0,u_2_integral=0.0;

    ROS_INFO("start");

    ros::init(argc, argv, "move_fix_agent2x");
    ros::NodeHandle n;
    
    ros::Subscriber sub_odometryAgent2 = n.subscribe("/agent2/odom", 2, odomCallbackagent2);
    ros::Publisher movement_pubAgent2 = n.advertise<geometry_msgs::Twist>("/agent2/cmd_vel",2);
    //for sensors the value after , should be higher to get a more accurate result (queued)
    pub_pose2dAgent2 = n.advertise<geometry_msgs::Pose2D>("/agent2/pose2d", 2);
    
    ros::Rate rate(10); //the larger the value, the "smoother" , try value of 1 to see "jerk" movement
    
    ros::Subscriber sub_odometry = n.subscribe("/agent1/odom", 2, odomCallbackAgent1);
    //for sensors the value after , should be higher to get a more accurate result (queued)
    pub_pose2d = n.advertise<geometry_msgs::Pose2D>("/agent1/pose2d", 2);

    ROS_INFO("Fixed-Time Tracking Control");
    t_last = ros::Time::now();
    while(ros::ok() && ((t_last.toSec()) <= (time_sw)) )
    {
		geometry_msgs::Twist move;
		
		t_first = ros::Time::now();
		delta_t = t_first - t_last;		
		t_last = ros::Time::now();
		
		// calculate switching time
		time_sw=(2/sqrt(alpha_2))+(2/sqrt(beta_2))+((2*sqrt(2))/sqrt(alpha_1))+((2*sqrt(2))/sqrt(beta_1));
		
		// calculate e1
		e_1 = vx_agent2 - vx_agent1;//px_agent2 - px_agent1; // x_2 - x_2d; // position d'agent2 - position du agent1
				
		// calculate e2
		e_2 = u_1-u_1d;//vx_agent2 - vx_agent1; //vxa - vxl; // vitesse d'agent2 - vitesse du agent1
				
		//calculate sliding surface
		a_1=((pow(abs(e_2),2))*sign(e_2))+(alpha_1*e_1)+(beta_1*(pow(abs(e_1),3))*sign(e_1));
		s_1=e_2+(sqrt(abs(a_1))*sign(a_1));
	 
		//calculate sliding mode controller
		b_1=(alpha_2*s_1)+(beta_2*pow(abs(s_1),3)*sign(s_1));
		c_1=(alpha_1+(3*beta_1*pow(e_1,2))+(2*d_1))/2;
		phi_1=(-(c_1)*sign(s_1))-(sqrt(abs(b_1))*sign(b_1));
			
		// calculating u_1
		u_1=u_1d+(phi_1);
		if (delta_t.toSec() > 1.0)
		{		
			u_1_integral = u_1 * 0;
		}
		else
		{
			u_1_integral += u_1 * delta_t.toSec();
		}
				
        // calculating u_2
		u_2=0.0;
		
		move.linear.x = u_1_integral; 
		move.angular.z = u_2;
		movement_pubAgent2.publish(move);
	    /*
	    std::cout << "U1 integral : " << u_1_integral;
		std::cout << std::endl;
		std::cout << "Phi 1 : " << phi_1;
		std::cout << std::endl;		    
		std::cout << "U1 : " << u_1;
		std::cout << std::endl;
	    std::cout << "Velo 0f Agent : " << vx_agent2;
		std::cout << std::endl;
	    std::cout << "Error pos : " << e_1 << " ,Error velo : " << e_2 << " ,Delta time 1 : " << delta_t;
		std::cout << std::endl;
	    */
	    //t_last = t_first;
	    rate.sleep();
	    ros::spinOnce();
	}
	while (ros::ok() && ((t_last.toSec()) > (time_sw)) )
	{
			
		t_first = ros::Time::now();
		delta_t = t_first - t_last;
		t_last = ros::Time::now();
				
		// calculate switching time
		time_sw=(2/sqrt(alpha_2))+(2/sqrt(beta_2))+((2*sqrt(2))/sqrt(alpha_1))+((2*sqrt(2))/sqrt(beta_1));
		
		// calculate e3
		e_3 = vx_agent2*e_4 - vx_agent1*e_4;// vx_agent2*px_agent2 - vx_agent1*px_agent1; // x_2 - x_2d; // position d'agent2 - position du agent1
				
		// calculate e4
		e_4 = u_2-u_2d;//vx_agent2 - vx_agent1; //vxa - vxl; // vitesse d'agent2 - vitesse du agent1
		
		//calculate zeta_4
		zeta_4 = e_4 * px_agent1;
		
		//calculate sliding surface
		a_2=((pow(abs(zeta_4),2))*sign(zeta_4))+(alpha_1*e_3)+(beta_1*(pow(abs(e_3),3))*sign(e_3));
		s_2=zeta_4+(sqrt(abs(a_2))*sign(a_2));
	 	
		//calculate sliding mode controller
		b_2=(alpha_2*s_2)+(beta_2*pow(abs(s_2),3)*sign(s_2));
		c_2=(alpha_1+(3*beta_1*pow(e_3,2))+(2*d_2))/2;
		phi_2=(-(c_2)*sign(s_2))-(sqrt(abs(b_2))*sign(b_2));
			
		// calculating u_2
		u_2=u_2d-((e_4*u_1d)/px_agent1)+(phi_2/px_agent1);
		if (delta_t.toSec() > 1.0)
		{		
			u_2_integral = u_2 * 0;
		}
		else
		{
			u_2_integral += u_2 * delta_t.toSec();
		}		
				
        geometry_msgs::Twist move;
		move.linear.x = u_2_integral;
		move.linear.y = 0;
		move.angular.z = 0;
		movement_pubAgent2.publish(move);
		/*
		std::cout << "U2 integral : " << u_2_integral;
		std::cout << std::endl;
		std::cout << "Phi 2 : " << phi_2;
		std::cout << std::endl;		    
		std::cout << "U2 : " << u_2;
		std::cout << std::endl;
	    std::cout << "Velo 0f Agent : " << vx_agent2;
		std::cout << std::endl;
	    std::cout << "Error pos : " << e_3 << " ,Error velo : " << e_4 << " ,Delta time 2 : " << delta_t;
		std::cout << std::endl;
		*/
	    rate.sleep();
	    ros::spinOnce();
	}
	    
   	geometry_msgs::Twist move;
    //push to stop
			
    move.linear.x = 0;
    move.linear.y = 0;
    move.angular.z = 0;
    movement_pubAgent2.publish(move);
    
    ros::spin();
	return 0;
}

//Function of SIGN
int sign(double v)
{
	if (v<0) return -1;
	if (v>0) return 1;
	return 0;
}
