#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

int sign(double v); //Sign Function Declaration

//Callback function of Leader
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

//Callback function of Agent1
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
    
    double u_1=0.0,u_2=0.0,z_3=1.0,x_2d=10.0;
    double a_1,b_1,s_1,phi_1;
    double a_2,b_2,s_2,phi_2=0.0;
    double e_1,e_2,e_3,e_4=0.0;
    double zeta_4=0.0;
    double d_2=0.0;
    double time_sw;//By experience, feasible time is more than 20s
    double alpha_1=20,beta_1=20;
    double alpha_2=10,beta_2=10;
    double u_1d=0.3,u_2d=1.0;
    double x_2;
    double d_1=0.0;
    double e_1dot,e_2dot;
    ros::Duration delta_t;
    ros::Time t_last,t_first;
	double dt;
   
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
    
    t_first = ros::Time::now();
	std::cout << "Time running first: " << t_first.toSec();
	std::cout << std::endl;
	
    while(ros::ok())
    {
				
		geometry_msgs::Twist move;
		dt = delta_t.toSec();		
			
		// calculate switching time
		time_sw=(2/sqrt(alpha_2))+(2/sqrt(beta_2))+((2*sqrt(2))/sqrt(alpha_1))+((2*sqrt(2))/sqrt(beta_1));
		            
		//calculate sliding surface
		a_1=((pow(abs(e_2),2))*sign(e_2))+(alpha_1*e_1)+(beta_1*(pow(abs(e_1),3))*sign(e_1));
		s_1=e_2+sqrt(abs(a_1))*sign(a_1);
	 	/*
		a_2=((pow(abs(zeta_4),2))*sign(zeta_4))+(alpha_1*e_3)+(beta_1*(pow(abs(e_3),3))*sign(e_3));
		s_2=zeta_4+sqrt(abs(a_2))*sign(a_2);*/

		//calculate sliding mode controller
		b_1=(alpha_2*s_1)+(beta_2*sqrt(pow(abs(s_1),3))*sign(s_1));
		phi_1= (alpha_1)+((3*beta_1*(pow(e_1,2))+(2*d_1))/2)*sign(s_1)-(sqrt(abs(b_1))*sign(b_1));
		/*
		b_2=(alpha_2*s_2)+(beta_2*sqrt(pow(abs(s_2),3))*sign(s_2));
		phi_2= (alpha_1)+((3*beta_1*(pow(e_3,2))+(2*d_2))/2)*sign(s_2)-(sqrt(abs(b_2))*sign(b_2));*/
	
		// calculating u_1
		u_1=u_1d+phi_1;

		//Calculate e_dot
		x_2 = current_pose_agent1.x;
		e_1dot = x_2-x_2d;
		e_2dot = u_1+d_1-u_1d;
		
			
				
        // calculating u_2
		if (t_last.toSec() < (time_sw+60.0))
			t_last = ros::Time::now();
			
			//Calculate erreur
			e_1 = e_1dot*dt;
			e_2 = e_2dot*dt;
						
			u_2=0.0;
			geometry_msgs::Twist move;
			move.linear.x = u_1; 
			move.angular.z = 0;
			/*move.linear.x = u_1*t_last.toSec(); 
			move.angular.z = u_2*t_last.toSec();*/
			movement_pubAgent1.publish(move);
		    
		    std::cout << "Time first 1 : " << t_first;
			std::cout << std::endl;
			std::cout << "Time last 1 : " << t_last;
			std::cout << std::endl;
		    
		    delta_t = t_last - t_first;
		    std::cout << "Delta time 1 : " << delta_t;
			std::cout << std::endl;
		    
		    //t_last = t_first;
		    ros::spinOnce();
		    rate.sleep();
		}
		else
		{
			t_last = ros::Time::now();
			
			move.linear.x = 0;
			move.linear.y = 0;
			move.angular.z = 0;
			movement_pubAgent1.publish(move);
		
		    std::cout << "Time first 2 : " << t_first;
			std::cout << std::endl;
			std::cout << "Time last 2 : " << t_last;
			std::cout << std::endl;
			delta_t = t_last - t_first;
			std::cout << "Delta time 2 : " << delta_t;
			std::cout << std::endl;
			//t_last = t_first;
	    	ros::spinOnce();
	    	rate.sleep();
	    }
    
		//push to stop
			
		move.linear.x = 0;
		move.linear.y = 0;
		move.angular.z = 0;
		movement_pubAgent1.publish(move);
    }
    
   	geometry_msgs::Twist move;
    //push to stop
			
    move.linear.x = 0;
    move.linear.y = 0;
    move.angular.z = 0;
    movement_pubAgent1.publish(move);
    
	return 0;
}

//Function of SIGN
int sign(double v)
{
	if (v<0) return -1;
	if (v>0) return 1;
	return 0;
}


		/*	u_2=u_2d-((e_4*u_1d)/x_2)+(phi_2/x_2d);
	    	
			//velocity controls
			geometry_msgs::Twist move;
		
	    	move.linear.x = (u_2+z_3*u_1)*t_last.toSec(); 
	    	move.angular.z = u_1*t_last.toSec();
	    	movement_pubAgent1.publish(move);
		*/
