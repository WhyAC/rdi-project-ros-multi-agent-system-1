#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <math.h>


geometry_msgs::Pose2D current_pose_agent1;
ros::Publisher pub_pose2d;
ros::Time tk1_2;
float px_agent1,py_agent1,vx_agent1;
void odomCallbackagent1(const nav_msgs::OdometryConstPtr& msg)
{
    // linear position
    current_pose_agent1.x = msg->pose.pose.position.x;
    current_pose_agent1.y = msg->pose.pose.position.y;
    tk1_2 = msg->header.stamp;
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
geometry_msgs::Pose2D current_pose_agent2;
ros::Time tk2_2;
ros::Publisher pub_pose2dagent2;
float px_agent2,py_agent2,vx_agent2;
void odomCallbackagent2(const nav_msgs::OdometryConstPtr& msg)
{
    // linear position
    current_pose_agent2.x = msg->pose.pose.position.x;
    current_pose_agent2.y = msg->pose.pose.position.y;
    tk2_2 = msg->header.stamp;
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
    current_pose_agent1.theta = yaw;
    pub_pose2d.publish(current_pose_agent2);
}

int main(int argc, char **argv)
{
    const double PI = 3.14159265358979323846;
    double u,u_integral = 0;
    ros::Time int_time,current_time,prev_time;
    ros::Duration delta_t;
    double dvhat1_2,vhat1_2=0,dpxhat1_2,pxhat1_2=0,z0=0;
    double dvhat2_2,vhat2_2=0,dpxhat2_2,pxhat2_2=0,z1=0;
    double theta=2, lembda=.3; // Gains
    ROS_INFO("start");
    ros::init(argc, argv, "move_pub_agent2");
    ros::NodeHandle n;
    
    ros::Subscriber sub_odometryagent2 = n.subscribe("/agent2/odom", 10, odomCallbackagent2);
    ros::Publisher movement_pubagent2 = n.advertise<geometry_msgs::Twist>("/agent2/cmd_vel",10); 
    //for sensors the value after , should be higher to get a more accurate result (queued)
    pub_pose2dagent2 = n.advertise<geometry_msgs::Pose2D>("/agent2/pose2d", 10);
    ros::Rate rate(10); //the larger the value, the "smoother" , try value of 1 to see "jerk" movement
    ros::Subscriber sub_odometry = n.subscribe("/agent1/odom", 10, odomCallbackagent1);
   //for sensors the value after , should be higher to get a more accurate result (queued)
    pub_pose2d = n.advertise<geometry_msgs::Pose2D>("/agent1/pose2d", 10);
    
    //move
    ros::Time start1 = ros::Time::now();
    while(ros::ok())
    {
	geometry_msgs::Twist move;
	ros::Time t = ros::Time::now();
   	// calculate delta_t
	delta_t = t - prev_time;
	prev_time = ros::Time::now();

        //Observer to estimate agent1's states
        z0=exp(-2*theta*(t.toSec()-tk1_2.toSec()))*(pxhat1_2-px_agent1);
        dvhat1_2=-theta*theta*z0;
	vhat1_2 +=dvhat1_2*delta_t.toSec();
	dpxhat1_2=vhat1_2-2*theta*z0;
	pxhat1_2 +=dpxhat1_2*delta_t.toSec();
        
	//Observer to estimate agent2's states
        z1=exp(-2*theta*(t.toSec()-tk2_2.toSec()))*(pxhat2_2-px_agent2);
        dvhat2_2=-theta*theta*z1;
	vhat2_2 +=dvhat2_2*delta_t.toSec();
	dpxhat2_2=vhat2_2-2*theta*z1;
	pxhat2_2 +=dpxhat2_2*delta_t.toSec();

	// calculating u
	u=-(lembda*lembda)*(pxhat2_2-pxhat1_2)-lembda*(vhat2_2-vhat1_2);
	u_integral += u*delta_t.toSec();
        
	//velocity controls
        move.linear.x = u_integral; //speed value m/s
        move.angular.z = 0;
        movement_pubagent2.publish(move);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
