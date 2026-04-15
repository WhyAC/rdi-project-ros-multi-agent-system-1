/*	
	Move to Goal Program
	Time independent
	Based on Triangle Formula
	28/09/2018
*/

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

#include <math.h>

geometry_msgs::Pose2D current_pose, goal_pose;
ros::Publisher pub_pose2d;
//Function Speed Linear Declaration
double speed_linear(double posx, double posy, double gposx, double gposy);
//Function Speed Angular Declaration
double speed_angular(double posx, double posy, double gposx, double gposy, double postheta);

//Function Callback Odometry
void odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    // linear position
    current_pose.x = msg->pose.pose.position.x;
    current_pose.y = msg->pose.pose.position.y;

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
    current_pose.theta = yaw;
    pub_pose2d.publish(current_pose);
}


int main(int argc, char **argv)
{
    const double PI = 3.14159265358979323846;
	double goal_posex, goal_posey, mx, mz;

    ROS_INFO("start");

    ros::init(argc, argv, "move_goal");
    ros::NodeHandle n;
    ros::Subscriber sub_odometry = n.subscribe("/leader/odom", 1, odomCallback);
    //for sensors the value after , should be higher to get a more accurate result (queued)
    ros::Publisher movement_pub = n.advertise<geometry_msgs::Twist>("/leader/cmd_vel",1); 
    pub_pose2d = n.advertise<geometry_msgs::Pose2D>("/leader/pose2d", 1);
    //the larger the value, the "smoother" , try value of 1 to see "jerk" movement    
    ros::Rate rate(10);


	//Get the input from the user.

	//Get Goal position x
	std::cout << "Posisi x tujuan: "; 
	std::cin >> goal_posex;
	std::cout << std::endl;		
	//Get Goal position y
	std::cout << "Posisi y tujuan: "; 
	std::cin >> goal_posey;
	std::cout << std::endl;
	
    //move to goal position
    ROS_INFO("Move to goal position");
    //double t0 = ros::Time::now().toSec();

	mx = speed_linear(current_pose.x, current_pose.y, goal_posex, goal_posey);
	mz = speed_angular(current_pose.x, current_pose.y, goal_posex, goal_posey, current_pose.theta);	

	while((ros::ok() && (mx/0.5) >= 0.05))
	{
		mx = speed_linear(current_pose.x, current_pose.y, goal_posex, goal_posey);
		mz = speed_angular(current_pose.x, current_pose.y, goal_posex, goal_posey, current_pose.theta);	
			
		std::cout << "Current Theta: " << current_pose.theta << std::endl;
		std::cout << "Current X: " << current_pose.x << std::endl;
		std::cout << "Current Y: " << current_pose.y << std::endl;	
		std::cout << "Current R: " << (mx/0.5) << std::endl;	
		
		geometry_msgs::Twist move;
				
		move.linear.x = mx; //x linear speed value m/s
		move.angular.z = mz; //z rotation speed value m/s		
		movement_pub.publish(move);

        ros::spinOnce();
        rate.sleep();
	}
	
	geometry_msgs::Twist move;
	move.linear.x = 0.0; //x linear speed value m/s
	move.linear.y = 0.0; //y linear speed value m/s
   	move.angular.z = 0.0; //z rotation speed value m/s		
	movement_pub.publish(move);	    
    return 0;
}

//Function Speed Linear
double speed_linear(double posx, double posy, double gposx, double gposy)
{
	return (0.5 * sqrt(pow((gposx - posx), 2) + pow((gposy - posy), 2)));
}

//Function Speed Angular
double speed_angular(double posx, double posy, double gposx, double gposy, double postheta)
{
	return (3 * (atan2(gposy - posy, gposx - posx) - postheta));
}
