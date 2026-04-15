#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <math.h>

//#include <minilab_navigation/Input.h>

geometry_msgs::Pose2D current_pose_leader;
ros::Publisher pub_pose2d;
float px_leader;
float py_leader;
float ptheta_leader;

void odomCallbackLeader(const nav_msgs::OdometryConstPtr& msg)
{
    // linear position
    current_pose_leader.x = msg->pose.pose.position.x;
    current_pose_leader.y = msg->pose.pose.position.y;
    px_leader=current_pose_leader.x ;
    py_leader=current_pose_leader.y ;
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

int main(int argc, char **argv)
{
    const double PI = 3.14159265358979323846;
    double u1=0, u1_integral = 0.0, last_u1=0;
    double u2=0, u2_integral = 0.0, last_u2=0;
    double v=0, last_v=0;
    double w=0, last_w=0;
    double delta_x=0, delta_y=0, delta_th=0; 
    ros::Time int_time,current_time;
    
    ros::Duration dt;
    ROS_INFO("start");

    ros::init(argc, argv, "move_fix_Leader");
    ros::NodeHandle n;
    ros::Subscriber sub_odometry = n.subscribe("/minilab3/odom", 10, odomCallbackLeader);
    ros::Publisher movement_pub = n.advertise<geometry_msgs::Twist>("/minilab3/cmd_vel",10); 

//    ros::Publisher input_publeader = n.advertise<minilab_navigation::Input>("/minilab3/input",10); 

    ros::Rate rate(10); 

/*  //move
    ROS_INFO("move leader");
    ros::Time initial_time = ros::Time::now();
    minilab_navigation::Input input_leader;
    ros::Time last_time= ros::Time::now();

    while(ros::ok() && current_pose_leader.x <5.8)
    {
        geometry_msgs::Twist move;
	    ros::Time t = ros::Time::now();
   	    
        // calculate delta_t
	    dt = (current_time - last_time).toSec();
	    last_time = ros::Time::now();
	
        move.linear.x = vx;      
	    move.angular.z = w;
	
        delta_x = (vx*cos(th)-vy*sin(th))*dt;
        delta_y = (vx*sin(th)+vy*cos(th))*dt;
        delta_th = w*dt;
        
        x += delta_x;
        y += delta_y;
        th += delta_th;

	    q_1 += th*dt;
        q_2 += w*dt.toSec();
        q_3 += (x*cos(th)+y*sin(th))*dt;
        q_4 += (x*sin(th)-y*cos(th))*dt;


        movement_pub.publish(move);
	    input_leader.input=u;
	    input_leader.header.stamp = ros::Time::now();
	    input_leader.header.frame_id = "/world";
	    input_publeader.publish(input_leader);
        ros::spinOnce();
        rate.sleep();
*/
        //move forward 1
        ROS_INFO("move forward");
        ros::Time start1 = ros::Time::now();
        while(ros::ok())
        {
        geometry_msgs::Twist move;
       	ros::Time t = ros::Time::now();
        
        //move.linear.x =sin((2*PI*ros::Time::now().toSec())/100);//0.1+ velocity; //speed value m/s
	    move.linear.x =0.6;//*(2*(PI)*cos(2*PI*t.toSec()/50))/50;       
        move.angular.z = 0;
        movement_pub.publish(move);

        ros::spinOnce();
        rate.sleep();
        }

 
        // just stop
        while(ros::ok()) {
        geometry_msgs::Twist move;
        move.linear.x = 0;
        move.angular.z = 0;
        movement_pub.publish(move);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


