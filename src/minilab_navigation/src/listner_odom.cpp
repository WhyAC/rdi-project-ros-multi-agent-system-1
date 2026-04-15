#include <ros/ros.h>
#include <nav_msgs/Odometry.h>



void listnerCallbackagent1(nav_msgs::Odometry msg)
{
  float px,py,vx,vy;
  ros::Rate rate(0.0333);
  px = msg.pose.pose.position.x;
  py = msg.pose.pose.position.y;
  vx = msg.twist.twist.linear.x;
  vy = msg.twist.twist.linear.y;
  ROS_INFO("HEllO ! Agent1 position is  [%f] [%f]",px,py);
  ros::Duration(0.5).sleep();
  ROS_INFO("HEllO ! Agent1 velocity is  [%f] [%f]",vx,vy);
  //ros::Duration(0.5).sleep();
  ros::spinOnce();
  rate.sleep();
}

void listnerCallbackleader(nav_msgs::Odometry msg)
{
  float px,py,vx,vy;
  ros::Rate rate(0.0333);
  px = msg.pose.pose.position.x;
  py = msg.pose.pose.position.y;
  vx = msg.twist.twist.linear.x;
  vy = msg.twist.twist.linear.y;
  ROS_INFO("HEllO ! leader position is  [%f] [%f]",px,py);
  ros::Duration(0.5).sleep();
  ROS_INFO("HEllO ! leader velocity is  [%f] [%f]",vx,vy);
  ros::spinOnce();
  rate.sleep();  
//ros::Duration(0.5).sleep();
}
int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_listner");

  ros::NodeHandle leader;
  ros::NodeHandle agent1;
  ros::Subscriber sub2 = agent1.subscribe("/agent1/odom", 1, listnerCallbackagent1);
  ros::Subscriber sub = leader.subscribe("/leader/odom", 1, listnerCallbackleader);
  
 ros::spin();
}

