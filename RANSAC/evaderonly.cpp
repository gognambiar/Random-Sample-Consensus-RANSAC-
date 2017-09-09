#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
 {
 geometry_msgs::Twist vel;

 ros::NodeHandle n;
 ros::Publisher vel_pub_=n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
 ros::Rate rate(100);
 
 int flag = 0;

while (ros::ok())
{   
for (unsigned int x=0;x< scan_msg->ranges.size();x++)
{
             if(scan_msg->ranges[x]<0.8) {
flag = 1;

                }
}
		if(flag == 1)
{
                vel.linear.x = 0.0;
                vel.angular.z = 2.0;
                vel_pub_.publish(vel);
}
              else
{

                vel.linear.x = 2.0;
                vel.angular.z = 0.0;
                vel_pub_.publish(vel);

                }

ros::spin();
}        



 }
int main(int argc, char** argv){
  ros::init(argc, argv, "evader");

  ros::NodeHandle n;
  ros::Publisher vel_pub_=n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Subscriber scan_sub = n.subscribe("base_scan", 1, scanCallback);

  ros::spin();
  return 0;

}
