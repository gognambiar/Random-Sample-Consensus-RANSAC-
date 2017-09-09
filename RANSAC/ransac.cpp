#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <math.h>
#include <stdlib.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

class cartesian
{
public:
float x,y;
void cartset(float t, float p)
{
t = t*0.00872664619237 - 1.57079637051;
y = p*sin(t);
x = p*cos(t);
}
};

/*float calcdist(cartesian a, cartesian b, cartesian c)
{
float m = ((b.y - a.y)/(b.x - a.x));
float dfdis = ((m*c.x - c.y + a.y - m*a.x)/sqrt(pow(m,2) + 1));
return (dfdis);
}*/

float calcdist(cartesian a, cartesian b, cartesian c)
{
float dfdis;
float dfx = b.x - a.x;
float dfy = b.y - a.y;
if ((dfx == 0) && (dfy == 0))
{
   dfx = c.x - a.x;
   dfy = c.y - a.y;
   dfdis = sqrt(dfx * dfx + dfy * dfy);
}

float d = ((c.x - a.x) * dfx + (c.y - a.y) * dfy) / (dfx * dfx + dfy * dfy);

if (d < 0)
    {
        dfx = c.x - a.x;
        dfy = c.y - a.y;
    }
    else if (d > 1)
    {
        dfx = c.x - b.x;
        dfy = c.y - b.y;
    }
    else
    {
        dfx = c.x - (a.x + d * dfx);
        dfy = c.y - (a.y + d * dfy);
    }

   dfdis = sqrt(dfx * dfx + dfy * dfy);

return (dfdis);
}

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
ros::NodeHandle n;
ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
ros::Rate r(100);
while (ros::ok())
{
cartesian max_otl[200], s1[200],s4[200];
int i, j,ff =0,eff= 0,ef=0,e=0,fi=0,fff=0,fii=0,ft = 0,ftt=0,flag =0,chk;
int no_max_inl = 0,b,c=0,dd=0;
float dfdis;

for(b=0;b< scan_msg->ranges.size();b++)
{
  if(scan_msg->intensities[b] == 1.0)
  {
  s1[c].cartset(c, scan_msg->ranges[b]);
  c++;
  }
}

if(c>10)
{
flag = 1;
ff = c;
for(dd = 0; dd < 10; dd++)
{
if(ft > 10 || dd == 0)
{
if(dd != 0)
{
ff = ft;
}
for(e =0; e < 60; e++)
{
i = rand() % ff;
j = rand() % ff;
fi = 0;ft = 0;
for(ef = 0; ef < ff; ef++)
{
 
 dfdis = calcdist(s1[i], s1[j], s1[ef]);
 if(dfdis < 0.1)
 {
   fi++;
 }
 else
 {
   max_otl[ft] = s1[ef];
   ft++;
 }
}//ef

 if(no_max_inl <= fi)
 {
 no_max_inl = fi;
 s4[2*e] = s1[i];s4[2*e+1] = s1[j]; //collection of best points

 for (ftt = 0; ftt < ft; ftt++)
 {
  s1[ftt] = max_otl[ftt]; //collection of outliers
 }//ftt
 
 } 


}//e
}//if c
}//if max_otl
}//for dd


visualization_msgs::Marker line_list;
line_list.header.frame_id = "base_link";
line_list.header.stamp = ros::Time::now();
line_list.ns = "points_and_lines";
line_list.action = visualization_msgs::Marker::ADD;
line_list.pose.orientation.w = 1.0;

line_list.id = 0;
line_list.type = visualization_msgs::Marker::LINE_LIST;
line_list.scale.x = 0.01;
line_list.color.b = 1.0;
line_list.color.a = 1.0;

geometry_msgs::Point p;

if(c>10)
{
for(fii = 0; fii < (2*e); fii += 2)
{
if(!(isnan(s4[fii].x) || isnan(s4[fii].y) || isnan(s4[fii+1].x) || isnan(s4[fii+1].y)))
{
p.x = s4[fii].x;
p.y = s4[fii].y;
p.z = 0;
line_list.points.push_back(p);
p.x = s4[fii+1].x;
p.y = s4[fii+1].y;
p.z = 0;
line_list.points.push_back(p);
}
}
}//if flag
else
{
p.x = 0;
p.y = 0;
p.z = 0;
line_list.points.push_back(p);
p.x = 0;
p.y = 0;
p.z = 0;
line_list.points.push_back(p);
}
marker_pub.publish(line_list);

ros::spinOnce();
//r.sleep();

}//rosok
}//function        


int main(int argc, char** argv)
{
  
ros::init(argc, argv, "evader");
ros::NodeHandle n;
ros::Subscriber scan_sub = n.subscribe("base_scan", 1, scanCallback);
ros::spin();
return(0);
}
