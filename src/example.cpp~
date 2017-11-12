#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include<cmath>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include<math.h>
#include<string.h>


 

class LaserScanToPointCloud{

public:

  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher scan_pub_;

  LaserScanToPointCloud(ros::NodeHandle n) : 
    n_(n),
    laser_sub_(n_, "base_scan", 10),
    laser_notifier_(laser_sub_,listener_, "base_link", 10)
  {
    laser_notifier_.registerCallback(
      boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.01));
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/my_cloud",1);
  }

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    sensor_msgs::PointCloud cloud;
    try
    {
        projector_.transformLaserScanToPointCloud(
          "base_link",*scan_in, cloud,listener_);
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }
    
    // Do something with cloud.

    scan_pub_.publish(cloud);

  }
};

class segment{

 public:
	
 	segment( const ros::Publisher& marker_pub)
				:marker_pub_(marker_pub)
	{

	}
    void segmentCallBack(const sensor_msgs::PointCloud:: ConstPtr& msg)
	{
		
		visualization_msgs::Marker points;
    	points.header.frame_id = "/base_link";
		points.header.stamp = ros::Time::now();
		points.ns = "points";
		points.action = visualization_msgs::Marker::ADD;
		points.pose.orientation.w = 1.0;

		points.id = 0;		
	
		points.type = visualization_msgs::Marker::POINTS;

		points.scale.x =0.02;
		points.scale.y =0.02;

		points.color.g = 1.0f;
		points.color.a = 1.0;

		
		for(int i =0; i< 1040;i++)
		{
			if( i!=0)
			{
			 	x = msg->points[i].x;
			 	y = msg->points[i].y;
				z = msg->points[i].z;
				prev_x = msg->points[i-1].x;
				prev_y = msg->points[i-1].y;
				prev_z = msg->points[i-1].z;
				distance = distanceCal( x,y, prev_x,prev_y);

				if(distance > 0.05 )
				{	
					geometry_msgs::Point p;
					if( check == false)
					{	
						int_store1 = i;
						p.x = x;
						p.y = y;
						p.z = z;
						points.points.push_back(p);	
						check = !check;
					}
					else
					{
						int_store2 = i;
						p.x = prev_x;
						p.y= prev_y;
						p.z= prev_z;
						points.points.push_back(p);
						check = !check;
					}
					ROS_INFO("segment value: %d ", abs(int_store2 - int_store1));
				}
								
			}
			
			
		}
		
		
        marker_pub_.publish(points); 
		
        
	}
   
    

private:
	ros::NodeHandle n_;
	double x =0.0;
	double y = 0.0;
	double z = 0.0;
	double prev_x = 0.0;
	double prev_y = 0.0;
	double prev_z = 0.0;
    double distance = 0.0;
	int int_store1 = 0;
	int int_store2 = 0;
	bool check = false;
	ros::Publisher marker_pub_;
	double distanceCal(double curr_x,double curr_y, double before_x, double before_y)
	{
		double val = 0.0;
		val = (pow((curr_x - before_x),2)) + (pow((curr_y - before_y),2));
		val = sqrt(val);
		return val;
	} 


};



int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "my_pcl_tutorial");
  ros::NodeHandle n;
  LaserScanToPointCloud lstopc(n);
  ros:: Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visual_marker", 10);
  segment ptr(marker_pub);
  ros::Subscriber sub = n.subscribe("/my_cloud",1000, &segment::segmentCallBack,&ptr);  
 
  ros::spin();
  
  return 0;
}

