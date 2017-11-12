/*
	Torso_detection code
	implemented by Vineeth Rajamohan, Santosh

*/


// necessary header files
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <cmath>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>
#include <math.h>
#include <string.h>

// pcl header files
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

//sensor_msgs header for point cloud2
#include <sensor_msgs/PointCloud2.h>

// eigen library
#include <Eigen/Dense>
using Eigen::MatrixXd;

unsigned int num_points = 100;
using sensor_msgs::PointCloud;

// class LaserSacnToPointCloud 
// objective: to convert laser scan to point cloud
// implemented by Santosh 
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

// class segment 
//implemented by vineeth Rajamohan
class segment{

 public:
	// publisher object	
	ros::Publisher cloud_pub_;

	//constructor to initialize publisher marker and cloud
 	segment( const ros::Publisher& marker_pub,const ros::Publisher& cloud_pub )
				:marker_pub_(marker_pub)
	{
		cloud_pub_ = cloud_pub;
	}

	// call back function
    void segmentCallBack(const sensor_msgs::PointCloud:: ConstPtr& msg)
	{

		// pcl and sensor_msgs objects
		pcl::PointCloud<pcl::PointXYZ> cloud;
		sensor_msgs::PointCloud2 output;

		// cloud setup
		cloud.width  = 50000;
    	cloud.height = 2;
   		cloud.points.resize(cloud.width * cloud.height);

   		// marker setup
		visualization_msgs::Marker points;
    	points.header.frame_id = "base_link";
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

				// cal distance between two points(consecutive)
				distance = distanceCal( x,y, prev_x,prev_y);

				// is distance is > 0.5m
				if(distance > 0.05 )
				{	
					// point object 
					geometry_msgs::Point pi;

					// store the first points at the first instance
					if( check == false)
					{	
						int_store1 = i;
						pi.x = x;
						pi.y = y;
						pi.z = z;
						points.points.push_back(pi);	
						check = !check;
					}

					// store the end points at the second instance
					else
					{
						int_store2 = i;
						pi.x = prev_x;
						pi.y= prev_y;
						pi.z= prev_z;
						points.points.push_back(pi);
						check = !check;
						check2 = true;
					}
					
					// if both are true 
					if(check2 == true)
					{
						// val function to cal the length of the segment 
						val= abs(int_store2 - int_store1);

						// if segment length is greater than 2 and less than 15 keep it
						// discard anything else
						if(val >=2 && val < 15)
						{
							ROS_INFO("start");
							//ROS_INFO("value1: %d, value2: %d", int_store1,int_store2);
							// store the necessary points to the pointcloud
							if(int_store2 > int_store1)
							{
								for(int i = int_store1; i <= int_store2; i++)
								{
								
									cloud.points[i].x = msg->points[i].x;
									cloud.points[i].y = msg->points[i].y;
									cloud.points[i].z = msg->points[i].z;
									store_x[adder] = msg->points[i].x;
									store_y[adder] = msg->points[i].y;
									ROS_INFO(" xvalue: %f, yvalue: %f, zvalue: %f", msg->points[i].x, msg->points[i].y, msg->points[i].z);
									adder++;
								}
							}
							if(int_store1 > int_store2)
							{
								for(int i = int_store2; i <= int_store1; i++)
								{
								
									cloud.points[i].x = msg->points[i].x;
									cloud.points[i].y = msg->points[i].y;
									cloud.points[i].z = msg->points[i].z;
									store_x[adder] = msg->points[i].x;
									store_y[adder] = msg->points[i].y;
									ROS_INFO(" xvalue: %f, yvalue: %f, zvalue: %f", msg->points[i].x, msg->points[i].y, msg->points[i].z);
									adder++;
								}
							}

							ROS_INFO("end");

							// adder is the no of points
							// dimension is set to 2

							MatrixXd p(dimension,adder);
							for(int i = 0; i<adder;i++)
							{
								p(0,i) = store_x[i];
								p(1,i) = store_y[i];
							}
							MatrixXd q = p;
							q.conservativeResize(p.rows() + 1,p.cols());
							
							for(size_t i = 0;i<q.cols();i++)
							{
								q(q.rows() - 1,i) = 1;
							}

							int count = 1;
							double err = 1;
							const double init_u = 1.0 / (double) adder;
							MatrixXd u = MatrixXd::Constant(adder,1,init_u);

							while(err > tolerance)
							{
								MatrixXd Q_tr = q.transpose();
								MatrixXd X = q * u.asDiagonal() * Q_tr;
								MatrixXd M = (Q_tr * X.inverse() * q).diagonal();

								int j_x, j_y;
								double maximum = M.maxCoeff(&j_x, &j_y);
								double step_size = (maximum - dimension - 1) / ((dimension + 1) * (maximum +1));

								MatrixXd new_u = (1 - step_size) * u;
								new_u(j_x,0) += step_size;

								// Find err
								MatrixXd u_diff = new_u - u;
								for(size_t i =0; i < u_diff.rows();i++)
								{
									for(size_t j =0; j< u_diff.cols();j++)
									{
										u_diff(i, j) *= u_diff(i,j);
									}
									err = sqrt(u_diff.sum());
									count++;
									u = new_u;
								}
							}

							MatrixXd U = u.asDiagonal();
							//MatrixXd A = (1.0 / (double) dimension) * (p * U * p.transpose() - (p * u) * (p * u).transpose()).inverse();
							MatrixXd temp =  (p* u).transpose();
							MatrixXd temp2 = (p*u) * temp;
							MatrixXd temp3 = (p* U* p.transpose() - temp2).inverse();
							MatrixXd temp4 = (1.0/(double) dimension) * temp3;
							//ROS_INFO(" %f, %f", A(0,0), A(0,1));
							MatrixXd c = p * u;
							//cout<<endl;
							//cout<<endl;
							ROS_INFO("value_xx %f value_xy %f value_yy %f", temp4(0,0), temp4(0,1), temp4(1,1));
							ROS_INFO("shift_x %f shift_y %f", c(0,0), c(1,0));

							adder = 0;
						}
						


					} 
				}
								
			}
			
			
		} 
		// pcl to ROS msg 
		pcl::toROSMsg(cloud, output);
		//set frame ID
    	output.header.frame_id = "base_link";
		
		// publish marker and cloud
        marker_pub_.publish(points); 
		cloud_pub_.publish(output);
        
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

    double major_axis = 0.0;
    double minor_axis = 0.0;

    double tolerance = 0.2;
    double center_x = 0.0;
    double center_y = 0.0;
	int int_store1 = 0;
	int int_store2 = 0;
	int adder = 0;
	int dimension = 2;
	bool check = false;
	bool check2 = false;
	double val = 0.0;

	double store_x[100];
	double store_y[100];

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
  ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud2> ("cloud", 50);
  segment ptr(marker_pub, cloud_pub);
  ros::Subscriber sub = n.subscribe("/my_cloud",1000, &segment::segmentCallBack,&ptr);  
 
  ros::spin();
  
  return 0;
}












/*

#include <ros/ros.h>

// point cloud headers
#include <pcl/point_cloud.h>
//Header which contain PCL to ROS and ROS to PCL conversion functions
#include <pcl_conversions/pcl_conversions.h>

//sensor_msgs header for point cloud2
#include <sensor_msgs/PointCloud2.h>

main (int argc, char **argv)
{
    ros::init (argc, argv, "pcl_create");


    ROS_INFO("Started PCL publishing node");

    ros::NodeHandle nh;

//Creating publisher object for point cloud

    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);

//Creating a cloud object
    pcl::PointCloud<pcl::PointXYZ> cloud;

//Creating a sensor_msg of point cloud

    sensor_msgs::PointCloud2 output;

    //Insert cloud data
    cloud.width  = 50000;
    cloud.height = 2;
    cloud.points.resize(cloud.width * cloud.height);

//Insert random points on the clouds

    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
        cloud.points[i].x = 512 * rand () / (RAND_MAX + 1.0f);
        cloud.points[i].y = 512 * rand () / (RAND_MAX + 1.0f);
        cloud.points[i].z = 512 * rand () / (RAND_MAX + 1.0f);
    }

    //Convert the cloud to ROS message
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "point_cloud";

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        //publishing point cloud data
      pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

*/