/*
 * listen.cpp
 *
 *  Created on: Jun 10, 2012
 *      Author: daniel
 */

#include "ros/ros.h"

#include <string>

#include <pcl-1.5/pcl/point_types.h>
#include <pcl-1.5/pcl/io/pcd_io.h>

#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"

#include "boost/date_time/posix_time/posix_time.hpp"
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/algorithm/string/erase.hpp>

typedef pcl::PointXYZRGBA Point;


void callBack(const sensor_msgs::PointCloud2::ConstPtr msg)
{
	boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();

	std::string filename = "Capture/v2_" +  boost::lexical_cast<std::string,boost::posix_time::ptime>( now ) +  ".pcd";

	boost::algorithm::erase_all(filename,":");

	std::cout << filename << std::endl;

	boost::shared_ptr<pcl::PointCloud<Point> > pointCloud(new pcl::PointCloud<Point>);

	pcl::io::savePCDFileBinary<Point>(filename,*pointCloud);

	ROS_INFO("Height: [%i]",msg->height);
	ROS_INFO("Width: [%i]",msg->width);

}

int main(int argc,char ** argv)
{
	ros::init(argc,argv,"listen");
	ros::NodeHandle nodeHandle;
	ros::Subscriber subscripber = nodeHandle.subscribe("/camera/depth/points",20,callBack);
	ros::spin();

	return 0;
}


