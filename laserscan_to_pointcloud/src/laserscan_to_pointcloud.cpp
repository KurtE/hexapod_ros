#include <laserscan_to_pointcloud.h>
#include <ros/package.h>

//==============================================================================
// Constructor
//==============================================================================

LaserScanFilter::LaserScanFilter(){
        scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 100, &LaserScanFilter::scanCallback, this);
        point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/cloud", 100, false);
        //tfListener_.setExtrapolationLimit(ros::Duration(0.1));
        //tfListener_.setTolerance(ros::Duration(0.1));
}

//==============================================================================
// Callback function
//==============================================================================
void LaserScanFilter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    sensor_msgs::PointCloud2 cloud;
    try
    {
      projector_.transformLaserScanToPointCloud("base_link", *scan, cloud, tfListener_);
    }
    catch (tf::TransformException& e)
    {
      ROS_WARN("Exception: %s", e.what());
    }
    point_cloud_publisher_.publish(cloud);
}


//==============================================================================
// Callback function
//==============================================================================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "LaserScanFilter");

    LaserScanFilter filter;

    ros::spin();

    return 0;
}