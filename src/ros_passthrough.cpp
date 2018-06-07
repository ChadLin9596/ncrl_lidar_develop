#include <ros/ros.h>
#include <ncrl_lidar_develop/pcl_lb.h>

PointCloudXYZ::Ptr    cloudxyz    (new PointCloudXYZ);
PointCloudXYZ::Ptr    cloud_filter(new PointCloudXYZ);

ros::Publisher pub_XYZ;
bool lock = false;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  if (!lock){
    lock = true;
    ROS_INFO("Pass Through Filter Operating");
    // start to msg conversion
    pcl::fromROSMsg (*input, *cloudxyz); //covert from ros type to pcl type

    //create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloudxyz);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.0,0.5);
    pass.filter(*cloud_filter);

    pub_XYZ.publish(*cloud_filter);
    // end process
  }
  else
  {
    std::cout << "lock" << std::endl;
  }
  lock = false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_passthrough");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 1, cloud_cb);
  pub_XYZ = nh.advertise<PointCloudXYZ> ("/pcl_filter", 1);
  ros::spin();
}
