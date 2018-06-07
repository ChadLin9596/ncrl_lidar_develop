#include <ros/ros.h>
#include <ncrl_lidar_develop/pcl_lb.h>

PointCloud2::Ptr    cloud       (new PointCloud2);
PointCloud2::Ptr    cloud_filter(new PointCloud2);

ros::Publisher pub_XYZ;
bool lock = false;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  if (!lock){
    lock = true;
    // start to msg conversion
    ROS_INFO("Voxel Grid Filter Operating");
    pcl_conversions::toPCL (*input, *cloud); //covert from ros type to pcl type

    // VoxelGrid filter
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.05f, 0.05f, 0.05f);
    sor.filter (*cloud_filter);
    //pcl_conversions::fromPCL()

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
  ros::init(argc, argv, "ros_voxelgrid");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 1, cloud_cb);
  pub_XYZ = nh.advertise<PointCloud2> ("/pcl_voxel", 1);
  ros::spin();
}
