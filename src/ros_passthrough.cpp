#include <ros/ros.h>
#include <ncrl_lidar_develop/pcl_lb.h>

PointCloudXYZ::Ptr    cloudxyz    (new PointCloudXYZ);
PointCloudXYZ::Ptr    cloud_filter(new PointCloudXYZ);
PointCloudXYZRGB::Ptr cloudxyzrgb (new PointCloudXYZRGB);

bool lock = false;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  if (!lock){
    lock = true;

    // start to msg conversion
    pcl::fromROSMsg (*input, *cloudxyz); //covert from ros type to pcl type
    cloudxyz->width = 5;
    cloudxyz->height = 1;
    cloudxyz->points.resize(cloudxyz->width * cloudxyz->height);

    for (size_t i = 0; i < cloudxyz->points.size(); ++i)
    {
      cloudxyz->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
      cloudxyz->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
      cloudxyz->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }

    std::cerr << "Cloud before filtering: " << std::endl;
    for (size_t i = 0; i < cloudxyz->points.size (); ++i)
    std::cerr << "    " << cloudxyz->points[i].x << " "
                        << cloudxyz->points[i].y << " "
                        << cloudxyz->points[i].z << std::endl;

    //create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloudxyz);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0,1.0);
    pass.filter(*cloud_filter);

    std::cerr << "Cloud after filtering: " << std::endl;
    for (size_t i = 0; i < cloud_filter->points.size (); ++i)
    std::cerr << "    " << cloud_filter->points[i].x << " "
                        << cloud_filter->points[i].y << " "
                        << cloud_filter->points[i].z << std::endl;
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
  ros::spin();
}
