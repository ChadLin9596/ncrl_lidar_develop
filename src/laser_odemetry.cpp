#include <ros/ros.h>
#include <ncrl_lidar_develop/pcl_lb.h>
//define point cloud type

float distance = 0;
float range = 0.7;
//declare point cloud pointer
PointCloudXYZ::Ptr cloud_XYZ (new PointCloudXYZ);
PointCloudXYZRGB::Ptr cloud_XYZRGB (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr result (new PointCloudXYZRGB);
sensor_msgs::PointCloud2 ros_output;

//declare publisher
ros::Publisher pub_XYZRGB;
ros::Publisher pub_avoid;

//declare global variable
bool lock = false;
void pointcloud_processing(void);
geometry_msgs::Point xyz;

//call back function
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  if (!lock){
    lock = true;

    pcl::fromROSMsg (*input, *cloud_XYZ); //covert from ros type to pcl type
    copyPointCloud(*cloud_XYZ, *cloud_XYZRGB); //it will include frame information

    for (size_t i = 0; i < cloud_XYZRGB->points.size(); i++)
    {
        cloud_XYZRGB->points[i].r = 255;
        cloud_XYZRGB->points[i].g = 255;
        cloud_XYZRGB->points[i].b = 255;
    }
    pointcloud_processing();
  }
  else
  {
    std::cout << "lock" << std::endl;
  }
}

void getcircle()
{
  int count = 1;
  xyz.x = 0;
  xyz.y = 0;
  for (size_t i = 0; i < cloud_XYZRGB->points.size(); i++)
  {
    float X = cloud_XYZRGB->points[i].x;
    float Y = cloud_XYZRGB->points[i].y;
    float Z = cloud_XYZRGB->points[i].z;
    distance = sqrt(X*X + Y*Y);
    if (distance < range && cloud_XYZRGB->points[i].z > 0)
    {
      count += 1;
      cloud_XYZRGB->points[i].r = 255;
      cloud_XYZRGB->points[i].g = 0;
      cloud_XYZRGB->points[i].b = 0;
      xyz.x += X;
      xyz.y += Y;
    }
  }
  xyz.x = xyz.x/count;
  xyz.y = xyz.y/count;
}

//void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
void pointcloud_processing()
{
  getcircle();
  ROS_INFO("x: %f y: %f",xyz.x,xyz.y);

  pub_XYZRGB.publish(*cloud_XYZRGB);
  pub_avoid.publish(xyz);
  lock = false;
}

int main (int argc, char** argv)
{
     // Initialize ROS
     ros::init (argc, argv, "laser_odemetry");
     ros::NodeHandle nh;

     // Create a ROS subscriber for the input point cloud
     ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 1, cloud_cb);

     // Create a ROS publisher for the output point cloud
     pub_XYZRGB = nh.advertise<PointCloudXYZRGB> ("/output_pcl", 1);
     pub_avoid = nh.advertise<geometry_msgs::Point> ("/velodyne/obstacle",1);

     // Spin
     ros::spin ();
}
