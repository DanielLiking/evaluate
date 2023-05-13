#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float32.h>
nav_msgs::OccupancyGridConstPtr grid_map_msg;
ros::Publisher result_pub;
void grid_map_callback(const nav_msgs::OccupancyGridConstPtr& msg) 
{
  grid_map_msg = msg;
}

void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) 
{
  // Define the transform listener and transform broadcaster
  static tf::TransformListener listener;

  // Lookup the transform from the base frame to the map frame
  tf::StampedTransform transform;
  try 
  {
    listener.waitForTransform("map", "base_link", cloud_msg->header.stamp, ros::Duration(1.0));
    listener.lookupTransform("map", "base_link", cloud_msg->header.stamp, transform);
  }
  catch (tf::TransformException& ex) 
  {
    ROS_ERROR_STREAM("Transform error: " << ex.what());
    return;
  }

  // Transform the point cloud to the map frame
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *cloud);
  pcl_ros::transformPointCloud(*cloud, *cloud, transform);
  int points_size = cloud->size();
  // Project the point cloud to the grid map
  //这个一维数组按照栅格地图从左到右、从上到下的顺序存储了每个格子的值。
  
  if (grid_map_msg) 
  {
    //
    std::vector<signed char> grid_data = grid_map_msg->data;
    float grid_resolution = grid_map_msg->info.resolution;
    const int grid_width = grid_map_msg->info.width;
    const int grid_height = grid_map_msg->info.height;
    float grid_origin_x = grid_map_msg->info.origin.position.x;
    float grid_origin_y = grid_map_msg->info.origin.position.y;
    const int size = grid_width * grid_height;
    int  hit =0;
    int count = 0;

    std::vector<std::vector<int>> grid_array(grid_height,std::vector<int>(grid_width));
    for(int i = 0; i< grid_height; i++)
    {
      for(int j =0; j<grid_width; j++)
      {
        int index = i * grid_width + j;
        if (index < 0 || index > size) continue;
        grid_array[i][j] = grid_data[index];
      }

    }
    // std::ofstream outfile;
    // outfile.open("/home/tjark/array.txt");
    // for(int i=0; i<grid_height; i++) 
    // {
    //     for(int j=0; j<grid_width; j++) 
    //     {
    //         outfile << grid_array[i][j]<< " ";
    //     }
    //     outfile << "\n";
    // }

    // // 关闭文件
    // outfile.close();

    // 使用二维数组 grid_data 进行处理
    // std::vector<std::vector<int>> laser(grid_height, std::vector<int>(grid_width, 1));
    for (const auto& point : cloud->points) 
    {
      int x = (point.x - grid_origin_x) / (grid_resolution);
      int y = (point.y - grid_origin_y) / (grid_resolution);
      // ROS_INFO_STREAM("x:"<<x<<"point.x"<<" "<<point.x);
      // ROS_INFO_STREAM("y:"<<y<<"point.y"<<" "<<point.y);
      if(x < 0 || x >= grid_width || y < 0 ||y >= grid_height)
      {
        // ROS_INFO_STREAM("Invalidate parameter");
        continue;
      }
      
      count = 0;
      for (int i = -1; i <= 1; i++) 
      {
          for (int j = -1; j <= 1; j++) 
          {
              // if (i == 0 && j == 0 ) continue;
              int nx = x + i;
              int ny = y + j;
              // laser[nx][ny] = 0;
              if(nx < 0 || nx >= grid_width || ny < 0 ||ny >= grid_height)  continue;
              if (grid_array[nx][ny] <= 0)
              {
                count++;
              } 
          }
      }
       if(count>=3)
      {
          hit++;
      }
    
    // std::ofstream outfile1;
    // outfile1.open("/home/tjark/laser.txt");
    // for(int i=0; i<grid_height; i++) 
    // {
    //     for(int j=0; j<grid_width; j++) 
    //     {
    //         outfile1 << laser[i][j]<< " ";
    //     }
    //     outfile << "\n";
    // } 
    // outfile1.close(); 
} 
  
    // ROS_INFO_STREAM("hit:"<<hit);
    // ROS_INFO_STREAM(" points_size:"<< points_size);
    float precision = ((float)hit/(float) points_size)*100;
    std_msgs::Float32 precision_msg;
    precision_msg.data = precision;
    result_pub.publish(precision_msg);
    


  }
}
   

int main(int argc, char** argv) {
  ros::init(argc, argv, "my_node");
  ros::NodeHandle nh;
  // Create a publisher for the result
  result_pub = nh.advertise<std_msgs::Float32>("/result_topic", 10);
  // Subscribe to the grid map topic
  ros::Subscriber grid_map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 10, grid_map_callback);

  // Subscribe to the point cloud topic
  ros::Subscriber pointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/scan_matched_points2", 10, pointcloud_callback);

  // Spin the node
  ros::spin();

  return 0;
}