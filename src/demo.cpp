#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

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
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *point_cloud);
  pcl_ros::transformPointCloud(*point_cloud, *point_cloud, transform);
  int points_size = point_cloud->size();
  // Project the point cloud to the grid map
  //这个一维数组按照栅格地图从左到右、从上到下的顺序存储了每个格子的值。
  
  if (grid_map_msg) 
  {
    //栅格地图的数组
    
    std::vector<signed char> grid_data = grid_map_msg->data;
    float grid_resolution = grid_map_msg->info.resolution;
    const int grid_width = grid_map_msg->info.width;
    const int grid_height = grid_map_msg->info.height;
    float grid_origin_x = grid_map_msg->info.origin.position.x;
    float grid_origin_y = grid_map_msg->info.origin.position.y;
    const int size = grid_width * grid_height;
    int  hit =0;
    int count = 0;
  //点云数据的数组
 
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr grid_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (int row = 0; row < grid_height; row++) 
    {
        for (int col = 0; col < grid_width; col++) 
        {
            // 获取栅格地图中的值
            int grid_value = grid_array[row][col];
            
            // 判断栅格的状态，决定是否将其转换为点云
            if (grid_value == 0) 
            {
                // 计算点的坐标（假设栅格的中心为点的位置）
                float x = col * grid_resolution + grid_origin_x;
                float y = row * grid_resolution + grid_origin_y;
                float z = 0.0;  // 栅格地图通常没有高度信息，设置为0
                
                // 创建一个点并添加到点云中
                pcl::PointXYZ point(x, y, z);
                grid_cloud->push_back(point);
            }
        }
    }


  //进行匹配程度评估
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(grid_cloud);
    voxel_grid.setLeafSize(0.01f, 0.01f, 0.01f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_grid.filter(*downsampled_source_cloud);

    voxel_grid.setInputCloud(point_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_grid.filter(*downsampled_target_cloud);

    // 创建ICP对象并设置参数
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaxCorrespondenceDistance(0.05);  // 设置最大对应点距离
    icp.setMaximumIterations(50);  // 设置最大迭代次数
    icp.setTransformationEpsilon(1e-8);  // 设置变换阈值
    icp.setEuclideanFitnessEpsilon(0.01);  // 设置收敛条件

    // 执行配准
    icp.setInputSource(downsampled_source_cloud);
    icp.setInputTarget(downsampled_target_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*aligned_cloud);

    // 获取配准结果
    Eigen::Matrix4f transformation = icp.getFinalTransformation();
    bool converged = icp.hasConverged();
    double fitness_score = icp.getFitnessScore();
    ROS_INFO_STREAM("fitness_score:"<<fitness_score);

  
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



