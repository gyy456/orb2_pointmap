/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include<pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/passthrough.h>
#include<opencv2/core/core.hpp>
#include <pcl/filters/extract_indices.h>
#include <cmath>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/time.h>
#include <pcl/filters/radius_outlier_removal.h>

#include"../../../include/System.h"
#include<tf/transform_broadcaster.h>
#include "../../../include/Converter.h"
#include <nav_msgs/Path.h>
#include "geometry_msgs/PoseStamped.h"
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>


using namespace std;
ros::Publisher pub_pointcloud;
ros::Publisher pose_pub;
nav_msgs::Path rgbd_path;
ros::Publisher RGBD_path_pub;
ros::Publisher ocp_map_topic_pub;
nav_msgs::OccupancyGrid occ_map;
class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}
    
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
    pcl::PointCloud<pcl::PointXYZRGBA> ::Ptr outputMap;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;
    pub_pointcloud= nh.advertise<sensor_msgs::PointCloud2> ("/jackal0/PointCloudOutput", 10);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/pointcloud_mapping/ORB_SLAM/pose", 5);//
    RGBD_path_pub = nh.advertise<nav_msgs::Path>("ORB_SLAM/path",10);//
    ocp_map_topic_pub =nh.advertise<nav_msgs::OccupancyGrid>("ORB_SALM/occpancy_map", 1);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/jackal0/front/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/jackal0/front/depth/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
                    nav_msgs::OccupancyGrid &msg) {
  msg.header.seq = 0;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";

  msg.info.map_load_time = ros::Time::now();
  double map_resolution=0.1;
  msg.info.resolution = map_resolution;

  double x_min, x_max, y_min, y_max;
//   double z_max_grey_rate = 0.05;
//   double z_min_grey_rate = 0.95;
  //? ? ??
//   double k_line =
//       (z_max_grey_rate - z_min_grey_rate) / (thre_z_max - thre_z_min);
//   double b_line =
//       (thre_z_max * z_min_grey_rate - thre_z_min * z_max_grey_rate) /
//       (thre_z_max - thre_z_min);

  if (cloud->points.empty()) {
    ROS_WARN("pcd is empty!\n");
    return;
  }

  for (int i = 0; i < cloud->points.size() - 1; i++) {
    if (i == 0) {
      x_min = x_max = cloud->points[i].x;
      y_min = y_max = cloud->points[i].y;
    }

    double x = cloud->points[i].x;
    double y = cloud->points[i].y;

    if (x < x_min)
      x_min = x;
    if (x > x_max)
      x_max = x;

    if (y < y_min)
      y_min = y;
    if (y > y_max)
      y_max = y;
  }
  // origin的确定
  msg.info.origin.position.x = x_min;
  msg.info.origin.position.y = y_min;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.info.origin.orientation.w = 1.0;
  //设置栅格地图大小
  msg.info.width = int((x_max - x_min) / map_resolution);
  msg.info.height = int((y_max - y_min) / map_resolution);
  //实际地图中某点坐标为(x,y)，对应栅格地图中坐标为[x*map.info.width+y]
  msg.data.resize(msg.info.width * msg.info.height);
  msg.data.assign(msg.info.width * msg.info.height, 0);

  ROS_INFO("data size = %d\n", msg.data.size());

  for (int iter = 0; iter < cloud->points.size(); iter++) {
    int i = int((cloud->points[iter].x - x_min) / map_resolution);
    if (i < 0 || i >= msg.info.width)
      continue;

    int j = int((cloud->points[iter].y - y_min) / map_resolution);
    if (j < 0 || j >= msg.info.height - 1)
      continue;
    // 栅格地图的占有概率[0,100]，这里设置为占据
    msg.data[i + j * msg.info.width] = 100;
    //    msg.data[i + j * msg.info.width] = int(255 * (cloud->points[iter].z *
    //    k_line + b_line)) % 255;
  }
}




void RadiusOutlierFilter(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pcd_cloud0,
                         const double &radius, const int &thre_count) {
  //创建滤波器
  pcl::RadiusOutlierRemoval<pcl::PointXYZRGBA> radiusoutlier;
  //设置输入点云
  radiusoutlier.setInputCloud(pcd_cloud0);
  //设置半径,在该范围内找临近点
  radiusoutlier.setRadiusSearch(radius);
  //设置查询点的邻域点集数，小于该阈值的删除
  radiusoutlier.setMinNeighborsInRadius(thre_count);
  radiusoutlier.filter(*pcd_cloud0);
  // test 保存滤波后的点云到文件
  std::cout << "半径滤波后点云数据点数：" << pcd_cloud0->points.size()
            << std::endl;
}






void cloud_filter(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud_source )
{
    pcl::NormalEstimationOMP<pcl::PointXYZRGBA,pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());
    n.setNumberOfThreads(12);
    n.setInputCloud(cloud_source);
    n.setSearchMethod(tree);
    n.setRadiusSearch(0.04);
    n.compute(*normals);
    // for (int i=1;i<cloud_source->points.size();)
    // {
    //     if (cloud_source->points[i].y>0.05)
    //     {
    //         cloud_source->erase(cloud_source->begin()+i);
    //         // cloud_source->removeIndices(i);
    //     }
    //     else
    //     {
    //         i=i+1;
    //     }
        
    // }
    for (int i=1;i<cloud_source->points.size();i=i+1)
    {   float k=abs( normals->points[i].normal_z)/sqrt(normals->points[i].normal_x*normals->points[i].normal_x+normals->points[i].normal_y*normals->points[i].normal_y);
        if (k>0.2)
        {
           cloud_source->points[i].x=std::numeric_limits <float>::quiet_NaN();
           cloud_source->points[i].y=std::numeric_limits <float>::quiet_NaN();
           cloud_source->points[i].y=std::numeric_limits <float>::quiet_NaN();
        }
       
        
    }
    cloud_source->is_dense =false;
    std::vector<int> maping;
    pcl::removeNaNFromPointCloud(*cloud_source,*cloud_source, maping);
    cloud_source->is_dense =true;

    double thre_radius = 0.1;
    int thres_point_count = 4;
    std::cout << "半径滤波前点云数据点数：" << cloud_source->points.size()<< std::endl;
    RadiusOutlierFilter(cloud_source,thre_radius,thres_point_count);



}


void pointcloudpublish(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& inputMap )
 { 
	//pcl::PointCloud<pcl::PointXYZ> cloud; 	
	sensor_msgs::PointCloud2 output;   //声明的输出的点云的格式
    
    // pcl::removeNaNFromPointCloud(*inputMap,*inputMap, mapping);
	//pcl::toPCLPointCloud2(*p,p2); //转换到 PCLPointCloud2 类型
	//pcl_conversions::fromPCL(p2, output);    //转换到 ros 下的格式
    // if(inputMap.width!=0)
	//     {
    //         pcl::io::savePCDFile ("/home/gyy/Downloads/orb_map/test1.pcd", inputMap); 
    //     } 
    cloud_filter(inputMap);
    SetMapTopicMsg(inputMap,occ_map);
    ocp_map_topic_pub.publish(occ_map);
	toROSMsg(*inputMap,output);
	output.header.stamp=ros::Time::now();
	output.header.frame_id  ="map";
	pub_pointcloud.publish(output);

 }


//转换为栅格地图数据并发布




void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat Tcw;
    Tcw =mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    // mpSLAM->getPointCloudMap(outputMap);
    // if (outputMap->points.size()>0)
    {   
        mpSLAM->getPointCloudMap(outputMap);


        pointcloudpublish(outputMap);
    }
    cv::Mat T2=cv::Mat::eye(4,4,CV_32F);
    T2.at<float>(1,1)=0;
    T2.at<float>(1,2)=1;
    T2.at<float>(2,1)=-1;
    T2.at<float>(2,2)=0;
    
    // mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id ="map";
    if(!Tcw.empty()){
        // cv::Mat Tcw1=Tcw*T2;
        cv::Mat Rwc = T2.rowRange(0,3).colRange(0,3)*Tcw.rowRange(0,3).colRange(0,3).t(); // Rotation information
        cv::Mat twc =-Rwc*Tcw.rowRange(0,3).col(3); // translation information
        vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

        tf::Transform new_transform;
        new_transform.setOrigin(tf::Vector3(twc.at<float>(0, 0), twc.at<float>(0, 1), twc.at<float>(0, 2)));

        tf::Quaternion quaternion(q[0], q[1], q[2], q[3]);
        new_transform.setRotation(quaternion);

        tf::poseTFToMsg(new_transform, pose.pose);
        pose_pub.publish(pose);
        rgbd_path.header.frame_id="map";
        rgbd_path.header.stamp=ros::Time::now();
        rgbd_path.poses.push_back(pose);
        RGBD_path_pub.publish(rgbd_path);

    }
}


