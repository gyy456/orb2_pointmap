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
#include <std_msgs/Int16.h>
#include"../../../include/System.h"

using namespace std;
ros::Publisher pub_pointcloud;
// ros::Publisher id_pub;
// std_msgs::Int16 tt;
class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}
    
    void GrabRGBD_1(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
    pcl::PointCloud<pcl::PointXYZRGBA> ::Ptr outputMap;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD_1");
    ros::start();
    // tt.data=1
    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM_1(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb_1(&SLAM_1);

    ros::NodeHandle nh;
    pub_pointcloud= nh.advertise<sensor_msgs::PointCloud2> ("/jackal1/PointCloudOutput", 10);
    // id_pub= nh.advertise<std_msgs::Int16> ("/jackal1/key_id", 10);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/jackal1/front/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/jackal1/front/depth/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol_1;
    message_filters::Synchronizer<sync_pol_1> sync(sync_pol_1(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD_1,&igb_1,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM_1.Shutdown();

    // Save camera trajectory
    SLAM_1.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}


void pointcloudpublish(pcl::PointCloud<pcl::PointXYZRGBA> inputMap )
 { 
	//pcl::PointCloud<pcl::PointXYZ> cloud; 	
    // std::vector<int> mapping;
    // pcl::removeNaNFromPointCloud(inputMap,inputMap, mapping);
	sensor_msgs::PointCloud2 output1;   //声明的输出的点云的格式
	//pcl::toPCLPointCloud2(*p,p2); //转换到 PCLPointCloud2 类型
	//pcl_conversions::fromPCL(p2, output);    //转换到 ros 下的格式
    // if(inputMap.width!=0)
	//     {pcl::io::savePCDFile ("/home/gyy/Downloads/orb_map/test2.pcd", inputMap);}  
	toROSMsg(inputMap,output1);
	output1.header.stamp=ros::Time::now();
	output1.header.frame_id  ="jackal1/base_link";
	pub_pointcloud.publish(output1);
    // id_pub.publish(tt);
 }


void ImageGrabber::GrabRGBD_1(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
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

    mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    mpSLAM->getPointCloudMap(outputMap);
    pointcloudpublish(*outputMap);
    // tt.data=tt.data+1
}


