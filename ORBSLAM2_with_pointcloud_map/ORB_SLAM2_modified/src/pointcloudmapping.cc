/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#include "pointcloudmapping.h"
#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include "Converter.h"
#include "cmath"
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>




#include <pcl/search/kdtree.h>
#include <pcl/filters/radius_outlier_removal.h>

PointCloudMapping::PointCloudMapping(double resolution_)
{
    this->resolution = resolution_;
    voxel.setLeafSize( 0.02f, 0.02f, 0.02f);
    globalMap = boost::make_shared< PointCloud >( );
    p = boost::make_shared< PointCloud >( );  //gyy的修改
    viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );
}
// void PointCloudMapping::RadiusOutlierFilter(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pcd_cloud0,
//                          const double &radius, const int &thre_count) {
//   //创建滤波器
//   pcl::RadiusOutlierRemoval<pcl::PointXYZRGBA> radiusoutlier;
//   //设置输入点云
//   radiusoutlier.setInputCloud(pcd_cloud0);
//   //设置半径,在该范围内找临近点
//   radiusoutlier.setRadiusSearch(radius);
//   //设置查询点的邻域点集数，小于该阈值的删除
//   radiusoutlier.setMinNeighborsInRadius(thre_count);
//   radiusoutlier.filter(*pcd_cloud0);
//   // test 保存滤波后的点云到文件
//   std::cout << "半径滤波后点云数据点数：" << pcd_cloud0->points.size()
//             << std::endl;
// }


// void PointCloudMapping::cloud_filter(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud_source )
// {
//     pcl::NormalEstimationOMP<pcl::PointXYZRGBA,pcl::Normal> n;
//     pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//     pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());
//     n.setNumberOfThreads(12);
//     n.setInputCloud(cloud_source);
//     n.setSearchMethod(tree);
//     n.setRadiusSearch(0.1);
//     n.compute(*normals);
//     // for (int i=1;i<cloud_source->points.size();)
//     // {
//     //     if (cloud_source->points[i].y>0.05)
//     //     {
//     //         cloud_source->erase(cloud_source->begin()+i);
//     //         // cloud_source->removeIndices(i);
//     //     }
//     //     else
//     //     {
//     //         i=i+1;
//     //     }
        
//     // }
//     for (int i=1;i<cloud_source->points.size();i=i+1)
//     {   float k=abs( normals->points[i].normal_y)/sqrt(normals->points[i].normal_x*normals->points[i].normal_x+normals->points[i].normal_z*normals->points[i].normal_z);
//         if (0.2<k&&k<100)
//         {
//            cloud_source->points[i].x=std::numeric_limits <float>::quiet_NaN();
//            cloud_source->points[i].y=std::numeric_limits <float>::quiet_NaN();
//            cloud_source->points[i].y=std::numeric_limits <float>::quiet_NaN();
//         }
       
        
//     }
//     cloud_source->is_dense =false;
//     std::vector<int> maping;
//     pcl::removeNaNFromPointCloud(*cloud_source,*cloud_source, maping);
//     cloud_source->is_dense =true;

//     double thre_radius = 0.1;
//     int thres_point_count = 20;
//     RadiusOutlierFilter(cloud_source,thre_radius,thres_point_count);



// }





void PointCloudMapping::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    viewerThread->join();
}

void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    cout<<"receive a keyframe, id = "<<kf->mnId<<endl;
    unique_lock<mutex> lck(keyframeMutex);
    keyframes.push_back( kf );
    colorImgs.push_back( color.clone() );
    depthImgs.push_back( depth.clone() );
    
    keyFrameUpdated.notify_one();
}

pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    PointCloud::Ptr tmp( new PointCloud() );
    // point cloud is null ptr
    for ( int m=0; m<depth.rows; m+=3 )
    {
        for ( int n=0; n<depth.cols; n+=3 )
        {
            float d = depth.ptr<float>(m)[n];
            if ( d>2 ||d<0.01)
                {continue;}
            PointT p;
            p.z = d;
            p.x = ( n - kf->cx) * p.z / kf->fx;
            p.y = ( m - kf->cy) * p.z / kf->fy;
            
            p.b = color.ptr<uchar>(m)[n*3];
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.r = color.ptr<uchar>(m)[n*3+2];
                
            tmp->points.push_back(p);
        }
    }
    
    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
    PointCloud::Ptr cloud(new PointCloud);
    pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
    cloud->is_dense = false;
    Eigen::Isometry3d T1=Eigen::Isometry3d::Identity();
    T1(1,1)=0;
    T1(1,2)=1;
    T1(2,1)=-1;
    T1(2,2)=0;
    pcl::transformPointCloud( *cloud, *tmp, T1.matrix());
    cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<tmp->points.size()<<endl;
    return tmp;
}







void PointCloudMapping::viewer()
{
    pcl::visualization::CloudViewer viewer("viewer");
    while(1)
    {
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
            keyFrameUpdated.wait( lck_keyframeUpdated );
        }
        
        // keyframe is updated 
        size_t N=0;
        {
            unique_lock<mutex> lck( keyframeMutex );
            N = keyframes.size();
        }
        
        for ( size_t i=lastKeyframeSize; i<N ; i++ )
        {   
            p = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );
            // if(i==1)
            // {   pcl::PointCloud<pcl::PointXYZRGBA> ::Ptr cloud;
            //     cloud=p;
            //     cloud->height=1;
            //     cloud->width=36594;
            //     std::vector<int> mapping;
            //     pcl::removeNaNFromPointCloud(*cloud, *cloud, mapping);
            //     // cout<<a<<endl;
            //     pcl::io::savePCDFile ("/home/gyy/Downloads/pcd_data/test2.pcd",*cloud);
            // }
            // if(i==165)
            // {   pcl::PointCloud<pcl::PointXYZRGBA> ::Ptr cloud;
            //     cloud=globalMap;
            //     // cloud->height=1;
            //     // cloud->width=3516891;
            //     std::vector<int> mapping;
            //     pcl::removeNaNFromPointCloud(*cloud,*cloud, mapping);
            //     pcl::io::savePCDFile ("/home/gyy/Downloads/pcd_data/global170.pcd",*cloud);
            // }

        // PointCloud::Ptr tmp(new PointCloud());
        voxel.setInputCloud( p );
        voxel.filter( *p );
        std::vector<int> mapping;
        pcl::removeNaNFromPointCloud(*p,*p, mapping); 
        cout<<"show loacl map, size="<<p->points.size()<<endl;  
        *globalMap += *p;
        }
        viewer.showCloud( globalMap );
        cout<<"show global map, size="<<globalMap->points.size()<<endl;
        lastKeyframeSize = N;
    }
}

void PointCloudMapping::getGlobalCloudMap(pcl::PointCloud<pcl::PointXYZRGBA> ::Ptr &outputMap)
{
	   unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );   

       outputMap=p;

    //    pcl::removeNaNFromPointCloud(*outputMap,*outputMap, maping);
       
}