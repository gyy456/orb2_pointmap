#include <iostream>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include"../../../include/System.h"
#include <string> 
#include <std_msgs/Int16.h>


using namespace std;
int t=1;
pcl::visualization::PCLVisualizer viewer;
int start_fusion=0;
// void keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event, void* nothing)
// {
//     if (event.getKeySym () == "space" && event.keyDown ())
//         next_iteration = true;
// }

void down_sample(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in,
                 const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out)
{
    pcl::ScopeTime time("down sample");

    int down_sample_rate = 8;

    int height = cloud_in->height;
    int width = cloud_in->width;

    int height_ = height / down_sample_rate;
    int width_ = width / down_sample_rate;

    cloud_out->resize(height_ * width_);

    for (int i = 0; i < height; i = i + down_sample_rate)
    {
        for (int j = 0; j < width; j = j + down_sample_rate)
        {
            int index = i * width + j;
            int index_ = i * width / down_sample_rate / down_sample_rate + j / down_sample_rate;

            cloud_out->points[index_] = cloud_in->points[index];
        }
    }

    cloud_out->height = height_;
    cloud_out->width = width_;
}

void remove_nan(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_1,
                const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_2)
{
    cloud_1->is_dense = false;
    cloud_2->is_dense = false;

    std::vector<int> indices_1;
    pcl::removeNaNFromPointCloud(*cloud_1, *cloud_1, indices_1);

    std::vector<int> indices_2;
    pcl::removeNaNFromPointCloud(*cloud_2, *cloud_2, indices_2);

    cloud_1->is_dense = true;
    cloud_2->is_dense = true;
}

void icp_function(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_source,
                  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_target,
                  Eigen::Matrix4d transformation_matrix_raw)
{
    int iterations = 1;
    // int id_key=id->data;

    // std::string s = std::to_string(id_key);

    Eigen::Matrix4d transformation_matrix = transformation_matrix_raw;

    // ICP 算法
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setMaximumIterations (iterations);
    icp.setTransformationEpsilon(1e-10);
    icp.setEuclideanFitnessEpsilon(1e-8); 
    //icp.setMaximumIterations(1);
    icp.setInputSource(cloud_source);
    icp.setInputTarget(cloud_target);
    icp.align(*cloud_source);
    icp.setMaximumIterations(1); 
    if (icp.hasConverged ())
    {
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
        std::cout << "\nRMSE: " << sqrt(icp.getFitnessScore()) << std::endl;
        std::cout << "\nICP transformation " << iterations << " : cloud_source -> cloud_target" << std::endl;

        transformation_matrix *= icp.getFinalTransformation ().cast<double>();
        std::cout << transformation_matrix << std::endl;
    }
    else
    {
        std::cerr << "\nICP has not converged." << std::endl;
    }

    t=t+1;
    std::string s = std::to_string(t);
    // white,c2
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> white(cloud_target, 250, 250, 250);
    viewer.addPointCloud<pcl::PointXYZRGB> (cloud_target, white, "cloud_target"+s);

    // red,c1
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red (cloud_source, 250, 0, 0);
    viewer.addPointCloud (cloud_source, red, "cloud_source"+s);

    // callback
    // viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) nullptr);
    viewer.spinOnce ();
    
    
        // ;

        // if you pressed "space"
        if (t<20)
        {   t=t+1;
            icp.align (*cloud_source);

            if (icp.hasConverged ())
            {
                // printf ("\033[15A");
                std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
                std::cout << "\nRMSE: " << sqrt(icp.getFitnessScore()) << std::endl;
                std::cout << "\nICP transformation " << ++iterations << " : cloud_source -> cloud_target" << std::endl;

                transformation_matrix *= icp.getFinalTransformation ().cast<double>();
                std::cout << transformation_matrix << std::endl;

                viewer.updatePointCloud (cloud_source, red, "cloud_source");
            }
            else
            {
                std::cerr << "\nICP has not converged.\n" << std::endl;
            }
        }

    
}


void  icp_fusion( const  sensor_msgs:: PointCloud2ConstPtr& ros_pcd1,const  sensor_msgs:: PointCloud2ConstPtr& ros_pcd2);

void keyCallback(const std_msgs::Int16& msg){
  start_fusion=msg.data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "icp_fusion");
    ros::start();
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud1_sub(nh, "/jackal0/PointCloudOutput", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud2_sub(nh, "/jackal1/PointCloudOutput", 1);
    // message_filters::Subscriber<std_msgs::Int16> id_sub(nh, "/jackal1/key_id", 1);
    ros::Subscriber key_sub = nh.subscribe("/start_fusion", 10,keyCallback);  //添加键盘回调
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), cloud1_sub,cloud2_sub);
    sync.registerCallback(boost::bind(&icp_fusion,_1,_2));
    ros::spin();
    return 0;
}




void  icp_fusion(const sensor_msgs:: PointCloud2ConstPtr& ros_pcd1,const sensor_msgs:: PointCloud2ConstPtr& ros_pcd2)   
{    
    

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZRGB>); 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZRGB>); 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZRGB>);  // tmp 
    // pcl_conversions::toPCL(*ros_pcd1, *cloud_source);
    pcl::fromROSMsg(*ros_pcd1,*cloud_source);
    // pcl::io::loadPCDFile<pcl::PointXYZRGB>(name_1, *cloud_source);

    // Defining a rotation matrix and translation vector
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

    transformation_matrix (0, 0) = 1; // 8.22_carpark_v
    transformation_matrix (0, 1) = 0;
    transformation_matrix (0, 2) = 0;
    transformation_matrix (0, 3) = 0;
    transformation_matrix (1, 0) = 0;
    transformation_matrix (1, 1) = 1;
    transformation_matrix (1, 2) = 0;
    transformation_matrix (1, 3) = 0;        
    transformation_matrix (2, 0) =0;
    transformation_matrix (2, 1) = 0;
    transformation_matrix (2, 2) = 1;
    transformation_matrix (2, 3) = 0;

    // Display in terminal the transformation matrix
    std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
    //print4x4Matrix (transformation_matrix);


    pcl::fromROSMsg(*ros_pcd2,*cloud_target);
    // while (cloud_source->points.size()>109782)
    // {
    //     down_sample(cloud_source,cloud_source);
    // }

    // while (cloud_source->points.size()>109782)
    // {
    //     down_sample(cloud_target,cloud_target);
    // }


    // Executing the transformation
    pcl::transformPointCloud (*cloud_source, *cloud_tmp, transformation_matrix);
    // std::string name_2 = "/home/gyy/Downloads/pcd_data/rot2test2.pcd" ;
    // pcl::io::loadPCDFile<pcl::PointXYZRGB>(name_2, *cloud_target);





    // pcl_conversions::toPCL(*ros_pcd2, *cloud_target);


    // white,c2
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> white(cloud_tmp, 250, 250, 250);
    // viewer.addPointCloud<pcl::PointXYZRGB> (cloud_tmp, white, "cloud_tmp");
    // // red,c1
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red (cloud_source, 250, 0, 0);
    // viewer.addPointCloud (cloud_source, red, "cloud_source");
    // // red,c1
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> green (cloud_target, 0, 250, 0);
    // viewer.addPointCloud (cloud_target, green, "cloud_target");
    if(start_fusion==1){
    pcl::PointCloud<pcl::PointXYZRGB> input_trans;
	  pcl::transformPointCloud(*cloud_source, input_trans, transformation_matrix);
	  std::vector<int> nn_indices ;
	  std::vector<float> nn_dists;
	  std::vector<float> all_dists;
	  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_(new pcl::search::KdTree<pcl::PointXYZRGB>);
	  tree_->setInputCloud(cloud_target);
	  for (const auto& point : input_trans) {
		// Find its nearest neighbor in the target
		// cntpoints++;
		tree_->nearestKSearch(point, 1, nn_indices, nn_dists);
		all_dists.push_back(sqrt(nn_dists[0]));
	}
	// std::cout << "num of points pair: " << cntpoints << all_dists.size() << std::endl;
	int cnt = 0;
	for(auto& n : all_dists){
		cnt++;
		std::cout<<"points pair"<<cnt<< "'s distance: " <<n <<std::endl;
	}
    // std::cout << "[Cloud 1] Number of Points = " << cloud_source->points.size () << std::endl;
    // std::cout << "[Cloud 2] Number of Points = " << cloud_target->points.size () << std::endl;
    // ros::Duration(1.0).sleep();
    icp_function(cloud_tmp, cloud_target,transformation_matrix);
    start_fusion=0;
    }
}
