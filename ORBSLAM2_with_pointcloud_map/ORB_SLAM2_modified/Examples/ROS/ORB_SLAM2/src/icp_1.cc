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
#include <pcl/filters/extract_indices.h>
#include <cmath>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>



bool next_iteration = false;

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event, void* nothing)
{
    if (event.getKeySym () == "space" && event.keyDown ())
        next_iteration = true;
}

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
    Eigen::Matrix4d transformation_matrix = transformation_matrix_raw;

    // ICP 算法
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setMaximumIterations (iterations);
    icp.setMaxCorrespondenceDistance(0.1);
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

    pcl::visualization::PCLVisualizer viewer;

    // white,c2
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> white(cloud_target, 250, 250, 250);

    viewer.addPointCloud<pcl::PointXYZRGB> (cloud_target, white, "cloud_target");

    // red,c1
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red (cloud_source, 250, 0, 0);
    viewer.addPointCloud (cloud_source, red, "cloud_source");

    // callback
    viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) nullptr);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce ();

        // if you pressed "space"
        if (next_iteration)
        {
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

        next_iteration = false;
    }
}


void RadiusOutlierFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcd_cloud0,const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_after_Radius,
                         const double &radius, const int &thre_count) {
  //创建滤波器
  pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> radiusoutlier;
  //设置输入点云
  radiusoutlier.setInputCloud(pcd_cloud0);
  //设置半径,在该范围内找临近点
  radiusoutlier.setRadiusSearch(radius);
  //设置查询点的邻域点集数，小于该阈值的删除
  radiusoutlier.setMinNeighborsInRadius(thre_count);
  radiusoutlier.filter(*cloud_after_Radius);
  // test 保存滤波后的点云到文件
  std::cout << "半径滤波后点云数据点数：" << cloud_after_Radius->points.size()
            << std::endl;
}





void cloud_filter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_source )
{
    pcl::NormalEstimationOMP<pcl::PointXYZRGB,pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    n.setNumberOfThreads(12);
    n.setInputCloud(cloud_source);
    n.setSearchMethod(tree);
    // n.setKSearch(50);
    n.setRadiusSearch(0.1);
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
    {   float k=abs( normals->points[i].normal_y)/sqrt(normals->points[i].normal_x*normals->points[i].normal_x+normals->points[i].normal_z*normals->points[i].normal_z);
        if (0.2<k&&k<100)
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

}
int main()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZRGB>); 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZRGB>); 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZRGB>);  // tmp

    std::string name_1 = "/home/gyy/Downloads/pcd_data/global170.pcd" ;

    
    pcl::io::loadPCDFile<pcl::PointXYZRGB>(name_1, *cloud_source);

    // Defining a rotation matrix and translation vector
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

    // pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    // pcl::ExtractIndices<pcl::PointXYZ> extract;
    cout<<cloud_source->points.size();
    // cout<<"\n"<<cloud_source->height;
    // cout<<"\n"<<cloud_source->width;

    pcl::VoxelGrid<pcl::PointXYZRGB>  voxel;

    voxel.setLeafSize( 0.02f, 0.02f, 0.02f);

    
    voxel.setInputCloud( cloud_source );
    voxel.filter( *cloud_tmp );
        

    


    cloud_source=cloud_tmp ;
    cout<<"\n"<<cloud_source->points.size();

    cloud_filter(cloud_source);

    cout<<"\n"<<cloud_source->points.size();
    // extract.setInputCloud(cloud_source);
    // extract.setIndices(indices);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source1 (new pcl::PointCloud<pcl::PointXYZRGB>); 
    // extract.setNegative(true);
    // extract.filter(cloud_source1);
    double thre_radius = 0.1;
    int thres_point_count = 20;
    RadiusOutlierFilter(cloud_source,cloud_tmp,thre_radius,thres_point_count);
    cloud_source=cloud_tmp ;



    transformation_matrix (0, 0) =  1; // 8.22_carpark_v
    transformation_matrix (0, 1) = 0;
    transformation_matrix (0, 2) = 0;
    transformation_matrix (0, 3) = 0;
    transformation_matrix (1, 0) = 0;
    transformation_matrix (1, 1) = 1;
    transformation_matrix (1, 2) = 0;
    transformation_matrix (1, 3) = 0;        
    transformation_matrix (2, 0) = 0;
    transformation_matrix (2, 1) = 0;
    transformation_matrix (2, 2) =  1;
    transformation_matrix (2, 3) = 0;

    // Display in terminal the transformation matrix
    std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
    //print4x4Matrix (transformation_matrix);

    // Executing the transformation
    pcl::transformPointCloud (*cloud_source, *cloud_tmp, transformation_matrix);
    std::string name_2 = "/home/gyy/Downloads/pcd_data/global170.pcd" ;
    pcl::io::loadPCDFile<pcl::PointXYZRGB>(name_2, *cloud_target);



    //     for (int i=1;i<cloud_target->points.size();){
    //     if (cloud_target->points[i].y>0.05)
    //     {
    //         cloud_target->erase(cloud_target->begin()+i);
    //         // cloud_source->removeIndices(i);
    //     }
    //     else
    //     {i=i+1;
    //     }
        

    // }


    pcl::visualization::PCLVisualizer viewer;
    // white,c2
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> white(cloud_tmp, 250, 250, 250);
    // viewer.addPointCloud<pcl::PointXYZRGB> (cloud_tmp, white, "cloud_tmp");
    // red,c1
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red (cloud_source, 250, 0, 0);
    viewer.addPointCloud (cloud_source, red, "cloud_source");
    // red,c1
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> green (cloud_target, 0, 250, 0);
    // viewer.addPointCloud (cloud_target, green, "cloud_target");

    pcl::PointCloud<pcl::PointXYZRGB> input_trans;
	pcl::transformPointCloud(*cloud_source, input_trans, transformation_matrix);
	std::vector<int> nn_indices;
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
	// int cnt = 0;
	// for(auto& n : all_dists){
	// 	cnt++;
	// 	std::cout<<"points pair"<<cnt<< "'s distance: " <<n <<std::endl;
	// }
    // std::cout << "[Cloud 1] Number of Points = " << cloud_source->points.size () << std::endl;
    // std::cout << "[Cloud 2] Number of Points = " << cloud_target->points.size () << std::endl;

    // icp_function(cloud_tmp, cloud_target,transformation_matrix);
    
    
    while (!viewer.wasStopped())
    {
        viewer.spinOnce ();

    }
    return 0;
}
