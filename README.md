# orb2_pointmap
下载库 eigen3.2.1 opence3.4.5 pcl l.8.1 panglin 0.5 vtk7.1 这些库也有相关的依赖 按照网上步骤安装就行了

进入到g2o_with_orbslam2文件夹下 下载修改后的g20库即 在该文件下 创建build （mkdir build） ; 进入 cd build ; cmake .. ; make ;编译完成后在进行安装 sudo make install; 至此 g20库安装完毕

进入到orb_modified 文件夹下 对项目进行编译 ./build.sh 编译完成后 再对ros版本进行编译 ./build_ros.sh

运行demo：
运行已经录好的rosbag进行测试 rosbag的压缩包在orb2_pointcloud_map/ORBSLAM2_with_pointcloud_map
rosbag play --pause 20230922_165939.bag device_0/sensor_1/Color_0/image/data:=/jackal0//front/image_raw /device_0/sensor_0/Depth_0/image/data:=/jackal0/front/depth/image_raw
进入 cd ORB_SLAM2_modified/Examples/ROS/ORB_SLAM2/src
roslaunch demo.launch
rosrun rviz rviz -d viewea.rviz
