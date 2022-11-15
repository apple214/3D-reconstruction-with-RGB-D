# 3D-reconstruction-with-RGB-D
3D reconstruction project with RGB-D camera, using SLAM algorithm

...is based on the open source projects, ROS, OpenCV, PCL, g2o, and more, online data from nyu https://cs.nyu.edu/~silberman/datasets/nyu_depth_v1.html, Dr.Shan He with data collection - Thanks!

# File description
 1. data：self-collected data set (trans to .jpg using ROS)
 2. dataOnline: online data set (with corresponding parameters)
 3. src：code

# Prerequisites
 1. ubuntu16.04 http://mirrors.aliyun.com/ubuntu-releases/16.04/
 2. OpenCV 2.4.9  
      sudo apt-get install libopencv-dev 
      pkg-config --modversion opencv
 3. PCL
      sudo apt-get install libpcl-dev pcl-tools 
 4. g2o
      sudo apt-get install libeigen3-dev libsuitesparse-dev libqt4-dev qt4-qmake libqglviewer-dev
      go into g2o file, then run following command
        mkdir build
        cd build 
        cmake ..
        make
        sudo make install
 5.  go back to the project file and create a build file
         mkdir build
         cd build 
         cmake ..
         make
 6. run ./bin/slam
  1. check for “Reading files 146” etc. log
  2. check for "saving the point cloud map..."
 7. show the result 
      pcl_viewer result.pcd
 8. if change data set, please change the parameters
     

# reference 
Xiang Gao, Tao Zhang, Yi Liu, Qinrui Yan, 14 Lectures on Visual SLAM: From Theory to Practice, Publishing House of Electronics Industry, 2017.
