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
#include<iomanip>
#include<chrono>
#include<opencv2/core/core.hpp>
#include<System.h>
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "std_msgs/String.h"
#include "FileYaml.h"
#include <queue>
#include <mutex>
#include <thread>
#include <condition_variable>
using namespace std;
ros::Subscriber image_l_sub;
ros::Subscriber image_r_sub;

int image_l_count = 0;
queue<cv::Mat> que_image_l;
queue<cv::Mat> que_image_r;
queue<long double> que_stamp;

std::mutex limage_mutex;
std::mutex rimage_mutex;
std::condition_variable left_data_cond;
std::condition_variable right_data_cond;
// 	std::lock_guard<std::mutex> lock(rimage_mutex);
//     std::thread show_thread{show_image}; //visualization thread
//     show_thread.detach();

cv::Mat imLeft;
cv::Mat imRight;
ros::Time ros_stamp;
long double stamp;

long double time_tranform(int64_t time)
{
    
    //取后13位
    int b = time/1e13;
    int64_t temp = b * 1e13;
    int64_t c = time - temp;
    
    //小数点后9位
    long double d = c / 1e9;
    return d;
}
//image_l回调函数
void imagelCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
    cv_ptr->image.copyTo(imLeft);
    image_l_count = cv_ptr->header.seq;
    ros_stamp = cv_ptr->header.stamp;
    std::lock_guard<std::mutex> lock_l(limage_mutex);
    stamp = time_tranform(ros_stamp.toNSec());
    //cout<<"ros_stamp: "<<ros_stamp<<endl;
    que_image_l.push(imLeft);
    que_stamp.push(stamp);
    
  

}
//image_r回调函数
void imagerCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
    cv_ptr->image.copyTo(imRight);
    std::lock_guard<std::mutex> lock_r(rimage_mutex);
    que_image_r.push(imRight);

}
Camera_Other_Parameter vecCamerasParam;
cv::Mat M1l,M2l,M1r,M2r;
cv::Mat data_left;
cv::Mat data_right;
long double frame_time;
cv::Mat imLeftRect, imRightRect;
void show_ORB()
{
    //-----------------ORB_SLAM2 Init---------------------------------------------
    const string param1_ORBvoc = "Vocabulary/ORBvoc.txt";
    
    const string param3_ORByaml = "Examples/Stereo/EuRoC.yaml";
    
    
    ORB_SLAM2::System SLAM(param1_ORBvoc,param3_ORByaml,ORB_SLAM2::System::STEREO,true);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    while(true)
    {
	std::this_thread::sleep_for(std::chrono::milliseconds(30));
	std::unique_lock<std::mutex> lock_l(limage_mutex);
        data_left = que_image_l.front();
	frame_time = que_stamp.front();
	que_image_l.pop();
	que_stamp.pop();
	lock_l.unlock();
	std::unique_lock<std::mutex> lock_r(rimage_mutex);
        data_right = que_image_r.front();
	que_image_r.pop();
	lock_r.unlock();
// 	cout.precision(13);
//	cout<<"frame: "<<frame_time<<endl;
	
	cv::resize(data_left,data_left,cv::Size(vecCamerasParam.cols,vecCamerasParam.rows));
	cv::resize(data_right,data_right,cv::Size(vecCamerasParam.cols,vecCamerasParam.rows));
	cv::remap(data_left,imLeftRect,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(data_right,imRightRect,M1r,M2r,cv::INTER_LINEAR);
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
	SLAM.TrackStereo(imLeftRect,imRightRect,frame_time);
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
	double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
	
	// Wait to load the next frame
        double T = 0;
            T = que_stamp.front() - frame_time;

        if(ttrack < T)
            usleep((T-ttrack)*1e6);
	
/*	
	cv::imshow("left",data_left);
	cv::imshow("right",data_right);
	cv::waitKey(1);*/
    }
    // Stop all threads
    SLAM.Shutdown();
    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
}
int main(int argc, char **argv)
{

    ros::init(argc,argv,"ORB_SLAM2");
    ros::NodeHandle n;
    image_l_sub = n.subscribe("/module/image_left",100,imagelCallback);
    image_r_sub = n.subscribe("/module/image_right", 100,imagerCallback);
    
    
    const char *param2_SDKyaml =  "/home/indemind/u/SDK-Linux-ros/lib/1604/headset.yaml";
    readConfig(param2_SDKyaml,vecCamerasParam);
    //-----------------fisheye rectify---------------------------------------------
    cv::Mat Q;
    
    if(vecCamerasParam.K_l.empty() || vecCamerasParam.K_r.empty() || vecCamerasParam.P_l.empty() || vecCamerasParam.P_r.empty() || vecCamerasParam.R_l.empty() || 
	    vecCamerasParam.R_r.empty() || vecCamerasParam.D_l.empty() || vecCamerasParam.D_r.empty() || vecCamerasParam.rows==0 || vecCamerasParam.cols==0)
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return -1;
    }
    
    cv::fisheye::initUndistortRectifyMap(vecCamerasParam.K_l,vecCamerasParam.D_l,vecCamerasParam.R_l,vecCamerasParam.P_l.rowRange(0,3).colRange(0,3),
					 cv::Size(vecCamerasParam.cols,vecCamerasParam.rows),CV_32FC1,M1l,M2l);
    cv::fisheye::initUndistortRectifyMap(vecCamerasParam.K_r,vecCamerasParam.D_r,vecCamerasParam.R_r,vecCamerasParam.P_r.rowRange(0,3).colRange(0,3),
					 cv::Size(vecCamerasParam.cols,vecCamerasParam.rows),CV_32FC1,M1r,M2r);
    cout << "the P_l of initUndistortRectifyMap after" << endl;
    for(int i = 0;i < 3;++i)
      for(int j = 0;j < 3;++j)
      {
	  double *ptr = vecCamerasParam.P_l.ptr<double>(i,j);
	  cout << *ptr<<endl;
      }

    cout << "the P_r of initUndistortRectifyMap after" << endl;
    for(int i = 0;i < 3;++i)
      for(int j = 0;j < 3;++j)
      {
	  double *ptr = vecCamerasParam.P_r.ptr<double>(i,j);
	  cout << *ptr<<endl;
      }
    cv::stereoRectify(vecCamerasParam.K_l,vecCamerasParam.D_l,vecCamerasParam.K_r,vecCamerasParam.D_r,cv::Size(vecCamerasParam.cols,vecCamerasParam.rows),
		      vecCamerasParam.R,vecCamerasParam.t,vecCamerasParam.R_l,vecCamerasParam.R_r,vecCamerasParam.P_l,vecCamerasParam.P_r,Q,cv::CALIB_ZERO_DISPARITY,0);

	
    //-----------------fisheye end---------------------------------------------
    std::thread show_thread{show_ORB}; //visualization thread
        

    show_thread.detach();
    ros::spin();
    

    return 0;
}

