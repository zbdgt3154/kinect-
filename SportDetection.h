#pragma once

#ifndef SPORTDETECTION_H
#define SPORTDETECTION_H

// C++
#include <iostream>
#include <fstream>
#include <chrono>
#include <string>
#include <time.h>
#include <sstream>
#include <iomanip>
#include <Windows.h>
#include <stdlib.h>
// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// Kinect DK
#include <k4a/k4a.hpp>

class SportDetection
{
public:
	k4a::device m_device;
	k4a::capture m_capture;

private:
	k4a_device_configuration_t m_config;

	//获取到的图像
	k4a::image rgbImage;
	k4a::image depthImage;
	k4a::image irImage;
	k4a::image transformed_depthImage;

	//将获取到的图像转化为Opencv可以处理的格式
	cv::Mat cv_depth;
	cv::Mat cv_depth_8U;
	cv::Mat cv_rgbImage_with_alpha;
	cv::Mat cv_rgbImage_no_alpha;
	k4a::calibration m_k4aCalibration;
	k4a::transformation m_k4aTransformation;

	//double savetimestemp;		// 存储图像时间戳
	double devicetimestemp;		// 设备图像时间戳

	double average;					// 图像均值
	double threshold;			// 判断门限
	bool issave;				// 是否保存此帧图像
	int saveTotalNo;			// 单次触发保存后保存的总张数
	int saveCurrentNo;			// 单次触发保存保存的当前张数

	long depthImageNo;			// 深度图像编号
	long rgbImageNo;			// 彩色图像编号

public:
	bool Init();									// 初始化
	void Run();										// 视频流处理
	void ProcessCommand(int argc, char* argv[]);	// 初始化命令行参数

private:
	bool isOnlyOneKinect();										// 发现机器是否只连接唯一一台Kinect
	void config();												// 配置kinect
	bool isEnableCapture();										// 是否能够捕获视频流	
	

	void processDepthImage(k4a::image image);					// 处理深度图像
	void processRGBImage(k4a::image image);						// 处理RGB图像
	double depthAve(cv::Mat image);								// 计算平均深度

	// TODO: 调用系统自带的recoder
	void startrecorder();										// 启动记录程序
	void saveRGBImage(cv::Mat rgbImage, double timeStemp);		// 保存RGB图像
	void saveDepthImage(cv::Mat depthImage, double timeStemp);	// 保存深度图像
	std::string getSysTime();									// 获取当前系统时间

	k4a::image captureDepthImage(k4a::capture capture);			// 获取原始深度图	
	void getImageInfo(k4a::image image);						// 获取深度图信息

	k4a::image captureRGBImage(k4a::capture capture);			// 获取原始RGB图	

};

#endif