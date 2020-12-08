#include "SportDetection.h"

bool SportDetection::Init()
{
    // TODO: 是否可以命令行读取配置？
    // TODO: 能够在摄像机意外关闭时自启动？
    if(threshold == 0)
    threshold = 500;

    saveTotalNo = 10;
    saveCurrentNo = 0;
    depthImageNo = 1;
    rgbImageNo = 1;
    if (isOnlyOneKinect())
    {
        m_device = k4a::device::open(K4A_DEVICE_DEFAULT);
        
        config();        //TODO: 可变初始化设置  

        if(isEnableCapture())
            return true;

    }
    else
    {
        std::cout << "No Kinect divice found!" << std::endl;
        return false;
    }
}

void SportDetection::Run()
{

    /*********************************************** 
    一、捕获视频流
    二、直接进行一个运动检测判的定
    三、如果符合存在运动，则开始（录像/连续截图）？
    ************************************************/

    // TODO: 减少计算量，提高检测的速度，可选方法如下
    /***********************************************
    1、 不要每一帧都进行判定，每间隔k帧进行检测。
    2、 一旦判定成功后，停止判定算法，连续储存m帧。
    3、 目前测出来耗费时间最长的部分为存储，考虑先将m帧存入内存，再保存为图片或者视频格式。
    ************************************************/
    
    while (true)
    {
        if (m_device.get_capture(&m_capture))
        {
            std::cout << "Receive new image!" << std::endl;

            k4a::image l_depthImage = captureDepthImage(this->m_capture);
            getImageInfo(l_depthImage);
            processDepthImage(l_depthImage);
            // TODO: 处理的与保存的逻辑还需要调整
            k4a::image l_rgbImage = captureRGBImage(this->m_capture);
            processRGBImage(l_rgbImage);
            // TODO: 添加检测到时候图像显示框的文字变化。
            // TODO: 将显示图像封装成单独的方法。
            imshow("运动检测", cv_rgbImage_no_alpha);
            cvWaitKey(33);
            std::cout << "显示图片" << std::endl;
            
            if (issave)
            {
                saveRGBImage(cv_rgbImage_no_alpha, devicetimestemp);                
            }
           
        }
    }
}

void SportDetection::ProcessCommand(int argc, char* argv[])
{
    //std::cout << "argc = " << argc << std::endl;
    //std::cout << "argv = " << argv[0] << std::endl;
    switch (argc)
    {
        // 设置门限
        case 2:
            threshold = strtod(argv[1],NULL);
            std::cout << "threshold = " << threshold << std::endl;
            break;

        case 3:
            break;
        default:
            break;
    }


}

bool SportDetection::isOnlyOneKinect()
{
    const uint32_t device_count = k4a::device::get_installed_count();
    if (0 == device_count) {
        std::cout << "Error: no K4A devices found. " << std::endl;
        return false;
    }
    else {
        std::cout << "Found " << device_count << " connected devices. " << std::endl;
        if (1 != device_count)
        {
            std::cout << "Error: more than one K4A devices found. " << std::endl;
            return false;
        }
        else
        {
            std::cout << "Done: found 1 K4A device. " << std::endl;
        }
    }
    return true;
}

void SportDetection::config()
{
    // TODO: 将配置设置成为命令行参数可以修改的模式
    m_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    m_config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    m_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    m_config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    m_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    m_config.synchronized_images_only = true;
    m_device.start_cameras(&m_config);
    std::cout << "Done: start camera." << std::endl;

    m_k4aCalibration = m_device.get_calibration(m_config.depth_mode, m_config.color_resolution);
    m_k4aTransformation = k4a::transformation(m_k4aCalibration);
}

bool SportDetection::isEnableCapture()
{
    int iAuto = 0;
    int iAutoError = 0;
    //cv::namedWindow("运动检测", cv::WINDOW_AUTOSIZE);
    while (true) {
        if (m_device.get_capture(&m_capture)) {
            std::cout << iAuto << ". Capture several frames to give auto-exposure" << std::endl;


            if (iAuto != 30) {
                iAuto++;
                continue;
            }
            else {
                std::cout << "Done: auto-exposure" << std::endl;
                return true;
            }

        }
        else {
            std::cout << iAutoError << ". K4A_WAIT_RESULT_TIMEOUT." << std::endl;
            if (iAutoError != 30) {
                iAutoError++;
                continue;
            }
            else {
                std::cout << "Error: failed to give auto-exposure. " << std::endl;
                return false;
            }
        }
    }
    return false;
}

void SportDetection::processDepthImage(k4a::image image)
{

    transformed_depthImage = m_k4aTransformation.depth_image_to_color_camera(image);
    cv_depth = cv::Mat(transformed_depthImage.get_height_pixels(), transformed_depthImage.get_width_pixels(), CV_16U,
        (void*)transformed_depthImage.get_buffer(), static_cast<size_t>(transformed_depthImage.get_stride_bytes()));
    
    // TODO: 使用更加精确的方法进行运动检测，使用命令行选择不同的模式，可选方案如下
    /**************************************************
    1、 计算平均深度的方式来测算
    2、 计算深度图的分布检测
    3、 使用逐帧法判断运动
    ***************************************************/

    if (saveCurrentNo == 0)
    {
        //求取深度图平均的方法触发保存
        average = depthAve(cv_depth);
        
        //求深度图分布的方法
        // TODO: 计算分布模型
    }



    if (average<threshold && saveCurrentNo<=saveTotalNo && saveCurrentNo != 0)
    {
        //if (depthImageNo < 100)
        //{
            normalize(cv_depth, cv_depth_8U, 0, 256 * 256 - 1, cv::NORM_MINMAX);
            saveDepthImage(cv_depth_8U, devicetimestemp);
        //}
        //TODO: 连续保存10帧图像.
    }
    else
    {
        std::cout << "mean = " << average << std::endl;
        std::cout << "No need to save" << std::endl;
        saveCurrentNo = 0;
    }
    
}

void SportDetection::processRGBImage(k4a::image image)
{
    cv_rgbImage_with_alpha = cv::Mat(image.get_height_pixels(), image.get_width_pixels(), CV_8UC4,
        (void*)image.get_buffer());

    cv::cvtColor(cv_rgbImage_with_alpha, cv_rgbImage_no_alpha, cv::COLOR_BGRA2BGR);
    //if (rgbImageNo < 100)
    //{
        //saveRGBImage(cv_rgbImage_no_alpha, devicetimestemp);
    //}

}

double SportDetection::depthAve(cv::Mat image)
{
    cv::Mat l_average, l_stddev;
    double l_mean;
    cv::meanStdDev(image, l_average, l_stddev);
    l_mean = l_average.at<double>(0, 0);
    std::cout << "mean = " << l_mean << std::endl;
    return l_mean;
}

void SportDetection::saveRGBImage(cv::Mat rgbImage, double timeStemp)
{
    std::string l_timeStemp = std::to_string(timeStemp);
    std::string fliename = "./SportImageRGB/" + getSysTime() + "_" + l_timeStemp + "_" + std::to_string(rgbImageNo) + ".png";
    cv::imwrite(fliename, rgbImage);
    std::cout << "save rgbImage successful" << std::endl;
    rgbImageNo++;
    issave = false;
}

void SportDetection::saveDepthImage(cv::Mat depthImage, double timeStemp)
{
    std::string l_timeStemp = std::to_string(timeStemp);
    std::string fliename = "./SportImageDepth/" + getSysTime() + "_" + l_timeStemp + "_" + std::to_string(depthImageNo) + ".png";
    cv::imwrite(fliename,depthImage);
    std::cout << "save depthImage successful" << std::endl;
    depthImageNo++;
    saveCurrentNo++;
    issave = true;
}

std::string SportDetection::getSysTime()
{
    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    //转为字符串
    std::stringstream ss;
    ss << std::put_time(std::localtime(&t), "%Y-%m-%d-%H-%M-%S");
    std::string str_time = ss.str();
    return str_time;
}

k4a::image SportDetection::captureDepthImage(k4a::capture capture)
{
    return capture.get_depth_image();
}

void SportDetection::getImageInfo(k4a::image image)
{
    //将需要的图像信息写成单独的数据结构，读取进来后开始保存
    devicetimestemp = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
        image.get_device_timestamp()).count());
}

k4a::image SportDetection::captureRGBImage(k4a::capture capture)
{
    return capture.get_color_image();
}
