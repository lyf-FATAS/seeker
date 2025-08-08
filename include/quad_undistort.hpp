// quad_undistort.hpp

#ifndef __QUAD_UNDISTORT_H__
#define __QUAD_UNDISTORT_H__


#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
#include "seeker_type.hpp"

// namespace SEEKNS {
class QuadUndistort {
public:
    enum Backend {
        OPENCV_BACKEND,
        VPI_BACKEND
    };

    // 初始化接口
    QuadUndistort(const std::string& path = "/home/jetson/catkin_ws_seeker/cali", 
                  Backend backend = OPENCV_BACKEND);
                  
    QuadUndistort(int width, int height, int fx, int fy,
                  const std::string& path = "/home/jetson/catkin_ws_seeker/cali", 
                  Backend backend = OPENCV_BACKEND);
    
    ~QuadUndistort();

    // 解畸变接口
    int undistort(cv::Mat &input, cv::Mat &output);

    // 禁用拷贝和赋值
    QuadUndistort(const QuadUndistort&) = delete;
    QuadUndistort& operator=(const QuadUndistort&) = delete;
    void init_impl(const device_event_t& param, int width, int height, 
                              int fx, int fy, Backend backend);
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

// } // SEEKNS

#endif // __QUAD_UNDISTORT_H__
