#include "seeker.hpp"
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <stereo_msgs/DisparityImage.h>
#include "NvJpegDecoder.h"
#include "NvBufSurface.h"
#include "quad_undistort.hpp"

class SeekRosNodeimpl {
public:
    SeekRosNodeimpl(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh) :
        nh_(nh), private_nh_(private_nh) {
            printf("backend\n");
        jpegdec = NvJPEGDecoder::createJPEGDecoder("jpegdec");

        private_nh_.param("publish_bgra", publish_bgra_, true);
        private_nh_.param("publish_gray", publish_gray_, false);
        private_nh_.param("undistort_color", undistort_color_, true);
        private_nh_.param("undistort_gray", undistort_gray_, false);

        transform_params.src_top = 0;
        transform_params.src_left = 0;
        transform_params.src_width = 1088;
        transform_params.src_height = 5120;
        transform_params.dst_top = 0;
        transform_params.dst_left = 0;
        transform_params.dst_width = 1088;
        transform_params.dst_height = 5120;
        transform_params.flag = NVBUFSURF_TRANSFORM_FILTER;
        transform_params.flip = NvBufSurfTransform_None;
        transform_params.filter = NvBufSurfTransformInter_Nearest;

        NvBufSurf::NvCommonAllocateParams params;

        // bgra
        if (publish_bgra_) {
            params.memType = NVBUF_MEM_SURFACE_ARRAY;
            params.width = 1088;
            params.height = 5120;
            params.layout = NVBUF_LAYOUT_PITCH;
            params.colorFormat = NVBUF_COLOR_FORMAT_BGRA;
            params.memtag = NvBufSurfaceTag_VIDEO_CONVERT;
            int ret = NvBufSurf::NvAllocate(&params, 1, &dst_dma_fd);
            if (ret == -1) {
                printf("create dmabuf failed\n");
            }
            ret = NvBufSurfaceFromFd(dst_dma_fd, (void**)(&nvbuf_surf));
            if (ret != 0) {
            }
            ret = NvBufSurfaceMap(nvbuf_surf, 0, 0, NVBUF_MAP_READ_WRITE);
            if (ret < 0) {
                printf("NvBufSurfaceMap failed\n");
            }
        }
        // gray
        if (publish_gray_) {
            params.memType = NVBUF_MEM_SURFACE_ARRAY;
            params.width = 1088;
            params.height = 5120;
            params.layout = NVBUF_LAYOUT_PITCH;
            //params.colorFormat = NVBUF_COLOR_FORMAT_BGR;
            params.colorFormat = NVBUF_COLOR_FORMAT_GRAY8;
            params.memtag = NvBufSurfaceTag_VIDEO_CONVERT;
            int ret = NvBufSurf::NvAllocate(&params, 1, &dst_dma_gray_fd);
            if (ret == -1) {
                printf("create dmabuf failed\n");
            }
            ret = NvBufSurfaceFromFd(dst_dma_gray_fd, (void**)(&nvbuf_gray_surf));
            if (ret != 0) {
            }
            ret = NvBufSurfaceMap(nvbuf_gray_surf, 0, 0, NVBUF_MAP_READ_WRITE);
            if (ret < 0) {
                printf("NvBufSurfaceMap failed\n");
            }
        }

        // 读取参数
        private_nh_.param("use_image_transport", use_image_transport_, true);
        private_nh_.param("pub_disparity_img", pub_disparity_img_, false);
        private_nh_.param("pub_disparity", pub_disparity_, true);
        private_nh_.param("pub_imu", pub_imu_, true);
        private_nh_.param("time_sync", time_sync_, true);
        private_nh_.param("imu_link", imu_link_, std::string("imu_link"));
        private_nh_.param("imu_topic", imu_topic_, std::string("imu_data_raw"));
        std::string cali_path_;
        private_nh_.param("cali_path", cali_path_, std::string("cali_path"));

        // 初始化image_transport
        if (use_image_transport_) {
            it_.reset(new image_transport::ImageTransport(nh_));
        }

        // 初始化设备
        std::vector<seeker_device_t> devices = seek.find_devices();
        if (devices.empty()) {
            printf("No Seeker Devices Found\n");
            return;
        }
        seek.open(devices[0]);
        undistort = std::make_shared<QuadUndistort>(cali_path_, QuadUndistort::Backend::VPI_BACKEND);

        // 设置回调
        seek.set_event_callback(std::bind(&SeekRosNodeimpl::onEvent, this, std::placeholders::_1, std::placeholders::_2));
        // seek.set_event_callback([this](auto&& ph, auto&& e) { onEvent(ph, e); });
        seek.set_mjpeg_callback(std::bind(&SeekRosNodeimpl::onMjpeg, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        // seek.set_mjpeg_callback([this](auto&& ph, auto&& d, auto&& l) { onMjpeg(ph, d, l); });
        seek.set_depth_callback(std::bind(&SeekRosNodeimpl::onDepth, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        // seek.set_depth_callback([this](auto&& ph, auto&& d, auto&& l) { onDepth(ph, d, l); });
        if (time_sync_) {
            seek.set_timer_callback(std::bind(&SeekRosNodeimpl::onTimer, this, std::placeholders::_1, std::placeholders::_2));
            // seek.set_timer_callback([this](auto&& tg, auto&& ts) { return onTimer(tg, ts); });
        }

        // 获取设备信息
        seek.get_dev_info(sdev);

        // 初始化发布者
        const std::vector<std::string> image_topics = {
            "/fisheye/left/image_raw",
            "/fisheye/right/image_raw",
            "/fisheye/bright/image_raw",
            "/fisheye/bleft/image_raw",
        };
        const std::vector<std::string> gray_topics = {
            "/fisheye/gray/left/image_raw",
            "/fisheye/gray/right/image_raw",
            "/fisheye/gray/bright/image_raw",
            "/fisheye/gray/bleft/image_raw",
        };
        const std::vector<std::string> depth_topics = {
            "/front/disparity/image_raw",
            "/right/disparity/image_raw",
            "/back/disparity/image_raw",
            "/left/disparity/image_raw"
        };
        const std::vector<std::string> disparity_topics = {
            "/front/disparity",
            "/right/disparity",
            "/back/disparity",
            "/left/disparity"
        };
        compressed_image_pub_ = nh_.advertise<sensor_msgs::CompressedImage>("/all/compressed", 10);

        // 图像发布者
        for (size_t i = 0; i < sdev.dev_info.rgb_camera_number; ++i) {
            if (use_image_transport_) {
                image_pubs_it_.push_back(it_->advertise(image_topics[i], 1));
            } else {
                if (publish_bgra_) {
                    image_pubs_ros_.push_back(nh_.advertise<sensor_msgs::Image>(image_topics[i], 10));
                    if (undistort_color_) {
                        image_rectify_pubs_ros_ = nh_.advertise<sensor_msgs::Image>("/rectify", 10);
                    }
                }
                if (publish_gray_) {
                    image_gray_pubs_ros_.push_back(nh_.advertise<sensor_msgs::Image>(gray_topics[i], 10));
                    if (undistort_gray_) {
                        image_rectify_gray_pubs_ros_ = nh_.advertise<sensor_msgs::Image>("/rectify_gray", 10);
                    }
                }
            }
        }

        // 深度和视差发布者
        for (size_t i = 0; i < sdev.dev_info.depth_camera_number; ++i) {
            if (pub_disparity_img_) {
                depth_pubs_.push_back(nh_.advertise<sensor_msgs::Image>(depth_topics[i], 10));
            }
            if (pub_disparity_) {
                disparity_pubs_.push_back(nh_.advertise<stereo_msgs::DisparityImage>(disparity_topics[i], 10));
            }
        }

        // IMU发布者
        if (pub_imu_) {
            imu_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_topic_, 200);
        }

        // 启动设备流
        seek.start_event_stream();
        seek.start_image_stream();
        seek.start_depth_stream();
    }

    ~SeekRosNodeimpl() {
        seek.stop_event_stream();
        seek.stop_image_stream();
        seek.stop_depth_stream();
        seek.close();
        delete jpegdec;
        int ret = NvBufSurfaceUnMap(nvbuf_surf, 0, 0);
        if (ret < 0) {
            printf("NvBufSurfaceUnMap failed\n");
        }
    }

private:
    void onDepth(event_header_t& pheader, const uint8_t* data, int len) {
        const int depth_camera_number = sdev.dev_info.depth_camera_number;
        const int height = sdev.dev_info.depth_resolution_height/depth_camera_number;
        const int width = sdev.dev_info.depth_resolution_width;
        
        std::vector<cv::Mat> images(depth_camera_number);
        for (int i = 0; i < depth_camera_number; i++) {
            images.at(i) = cv::Mat(height, width, CV_16UC1, (void *)data+i*height*width*2);
        }
        std_msgs::Header header;
        header.stamp = ros::Time(pheader.sec, pheader.nsec);  // 使用当前时间作为时间戳
        header.seq = pheader.seq;
        // 发布视差图像
        for (size_t i = 0; i < depth_camera_number; ++i) {
            header.frame_id = "depth" + std::to_string(i);
            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "16UC1", images[i]).toImageMsg();
            // 注意：这里假设深度图像是单通道16位无符号整数（"16UC1"）
            depth_pubs_[i].publish(img_msg);
        }
        // 发布disparity
        for (size_t i = 0; i < depth_camera_number; ++i) {
            const int DPP = 256/4;
            const double inv_dpp = 1.0 / DPP;
            stereo_msgs::DisparityImage disparity;
            sensor_msgs::Image& dimage = disparity.image;
            dimage.height = images[i].rows;
            dimage.width = images[i].cols;
            dimage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            dimage.step = dimage.width * sizeof(float);
            dimage.data.resize(dimage.step * dimage.height);
            cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
            images[i].convertTo(dmat, dmat.type(), inv_dpp, 0);
            ROS_ASSERT(dmat.data == &dimage.data[0]);

            // Stereo eventeters
            disparity.f = 320.0;
            disparity.T = 0.04625;

            // Disparity search range
            disparity.min_disparity = 0.0;
            disparity.max_disparity = 192;
            disparity.delta_d = inv_dpp;
            header.frame_id = "depth" + std::to_string(i);
            disparity.header = header;
            disparity_pubs_[i].publish(disparity);
        }
        return;
    }

    // 回调函数和成员变量与原始代码类似，增加参数判断
    void onEvent(event_header_t& header, device_event_t& event) {
        if (event.type == EVENT_TYPE_SENSOR_CUSTOM && pub_imu_) {
            sensor_msgs::Imu imu_msg;
            imu_msg.header.stamp = ros::Time(header.sec, header.nsec);
            imu_msg.header.frame_id = imu_link_;
            
            imu_msg.angular_velocity.x = event.event.sensor_custom.angular_velocity_x;
            imu_msg.angular_velocity.y = event.event.sensor_custom.angular_velocity_y;
            imu_msg.angular_velocity.z = event.event.sensor_custom.angular_velocity_z;

            imu_msg.linear_acceleration.x = event.event.sensor_custom.linear_acceleration_x;
            imu_msg.linear_acceleration.y = event.event.sensor_custom.linear_acceleration_y;
            imu_msg.linear_acceleration.z = event.event.sensor_custom.linear_acceleration_z;

            imu_pub_.publish(imu_msg);
        }
    }

    void onImageBGRA(event_header_t& eheader, cv::Mat& frame) {
        const int cam_num = sdev.dev_info.rgb_camera_number;
        const int h = frame.rows / cam_num;
        const int w = frame.cols;
        std_msgs::Header header;
        header.stamp = ros::Time(eheader.sec, eheader.nsec);
        header.seq = eheader.seq;

        for (int i = 0; i < cam_num; i++) {
            header.frame_id = "cam" + std::to_string(i);;  // 可选：设置帧ID，比如相机的参考坐标系
            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "bgr8", frame(cv::Rect(0, i * h, w, h))).toImageMsg();
            if (use_image_transport_) {
                image_pubs_it_[i].publish(img_msg);
            } else {
                image_pubs_ros_[i].publish(img_msg);
            }
        }
    }

    void onImageGRAY(event_header_t& eheader, cv::Mat& frame) {
        const int cam_num = sdev.dev_info.rgb_camera_number;
        const int h = frame.rows / cam_num;
        const int w = frame.cols;
        std_msgs::Header header;
        header.stamp = ros::Time(eheader.sec, eheader.nsec);
        header.seq = eheader.seq;

        for (int i = 0; i < cam_num; i++) {
            header.frame_id = "cam" + std::to_string(i);;  // 可选：设置帧ID，比如相机的参考坐标系
            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "mono8", frame(cv::Rect(0, i * h, w, h))).toImageMsg();
            image_gray_pubs_ros_[i].publish(img_msg);
        }
    }

    void onRectiryBGRA(const event_header_t& eheader, const cv::Mat& frame) {
        std_msgs::Header header;
        header.stamp = ros::Time(eheader.sec, eheader.nsec);
        header.seq = eheader.seq;

        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        image_rectify_pubs_ros_.publish(img_msg);
    }

    void onRectiryGRAY(const event_header_t& eheader, const cv::Mat& frame) {
        std_msgs::Header header;
        header.stamp = ros::Time(eheader.sec, eheader.nsec);
        header.seq = eheader.seq;

        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "mono8", frame).toImageMsg();
        image_rectify_gray_pubs_ros_.publish(img_msg);
    }

    int dump_dmabuf(NvBufSurface *nvbuf_surf, uint8_t *data, int &byteuse) {
        unsigned int plane = 0;
        int ret = -1;
        // NvBufSurfaceSyncForCpu (nvbuf_surf, 0, plane);
        for (uint i = 0; i < nvbuf_surf->surfaceList->planeParams.height[plane]; ++i) {
            memcpy(data + byteuse, (char *)nvbuf_surf->surfaceList->mappedAddr.addr[plane] + i * nvbuf_surf->surfaceList->planeParams.pitch[plane],
                    nvbuf_surf->surfaceList->planeParams.width[plane] * nvbuf_surf->surfaceList->planeParams.bytesPerPix[plane]);
            byteuse += nvbuf_surf->surfaceList->planeParams.width[plane] * nvbuf_surf->surfaceList->planeParams.bytesPerPix[plane];
        }
        return 0;
    }

    void onMjpeg(event_header_t& pheader, const uint8_t* data, int len) {
        sensor_msgs::CompressedImage compressed;
        std_msgs::Header header;
        header.stamp = ros::Time(pheader.sec, pheader.nsec);
        header.seq = pheader.seq;
        compressed.header = header;
        compressed.format += "rgb8; jpeg compressed";
        compressed.data.resize(len);
        memcpy(&compressed.data[0], data, len);
        compressed_image_pub_.publish(compressed);

        int fd, ret;
        uint32_t width, height, pixfmt;
        static uint8_t cpu_data_bgra[1088*5120*4];
        static uint8_t cpu_data_gray[1088*5120];

        if (jpegdec->decodeToFd(fd, (unsigned char*)data,
                        len, pixfmt, width, height) < 0)
                        printf("Cannot decode MJPEG");
        if (publish_bgra_ && NvBufSurf::NvTransform(&transform_params, fd, dst_dma_fd)) {
            printf("Cannot NvTransform\n");
        }
        if (publish_gray_ && NvBufSurf::NvTransform(&transform_params, fd, dst_dma_gray_fd)) {
            printf("Cannot NvTransform\n");
        }

        int byte_use = 0;
        cv::Size szSize(1088,5120);
        
        if (publish_bgra_) {
            dump_dmabuf(nvbuf_surf, cpu_data_bgra, byte_use);
            cv::Mat frame(5120, 1088, CV_8UC4, cpu_data_bgra);
            cv::Mat bgr8_frame;
            cv::cvtColor(frame, bgr8_frame, cv::COLOR_BGRA2BGR);
            onImageBGRA(pheader, bgr8_frame);
            if (undistort_color_) {
                cv::Mat img_rectify = cv::Mat::zeros(480*8, 640, CV_8UC4);
                undistort->undistort(frame, img_rectify);
                cv::Mat img_rectify2;
                cv::cvtColor(img_rectify, img_rectify2, cv::COLOR_BGRA2BGR);
                onRectiryBGRA(pheader, img_rectify2);
            }
        }
        byte_use = 0;
        if (publish_gray_) {
            dump_dmabuf(nvbuf_gray_surf, cpu_data_gray, byte_use);
            // printf("2byte_use=%d\n", byte_use);
            cv::Mat frame(5120, 1088, CV_8UC1, cpu_data_gray);
            onImageGRAY(pheader, frame);
            if (undistort_gray_) {
                cv::Mat img_rectify = cv::Mat::zeros(480*8, 640, CV_8U);
                undistort->undistort(frame, img_rectify);
                onRectiryGRAY(pheader, img_rectify);
            }
        }
    }

    bool onTimer(std::pair<uint64_t, uint64_t>& timer_get, std::pair<uint64_t, uint64_t>& timer_set) {
        ros::Time now = ros::Time::now();
        timer_set.first = now.sec;
        timer_set.second = now.nsec;
        return true;
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    SEEKNS::SEEKER seek;
    seeker_device_t sdev;
    std::unique_ptr<image_transport::ImageTransport> it_;
    std::vector<image_transport::Publisher> image_pubs_it_;
    ros::Publisher compressed_image_pub_;
    std::vector<ros::Publisher> image_pubs_ros_;
    std::vector<ros::Publisher> image_gray_pubs_ros_;
    ros::Publisher image_rectify_pubs_ros_;
    ros::Publisher image_rectify_gray_pubs_ros_;
    std::vector<ros::Publisher> depth_pubs_;
    std::vector<ros::Publisher> disparity_pubs_;
    ros::Publisher imu_pub_;
    
    // 参数
    bool use_image_transport_;
    bool pub_disparity_img_;
    bool pub_disparity_;
    bool pub_imu_;
    bool time_sync_;
    bool publish_bgra_;
    bool publish_gray_;
    bool undistort_color_;
    bool undistort_gray_;
    std::string imu_link_;
    std::string imu_topic_;

    // jetson
    NvJPEGDecoder *jpegdec;
    int dst_dma_fd;
    NvBufSurface *nvbuf_surf;
    NvBufSurf::NvCommonTransformParams transform_params;
    int dst_dma_gray_fd;
    NvBufSurface *nvbuf_gray_surf;
    std::shared_ptr<QuadUndistort> undistort;
};

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

class SeekRosNode : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      v4l2_image_publish_ = std::make_shared<SeekRosNodeimpl>(
          getNodeHandle(), getPrivateNodeHandle());
    } catch (std::runtime_error e) {
      ROS_ERROR("%s", e.what());
    }
  }

  std::shared_ptr<SeekRosNodeimpl> v4l2_image_publish_;
};

PLUGINLIB_EXPORT_CLASS(SeekRosNode, nodelet::Nodelet)
