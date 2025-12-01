#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "seeker.hpp"
#include "NvJpegDecoder.h"
#include "NvBufSurface.h"
#include "quad_undistort.hpp"

class SeekRosNode : public rclcpp::Node {
public:
  explicit SeekRosNode() : Node("seeker_node", rclcpp::NodeOptions().use_intra_process_comms(true)) {
    // 参数声明与获取
    // qos_profile.data_sharing_kind(RMW_DATA_SHARING_AUTOMATIC);
    // declare_parameter("use_image_transport", true);
    declare_parameter("pub_disparity_img", false);
    declare_parameter("pub_disparity", true);
    declare_parameter("pub_imu", true);
    declare_parameter("time_sync", false);
    declare_parameter("imu_link", "imu_link");
    declare_parameter("imu_topic", "/imu_data_raw");
    declare_parameter("publish_bgra", true);
    declare_parameter("publish_gray", false);
    declare_parameter("undistort_color", true);
    declare_parameter("undistort_gray", false);
    declare_parameter("cali_path", "cali_path");

    // use_image_transport_ = get_parameter("use_image_transport").as_bool();
    pub_disparity_img_ = get_parameter("pub_disparity_img").as_bool();
    pub_disparity_ = get_parameter("pub_disparity").as_bool();
    pub_imu_ = get_parameter("pub_imu").as_bool();
    time_sync_ = get_parameter("time_sync").as_bool();
    imu_link_ = get_parameter("imu_link").as_string();
    imu_topic_ = get_parameter("imu_topic").as_string();
    publish_bgra_ = get_parameter("publish_bgra").as_bool();
    publish_gray_ = get_parameter("publish_gray").as_bool();
    undistort_color_ = get_parameter("undistort_color").as_bool();
    undistort_gray_ = get_parameter("undistort_gray").as_bool();
    std::string cali_path_ = get_parameter("cali_path").as_string();

    // 初始化image_transport
    // if (use_image_transport_) {
    //   it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    // }

    // 初始化设备
    std::vector<seeker_device_t> devices = seek_.find_devices();
    if (devices.empty()) {
        RCLCPP_ERROR(get_logger(), "No Seeker Devices Found");
        std::exit(EXIT_FAILURE);
        return;
    }
    seek_.open(devices[0]);

    // 设置回调
    seek_.set_event_callback([this](auto&& ph, auto&& e) { onEvent(ph, e); });
    seek_.set_mjpeg_callback([this](auto&& ph, auto&& d, auto&& l) { onMjpeg(ph, d, l); });
    seek_.set_depth_callback([this](auto&& ph, auto&& d, auto&& l) { onDepth(ph, d, l); });
    
    if (time_sync_) {
        seek_.set_timer_callback([this](auto&& tg, auto&& ts) { return onTimer(tg, ts); });
    }

    // 获取设备信息
    seek_.get_dev_info(sdev_);
    undistort = std::make_shared<QuadUndistort>(cali_path_, QuadUndistort::Backend::VPI_BACKEND);

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

    compressed_image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("all/compressed", 10);

    // 图像发布者
    for (size_t i = 0; i < sdev_.dev_info.rgb_camera_number; ++i) {
    // if (use_image_transport_) {
    //   image_pubs_it_.push_back(it_->advertise(image_topics[i], 1));
    // } else {
        // image_pubs_ros_.push_back(create_publisher<sensor_msgs::msg::Image>(image_topics[i], 10));
        if (publish_bgra_) {
        image_pubs_ros_.push_back(create_publisher<sensor_msgs::msg::Image>(image_topics[i], 10));
            if (undistort_color_) {
                image_rectify_pubs_ros_ = create_publisher<sensor_msgs::msg::Image>("/rectify", 10);
            }
        }
        if (publish_gray_) {
            image_gray_pubs_ros_.push_back(create_publisher<sensor_msgs::msg::Image>(gray_topics[i], 10));
            if (undistort_gray_) {
                image_rectify_gray_pubs_ros_ = create_publisher<sensor_msgs::msg::Image>("/rectify_gray", 10);
            }
        }
    }

    // 深度和视差发布者
    for (size_t i = 0; i < sdev_.dev_info.depth_camera_number; ++i) {
        if (pub_disparity_img_) {
            depth_pubs_.push_back(create_publisher<sensor_msgs::msg::Image>(depth_topics[i], 10));
        }
        if (pub_disparity_) {
            disparity_pubs_.push_back(create_publisher<stereo_msgs::msg::DisparityImage>(disparity_topics[i], 10));
        }
    }

    // IMU发布者
    if (pub_imu_) {
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 200);
    }

    jpegdec = NvJPEGDecoder::createJPEGDecoder("jpegdec");

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

    // 启动设备流
    seek_.start_event_stream();
    seek_.start_image_stream();
    seek_.start_depth_stream();
}

~SeekRosNode() {
    seek_.stop_event_stream();
    seek_.stop_image_stream();
    seek_.stop_depth_stream();
    seek_.close();
}

private:
    void onDepth(const event_header_t& eheader, const uint8_t* data, int len) {
        const int depth_camera_number = sdev_.dev_info.depth_camera_number;
        const int height = sdev_.dev_info.depth_resolution_height / depth_camera_number;
        const int width = sdev_.dev_info.depth_resolution_width;

        std::vector<cv::Mat> images;
        for (int i = 0; i < depth_camera_number; i++) {
            images.emplace_back(height, width, CV_16UC1, const_cast<uint8_t*>(data) + i * height * width * 2);
        }

        // 创建消息头
        auto header = std::make_shared<std_msgs::msg::Header>();
        header->stamp = rclcpp::Time(eheader.sec, eheader.nsec);

        // 发布视差图像
        for (size_t i = 0; i < depth_camera_number; ++i) {
            header->frame_id = "depth" + std::to_string(i);
            sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(*header, "16UC1", images[i]).toImageMsg();
            depth_pubs_[i]->publish(*img_msg);
        }

        // 发布视差消息
        for (size_t i = 0; i < depth_camera_number; ++i) {
            const int DPP = 256/4;
            const double inv_dpp = 1.0 / DPP;
            auto disparity_msg = std::make_shared<stereo_msgs::msg::DisparityImage>();

            // 设置视差图像
            auto& dimage = disparity_msg->image;
            dimage.header = *header;
            dimage.height = images[i].rows;
            dimage.width = images[i].cols;
            dimage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            dimage.step = dimage.width * sizeof(float);
            dimage.data.resize(dimage.step * dimage.height);

            cv::Mat_<float> dmat(images[i].rows, images[i].cols, 
                                reinterpret_cast<float*>(dimage.data.data()));
            images[i].convertTo(dmat, CV_32F, inv_dpp, 0);

            // 设置视差参数
            disparity_msg->f = 320.0;
            disparity_msg->t = 0.04625;
            disparity_msg->min_disparity = 0.0;
            disparity_msg->max_disparity = 192;
            disparity_msg->delta_d = inv_dpp;
            disparity_msg->header = *header;

            disparity_pubs_[i]->publish(*disparity_msg);
        }
    }

    void onEvent(const event_header_t& header, const device_event_t& event) {
        if (event.type == EVENT_TYPE_SENSOR_CUSTOM && pub_imu_) {
            auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
            imu_msg->header.stamp = rclcpp::Time(header.sec, header.nsec);
            imu_msg->header.frame_id = imu_link_;
            
            imu_msg->angular_velocity.x = -event.event.sensor_custom.angular_velocity_x;
            imu_msg->angular_velocity.y = event.event.sensor_custom.angular_velocity_y;
            imu_msg->angular_velocity.z = -event.event.sensor_custom.angular_velocity_z;

            imu_msg->linear_acceleration.x = -event.event.sensor_custom.linear_acceleration_x;
            imu_msg->linear_acceleration.y = event.event.sensor_custom.linear_acceleration_y;
            imu_msg->linear_acceleration.z = -event.event.sensor_custom.linear_acceleration_z;

            imu_pub_->publish(*imu_msg);
        }
    }

    void onImageBGRA(const event_header_t& eheader, cv::Mat& frame) {
        const int cam_num = sdev_.dev_info.rgb_camera_number;
        const int h = frame.rows / cam_num;
        const int w = frame.cols;
        
        auto header = std::make_shared<std_msgs::msg::Header>();
        header->stamp = rclcpp::Time(eheader.sec, eheader.nsec);

        for (int i = 0; i < cam_num; i++) {
            header->frame_id = "cam" + std::to_string(i);
            auto img_msg = cv_bridge::CvImage(*header, "bgr8", frame(cv::Rect(0, i * h, w, h))).toImageMsg();
            image_pubs_ros_[i]->publish(*img_msg);
        }
    }

    void onImageGRAY(const event_header_t& eheader, cv::Mat& frame) {
        const int cam_num = sdev_.dev_info.rgb_camera_number;
        const int h = frame.rows / cam_num;
        const int w = frame.cols;
        
        auto header = std::make_shared<std_msgs::msg::Header>();
        header->stamp = rclcpp::Time(eheader.sec, eheader.nsec);

        for (int i = 0; i < cam_num; i++) {
            header->frame_id = "cam" + std::to_string(i);
            auto img_msg = cv_bridge::CvImage(*header, "mono8", frame(cv::Rect(0, i * h, w, h))).toImageMsg();
            image_gray_pubs_ros_[i]->publish(*img_msg);
        }
    }

    void onRectiryBGRA(const event_header_t& eheader, const cv::Mat& frame) {
        auto header = std::make_shared<std_msgs::msg::Header>();
        header->stamp = rclcpp::Time(eheader.sec, eheader.nsec);
        auto img_msg = cv_bridge::CvImage(*header, "bgr8", frame).toImageMsg();
        image_rectify_pubs_ros_->publish(*img_msg);
    }

    void onRectiryGRAY(const event_header_t& eheader, const cv::Mat& frame) {
        auto header = std::make_shared<std_msgs::msg::Header>();
        header->stamp = rclcpp::Time(eheader.sec, eheader.nsec);
        auto img_msg = cv_bridge::CvImage(*header, "mono8", frame).toImageMsg();
        image_rectify_gray_pubs_ros_->publish(*img_msg);
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

  void onMjpeg(const event_header_t& pheader, const uint8_t* data, int len) {
        // publish compressed img
        auto compressed = std::make_shared<sensor_msgs::msg::CompressedImage>();
        compressed->header.stamp = rclcpp::Time(pheader.sec, pheader.nsec);
        compressed->format = "jpeg";

        compressed->data.resize(len);
        memcpy(compressed->data.data(), data, len);

        compressed_image_pub_->publish(*compressed);

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
        auto now = this->now();
        timer_set.first = now.seconds();
        timer_set.second = now.nanoseconds() % 1000000000;
        return true;
    }

    // ROS2接口
    //   std::shared_ptr<image_transport::ImageTransport> it_;
    //   std::vector<image_transport::Publisher> image_pubs_it_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_pub_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> image_pubs_ros_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> image_gray_pubs_ros_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_rectify_pubs_ros_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_rectify_gray_pubs_ros_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> depth_pubs_;
    std::vector<rclcpp::Publisher<stereo_msgs::msg::DisparityImage>::SharedPtr> disparity_pubs_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    // 设备接口
    SEEKNS::SEEKER seek_;
    seeker_device_t sdev_;

    // 参数
    // bool use_image_transport_;
    bool pub_disparity_img_;
    bool pub_disparity_;
    bool pub_imu_;
    bool time_sync_;
    std::string imu_link_;
    std::string imu_topic_;

    //
    bool publish_bgra_;
    bool publish_gray_;
    bool undistort_color_;
    bool undistort_gray_;

    // nvidia
    NvJPEGDecoder *jpegdec;
    int dst_dma_fd;
    NvBufSurface *nvbuf_surf;
    NvBufSurf::NvCommonTransformParams transform_params;
    int dst_dma_gray_fd;
    NvBufSurface *nvbuf_gray_surf;
    std::shared_ptr<QuadUndistort> undistort;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SeekRosNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
