#include "seeker.hpp"
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stereo_msgs/DisparityImage.h>
#include <Eigen/Dense>
#include "quad_undistort.hpp"

class SeekRosNodeimpl {
public:
    SeekRosNodeimpl(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh) :
        nh_(nh), private_nh_(private_nh) {
            printf("backend\n");

        private_nh_.param("publish_bgra", publish_bgra_, true);
        private_nh_.param("publish_gray", publish_gray_, false);
        private_nh_.param("undistort_color", undistort_color_, true);
        private_nh_.param("undistort_gray", undistort_gray_, false);

        // 读取参数
        private_nh_.param("use_image_transport", use_image_transport_, true);
        private_nh_.param("pub_disparity_img", pub_disparity_img_, false);
        private_nh_.param("pub_disparity", pub_disparity_, true);
        private_nh_.param("pub_imu", pub_imu_, true);
        private_nh_.param("time_sync", time_sync_, true);
        private_nh_.param("imu_link", imu_link_, std::string("imu_link"));
        private_nh_.param("imu_topic", imu_topic_, std::string("imu_data_raw"));
        private_nh_.param("img_pub_intervals", img_pub_intervals_, 2);
        private_nh_.param("rect_cat_pub_intervals", rect_cat_pub_intervals_, 2);
        std::string cali_path_;
        private_nh_.param("cali_path", cali_path_, std::string("cali_path"));
        private_nh_.param("output_width", output_width_, 0);
        private_nh_.param("output_height", output_height_, 0);
        private_nh_.param("output_fx", output_fx_, 0);
        private_nh_.param("output_fy", output_fy_, 0);

        constexpr int kDefaultUndistortWidth = 640;
        constexpr int kDefaultUndistortHeight = 480;
        constexpr int kDefaultUndistortFx = 320;
        constexpr int kDefaultUndistortFy = 320;

        if (output_width_ <= 0) {
            output_width_ = kDefaultUndistortWidth;
        }
        if (output_height_ <= 0) {
            output_height_ = kDefaultUndistortHeight;
        }
        if (output_fx_ <= 0) {
            output_fx_ = kDefaultUndistortFx;
        }
        if (output_fy_ <= 0) {
            output_fy_ = kDefaultUndistortFy;
        }

        ROS_INFO("Undistort output params: width=%d height=%d fx=%d fy=%d",
                 output_width_, output_height_, output_fx_, output_fy_);
        if (img_pub_intervals_ < 1) {
            img_pub_intervals_ = 1;
        }
        if (rect_cat_pub_intervals_ < 1) {
            rect_cat_pub_intervals_ = 1;
        }

        // imu w.r.t. base_link
        imu_wrt_base_ << -1,  0,  0,
                          0, -1,  0,
                          0,  0,  1;

        img_pub_cnt_ = 0;
        rect_cat_pub_cnt_ = 0;

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
        if (undistort_color_) {
            undistort_color_impl_ = std::make_shared<QuadUndistort>(
                output_width_, output_height_ * 8, output_fx_, output_fy_,
                cali_path_, QuadUndistort::Backend::VPI_BACKEND);
        }
        if (undistort_gray_) {
            undistort_gray_impl_ = std::make_shared<QuadUndistort>(
                output_width_, output_height_* 8, output_fx_, output_fy_,
                cali_path_, QuadUndistort::Backend::VPI_BACKEND);
        }

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
        const std::vector<std::string> rect_cat_topics = {
            "/rect_cat/left/image_raw",
            "/rect_cat/right/image_raw",
            "/rect_cat/bright/image_raw",
            "/rect_cat/bleft/image_raw",
        };
        const std::vector<std::string> rect_cat_gray_topics = {
            "/rect_cat_gray/left/image_raw",
            "/rect_cat_gray/right/image_raw",
            "/rect_cat_gray/bright/image_raw",
            "/rect_cat_gray/bleft/image_raw",
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

        // 图像发布者
        for (size_t i = 0; i < sdev.dev_info.rgb_camera_number; ++i) {
            if (publish_bgra_) {
                if (use_image_transport_) {
                    image_pubs_it_.push_back(it_->advertise(image_topics[i], 10));
                } else {
                    image_pubs_ros_.push_back(nh_.advertise<sensor_msgs::Image>(image_topics[i], 10));
                }
            }
            if (publish_gray_) {
                if (use_image_transport_) {
                    image_gray_pubs_it_.push_back(it_->advertise(gray_topics[i], 10));
                } else {
                    image_gray_pubs_ros_.push_back(nh_.advertise<sensor_msgs::Image>(gray_topics[i], 10));
                }
            }
        }

        if (publish_bgra_ && undistort_color_) {
            for (const auto& topic : rect_cat_topics) {
                if (use_image_transport_) {
                    rect_cat_pubs_it_.push_back(it_->advertise(topic, 10));
                } else {
                    rect_cat_pubs_ros_.push_back(nh_.advertise<sensor_msgs::Image>(topic, 10));
                }
            }
        }

        if (publish_gray_ && undistort_gray_) {
            for (const auto& topic : rect_cat_gray_topics) {
                if (use_image_transport_) {
                    rect_cat_gray_pubs_it_.push_back(it_->advertise(topic, 10));
                } else {
                    rect_cat_gray_pubs_ros_.push_back(nh_.advertise<sensor_msgs::Image>(topic, 10));
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
    }

private:
    void onDepth(event_header_t& pheader, const uint8_t* data, int len) {
        const int depth_camera_number = sdev.dev_info.depth_camera_number;
        const int height = sdev.dev_info.depth_resolution_height/depth_camera_number;
        const int width = sdev.dev_info.depth_resolution_width;
        
        std::vector<cv::Mat> images(depth_camera_number);
        const uint16_t* depth_ptr = reinterpret_cast<const uint16_t*>(data);
        for (int i = 0; i < depth_camera_number; i++) {
            images.at(i) = cv::Mat(height, width, CV_16UC1,
                                   const_cast<uint16_t*>(depth_ptr + i * height * width));
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
            
            // 将IMU数据转换为 w.r.t. base_link 的坐标系
            Eigen::Vector3d angular_velocity(
                event.event.sensor_custom.angular_velocity_x,
                event.event.sensor_custom.angular_velocity_y,
                event.event.sensor_custom.angular_velocity_z
            );
            Eigen::Vector3d linear_acceleration(
                event.event.sensor_custom.linear_acceleration_x,
                event.event.sensor_custom.linear_acceleration_y,
                event.event.sensor_custom.linear_acceleration_z
            );
            Eigen::Vector3d angular_velocity_wrt_base = imu_wrt_base_ * angular_velocity;
            Eigen::Vector3d linear_acceleration_wrt_base = imu_wrt_base_ * linear_acceleration;

            imu_msg.angular_velocity.x = angular_velocity_wrt_base.x();
            imu_msg.angular_velocity.y = angular_velocity_wrt_base.y();
            imu_msg.angular_velocity.z = angular_velocity_wrt_base.z();

            imu_msg.linear_acceleration.x = linear_acceleration_wrt_base.x();
            imu_msg.linear_acceleration.y = linear_acceleration_wrt_base.y();
            imu_msg.linear_acceleration.z = linear_acceleration_wrt_base.z();
            
            // imu_msg.angular_velocity.x = event.event.sensor_custom.angular_velocity_x;
            // imu_msg.angular_velocity.y = event.event.sensor_custom.angular_velocity_y;
            // imu_msg.angular_velocity.z = event.event.sensor_custom.angular_velocity_z;

            // imu_msg.linear_acceleration.x = event.event.sensor_custom.linear_acceleration_x;
            // imu_msg.linear_acceleration.y = event.event.sensor_custom.linear_acceleration_y;
            // imu_msg.linear_acceleration.z = event.event.sensor_custom.linear_acceleration_z;

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
            if (use_image_transport_) {
                image_gray_pubs_it_[i].publish(img_msg);
            } else {
                image_gray_pubs_ros_[i].publish(img_msg);
            }
        }
    }

    void publishRectCat(const event_header_t& eheader, const cv::Mat& frame) {
        const int expected_cam_slices = 8;
        const bool has_publishers = use_image_transport_ ? !rect_cat_pubs_it_.empty() : !rect_cat_pubs_ros_.empty();
        if (frame.rows % expected_cam_slices != 0 || frame.cols == 0 || !has_publishers) {
            return;
        }

        const int single_h = frame.rows / expected_cam_slices;
        const int single_w = frame.cols;

        std::vector<cv::Mat> slices;
        slices.reserve(expected_cam_slices);
        for (int i = 0; i < expected_cam_slices; ++i) {
            slices.emplace_back(frame(cv::Rect(0, i * single_h, single_w, single_h)));
        }

        std::vector<cv::Mat> cat_imgs;
        cat_imgs.reserve(expected_cam_slices / 2);
        for (int i = 0; i < expected_cam_slices; i += 2) {
            cv::Mat cat;
            cv::hconcat(slices[i], slices[i + 1], cat);
            cat_imgs.push_back(cat);
        }

        std_msgs::Header header;
        header.stamp = ros::Time(eheader.sec, eheader.nsec);
        header.seq = eheader.seq;

        const std::vector<std::string> frame_ids = {
            "rect_cat_left",
            "rect_cat_right",
            "rect_cat_bright",
            "rect_cat_bleft"
        };

        const size_t available_pubs = use_image_transport_ ? rect_cat_pubs_it_.size() : rect_cat_pubs_ros_.size();
        const size_t publish_count = std::min(cat_imgs.size(), available_pubs);
        for (size_t i = 0; i < publish_count; ++i) {
            header.frame_id = i < frame_ids.size() ? frame_ids[i] : ("rect_cat_" + std::to_string(i));
            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "bgr8", cat_imgs[i]).toImageMsg();
            if (use_image_transport_) {
                rect_cat_pubs_it_[i].publish(img_msg);
            } else {
                rect_cat_pubs_ros_[i].publish(img_msg);
            }
        }
    }

    void publishRectCatGray(const event_header_t& eheader, const cv::Mat& frame) {
        const int expected_cam_slices = 8;
        const bool has_publishers = use_image_transport_ ? !rect_cat_gray_pubs_it_.empty() : !rect_cat_gray_pubs_ros_.empty();
        if (frame.rows % expected_cam_slices != 0 || frame.cols == 0 || !has_publishers) {
            return;
        }

        const int single_h = frame.rows / expected_cam_slices;
        const int single_w = frame.cols;

        std::vector<cv::Mat> slices;
        slices.reserve(expected_cam_slices);
        for (int i = 0; i < expected_cam_slices; ++i) {
            slices.emplace_back(frame(cv::Rect(0, i * single_h, single_w, single_h)));
        }

        std::vector<cv::Mat> cat_imgs;
        cat_imgs.reserve(expected_cam_slices / 2);
        for (int i = 0; i < expected_cam_slices; i += 2) {
            cv::Mat cat;
            cv::hconcat(slices[i], slices[i + 1], cat);
            cat_imgs.push_back(cat);
        }

        std_msgs::Header header;
        header.stamp = ros::Time(eheader.sec, eheader.nsec);
        header.seq = eheader.seq;

        const std::vector<std::string> frame_ids = {
            "rect_cat_gray_left",
            "rect_cat_gray_right",
            "rect_cat_gray_bright",
            "rect_cat_gray_bleft"
        };

        const size_t available_pubs = use_image_transport_ ? rect_cat_gray_pubs_it_.size() : rect_cat_gray_pubs_ros_.size();
        const size_t publish_count = std::min(cat_imgs.size(), available_pubs);
        for (size_t i = 0; i < publish_count; ++i) {
            header.frame_id = i < frame_ids.size() ? frame_ids[i] : ("rect_cat_gray_" + std::to_string(i));
            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "mono8", cat_imgs[i]).toImageMsg();
            if (use_image_transport_) {
                rect_cat_gray_pubs_it_[i].publish(img_msg);
            } else {
                rect_cat_gray_pubs_ros_[i].publish(img_msg);
            }
        }
    }

    void onMjpeg(event_header_t& pheader, const uint8_t* data, int len) {
        const bool publish_raw = (img_pub_cnt_ % img_pub_intervals_) == 0;
        const bool publish_rect = (rect_cat_pub_cnt_ % rect_cat_pub_intervals_) == 0;

        if (!publish_raw && !publish_rect) {
            img_pub_cnt_ += 1;
            rect_cat_pub_cnt_ += 1;
            return;
        }

        cv::Mat jpeg_buffer(1, len, CV_8UC1, const_cast<uint8_t*>(data));
        cv::Mat frame = cv::imdecode(jpeg_buffer, cv::IMREAD_COLOR);
        if (frame.empty()) {
            ROS_WARN("Failed to decode MJPEG frame with OpenCV");
            img_pub_cnt_ = 0;
            rect_cat_pub_cnt_ = 0;
            img_pub_cnt_ += 1;
            rect_cat_pub_cnt_ += 1;
            return;
        }

        cv::Mat gray_frame;
        const bool need_gray_frame = publish_gray_ && (publish_raw || (publish_rect && undistort_gray_ && undistort_gray_impl_));
        if (need_gray_frame) {
            cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
        }

        if (publish_raw) {
            if (publish_bgra_) {
                onImageBGRA(pheader, frame);
            }
            if (publish_gray_) {
                onImageGRAY(pheader, gray_frame);
            }
        }

        if (publish_rect) {
            if (publish_bgra_ && undistort_color_ && undistort_color_impl_) {
                cv::Mat frame_bgra;
                cv::cvtColor(frame, frame_bgra, cv::COLOR_BGR2BGRA);
                cv::Mat img_rectify = cv::Mat::zeros(output_height_ * 8, output_width_, CV_8UC4);
                undistort_color_impl_->undistort(frame_bgra, img_rectify);
                cv::Mat img_rectify_bgr;
                cv::cvtColor(img_rectify, img_rectify_bgr, cv::COLOR_BGRA2BGR);
                publishRectCat(pheader, img_rectify_bgr);
            }
            if (publish_gray_ && undistort_gray_ && undistort_gray_impl_) {
                if (gray_frame.empty()) {
                    cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
                }
                cv::Mat img_rectify = cv::Mat::zeros(output_height_ * 8, output_width_, CV_8U);
                undistort_gray_impl_->undistort(gray_frame, img_rectify);
                publishRectCatGray(pheader, img_rectify);
            }
        }

        if (publish_raw) {
            img_pub_cnt_ = 0;
        }
        if (publish_rect) {
            rect_cat_pub_cnt_ = 0;
        }

        img_pub_cnt_ += 1;
        rect_cat_pub_cnt_ += 1;
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
    std::vector<image_transport::Publisher> image_gray_pubs_it_;
    std::vector<image_transport::Publisher> rect_cat_pubs_it_;
    std::vector<image_transport::Publisher> rect_cat_gray_pubs_it_;
    std::vector<ros::Publisher> image_pubs_ros_;
    std::vector<ros::Publisher> image_gray_pubs_ros_;
    std::vector<ros::Publisher> rect_cat_pubs_ros_;
    std::vector<ros::Publisher> rect_cat_gray_pubs_ros_;
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
    int img_pub_intervals_;
    int img_pub_cnt_;
    int rect_cat_pub_intervals_;
    int rect_cat_pub_cnt_;
    int output_width_;
    int output_height_;
    int output_fx_;
    int output_fy_;

    Eigen::Matrix3d imu_wrt_base_;  // IMU w.r.t. base_link 的变换矩阵

    std::shared_ptr<QuadUndistort> undistort_color_impl_;
    std::shared_ptr<QuadUndistort> undistort_gray_impl_;
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
