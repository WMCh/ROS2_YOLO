#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"

using namespace std::chrono_literals;

class CameraPublisher : public rclcpp::Node
{
public:
    // 构造函数
    CameraPublisher() : Node("camera_publisher"), fps_(30)
    {
        // 声明参数：摄像头设备号、帧率
        this->declare_parameter<std::string>("video_device", "/dev/video0");
        this->declare_parameter<int>("fps", 30);

        // 获取参数
        this->get_parameter("video_device", video_device_);
        this->get_parameter("fps", fps_);

        // 创建图像发布者
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10);

        // 打开摄像头
        cap_.open(video_device_, cv::CAP_V4L);  // Linux 专用 V4L 后端，更稳定

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开摄像头: %s", video_device_.c_str());
            throw std::runtime_error("摄像头打开失败");
        }

        RCLCPP_INFO(this->get_logger(), "摄像头已打开: %s, 帧率: %d", video_device_.c_str(), fps_);

        // 创建定时器，按帧率发布
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / fps_),
            std::bind(&CameraPublisher::publishImage, this));
    }

    ~CameraPublisher()
    {
        if (cap_.isOpened()) {
            cap_.release();
        }
        RCLCPP_INFO(this->get_logger(), "摄像头已安全释放");
    }

private:
    // 发布图像回调函数
    void publishImage()
    {
        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "读取空帧");
            return;
        }

        // 转换为 ROS2 Image 消息
        auto msg = cv_bridge::CvImage(
            std_msgs::msg::Header(),
            "bgr8",  // OpenCV 默认格式
            frame).toImageMsg();

        // 发布
        image_pub_->publish(*msg);

        RCLCPP_INFO_ONCE(this->get_logger(), "开始发布图像 /camera/image_raw");
    }

    // 成员变量
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
    std::string video_device_;
    int fps_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}