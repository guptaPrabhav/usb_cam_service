#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageToggleService : public rclcpp::Node
{
public:
    ImageToggleService()
        : Node("image_toggle_service"), use_grayscale_(false)
    {
        // Subscription to the input image topic
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10,
            std::bind(&ImageToggleService::imageCallback, this, std::placeholders::_1));

        // Publisher to the output image topic
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/image_processed", 10);

        // Service for toggling grayscale mode
        toggle_service_ = this->create_service<std_srvs::srv::SetBool>(
            "toggle_grayscale",
            std::bind(&ImageToggleService::handleToggleRequest, this, std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "Image Toggle Service initialized.");
    }

private:
void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat input_image = cv_ptr->image;
    cv::Mat processed_image;

    // Check the number of channels in the input image
    int channels = input_image.channels();
    if (use_grayscale_)
    {
        if (channels == 3) // RGB/BGR
        {
            cv::cvtColor(input_image, processed_image, cv::COLOR_BGR2GRAY);
            cv_ptr->encoding = "mono8"; // Update encoding for grayscale
        }
        else if (channels == 4) // RGBA/BGRA
        {
            cv::cvtColor(input_image, processed_image, cv::COLOR_BGRA2GRAY);
            cv_ptr->encoding = "mono8";
        }
        else if (channels == 2) // 2-channel (assume YUV-like format)
        {
            RCLCPP_INFO(this->get_logger(), "Processing 2-channel YUV image for grayscale.");
            std::vector<cv::Mat> channels_vec;
            cv::split(input_image, channels_vec); // Split Y and UV channels
            processed_image = channels_vec[0];   // Use only the luminance channel (Y)
            cv_ptr->encoding = "mono8";
        }
        else if (channels == 1) // Already grayscale
        {
            RCLCPP_INFO(this->get_logger(), "Image is already grayscale.");
            processed_image = input_image;
            cv_ptr->encoding = "mono8";
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Unsupported number of channels: %d", channels);
            return;
        }
    }
    else
    {
        if (channels == 1) // Grayscale to BGR
        {
            cv::cvtColor(input_image, processed_image, cv::COLOR_GRAY2BGR);
            cv_ptr->encoding = "bgr8";
        }
        else if (channels == 3) // RGB/BGR, no change needed
        {
            processed_image = input_image;
            cv_ptr->encoding = "bgr8";
        }
        else if (channels == 4) // RGBA/BGRA to BGR
        {
            cv::cvtColor(input_image, processed_image, cv::COLOR_BGRA2BGR);
            cv_ptr->encoding = "bgr8";
        }
        else if (channels == 2) // 2-channel (assume YUV-like format)
        {
            RCLCPP_INFO(this->get_logger(), "Processing 2-channel YUV image for color.");
            std::vector<cv::Mat> channels_vec;
            cv::split(input_image, channels_vec); // Split Y and UV channels

            // Create a dummy UV channel with zero chroma for simplicity
            cv::Mat uv_dummy = cv::Mat::zeros(channels_vec[0].size(), channels_vec[1].type());

            // Merge back to a 3-channel YUV image for conversion
            cv::Mat yuv_image;
            cv::merge(std::vector<cv::Mat>{channels_vec[0], uv_dummy, uv_dummy}, yuv_image);

            // Convert YUV to BGR
            cv::cvtColor(yuv_image, processed_image, cv::COLOR_YUV2BGR);
            cv_ptr->encoding = "bgr8";
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Unsupported number of channels: %d", channels);
            return;
        }
    }

    // Update the processed image
    cv_ptr->image = processed_image;

    // Publish the processed image
    image_publisher_->publish(*cv_ptr->toImageMsg());
}




    void handleToggleRequest(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        use_grayscale_ = request->data; // Update the mode
        response->success = true;
        response->message = use_grayscale_ ? "Switched to grayscale mode." : "Switched to color mode.";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr toggle_service_;

    bool use_grayscale_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageToggleService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
