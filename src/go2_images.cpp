#include <unitree/robot/go2/video/video_client.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "video_publisher");
    ros::NodeHandle nh;

    // Create a ROS publisher
    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("go2_camera/image_raw", 10);

    /*
     * Initialize ChannelFactory
     */
    unitree::robot::ChannelFactory::Instance()->Init(0);
    unitree::robot::go2::VideoClient video_client;

    /*
     * Set request timeout 1.0s
     */
    video_client.SetTimeout(1.0f);
    video_client.Init();

    // Test Api
    std::vector<uint8_t> image_sample;
    int ret;

    while (ros::ok())
    {
        ret = video_client.GetImageSample(image_sample);

        if (ret == 0) {
            cv::Mat image(cv::imdecode(image_sample, cv::IMREAD_COLOR));

            if (!image.empty()) {
                // Convert OpenCV image to ROS Image message
                sensor_msgs::Image ros_image;
                cv_bridge::CvImage cv_image;
                cv_image.image = image;
                cv_image.encoding = "bgr8"; 

                cv_image.toImageMsg(ros_image);

                image_pub.publish(ros_image);
                // ROS_INFO("Image published successfully.");
            } else {
                ROS_ERROR("Failed to decode image.");
            }
        } else {
            ROS_WARN("Failed to get image sample.");
        }

    }

    return 0;
}
