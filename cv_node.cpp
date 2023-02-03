#include <geographic_msgs/GeoPoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <math.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <nav_msgs/Odometry.h>
#include <ros/duration.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>

#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"

class ImageConverter {
    ros::NodeHandle nh;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher pub;
    ros::Subscriber pose_sub;
    geometry_msgs::PoseStamped pose_pos;
    geometry_msgs::Point point_to_origin;

   public:
    void pose_cb(geometry_msgs::PoseStamped msg) {
        this->pose_pos = msg;
    }

    ImageConverter(ros::NodeHandle nh_) : it_(nh_) {
        this->nh = nh_;

        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/webcam/image_raw", 1, &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);

        cv::namedWindow("Detected red circles on the input image", cv::WINDOW_AUTOSIZE);
    }

    ~ImageConverter() {
        cv::destroyWindow("Detected red circles on the input image");
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat orig_image = cv_ptr->image.clone();

        cv::medianBlur(cv_ptr->image, cv_ptr->image, 3);
        cv::Mat hsv_image;
        cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

        cv::Mat lower_red_hue_range;
        cv::Mat upper_red_hue_range;
        cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
        cv::inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);

        cv::Mat red_hue_image;
        cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
        cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);

        // Use the Hough transform to detect circles in the combined threshold image
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(red_hue_image, circles, cv::HOUGH_GRADIENT, 1, red_hue_image.rows / 8, 100, 20, 0, 0);

        // Loop over all detected circles and outline them on the original image

        for (size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
            cv::Point center(std::round(circles[current_circle][0]), std::round(circles[current_circle][1]));
            int radius = std::round(circles[current_circle][2]);

            cv::circle(orig_image, center, radius, cv::Scalar(0, 255, 0), 5);
        }

        if (circles.size() > 0) {
            cv::Point center(std::round(circles[0][0]), std::round(circles[0][1]));

            int x_pos = center.x - (640 / 2);
            int y_pos = (480 / 2) - center.y;

            double alt = this->pose_pos.pose.position.z;
            double x_dist_to_origin, y_dist_to_origin;

            double x_angle = 0.1 * x_pos;
            x_dist_to_origin = alt * tan(x_angle * M_PI / 180);

            double y_angle = 0.1 * y_pos;
            y_dist_to_origin = alt * tan(y_angle * M_PI / 180);

            this->point_to_origin.x = x_dist_to_origin;
            this->point_to_origin.y = y_dist_to_origin;
            this->point_to_origin.z = 0;
        }

        // Show images

        cv::imshow("Detected red circles on the input image", orig_image);

        image_pub_.publish(cv_ptr->toImageMsg());
        cv::waitKey(3);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_converter");
    ros::NodeHandle image_converter;

    ImageConverter ic(image_converter);

    ros::spin();

    return 0;
}
