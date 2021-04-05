// opencv
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>

// eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

// ros
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>

// stl / cpp
#include <iostream>
#include <vector>
#include "math.h"


// data collection parameters
int num_data = 0;
int imu_counter = 0;

// data storage elements
Eigen::Vector3d sum_accel;

// sun sensor
cv::Point2f sun_sensor_pixel; // updated by vision code
bool detected_sun = false;

// blob
cv::Ptr<cv::SimpleBlobDetector> blob_detector; 

// camera calibration
cv::Mat K;
cv::Mat dist_coeffs;

// camera to body frame transformation
Eigen::Quaterniond q_bc;


// gravity
const double gravity = 9.80665;

void init_blob_detector()
{
    // Setup SimpleBlobDetector parameters.
    cv::SimpleBlobDetector::Params params;

    // Filter by Area.
    params.filterByArea = false;
    params.minArea = 100;
    params.maxArea = 100000;

    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0.6;
    params.maxCircularity = 1.0;

    // Filter by Convexity
    params.filterByConvexity = true;
    params.minConvexity= 0.01;

    // Filter by Inertia
    params.filterByInertia = true;
    params.minInertiaRatio = 0.8;

    blob_detector = cv::SimpleBlobDetector::create(params);
}

// convert image to grey scale
void color_to_gray(const cv::Mat & img, cv::Mat & img_gray)
{
    cv::cvtColor(img,img_gray,cv::COLOR_RGB2GRAY);
}

// blur the image to reduce noise
void blur_image(cv::Mat & img)
{
    cv::medianBlur(img, img, 5);
}

// threshold the image
void threshold_image(const cv::Mat & img_gray, cv::Mat & img_bw)
{
    cv::threshold(img_gray, img_bw, 245, 255, CV_THRESH_BINARY);
}

// find the center of the circle
bool find_circle(const cv::Mat& img_orig, const cv::Mat& img_bw, cv::Point2f & center)
{
    cv::Mat img_flip;
    cv::bitwise_not(img_bw,img_flip); // needs to be BW

    std::vector<cv::KeyPoint> blobs;
    blob_detector->detect( img_flip, blobs);

    // just want one blob
    if (!blobs.size() || (blobs.size()>1))
    {
        return false;
    }
    center = blobs[0].pt;
    return true;
    
    // // // uncomment if plotting
    // for (auto blob : blobs)
    // {
    //     auto circle = blob.pt;
    //     auto radius = blob.size/2;
    //     // std::cout << radius << std::endl;
    //     cv::circle(img_flip, circle, radius,  cv::Scalar(0, 0, 0));
    // }
    // cv::Mat im_with_keypoints;
    // cv::drawKeypoints( img_flip, blobs, im_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    // cv::imshow("keypoints", im_with_keypoints );
    // cv::waitKey(0);

}

// all image operations in image call back
void img_callback(const sensor_msgs::ImageConstPtr& image_ros)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
            cv_ptr = cv_bridge::toCvCopy(image_ros, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
            return;
    }
    cv::Mat img_gray = cv_ptr->image;
    cv::Mat img_bw;
    blur_image(img_gray);
    threshold_image(img_gray, img_bw);
    detected_sun = find_circle(img_gray, img_bw, sun_sensor_pixel);
}

// converts polar angles to unit vector
Eigen::Vector3d polar_angles_to_unit_vector(double angle_a, double angle_b)
{

}

// converts pixel to undistorted image point: x' = inv(K)*u, x'' = undistort(x')
Eigen::Vector3d pixel_to_unit_vector(const cv::Point2f u)
{
    std::vector<cv::Point2f>  u_vec = {u};
    std::vector<cv::Point2f> x_pp_vec;
    cv::undistortPoints(u_vec, x_pp_vec, K, dist_coeffs);
    cv::Point2f x_pp = x_pp_vec.at(0);
    Eigen::Vector3d x_pp_3d(x_pp.x, x_pp.y, 1);
    return x_pp_3d;
}

// convert unit vector in camera frame to body frame
Eigen::Vector3d camera_to_body_frame(const Eigen::Vector3d & x_camera, const Eigen::Quaterniond & q_bc)
{
    return q_bc._transformVector(x_camera);
}

// compute rotation matrix of stage (roll and pitch)
Eigen::Quaterniond compute_q_wb_stage(double angle_a, double angle_b)
{

}

// compute rotation matrix of IMU (roll and pitch)
Eigen::Quaterniond compute_q_wb_imu(Eigen::Vector3d sum_acc)
{
    // compute initial orientation
    Eigen::Vector3d g_b = sum_acc / num_data;

    // initial roll (phi) and pitch (theta)
    double phi = atan2(-g_b[1],-g_b[2]);
    double theta = atan2(g_b[0], sqrt(g_b[1]*g_b[1] + g_b[2]*g_b[2]));

    // set initial yaw to zero
    double psi = 0;

    // q is navigation to body transformation: R_bw
    // YPR: R_wb = R(yaw)R(pitch)R(Roll)
    // RPY: R_bw = R(-Roll)R(-Pitch)R(-yaw) 
    Eigen::Quaternion<double> q_bw = Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitX());

    // make q into body to navigation frame transformation: R_wb
    return q_bw.inverse(); 
}

// convert unit vector in body frame to global frame
Eigen::Vector3d body_to_global(const Eigen::Vector3d & x_body, const Eigen::Quaterniond & q_wb)
{
    return q_wb._transformVector(x_body);
}

// project 3D vector to the ground
Eigen::Vector2d project_3d_to_2d(const Eigen::Vector3d & x_3d)
{
    Eigen::Vector2d x_2d;
    x_2d << x_3d[0], x_3d[1];
    return x_2d;
}

// 2d vector to angle
double get_angle(const Eigen::Vector2d & x_2d)
{
    return atan2(x_2d[1],x_2d[0]);
}

// compute heading correction
double compute_correction(double time)
{
    return time; // fix this!
}

// heading correction
double heading_correction(double heading, double correction)
{
    return heading + correction;
}

bool compute_heading(Eigen::Vector3d sum_acc, double & heading)
{
    if (detected_sun)
    {
        // reset
        detected_sun = false;

        // get 3d ray to sun
        Eigen::Vector3d x_camera = pixel_to_unit_vector(sun_sensor_pixel);

        // reset
        sun_sensor_pixel = {0,0};

        // convert 3d ray to body frame
        Eigen::Vector3d x_body = camera_to_body_frame(x_camera, q_bc);

        // compute body to global transformation
        Eigen::Quaterniond q_wb = compute_q_wb_imu(sum_acc);

        // convert 3d ray to global frame
        Eigen::Vector3d x_world = body_to_global(x_body, q_wb);

        // convert 3d ray to 2d ray
        Eigen::Vector2d x_world_2d = project_3d_to_2d(x_world);

        // compute angle of 2d ray
        double heading = get_angle(x_world_2d);

        // correct the angle based on time
        double correction = compute_correction(0);
        heading = heading_correction(heading, correction);

        return true;
    }
    return false;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // get accelerometer data
    geometry_msgs::Vector3 a = msg->linear_acceleration;

    // Eigen
    Eigen::Vector3d v(a.x,a.y,a.x);

    // check if robot is stationary
    if (abs(v.norm() - gravity) < 0.1)
    {
        imu_counter++;
    } else 
    {
        sum_accel = {0,0,0};
        imu_counter = 0;
    }

    if (imu_counter < num_data)
    {
        // add to matrix
        sum_accel -= Eigen::Vector3d(a.x,a.y,a.z);

    } else 
    {
        double heading;
        if (compute_heading(sum_accel, heading))
        {
            // publish heading
            std::cout << heading << std::endl;
        }

        // publish the heading
        sum_accel = {0,0,0};
        imu_counter = 0;
    }
}


// service handle

int main(int argc, char **argv)
{
    ROS_INFO("sun sensor node started.");

    ros::init(argc, argv, "sun_sensor_node");

    // create node handle and pointer
    ros::NodeHandle n;

    // get number of data items to average from parameter server
    n.param("num_data", num_data,250);

    // camera calibration matrix
    double fx = 0, fy = 0, cx = 0, cy = 1;
    cv::Mat K = (cv::Mat_<float>(3, 3) <<  fx, 0, cx, 0, fy, cy, 0,  0, 1);;
    // TODO: store dist
    // TODO: store q_bc (use the matlab script)


    // code for testing
    // std::string image_path = "/home/X/Desktop/sun1.jpg";
    // cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
    // if(img.empty())
    // {
    //     std::cout << "Could not read the image: " << image_path << std::endl;
    //     return 1;
    // }

    // message_filters::Subscriber<sensor_msgs::Image> image1_sub(n, "/stereo/left/image_rect", 1);

    init_blob_detector();
    // img_callback(img);


    // imu callback
    ros::Subscriber sub_imu = n.subscribe("/imu/data", 10, imu_callback);
    ros::Subscriber sub_img = n.subscribe("/stereo/left/image_raw", 10, img_callback);

    ros::spin();
}
