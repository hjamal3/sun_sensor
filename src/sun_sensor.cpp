#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "math.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// debugging
#include <iostream>

// data collection parameters
int num_data = 0;
int imu_counter = 0;

// data storage elements
Eigen::Vector3d sum_accel;
Eigen::Vector3d sum_gyro;

// sun sensor
Eigen::Vector2d sun_sensor_pixel; // updated by vision code

// camera calibration
Eigen::Matrix3d K;
Eigen::Matrix3d K_inv;
Eigen::Quaterniond q_bc; // camera to body frame transformation


// converts pixel to image point: x = inv(K)*u
Eigen::Vector3d pixel_to_unit_vector(const Eigen::Vector2d & u, const Eigen::Matrix3d & K_inv)
{
	Eigen::Vector3d u_3d(u[0], u[1], u[2]);
	return K_inv*u_3d;
}

// convert unit vector in camera frame to body frame
Eigen::Vector3d camera_to_body_frame(const Eigen::Vector3d & x_camera, const Eigen::Quaterniond & q_bc)
{
	return q_bc._transformVector(x_camera);
}

Eigen::Quaterniond compute_q_wb()
{
	// compute initial orientation
	Eigen::Vector3d g_b = sum_accel / num_data;

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

double compute_heading()
{
	Eigen::Vector3d x_camera = pixel_to_unit_vector(sun_sensor_pixel, K_inv);
	Eigen::Vector3d x_body = camera_to_body_frame(x_camera, q_bc);
	Eigen::Quaterniond q_wb = compute_q_wb();
	Eigen::Vector3d x_world = body_to_global(x_body, q_wb);
	Eigen::Vector2d x_world_2d = project_3d_to_2d(x_world);
	double heading = get_angle(x_world_2d);
	double correction = compute_correction(0);
	return heading_correction(heading, correction);
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
	if (imu_counter < num_data)
	{
		// get accelerometer data
		geometry_msgs::Vector3 a = msg->linear_acceleration;

		// get gyroscope data
		geometry_msgs::Vector3 w = msg->angular_velocity;

		// add to matrix
		sum_accel -= Eigen::Vector3d(a.x,a.y,a.z);
		sum_gyro += Eigen::Vector3d(w.x,w.y,w.z);

		// increment counter
		imu_counter++;
	} else 
	{
		double heading = compute_heading();

		// publish the heading

		imu_counter = 0;
	}
}


// service handle

int main(int argc, char **argv)
{
  ROS_INFO("init_ekf node started.");

  ros::init(argc, argv, "init_imu_ekf_node");

  // create node handle and pointer
  ros::NodeHandle n;

  // get number of data items to average from parameter server
  n.param("num_data", num_data,1000);

  // camera calibration matrix
  double fx = 0, fy = 0, cx = 0, cy = 1;
  K <<  fx, 0, cx, 
  		0, fy, cy, 
  		0,  0, 1;
  K_inv = K.inverse();
  // imu callback
  ros::Subscriber sub_imu = n.subscribe("/imu/data", 10, imu_callback);
  ros::spin();
}
