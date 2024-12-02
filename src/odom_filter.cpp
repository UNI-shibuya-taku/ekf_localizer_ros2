#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
// #include <sensor_msgs/Imu.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/transform_listener.h>
# define M_PI       3.14159265358979323846 
class OdomFilter
{
public:
	OdomFilter();
	~OdomFilter();
	void process();

private:
	void odom_callback(const nav_msgs::OdometryConstPtr& msg);
	double get_yaw(geometry_msgs::Quaternion q);
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	ros::Subscriber odom_sub_;
	ros::Publisher odom_filter_pub_;
	nav_msgs::Odometry current_odom_;
	// nav_msgs::Odometry last_odom_;
	bool init_callback = false;
	double current_linear;
	double last_linear;
	double current_yaw;
	double last_yaw;
};
OdomFilter::OdomFilter() :
	private_nh_("~")
{
	odom_sub_ = nh_.subscribe("/atom/odometry",10,&OdomFilter::odom_callback,this);
	odom_filter_pub_ = nh_.advertise<nav_msgs::Odometry>("/atom/odometry/filter",10);
}
double OdomFilter::get_yaw(geometry_msgs::Quaternion q)
{
    double roll , pitch , yaw;
    tf::Quaternion quat(q.x, q.y, q.z, q.w);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return yaw;
}
OdomFilter::~OdomFilter()
{
}
void OdomFilter::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
	current_odom_ = *msg;
	// filterinng
	// current_yaw = get_yaw(current_odom_.pose.pose.orientation);
	current_linear = current_odom_.twist.twist.linear.x;
	double diff_linear = current_linear - current_linear;
	current_yaw = tf2::getYaw(current_odom_.pose.pose.orientation);
	double diff_yaw = current_yaw - last_yaw;
    diff_yaw = atan2(sin(diff_yaw), cos(diff_yaw));

	double left_wheel = ((diff_linear+0.245*tan(diff_yaw*M_PI/180)/2) / (M_PI*0.1025)*180);
	double right_wheel = ((diff_linear-0.245*tan(diff_yaw*M_PI/180)/2) / (M_PI*0.1025)*180);
	std::cout << "left_wheel: " << left_wheel << std::endl;
	std::cout << "right_wheel: " << right_wheel << std::endl;

	// std::cout << "curr_yaw: " << current_yaw << std::endl;
	// std::cout << "diff_yaw: " << diff_yaw << std::endl;
	if(init_callback == true && left_wheel < 20 && right_wheel < 20){
		odom_filter_pub_.publish(current_odom_);
	}
	last_yaw = current_yaw;
	last_linear = current_linear;
	init_callback = true;
}
void OdomFilter::process()
{
	while(ros::ok()){
		ros::spinOnce();
	}
}
int main (int argc,char **argv)
{
    ros::init(argc, argv, "odom_filter");
    OdomFilter odom_filter;
    odom_filter.process();
    return 0;
}