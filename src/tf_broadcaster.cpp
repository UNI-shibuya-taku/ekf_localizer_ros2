#include "tf/tf_broadcaster.h"

TFBroadcaster::TFBroadcaster() : Node("TFBroadcaster")
{
    this -> declare_parameter("is_odom_tf", true);
    this -> declare_parameter("odom_topic_name", "/odom");
    this -> declare_parameter("odom_frame_id", "odom");
    this -> declare_parameter("base_link_frame_id", "base_link");
    this -> declare_parameter("camera_frame_id", "camera_color_optical_frame");
    this -> declare_parameter("velodyne_frame_id", "velodyne");
    this -> declare_parameter("camera_x", 0.0);
    this -> declare_parameter("camera_y", 0.0);
    this -> declare_parameter("camera_z", 0.5);
    this -> declare_parameter("camera_roll", 0.0);
    this -> declare_parameter("camera_pitch", 0.0);
    this -> declare_parameter("camera_yaw", 0.0);
    this -> declare_parameter("velodyne_x", 0.0);
    this -> declare_parameter("velodyne_y", 0.0);
    this -> declare_parameter("velodyne_z", 1.3);
    this -> declare_parameter("velodyne_roll", 0.0);
    this -> declare_parameter("velodyne_pitch", 0.0);
    this -> declare_parameter("velodyne_yaw", 0.0);

    // this -> get_parameter("is_odom_tf", is_odom_tf_);
    // this -> get_parameter("odom_topic_name", odom_topic_name_);
    // this -> get_parameter("odom_frame_id", odom_frame_id_);
    // this -> get_parameter("base_link_frame_id", base_link_frame_id_);
    // this -> get_parameter("camera_link_frame_id", camera_frame_id_);
    // this -> get_parameter("velodyne_link_frame_id", velodyne_frame_id_);
    this -> get_parameter("camera_x", camera_coordinate_.x);
    this -> get_parameter("camera_y", camera_coordinate_.y);
    this -> get_parameter("camera_z", camera_coordinate_.z);
    this -> get_parameter("camera_roll", camera_coordinate_.roll);
    this -> get_parameter("camera_pitch", camera_coordinate_.pitch);
    this -> get_parameter("camera_yaw", camera_coordinate_.yaw);
    this -> get_parameter("velodyne_x", velodyne_coordinate_.x);
    this -> get_parameter("velodyne_y", velodyne_coordinate_.y);
    this -> get_parameter("velodyne_z", velodyne_coordinate_.z);
    this -> get_parameter("velodyne_roll", velodyne_coordinate_.roll);
    this -> get_parameter("velodyne_pitch", velodyne_coordinate_.pitch);
    this -> get_parameter("velodyne_yaw", velodyne_coordinate_.yaw);

	/*subscriber*/
    // odom_sub_ = nh_.subscribe(odom_topic_name_,1,&TFBroadcaster::odometry_callback,this);
    odom_sub_  = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::QoS(1).reliable(),
        std::bind(&TFBroadcaster::odometry_callback, this, std::placeholders::_1));
	
	camera_transform_ = create_transform_stamped_msg(base_link_frame_id_,camera_frame_id_,camera_coordinate_);
	velodyne_transform_ = create_transform_stamped_msg(base_link_frame_id_,velodyne_frame_id_,velodyne_coordinate_);

	// ↓大いなる不安要素
	// broadcaster_.reset(new tf2_ros::TransformBroadcaster);
	// static_broadcaster_.reset(new tf2_ros::StaticTransformBroadcaster);
}

TFBroadcaster::~TFBroadcaster() { }

void TFBroadcaster::odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
	if(is_odom_tf_){
		geometry_msgs::msg::TransformStamped transform;
		transform.header = msg->header;
		transform.header.frame_id = odom_frame_id_;
 	   	transform.child_frame_id = base_link_frame_id_;

    	transform.transform.translation.x = msg->pose.pose.position.x;
    	transform.transform.translation.y = msg->pose.pose.position.y;
    	transform.transform.translation.z = msg->pose.pose.position.z;
    	transform.transform.rotation = msg->pose.pose.orientation;

		broadcaster_->sendTransform(transform);
	}
}

geometry_msgs::msg::TransformStamped TFBroadcaster::create_transform_stamped_msg(std::string frame_id,std::string child_frame_id,Coordinate coordinate)
{
	geometry_msgs::msg::TransformStamped static_transform_stamped;
	static_transform_stamped.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
	static_transform_stamped.header.frame_id = frame_id;
	static_transform_stamped.child_frame_id = child_frame_id;

	static_transform_stamped.transform.translation.x = coordinate.x;
	static_transform_stamped.transform.translation.y = coordinate.y;
	static_transform_stamped.transform.translation.z = coordinate.z;

	tf2::Quaternion quaternion;
	quaternion.setRPY(coordinate.roll,coordinate.pitch,coordinate.yaw);
	static_transform_stamped.transform.rotation.x = quaternion.x();
	static_transform_stamped.transform.rotation.y = quaternion.y();
	static_transform_stamped.transform.rotation.z = quaternion.z();
	static_transform_stamped.transform.rotation.w = quaternion.w();

	return static_transform_stamped;
}

void TFBroadcaster::process()
{
    std::cout << "process()" << std::endl;
	static_broadcaster_->sendTransform({ camera_transform_, velodyne_transform_ });
    std::cout << "End process()" << std::endl;
}
// tf_broadcaster_node撤廃ver
int main(int argc,char** argv)
{
    std::cout << "START TfBroadcaster" << std::endl;
    rclcpp::init(argc, argv); // ノードの初期化
    std::cout << "End init()" << std::endl;
    auto node = std::make_shared<TFBroadcaster>();
    std::cout << "End make_shared" << std::endl;
    node->process();
    // rclcpp::spin(node);   // コールバック関数の実行
    // rclcpp::Rate loop_rate(10);
    std::cout << "End process" << std::endl;
    while(rclcpp::ok())
    {
        rclcpp::spin_some(node);   // コールバック関数の実行
        // loop_rate.sleep(); // 周期が終わるまで待つ
    }
    std::cout << "End while()" << std::endl;
    node.reset();
    std::cout << "End reset()" << std::endl;
    rclcpp::shutdown();
    std::cout << "End shutdown()" << std::endl;
    return 0;
}