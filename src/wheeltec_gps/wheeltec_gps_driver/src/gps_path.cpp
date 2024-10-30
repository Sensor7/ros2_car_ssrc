
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <math.h>
#define EARTH_RADIUS 6378.137
using std::placeholders::_1;
bool pose_init;

rclcpp::Node::SharedPtr nh_=nullptr;

class GpsPath : public rclcpp::Node
{
  public:
    GpsPath()
    : Node("GpsPath")
    {
      state_pub_ = create_publisher<nav_msgs::msg::Path>("gps_path", 10);
      subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps_topic", 1, std::bind(&GpsPath::gps_callback, this, _1));
    }
    struct lla_pose
    {
      double latitude;
      double longitude;
      double altitude;
    };
    double rad(double d) 
    {
    	return d * 3.1415926 / 180.0;
    }
  
  private:
  
 void publishTFFrames(const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg_ptr, const geometry_msgs::msg::PoseStamped& current_position) {

    //static tf2_ros::TransformBroadcaster tf_broadcaster;
    static  std::shared_ptr<tf2_ros::TransformBroadcaster>  tf_broadcaster;
    tf_broadcaster =std::make_shared<tf2_ros::TransformBroadcaster>(this);


    // 发布GPS轨迹原点的TF坐标
    geometry_msgs::msg::TransformStamped init_tf;
    init_tf.header.stamp = rclcpp::Time(gps_msg_ptr->header.stamp.sec, gps_msg_ptr->header.stamp.nanosec);
    init_tf.header.frame_id = "path"; // 假设GPS轨迹原点位于world坐标系
    init_tf.child_frame_id = "gps_origin";
    init_tf.transform.translation.x = 0.0;
    init_tf.transform.translation.y = 0.0;
    init_tf.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    init_tf.transform.rotation = tf2::toMsg(q);
    tf_broadcaster->sendTransform(init_tf);

    // 发布当前轨迹位置的TF坐标
    geometry_msgs::msg::TransformStamped current_tf;
    //current_tf.header.stamp = ros::Time::now();
    current_tf.header.stamp = rclcpp::Time(current_position.header.stamp.sec, current_position.header.stamp.nanosec);
    current_tf.header.frame_id = "path"; // 假设当前轨迹位置位于world坐标系
    current_tf.child_frame_id = "current_path_position";
    current_tf.transform.translation.x = current_position.pose.position.x;
    current_tf.transform.translation.y = current_position.pose.position.y;
    current_tf.transform.translation.z = current_position.pose.position.z;
    current_tf.transform.rotation = current_position.pose.orientation;
    q.setRPY(0, 0, 0); // 设置姿态为单位四元数
    current_tf.transform.rotation = tf2::toMsg(q);
    tf_broadcaster->sendTransform(current_tf);
} 
  
  
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg) 
    {
      if(!pose_init)
       {
        init_pose.latitude = gps_msg->latitude;
        init_pose.longitude = gps_msg->longitude;
        init_pose.altitude = gps_msg->altitude;
        pose_init = true;
        }
      else
        {
        //计算相对位置
            double radLat1 ,radLat2, radLong1,radLong2,delta_lat,delta_long,x,y;
            
            radLat1 = rad(init_pose.latitude);
            radLong1 = rad(init_pose.longitude);
            
            radLat2 = rad(gps_msg->latitude);
            radLong2 = rad(gps_msg->longitude);
                //计算x
            delta_lat = radLat2 - radLat1;
            delta_long = 0;
            
            
            if(delta_lat>0)
              x = -2*asin( sqrt( pow( sin( delta_lat/2 ),2) + cos( radLat1 )*cos( radLat2)*pow( sin( delta_long/2 ),2 ) ));
            else
              x = 2*asin( sqrt( pow( sin( delta_lat/2 ),2) + cos( radLat1 )*cos( radLat2)*pow( sin( delta_long/2 ),2 ) ));
                x = x*EARTH_RADIUS*1000;
        
                //计算y
            delta_lat = 0;
                delta_long = radLong2  - radLong1;
            if(delta_long>0)
              y = 2*asin( sqrt( pow( sin( delta_lat/2 ),2) + cos( radLat2 )*cos( radLat2)*pow( sin( delta_long/2 ),2 ) ) );
            else
              y = -2*asin( sqrt( pow( sin( delta_lat/2 ),2) + cos( radLat2 )*cos( radLat2)*pow( sin( delta_long/2 ),2 ) ) );
               y = y*EARTH_RADIUS*1000;
    
            //计算z
            double z = gps_msg->altitude - init_pose.altitude;
            // 更新当前位置
            geometry_msgs::msg::PoseStamped current_position;
            current_position.header.frame_id = "path";
            current_position.header.stamp = rclcpp::Node::now();
            current_position.pose.position.x = x;
            current_position.pose.position.y = y;
            current_position.pose.position.z = z;
            
            //·发布轨迹
            ros_path_.header.frame_id = "path";
            ros_path_.header.stamp = rclcpp::Node::now();  
            geometry_msgs::msg::PoseStamped pose;
            pose.header = ros_path_.header;
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = z;
            ros_path_.poses.push_back(pose);
            
            state_pub_->publish(ros_path_);
    
            RCLCPP_INFO(this->get_logger(),"( x:%0.6f ,y:%0.6f ,z:%0.6f)",x ,y ,z );

        }        
        
        
    }
    
    nav_msgs::msg::Path ros_path_;
    lla_pose init_pose;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr state_pub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  pose_init = false;
  rclcpp::spin(std::make_shared<GpsPath>());
  rclcpp::shutdown();
  return 0;
}



