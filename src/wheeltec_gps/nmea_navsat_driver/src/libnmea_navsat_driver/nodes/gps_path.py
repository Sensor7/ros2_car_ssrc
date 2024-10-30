import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

EARTH_RADIUS = 6378.137

class GpsPath(Node):
    def __init__(self):
        super().__init__('GpsPath')
        self.pose_init = False
        self.state_pub_ = self.create_publisher(Path, 'gps_path', 10)
        self.subscription_ = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            1
        )

    def rad(self, d):
        return d * math.pi / 180.0

    def gps_callback(self, gps_msg):
        if not self.pose_init:
            self.init_pose = self.lla_pose()
            self.init_pose.latitude = gps_msg.latitude
            self.init_pose.longitude = gps_msg.longitude
            self.init_pose.altitude = gps_msg.altitude
            self.pose_init = True
        else:
            radLat1 = self.rad(self.init_pose.latitude)
            radLong1 = self.rad(self.init_pose.longitude)
            radLat2 = self.rad(gps_msg.latitude)
            radLong2 = self.rad(gps_msg.longitude)
            
            delta_lat = radLat2 - radLat1
            delta_long = 0
            
            if delta_lat > 0:
                x = -2 * math.asin(math.sqrt(
                    math.pow(math.sin(delta_lat / 2), 2) +
                    math.cos(radLat1) * math.cos(radLat2) *
                    math.pow(math.sin(delta_long / 2), 2)
                ))
            else:
                x = 2 * math.asin(math.sqrt(
                    math.pow(math.sin(delta_lat / 2), 2) +
                    math.cos(radLat1) * math.cos(radLat2) *
                    math.pow(math.sin(delta_long / 2), 2)
                ))
            x *= EARTH_RADIUS * 1000
            
            delta_lat = 0
            delta_long = radLong2 - radLong1
            if delta_long > 0:
                y = 2 * math.asin(math.sqrt(
                    math.pow(math.sin(delta_lat / 2), 2) +
                    math.cos(radLat2) * math.cos(radLat2) *
                    math.pow(math.sin(delta_long / 2), 2)
                ))
            else:
                y = -2 * math.asin(math.sqrt(
                    math.pow(math.sin(delta_lat / 2), 2) +
                    math.cos(radLat2) * math.cos(radLat2) *
                    math.pow(math.sin(delta_long / 2), 2)
                ))
            y *= EARTH_RADIUS * 1000
            
            z = gps_msg.altitude - self.init_pose.altitude
            
            ros_path_ = Path()
            ros_path_.header.frame_id = 'path'
            ros_path_.header.stamp = self.get_clock().now().to_msg()
            pose = PoseStamped()
            pose.header = ros_path_.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            ros_path_.poses.append(pose)
            
            self.state_pub_.publish(ros_path_)
            
            self.get_logger().info("( x:%0.6f ,y:%0.6f ,z:%0.6f)", x, y, z)
    
    class lla_pose:
        def __init__(self):
            self.latitude = 0.0
            self.longitude = 0.0
            self.altitude = 0.0

def main(args=None):
    rclpy.init(args=args)
    gps_path = GpsPath()

    rclpy.spin(gps_path)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
