运行设备别名规则文件:
sudo sh wheeltec_gnss.sh

编译功能包需要的依赖：
sudo pip3 install transforms3d
sudo apt install ros-humble-tf-transformations
sudo apt install ros-humble-gps-umd
pip3 install serial
pip3 install pyserial
或一键安装依赖：
rosdep install --from-paths src --ignore-src -r -y

使用NMEA协议解析WHEELTEC G60/G70：
ros2 launch wheeltec_gps_driver wheeltec_nmea_driver.launch.py

--gps topic: /gps/fix
--gps frame_id: navsat_link

在RVIZ中记录经纬度轨迹(WHEELTEC G60/G70)：
ros2 launch wheeltec_gps_driver nmea_gps_path.launch.py

使用UBLOX协议解析WHEELTEC G70模块(仅WHEELTEC G70)：
ros2 launch wheeltec_gps_driver wheeltec_ublox_driver.launch.py
--gps topic: /ublox_gps_node/fix
--gps frame_id: navsat_link

在RVIZ中记录经纬度轨迹(仅WHEELTEC G70)：
ros2 launch wheeltec_gps_driver ublox_gps_path.launch.py
