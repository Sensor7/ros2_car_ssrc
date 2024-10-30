/**************************************************************************
作者：pjf && lyk
功能：多机编队
**************************************************************************/
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/LaserScan.h"
#include <std_msgs/Int32.h>

int multi_mode = 2;//编队模式选择：目前有模式1：整体自转 和模式2：各个小车原地自转

double e_linear_x = 0;    //从机与目标点x方向的偏差（小车前后方向的偏差）
double e_linear_y = 0;    //从机与目标点y方向的偏差（小车左右方向的偏差）
double e_angular_z = 0;    //从机与目标点角度偏差
double angular_turn = 0;     //修正从机运动方向

double slave_x = 0.8;    //目标点的x坐标
double slave_y = -0.8;    //目标点的y坐标

double max_vel_x=1.0;     //最大速度限制
double min_vel_x=0.05;     //最小速度限制
double max_vel_theta=1.0; //最大角速度限制
double min_vel_theta=0.05;//最小角速度限制

double k_v=1;   //调节前后方向偏差时，k_v越大，线速度越大
double k_l=1;   //调节左右方向偏差时，k_l越大，角速度越大
double k_a=1;   //调节角度偏差时，k_a越大，角速度越大

// tf变换相关参数
std::string base_frame;
std::string base_to_slave;
std::string tf_prefix_;

//主车速度
float odom_linear_x=0;    
float odom_linear_y=0;
float odom_angular_z=0;
/**************************************************************************
函数功能：底盘运动sub回调函数（原始数据）
入口参数：cmd_msg  command_recognition.cpp
返回  值：无
**************************************************************************/
void vel_Callback(const nav_msgs::Odometry& msg)
{
  odom_linear_x = msg.twist.twist.linear.x;
  odom_linear_y = msg.twist.twist.linear.y;
  odom_angular_z = msg.twist.twist.angular.z;
  if(fabs(odom_linear_x)<min_vel_x)odom_linear_x=0;
}
void multi_mode_Callback(const std_msgs::Int32& msg)
{
  multi_mode = msg.data;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "wheeltec_multi");

  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  ros::Publisher slave_vel = node.advertise<geometry_msgs::Twist>("cmd_vel_ori", 10); //发布原始数据
  ros::Subscriber vel_sub = node.subscribe("/odom", 1, vel_Callback);
  ros::Subscriber multi_mode_sub = node.subscribe("/multi_mode_topic", 1, multi_mode_Callback);

  private_nh.param<int>("multi_mode", multi_mode,2);//编队模式选择：目前有模式1 和模式2
  private_nh.param<std::string>("base_frame", base_frame, "base_footprint");
  private_nh.param<std::string>("base_to_slave", base_to_slave, "slave1");  
  private_nh.param<double>("slave_x", slave_x,0.6);
  private_nh.param<double>("slave_y", slave_y,-0.6);
  private_nh.param<double>("max_vel_x", max_vel_x,1.0);
  private_nh.param<double>("min_vel_x", min_vel_x,0.05);
  private_nh.param<double>("max_vel_theta", max_vel_theta,1.0);
  private_nh.param<double>("min_vel_theta", min_vel_theta,0.05);
  private_nh.param<double>("k_v", k_v,1);
  private_nh.param<double>("k_l", k_l,1);
  private_nh.param<double>("k_a", k_a,1);

// 使用tf_prefix参数将frame_name解析为frame_id
  tf_prefix_ = tf::getPrefixParam(private_nh); 
  base_frame = tf::resolve(tf_prefix_, base_frame);
  base_to_slave = tf::resolve(tf_prefix_, base_to_slave);

//中间变量
  geometry_msgs::Twist vel_msg;
  double e_angular_x = 0;
  double e_angular_y = 0;
  slave_x = slave_x+slave_y;
  slave_y = slave_x-slave_y;
  slave_x = -(slave_x-slave_y);//使得期望坐标slave_x slave_y是以主车为坐标原点，右边为x正方向，前面为y正方向的坐标系


  //tf变换中间变量
  tf::TransformListener listener1 ,listener2;
  tf::StampedTransform transformSM1 ,transformSM2;//中间变量
  geometry_msgs::Pose pose_orien1 ,pose_orien2;//用于获取终端姿态的变量
  tf::Quaternion RQ21 ,RQ22; //用于获取当前位姿与目标点的欧拉角 
  ros::Duration(3.0).sleep();
  ros::Rate rate(10.0);
  while (node.ok()){

    //根据编队模式不同，选择不同的运动模型
    if(multi_mode==1)
    {
      //由到从机与期望坐标做tf变换
      try{  
        listener1.waitForTransform(base_frame, base_to_slave, ros::Time(0), ros::Duration(3.0));

        listener1.lookupTransform(base_frame,  base_to_slave, ros::Time(0), transformSM1);
      }
      catch (tf::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue; 
      } 
      double angular_x1 , angular_y1 , angular_z1 ;
      pose_orien1.orientation.x=transformSM1.getRotation().getX();
      pose_orien1.orientation.y=transformSM1.getRotation().getY();
      pose_orien1.orientation.z=transformSM1.getRotation().getZ();
      pose_orien1.orientation.w=transformSM1.getRotation().getW();
      tf::quaternionMsgToTF(pose_orien1.orientation, RQ21);  //终端当前位姿从四元数形式转换为欧拉角形式
      tf::Matrix3x3(RQ21).getRPY(angular_x1, angular_y1, angular_z1);  //读取期望坐标点与从车的相对角度angular_z1
      e_angular_z = angular_z1; 
      if(fabs(odom_linear_x/odom_angular_z)<5.0)angular_turn = atan2(slave_y,slave_x+odom_linear_x/odom_angular_z);
      else angular_turn = atan2(slave_y,slave_x);
      if(fabs(odom_angular_z)>min_vel_theta)//主车转弯时，从车需要修正前进方向（前进方向为从车运动轨迹圆的切线方向），才能跟随主车运动
      {
         if(slave_x>0 && slave_y>=0)
        {
          if(odom_angular_z>0)e_angular_z=e_angular_z+angular_turn;
          else e_angular_z=e_angular_z-3.14+angular_turn;
        }
        else if(slave_x<=0 && slave_y>=0)
        {
          if(odom_angular_z>0)e_angular_z=e_angular_z+angular_turn;
          else e_angular_z=e_angular_z+angular_turn-3.14;
        }
        else if(slave_x<0 && slave_y<0)
        {
          if(odom_angular_z>=0)e_angular_z=e_angular_z+angular_turn;
          else e_angular_z=e_angular_z+3.14+angular_turn;
        }
        else if(slave_x>=0 && slave_y<0)
        {
          if(odom_angular_z>0)e_angular_z=e_angular_z+angular_turn;
          else e_angular_z=e_angular_z+3.14+angular_turn;
        }
        if(e_angular_z>4.71 && e_angular_z<7.85)e_angular_z=e_angular_z-6.28;
        if(e_angular_z<-4.71 && e_angular_z>-7.85)e_angular_z=e_angular_z+6.28;
      }
      e_linear_x = transformSM1.getOrigin().x();       //x方向的偏差（小车前后方向的偏差）
      e_linear_y = transformSM1.getOrigin().y();       //y方向的偏差（小车左右方向的偏差）
      e_angular_z = e_angular_z;                      //角度偏差


      printf("Deviation: \n e_linear_x = %f\n e_linear_y = %f\n e_angular_z =%f\n",e_linear_x,e_linear_y,e_angular_z);
      //计算得到从机的线速度
      if(fabs(odom_linear_x/odom_angular_z)<5.0)//odom_linear_x/odom_angular_z<5.0可视为小车在做非直线的运动
      {
        double d_r;                             //d_r为主车运动半径与期望跟随点的运动半径之差，
        if(slave_x+odom_linear_x/odom_angular_z>=0)
        {
          d_r = sqrt(pow(slave_x+odom_linear_x/odom_angular_z,2)+pow(slave_y,2))-odom_linear_x/odom_angular_z;
        }
        else 
        {
          d_r = -sqrt(pow(slave_x+odom_linear_x/odom_angular_z,2)+pow(slave_y,2))-odom_linear_x/odom_angular_z;
        }
        vel_msg.linear.x = (odom_linear_x+odom_angular_z*d_r)*fabs(cos(e_angular_z))+k_v*e_linear_x; //根据v/w=r，其中w从机=w主机，所以
                                    //  v从机/w从机=(odom_linear_x+odom_angular_z*d_r) /odom_angular_z=r主车运动半径+d_r = r从车运动半径，符合运动学规律
      }
      else 
      {
        vel_msg.linear.x = odom_linear_x+k_v*e_linear_x; 
      }
      float _k_a=k_a;
      float _k_l=k_l;
      if(vel_msg.linear.x<-min_vel_x)//从车后退时，修正参数的正负值与前进的相反
      {
        if(fabs(odom_angular_z)>min_vel_theta)_k_a=-k_a;//直行运动的_k_a参数为正
        _k_l=-k_l;
      }
      //计算得到从机的角速度
      vel_msg.angular.z = odom_angular_z+_k_l*e_linear_y+_k_a*sin(e_angular_z);
    }
    else if(multi_mode==2)
    {
        //由到从机与期望坐标做tf变换
      try{  
        listener1.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(3.0));
        listener2.waitForTransform("map", base_to_slave+"/base_link", ros::Time(0), ros::Duration(3.0));

        listener1.lookupTransform("map",  "base_link", ros::Time(0), transformSM1);
        listener2.lookupTransform("map",  base_to_slave+"/base_link", ros::Time(0), transformSM2);
      }
      catch (tf::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue; 
      } 
      double angular_x1 , angular_y1 , angular_z1 ;
      double angular_x2 , angular_y2 , angular_z2 ;
      //主车的TF变换
      pose_orien1.orientation.x=transformSM1.getRotation().getX();
      pose_orien1.orientation.y=transformSM1.getRotation().getY();
      pose_orien1.orientation.z=transformSM1.getRotation().getZ();
      pose_orien1.orientation.w=transformSM1.getRotation().getW();
      tf::quaternionMsgToTF(pose_orien1.orientation, RQ21);  //终端当前位姿从四元数形式转换为欧拉角形式
      tf::Matrix3x3(RQ21).getRPY(angular_x1, angular_y1, angular_z1);  //读取期望坐标点与从车的相对角度angular_z1

      // 从车的TF变换
      pose_orien2.orientation.x=transformSM2.getRotation().getX();
      pose_orien2.orientation.y=transformSM2.getRotation().getY();
      pose_orien2.orientation.z=transformSM2.getRotation().getZ();
      pose_orien2.orientation.w=transformSM2.getRotation().getW();
      tf::quaternionMsgToTF(pose_orien2.orientation, RQ22);  //终端当前位姿从四元数形式转换为欧拉角形式
      tf::Matrix3x3(RQ22).getRPY(angular_x2, angular_y2, angular_z2);  //读取期望坐标点与从车的相对角度angular_z2

      e_linear_x = transformSM1.getOrigin().x()-transformSM2.getOrigin().x()+slave_y;      //x方向的偏差（小车前后方向的偏差）
      e_linear_y = transformSM1.getOrigin().y()-transformSM2.getOrigin().y()-slave_x;       //y方向的偏差（小车左右方向的偏差）
      e_angular_z = angular_z1-angular_z2;                                                  //角度偏差
      printf("Deviation: \n e_linear_x = %f\n e_linear_y = %f\n e_angular_z =%f\n",e_linear_x,e_linear_y,e_angular_z);

      vel_msg.linear.x = odom_linear_x+k_v*e_linear_x*cos(angular_z2)+k_v*e_linear_y*sin(angular_z2); //根据当前从车的方向angular_z2，调整速度修正从车的x y方向偏差（例如从车朝向在x方向0角度时，cos(0)=1,调整小车速度修正x方向偏差）
      float _k_a=k_a;
      float _k_l=k_l;
      if(vel_msg.linear.x<-min_vel_x)//小车后退时，控制车头转动方向与前进时相反（例如从车在期望位置正右位置，前进时应该使车头向左偏修正，后退时应该使车头向右偏修正）
      {
        _k_l=-k_l;
      }
      //根据当前从车当前朝向方向angular_z2，在模仿主车运动的同时，调整车头方向，修正从车的x y ，角度方向偏差
      vel_msg.angular.z = odom_angular_z+0.5*_k_l*e_linear_y*cos(angular_z2)-0.5*_k_l*e_linear_x*sin(angular_z2)+_k_a*sin(e_angular_z);
    }

    //速度限制
    if(vel_msg.linear.x > max_vel_x)
      vel_msg.linear.x=max_vel_x;
    else if(vel_msg.linear.x < -max_vel_x)
      vel_msg.linear.x=-max_vel_x;
    if(fabs(vel_msg.linear.x) < min_vel_x)
      vel_msg.linear.x=0;
    if(vel_msg.angular.z > max_vel_theta)
      vel_msg.angular.z=max_vel_theta;
    else if(vel_msg.angular.z < -max_vel_theta)
      vel_msg.angular.z=-max_vel_theta;
    if(fabs(vel_msg.angular.z) < min_vel_theta)
      vel_msg.angular.z=0;

    slave_vel.publish(vel_msg);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
};
