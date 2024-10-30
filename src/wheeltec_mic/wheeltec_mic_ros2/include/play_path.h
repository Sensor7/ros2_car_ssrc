#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#include <iostream>
#include <unistd.h>
#include <string.h>

std::string head = "aplay -D plughw:CARD=Device,DEV=0 ";
std::string gnome_terminal = "dbus-launch gnome-terminal -- ";
std::string simple_follower = "ros2 launch simple_follower_ros2 ";
std::string wheeltec_mic_ros2 = "ros2 launch wheeltec_mic_ros2 ";
std::string audio_path;
std::string WHOLE;
std::string Launch;

#endif