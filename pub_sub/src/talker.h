#include <thread>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <string>

//message types
#include "/home/alex/Y3P/y3p/module_code/libraries/module_status_data/module_status_data.h"
#include "/home/alex/Y3P/y3p/module_code/libraries/ack/ack.h"
#include "/home/alex/Y3P/y3p/module_code/libraries/boot_broadcast/boot_broadcast.h"
#include "/home/alex/Y3P/y3p/module_code/libraries/command_data/command_data.h"

#include </home/alex/Y3P/catkin_workspace/src/pub_sub/src/udp_server.h>
#include "ros/ros.h"
#include "flat_bot_msgs/ModuleStatus.h"
#include <sstream>
#include "/home/alex/Y3P/catkin_workspace/src/flat_bot_msgs/src/action_server.h"
//for Actionserver
#include <mutex>
#include <actionlib/server/simple_action_server.h>

 // Custom actions
#include "flat_bot_msgs/ActuateConnectorAction.h"

#ifndef talker_h
#define talker_h

#define MAX_NUMBER_OF_MODULES 255 //!< Maximum number of modules supported by the system
#define MAX_SERIAL_SIZE 255 //!< Maximum size of packet on network
#define MSG_TYPE_POSITION 13 //!< Position in all packets in which the message type is stored
#define MODULE_IP "192.168.43.6"

Status module_status[MAX_NUMBER_OF_MODULES];
char recv_serial[MAX_SERIAL_SIZE];
uint8_t recv_empty;
uint32_t seq;

void chatterCallback(const flat_bot_msgs::ModuleStatus::ConstPtr& msg);

#endif
