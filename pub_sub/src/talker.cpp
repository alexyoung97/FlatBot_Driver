#include <thread>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <string>
#include "/home/alex/Y3P/y3p/module_code/libraries/module_status_data/module_status_data.h"
#include "/home/alex/Y3P/y3p/module_code/libraries/command_data/command_data.h"
#include </home/alex/Y3P/catkin_workspace/src/pub_sub/src/udp_server.h>
#include "ros/ros.h"
#include "flat_bot_msgs/ModuleStatus.h"
#include <sstream>
//for Actionserver
#include <mutex>
#include <actionlib/server/simple_action_server.h>

 // Custom actions
#include "flat_bot_msgs/ActuateConnectorAction.h"


using boost::asio::ip::udp;
//logging::add_console_log(std::cout, boost::log::keywords::format = ">> %Message%");

Status module_status;

// AJY change: for boost_1_65_1, use io_service instead of io_context

flat_bot_msgs::ModuleStatus populate_ModuleStatus_msg(Status in_status){
  flat_bot_msgs::ModuleStatus msg;
  msg.header.seq = module_status.get_status_data()->the_header.get_header()->seq;
  msg.header.stamp.sec = module_status.get_status_data()->the_header.get_header()->time.seconds;
  msg.header.stamp.nsec = module_status.get_status_data()->the_header.get_header()->time.nanoseconds;
  msg.uptime = module_status.get_status_data()->uptime;
  msg.bus_voltage = module_status.get_status_data()->bus_voltage; //works
  return msg;
}

void chatterCallback(const flat_bot_msgs::ModuleStatus::ConstPtr& msg)
{
  ROS_INFO("I heard: bus voltage = [%f]", msg->bus_voltage);
}

/** Module Driver main class
 *
 * This is the core class of this node, and follows the standard ROS convention for OO node design.
 *
 */
class ModuleDriver
{
public:
//	ModuleDriver(UDP_Server *a_server);
	ModuleDriver();

	~ModuleDriver();

private:
//	UDP_Server the_server;

	void ActuationHandler(const flat_bot_msgs::ActuateConnectorGoalConstPtr &goal);

	// The node handle property needs to be defined before the trajectory action server. The default constructor seems to require this!
	ros::NodeHandle n;
	actionlib::SimpleActionServer<flat_bot_msgs::ActuateConnectorAction> moduleActionServer;

	std::vector<std::string> moduleIds;
	std::mutex modileIdsLock;
};

/** Default Constructor
 *
 * Doesn't do very much at the moment. Inits the action server member, sets up two fake modules
 * then loops forever until the node crashes or received a SIGINT message (crtl C).
 */
//ModuleDriver::ModuleDriver(UDP_Server *a_server) : moduleActionServer(n, "/module_control", boost::bind(&ModuleDriver::ActuationHandler, this, _1), false)
ModuleDriver::ModuleDriver() : moduleActionServer(n, "/module_control", boost::bind(&ModuleDriver::ActuationHandler, this, _1), false)
{
	ros::NodeHandle n("~");
	//	the_server = *a_server;

	// get and validate parameters
	/*std::string portName, jointNamesParam, startupPolicy, shutdownPolicy;
	int BAUDRate;
	n.param<std::string>("port", portName, "/dev/ttyUSB0");
	n.param<int>("baud", BAUDRate, 115200);
	n.param<int>("joint_publish_rate", jointPublisherRate, 50);
	n.param<std::string>("joint_names", jointNamesParam, "");
	n.param<std::string>("startup_policy", startupPolicy, "torque_on");
	n.param<std::string>("shutdown_policy", shutdownPolicy, "torque_on");*/

	// Fake module names
	moduleIds.push_back("MirrorA");
	moduleIds.push_back("MirrorB");

	moduleActionServer.start();

	// this loop will get executed once per second doing something, nothing yet though!
    ros::Rate loopRate(1.0);
    int seq=0;
 /*   while(ros::ok())
    {
    	++seq;
    	ros::spinOnce();
    	loopRate.sleep();
    } */
}

ModuleDriver::~ModuleDriver()
{

}

/** Module actuation handler method
 *
 * This method is called every time an action goal is received, NOTE this is executed in a
 * different thread to the rest of the node.
 *
 * In this implementation execution remaining within this function until the action is completed
 * hence the need for the second thread.
 */
void ModuleDriver::ActuationHandler(const flat_bot_msgs::ActuateConnectorGoalConstPtr &goal)
{
	flat_bot_msgs::ActuateConnectorResult result;

	// verify the module id is valid
	if (std::find(moduleIds.begin(), moduleIds.end(), goal->module_id) == moduleIds.end())
	{
		result.result = flat_bot_msgs::ActuateConnectorResult::R_INVALID_MODULE_ID;
		ROS_ERROR("Error : Received an actuate connector action with an invalid module_id of \"%s\".", goal->module_id.c_str());
		moduleActionServer.setAborted(result, "Error : invalid module_id.");
		return;
	}

	// get module object from id.

	// verify the connector_id is valid
	if (goal->connector_id < 0 || goal->connector_id >= 3)
	{
		result.result = flat_bot_msgs::ActuateConnectorResult::R_INVALID_CONNECTOR_ID;
		ROS_ERROR("Error : Received an actuate connector action with an invalid connector_id of [%d].", goal->connector_id);
		moduleActionServer.setAborted(result, "Error : invalid connector_id.");
		return;
	}

	// verify the action type
	if (goal->action != flat_bot_msgs::ActuateConnectorGoal::A_OPEN && goal->action != flat_bot_msgs::ActuateConnectorGoal::A_CLOSE)
	{
		result.result = flat_bot_msgs::ActuateConnectorResult::R_INVALID_ACTION;
		ROS_ERROR("Error : Received an actuate connector action with an invalid action of [%d].", goal->action);
		moduleActionServer.setAborted(result, "Error : invalid action given.");
		return;
	}

	ROS_INFO("Received a valid module actuation action request.");

	// Fake actuator movement
/*	ros::Rate loopRate(20);
	ros::Time start = ros::Time::now();
	while (ros::Time::now() < start + ros::Duration(3.0))
	{
		float progress = (ros::Time::now() - start).toSec() / 3.0;

		// send feedback messages during movement
		flat_bot_msgs::ActuateConnectorFeedback progressMsg;
		progressMsg.completed = progress;

		if (goal->action == flat_bot_msgs::ActuateConnectorGoal::A_OPEN)
			progressMsg.status_msg = "Opening in progress";
		if (goal->action == flat_bot_msgs::ActuateConnectorGoal::A_CLOSE)
					progressMsg.status_msg = "Closing in progress";

		moduleActionServer.publishFeedback(progressMsg);

		printf("\r Faking action, %6.2f%% complete.", progress * 100.0);
		fflush(stdout);

		loopRate.sleep();
	}
	printf("\n");

	result.result = flat_bot_msgs::ActuateConnectorResult::R_SUCCESS;
	moduleActionServer.setSucceeded(result, "Module action completed okay."); */

	// Real actuator movement
	//Send action
	//Parse and forward on UDP received instructions, from action server
    Command c;
	c.set_cmd_action(goal->action);
	c.set_connector_id(goal->connector_id);
//	the_server.handle_send(c.serialise_cmd_data());
	//while(module status is "opening servo") stay in this loop
	//then return the final state (success/fail)

	ROS_INFO("Request completed okay.");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<flat_bot_msgs::ModuleStatus>("chatter", 1000);
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::Rate loop_rate(0.2);

  int count = 0; // count how many messages sent
  boost::asio::io_service io_service;
  UDP_Server server(io_service);
  //ModuleDriver driver(&server);
  ModuleDriver driver();

  while (ros::ok())
  {
	// UDP receive and forward to ROS topic
    server.start_receive();
    io_service.run_one();
    std::cout << "ros::ok\n";
    flat_bot_msgs::ModuleStatus msg; //message object
    msg.bus_voltage = module_status.get_status_data()->bus_voltage; //works
    ROS_INFO("bus voltage = %f", msg.bus_voltage);
    chatter_pub.publish(msg);



    // Spin the publisher, subscriber and ActionServer queues
    ros::spinOnce();
    loop_rate.sleep(); // code execution waits here (thread block)
    ++count;
    std::cout << "***********************************************\n\n";
  }

  return 0;
}
