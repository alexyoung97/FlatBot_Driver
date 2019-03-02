
#include <math.h>
#include <stdlib.h>
//#include <boost/algorithm/string.hpp>
#include <mutex>

// ROS libraries
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

 // Custom actions
#include "flat_bot_msgs/ActuateConnectorAction.h"

/** Module Driver main class
 *
 * This is the core class of this node, and follows the standard ROS convention for OO node design.
 *
 */
class ModuleDriver
{
public:
	ModuleDriver();
	~ModuleDriver();

private:

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
ModuleDriver::ModuleDriver() : moduleActionServer(n, "/module_control", boost::bind(&ModuleDriver::ActuationHandler, this, _1), false)
{
	ros::NodeHandle n("~");

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
    while(ros::ok())
    {
    	++seq;
    	ros::spinOnce();
    	loopRate.sleep();
    }
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
	ros::Rate loopRate(20);
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
					progressMsg.status_msg = "Closinging in progress";

		moduleActionServer.publishFeedback(progressMsg);

		printf("\r Faking action, %6.2f%% complete.", progress * 100.0);
		fflush(stdout);

		loopRate.sleep();
	}
	printf("\n");

	result.result = flat_bot_msgs::ActuateConnectorResult::R_SUCCESS;
	moduleActionServer.setSucceeded(result, "Module action completed okay.");

	ROS_INFO("Request completed okay.");
}

int main(int argc, char **argv)
{
	ROS_INFO("--[ Flatbot Module Action Server Node ]--");
	ros::init(argc, argv, "flatbot_module_driver");
	ros::start();

	ModuleDriver driver;

	ROS_INFO("--[ Flatbot Module Action Server Node: shutdown OK ]--\n");

	return 0;
}
