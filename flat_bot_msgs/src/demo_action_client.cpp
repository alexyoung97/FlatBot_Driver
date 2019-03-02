
#include <math.h>
#include <stdlib.h>
//#include <boost/algorithm/string.hpp>

// ROS libraries
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/server/simple_action_server.h>

 // Custom actions
#include "flat_bot_msgs/ActuateConnectorAction.h"

bool goalActive = false;

/// Called every time feedback is received for the goal
void feedbackCb(const flat_bot_msgs::ActuateConnectorFeedbackConstPtr& feedback)
{
  ROS_INFO("Received Feedback of [%f] \"%s\"", feedback->completed, feedback->status_msg.c_str());
}

/// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const flat_bot_msgs::ActuateConnectorResultConstPtr& result)
{
	ROS_INFO("Finished in state [%s]", state.toString().c_str());
	ROS_INFO("error_code: %d", result->result);

	goalActive = false;
}

/** Cheap and dirty node this time.
 *
 * No objects just a main and some global variables.
 * Good for throwing simple nodes together but not recommended for any
 * serious nodes.
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "module_action_client_test");

	ros::NodeHandle n("~");

	// create the action client
	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<flat_bot_msgs::ActuateConnectorAction> ac("/module_control", true);

	ROS_INFO("Waiting for action server to start.");
	ac.waitForServer(); //will wait for infinite time
	ROS_INFO("Action server started.");

	ros::Rate loopRate(0.1);
	while (ros::ok())
	{
		// if no action is current active then send a new one
		if (!goalActive)
		{
			// produce trajectory action goal
			flat_bot_msgs::ActuateConnectorGoal goal;
			goal.module_id = "MirrorA";
			goal.connector_id = rand() % 4; // One in 4 of these will fail because an id of 3 is invalid. Tests validation.
			goal.action = flat_bot_msgs::ActuateConnectorGoal::A_OPEN;

			// Not really a warning just was to highlight it in yellow text!
			ROS_WARN("Sending an open request to module %s connector #%d", goal.module_id.c_str(), goal.connector_id);

			// send a goal to the action server
			ac.sendGoal(goal,
						&doneCb,
						NULL,
						&feedbackCb);

			goalActive = true;
		}

		ros::spinOnce();
		loopRate.sleep();
	}

	//exit
	return 0;
}
