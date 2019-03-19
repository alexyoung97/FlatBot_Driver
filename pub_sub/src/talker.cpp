#include "/home/alex/Y3P/catkin_workspace/src/pub_sub/src/talker.h"

using boost::asio::ip::udp;
//logging::add_console_log(std::cout, boost::log::keywords::format = ">> %Message%");

// AJY change: for boost_1_65_1, use io_service instead of io_context

/*flat_bot_msgs::ModuleStatus populate_ModuleStatus_msg(Status* in_status){
 flat_bot_msgs::ModuleStatus msg;
 msg.header.seq = in_status->get_status_data()->the_header.get_header()->seq;
 msg.header.stamp.sec = in_status->get_status_data()->the_header.get_header()->time.seconds;
 msg.header.stamp.nsec = in_status->get_status_data()->the_header.get_header()->time.nanoseconds;
 msg.uptime = in_status->get_status_data()->uptime;
 msg.bus_voltage = in_status->get_status_data()->bus_voltage; //works
 return msg;
 }*/

void chatterCallback(const flat_bot_msgs::ModuleStatus::ConstPtr& msg) {
	ROS_INFO("I heard: bus voltage = [%f]", msg->bus_voltage);
}

/**
 * Server side of the boot_broadcast algorithm. When a module boots it sends a boot_broadcast message. Server checks if this has not already been ACKed, then sends ACK and adds module to the status table.
 */
void module_boot_received(udp_server* server){
	// Check if already ACKed

	// write some code here...

	// Send ACK
	Boot_Broadcast the_bb;
	the_bb.deserialise_bb(&recv_serial[0]);
	ACK the_ack;
	the_ack.populate_ack(the_bb.get_header()->get_header()->seq, &seq);
	char send_buf[200] = { 48 };
	server->send_to(MODULE_IP, send_buf, the_ack.serialise_ack(&send_buf[0]));
	//while(module status is "opening servo") stay in this loop
	//then return the final state (success/fail)
	ROS_INFO("Received boot_broadcast sequence #[%u]", the_bb.get_header()->get_header()->seq);
	seq++;

	//Add to status table

}

void parse_received(udp_server* server) {
	//parse received message
	// deserialise and populate the data into struct
	uint8_t msg_type = recv_serial[MSG_TYPE_POSITION];
	// skip switch case to debug whether it is causing messages to be lost
	module_boot_received(server);
	/*switch (msg_type) {
	case 0: //status
	{
		// Add to status table
		break;
	}
	case 1: //command
	{
		// ignore - ROS edge node sends commands only
		break;
	}
	case 2: //boot_broadcast
	{
		module_boot_received(server);
		break;
	}

	case 3: //ACK
	{
		// Implement some logic here later
		break;
	}
	} */
	recv_empty = 1;
}

int main(int argc, char **argv) {
	recv_empty = 1;
	seq = 0;
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise < flat_bot_msgs::ModuleStatus
			> ("chatter", 1000);
	ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
	ros::Rate loop_rate(0.2);

	int count = 0; // count how many messages sent
	boost::asio::io_service io_service;
	udp_server server(io_service, 1234, &recv_serial[0], &recv_empty);
	//attempt threading

	ModuleDriver driver(&server);
	//ModuleDriver driver();

	while (ros::ok()) {
		// UDP receive statuses and forward to ROS topic
		server.start_receive();
		io_service.run_one();
		std::cout << "ros::ok\n";
		flat_bot_msgs::ModuleStatus msg; //message object
		msg.bus_voltage = module_status[0].get_status_data()->bus_voltage; //works
		module_status[0].print_status();
		ROS_INFO("bus voltage = %f", msg.bus_voltage);
		chatter_pub.publish(msg);

		// Spin the publisher, subscriber and ActionServer queues
		ros::spinOnce();
		//loop_rate.sleep(); // code execution waits here (thread block)
		++count;
		std::cout << "***********************************************\n\n";

		if (!recv_empty) {
			parse_received(&server);
		}
	}

	return 0;
}
