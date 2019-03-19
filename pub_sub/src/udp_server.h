/**
 * Adapted from https://www.boost.org/doc/libs/1_65_1/doc/html/boost_asio/tutorial/tutdaytime6.html
 *
 */
//#include <thread>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <string>
#include <ros/ros.h> //for ROS_INFO
#include "/home/alex/Y3P/y3p/module_code/libraries/module_status_data/module_status_data.h"

#ifndef udp_server_h
#define udp_server_h

using boost::asio::ip::udp;

/**
 * When instantiated, the server binds to the given port and listens for incoming messages.
 */
class udp_server {
private:
	udp::socket socket_;
	udp::endpoint remote_endpoint_;
	boost::array<char, 200> recv_buffer_; // difference between this and char[]?
	//Status *module_status_ptr;
	char* recv_serial; //!< Pointer to a buffer created in calling function into which received serial data is to be copied
	uint8_t* recv_empty; //!< Pointer to a flag used to indicate if there is received data
	boost::thread server_thread;
public:
	/**
	 * udp_server constructor initialises class member socket to listen on port
	 * \param io_service Boost.IO io_service provides input/output functionality
	 * \param port UDP port used by module-ROS communication
	 * \param the_recv_serial Pointer to char[] buffer to write received serial data
	 * \param the_recv_empty Pointer to the flag used to indicate if there is a received packet
	 */
	udp_server(boost::asio::io_service& io_service, unsigned short port,
			char* the_recv_serial, uint8_t* the_recv_empty);
//private:

	/**
	 * Listens in the background for a new request  (incoming packet). Calls handle_receive when a request is received
	 */
	void start_receive();
	/**
	 * Check incoming packet for errors & copy to recv_serial for talker.cpp to use. After parsing the packet, call start_receive again to continue listening
	 */
	void handle_receive(const boost::system::error_code& error,
			std::size_t /*bytes_transferred*/);

	/**
	   * Send a message to a recipient IP address
	   *
	   * \param targetIP IP address to send to
	   * \param message_start pointer to the beginning of the message to send
	   * \param message_end pointer to the end of the message to send
	   */
	void send_to(std::string targetIP, char* message_start, char* message_end);

	/**
	 * Called after completing a UDP send operation.
	 */
	void handle_send(boost::shared_ptr<std::string> message/*message*/,
			const boost::system::error_code& /*error*/,
			std::size_t /*bytes_transferred*/);
	~udp_server();

};

#endif
