/**
 * udp_server.h
 * Adapted from https://www.boost.org/doc/libs/1_65_1/doc/html/boost_asio/tutorial/tutdaytime6.html
 */
#include <thread>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <string>
 
#ifndef udp_server_h
#define udp_server_h

using boost::asio::ip::udp;

class UDP_Server
{
private:
	  udp::socket socket_;
	  udp::endpoint remote_endpoint_;
	  boost::array<char, 200> recv_buffer_;
public:
	  UDP_Server(boost::asio::io_service& io_service) : socket_(io_service, udp::endpoint(udp::v4(), 1234));

//private:
  void start_receive();

  void handle_receive(const boost::system::error_code& error, std::size_t /*bytes_transferred*/);

  void handle_send(boost::shared_ptr<std::string> message/*message*/, const boost::system::error_code& /*error*/, std::size_t /*bytes_transferred*/);

};

#endif
