#include "/home/alex/Y3P/y3p/module_code/libraries/msg_header/udp_server.h"


public:
  UDP_Server::UDP_Server(boost::asio::io_service& io_service) : socket_(io_service, udp::endpoint(udp::v4(), 1234))
  {
    start_receive();
  }

//private:
  void UDP_Server::start_receive()
  {
    socket_.async_receive_from(boost::asio::buffer(recv_buffer_), remote_endpoint_,       boost::bind(&udp_server::handle_receive, this,          boost::asio::placeholders::error,          boost::asio::placeholders::bytes_transferred));
    module_status.deserialise_status(&recv_buffer_[0]);
//    module_status.print_status();
    /* this is added to try to forward messages onto ROS topic */
/*    std::cout << "module_status...bus_voltage: " << module_status.get_status_data()->bus_voltage << std::endl;
    flat_bot_msgs::ModuleStatus msg; //message object
    msg.bus_voltage = *(reinterpret_cast<float *>(&(module_status.get_status_data()->bus_voltage)));
    ROS_INFO("%f", msg.bus_voltage);
    publisher.publish(msg);
    ros::spinOnce(); */
    /* end of addition */
  }

  void UDP_Server::handle_receive(const boost::system::error_code& error,
      std::size_t /*bytes_transferred*/)
  {
    if (!error)
    {
  //    boost::shared_ptr<std::string> message(new std::string("Hello ESP1234"));

    //  socket_.async_send_to(boost::asio::buffer(*message), remote_endpoint_,          boost::bind(&udp_server::handle_send, this, message,           boost::asio::placeholders::error,            boost::asio::placeholders::bytes_transferred));

  //    start_receive();
    }
  }

  void UDP_Server::handle_send(boost::shared_ptr<std::string> message/*message*/, const boost::system::error_code& /*error*/, std::size_t /*bytes_transferred*/)
  {
//	  boost::shared_ptr<std::string> message(new std::string("Hello ESP1234"));

	  socket_.async_send_to(boost::asio::buffer(*message), remote_endpoint_,          boost::bind(&udp_server::handle_send, this, message, boost::asio::placeholders::error,            boost::asio::placeholders::bytes_transferred));
  }

  udp::socket socket_;
  udp::endpoint remote_endpoint_;
  boost::array<char, 200> recv_buffer_;
};
