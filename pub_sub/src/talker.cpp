#include <thread>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
//#include <thread> //original
#include <boost/thread.hpp>
#include <iostream>
#include <string>
#include "/home/alex/Y3P/y3p/module_code/libraries/module_status_data/module_status_data.h"
#include "ros/ros.h"
#include "flat_bot_msgs/ModuleStatus.h"
#include <stdint.h> // Added by AJY

#include <sstream>

using boost::asio::ip::udp;
//logging::add_console_log(std::cout, boost::log::keywords::format = ">> %Message%");

Status module_status;

// AJY change: for boost_1_65_1, use io_service instead of io_context

class udp_server
{
public:
  udp_server(boost::asio::io_service& io_service, ros::Publisher publisher) : socket_(io_service, udp::endpoint(udp::v4(), 1234))
  {
    start_receive(publisher);
  }

private:
  void start_receive(ros::Publisher publisher)
  {
    socket_.async_receive_from(boost::asio::buffer(recv_buffer_), remote_endpoint_,       boost::bind(&udp_server::handle_receive, this,          boost::asio::placeholders::error,          boost::asio::placeholders::bytes_transferred));
    module_status.deserialise_status(&recv_buffer_[0]);
    module_status.print_status();
    /* this is added to try to forward messages onto ROS topic */
/*    std::cout << "module_status...bus_voltage: " << module_status.get_status_data()->bus_voltage << std::endl;
    flat_bot_msgs::ModuleStatus msg; //message object
    msg.bus_voltage = *(reinterpret_cast<float *>(&(module_status.get_status_data()->bus_voltage)));
    ROS_INFO("%f", msg.bus_voltage);
    publisher.publish(msg);
    ros::spinOnce(); */
    /* end of addition */
  }

  void handle_receive(const boost::system::error_code& error,
      std::size_t /*bytes_transferred*/)
  {
    if (!error)
    {
  //    boost::shared_ptr<std::string> message(new std::string("Hello ESP1234"));

    //  socket_.async_send_to(boost::asio::buffer(*message), remote_endpoint_,          boost::bind(&udp_server::handle_send, this, message,           boost::asio::placeholders::error,            boost::asio::placeholders::bytes_transferred));

  //    start_receive();
    }
  }

  void handle_send(boost::shared_ptr<std::string> /*message*/,
      const boost::system::error_code& /*error*/,
      std::size_t /*bytes_transferred*/)
  {
  }

  udp::socket socket_;
  udp::endpoint remote_endpoint_;
  boost::array<char, 200> recv_buffer_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<flat_bot_msgs::ModuleStatus>("chatter", 1000);
  ros::Rate loop_rate(0.2);

  int count = 0; // count how many messages sent
  boost::asio::io_service io_service;
  try
  {
  //  boost::asio::io_service io_service; //original
  //  udp_server server(io_service); //original
//    udp_server server(io_service, chatter_pub); //original
//    boost::asio::deadline_timer t(io_service, boost::posix_time::seconds(4)); //added
//    t.async_wait(&u)

  //  udp_server server; //added
//    //t.reset(new std::thread(boost::bind(&udp_server::udp_server, server, &io_service))); //added
//    t.reset(new std::thread(boost::bind(&boost::asio::io_service::run, &io_service))); //added, compiles but runtume abort
    //std::thread t1(&io_service.run); //added
//    io_service.run(); //original
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  while (ros::ok())
  {
    udp_server server(io_service, chatter_pub);
    io_service.run_one();
    std::cout << "ros::ok\n";
    flat_bot_msgs::ModuleStatus msg; //message object
    //msg.bus_voltage = module_status.get_status_data()->bus_voltage;
    std::cout << "module_status...bus_voltage: " << module_status.get_status_data()->bus_voltage << std::endl;
    module_status.print_status();
    msg.bus_voltage = *(reinterpret_cast<float *>(&(module_status.get_status_data()->bus_voltage)));
    ROS_INFO("%f", msg.bus_voltage);
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep(); // code execution waits here (thread block)
    ++count;
    std::cout << "***********************************************\n\n";
  }

  return 0;
}
