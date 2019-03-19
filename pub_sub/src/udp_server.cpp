#include "/home/alex/Y3P/catkin_workspace/src/pub_sub/src/udp_server.h"

using boost::asio::ip::udp;


udp_server::udp_server(boost::asio::io_service& io_service, unsigned short port,  char* the_recv_serial, uint8_t* the_recv_empty) : socket_(io_service, udp::endpoint(udp::v4(), port))
  {
	recv_serial = the_recv_serial;
	recv_empty = the_recv_empty;
   // start_receive();
    server_thread = boost::thread(&udp_server::start_receive, this);
  }

udp_server::~udp_server(){
	server_thread.join();
}

void udp_server::start_receive()
  {
    socket_.async_receive_from(
        boost::asio::buffer(recv_buffer_), remote_endpoint_,
        boost::bind(&udp_server::handle_receive, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
  }


void udp_server::handle_receive(const boost::system::error_code& error,
      std::size_t /*bytes_transferred*/)
  {
    //proceed if no error or if the error is that the message is bigger than recv_buffer_
    if (!error || error == boost::asio::error::message_size)
    {
    //  boost::shared_ptr<std::string> message(new std::string("Received Status"));
    	*recv_empty = 0;
    	// copy message from recv_buffer_ into recv_serial
    	for (int i = 0; i < recv_buffer_[0]; i++){
    		*(recv_serial + i) = recv_buffer_[i];
    	}
      sleep(3);
      //send message to the client
    /*  socket_.async_send_to(boost::asio::buffer(*message), remote_endpoint_,
          boost::bind(&udp_server::handle_send, this, message,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred)); */

      //start listening again
      start_receive();
    }
  }


void udp_server::send_to(std::string targetIP, char* message_start, char* message_end){
    //make a message to send
  //  boost::shared_ptr<std::string> message(new std::string("Hello ESP1234"));
 //   sleep(3);
	boost::shared_ptr<std::string> message(new std::string(message_start, sizeof(char)*(message_end-message_start)));
	// Set IP address
	remote_endpoint_.address(boost::asio::ip::address_v4::from_string(targetIP));
    //send message to the client (run on a ROS::spin()) - use socket_.send_to to send immediately
  //  socket_.async_send_to(boost::asio::buffer(message_start, (std::size_t)(sizeof(char)*(message_end-message_start))), remote_endpoint_,
	remote_endpoint_.port(1234);
	socket_.async_send_to(boost::asio::buffer(*message), remote_endpoint_,
	boost::bind(&udp_server::handle_send, this, message,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
    ROS_INFO("Sent [%lu] bytes to [%s]", message->size(), targetIP.c_str());
}

//invoked after the service request has been completed
void udp_server::handle_send(boost::shared_ptr<std::string> /*message*/,
      const boost::system::error_code& /*error*/,
      std::size_t /*bytes_transferred*/)
  {
    //application level logic here
  }

