//
//  Hello World client in C++
//  Connects REQ socket to tcp://localhost:5555
//  Sends "Hello" to server, expects "World" back
//
//#define ZMQ_BUILD_DRAFT_API 1

#include <iostream>
#include <sstream>
#include <string>

#include "zmq.hpp"

#include "lib/sandbox/zmq/mission_commands.pb.h"

int main() {
  //  Prepare our context and socket
  zmq::context_t context(1);
  // zmq::socket_t socket(context, ZMQ_RADIO);

  // std::cout << "Connecting to hello world server…" << std::endl;
  // socket.connect("udp://127.0.0.1:6005");

  //  Do 10 requests, waiting each time for a response
  /*
  for (int request_nbr = 0; request_nbr != 10 || true; request_nbr++) {
    zmq::message_t request(5);
    memcpy(request.data(), "Hello", 5);
    std::cout << "Sending Hello " << request_nbr << "…" << std::endl;
    socket.send(request);


    //  Get the reply.
    zmq::message_t reply;
    socket.recv(&reply);

    ::std::string input(static_cast<char *>(reply.data()), reply.size());

    ::spinny::controls::ground_communicator::Command cmd;
    cmd.ParseFromString(input);
    std::cout << cmd.type() << std::endl;

    std::cout << "Received World " << reply.data() << std::endl;

  }
  */

  return 0;
}
