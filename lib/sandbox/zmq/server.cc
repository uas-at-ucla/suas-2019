//
//  Hello World server in C++
//  Binds REP socket to tcp://*:5555
//  Expects "Hello" from client, replies with "World"
//

//#define ZMQ_BUILD_DRAFT_API 1

#include <iostream>
#include <string>
#include <unistd.h>

#include "zmq.hpp"

#include "lib/sandbox/zmq/mission_commands.pb.h"

#include <ostream>
#include <sstream>

int main() {
  //  Prepare our context and socket
  zmq::context_t context(1);
  // zmq::socket_t socket(context, ZMQ_DISH);
  // socket.bind("udp://127.0.0.1:6005");

  /*
  while (true) {
    ::spinny::controls::ground_communicator::Command cmd;
    cmd.set_type("test");
    cmd.set_latitude(1.0);
    cmd.set_longitude(3.0);

    ::std::string output;
    cmd.SerializeToString(&output);

    zmq::message_t request;

    //  Wait for next request from client
    socket.recv(&request);
    std::cout << "Received Hello" << std::endl;

    //  Do some 'work'
    usleep(1e6 / 200);

    //  Send reply back to client
    zmq::message_t reply(output.size());
    memcpy((void *)reply.data(), output.c_str(), output.size());
    socket.send(reply);
  }
  */
  return 0;
}
