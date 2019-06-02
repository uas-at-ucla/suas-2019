#include "io.h"

#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <signal.h>
#include <stdlib.h>

// SIGINT code taken from here:
// https://answers.ros.org/question/27655/what-is-the-correct-way-to-do-stuff-before-a-node-is-shutdown/

sig_atomic_t volatile g_request_shutdown = 0;
static ::src::controls::io::IO *io = nullptr;
extern "C" void signal_handler(int signum) {
  ::std::cout << "GOT SIGNAL!" << ::std::endl;
  if (io != nullptr) {
    io->Quit(signum);
  }

  ::ros::shutdown();
  exit(0);
}

void shutdownCallback(XmlRpc::XmlRpcValue &params,
                      XmlRpc::XmlRpcValue &result) {
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();

  if (num_params > 1) {
    ::std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    g_request_shutdown = 1;
  }

  result = ::ros::xmlrpc::responseInt(1, "", 0);
}

int main(int argc, char **argv) {
  ::ros::init(argc, argv, "uasatucla_io", ::ros::init_options::NoSigintHandler);
  signal(SIGQUIT, signal_handler);
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);
  signal(SIGHUP, signal_handler);

  // ::ros::XMLRPCManager::instance()->unbind("shutdown");
  // ::ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

  ::ros::start();

// #ifndef RASPI_DEPLOYMENT
  // Log more verbose output if running in SITL.
  if (::ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ::ros::console::levels::Debug)) {
    ::ros::console::notifyLoggerLevelsChanged();
  }
// #endif

  io = new ::src::controls::io::IO();
  ::ros::spin();

  io->Quit(2);

  ::ros::shutdown();
}
