// Exercise real C++ DDS serialization without using a robot API topic.
// Fixed loopback interface and domain 223; never uses robot domain 0.
#include <unitree/robot/internal/internal_idl_decl/Request_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <iostream>

int main() {
  unitree::robot::ChannelFactory::Instance()->Init(223, "lo");
  unitree::robot::ChannelPublisher<unitree_api::msg::dds_::Request_>
      publisher("rt/b2_sdk_local_dds_write_check");
  publisher.InitChannel();
  unitree_api::msg::dds_::Request_ message;
  message.parameter("{}");
  if (!publisher.Write(message)) {
    std::cerr << "local DDS write failed" << std::endl;
    return 1;
  }
  std::cout << "local DDS write OK (domain=223, interface=lo, no robot API)"
            << std::endl;
  return 0;
}
