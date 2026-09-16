#pragma once
#include <cctype>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
namespace px4_vio_bridge
{
// Same role/path as Python ProcessSingleton. Acquire before any flight publisher.
class FlightAuthority
{
public:
  FlightAuthority()
  {
    const auto * value = std::getenv("ROS_DOMAIN_ID");
    std::string domain = value ? value : "0";
    for (auto & c : domain) {
      if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_' && c != '-' && c != '.') {c = '_';}
    }
    const auto path = "/tmp/px4_vio_bridge_flight_authority_ros_domain_" + domain + ".lock";
    fd_ = ::open(path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0644);
    if (fd_ < 0) {throw std::runtime_error("cannot open flight authority lock");}
    if (::flock(fd_, LOCK_EX | LOCK_NB) != 0) {
      ::close(fd_); fd_ = -1;
      throw std::runtime_error("another flight adapter owns " + path);
    }
  }
  ~FlightAuthority() {if (fd_ >= 0) {::close(fd_);}}
  FlightAuthority(const FlightAuthority &) = delete;
  FlightAuthority & operator=(const FlightAuthority &) = delete;
private:
  int fd_{-1};
};
}
