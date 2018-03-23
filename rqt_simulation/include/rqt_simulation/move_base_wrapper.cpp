#include <boost/python.hpp>

#include <string>

#include <ros/serialization.h>

#include <std_srvs/Empty.h>
//#include <std_msgs/Int64.h>

/* Read a ROS message from a serialized string.
  */
template <typename M>
M from_python(const std::string str_msg)
{
  size_t serial_size = str_msg.size();
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  for (size_t i = 0; i < serial_size; ++i)
  {
    buffer[i] = str_msg[i];
  }
  ros::serialization::IStream stream(buffer.get(), serial_size);
  M msg;
  ros::serialization::Serializer<M>::read(stream, msg);
  return msg;
}

/* Write a ROS message into a serialized string.
*/
template <typename M>
std::string to_python(const M& msg)
{
  size_t serial_size = ros::serialization::serializationLength(msg);
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  ros::serialization::OStream stream(buffer.get(), serial_size);
  ros::serialization::serialize(stream, msg);
  std::string str_msg;
  str_msg.reserve(serial_size);
  for (size_t i = 0; i < serial_size; ++i)
  {
    str_msg.push_back(buffer[i]);
  }
  return str_msg;
}

class MoveBaseWrapper : public move_base::MoveBase
{
public:
  MoveBaseWrapper() : MoveBase() {}

  std::string clearCostmapsService(const std::string& str_req, const std::string& str_resp)
  {
    nav_msgs::GetPlan::Request req = from_python<nav_msgs::GetPlan::Request>(str_req);
    nav_msgs::GetPlan::Response resp = from_python<nav_msgs::GetPlan::Response>(str_resp);

  }

}
