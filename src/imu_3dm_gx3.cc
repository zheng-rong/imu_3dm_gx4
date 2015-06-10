// Interface to the Microstrain 3DM-GX4-25
// N. Michael & Zheng Rong

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

static float extract_float(unsigned char* addr)
{
  float tmp;

  *((unsigned char*)(&tmp) + 3) = *(addr);
  *((unsigned char*)(&tmp) + 2) = *(addr+1);
  *((unsigned char*)(&tmp) + 1) = *(addr+2);
  *((unsigned char*)(&tmp)) = *(addr+3);

  return tmp;
}

bool validate_checksum(const unsigned char *data, unsigned short length)
{
  unsigned short chksum = 0;
  unsigned short rchksum = 0;

  for (unsigned short i = 0; i < length - 2; i++)
    chksum += data[i];

  rchksum = data[length - 2] << 8;
  rchksum += data[length - 1];

  return (chksum == rchksum);
}

inline void print_bytes(const unsigned char *data, unsigned short length)
{
  for (unsigned int i = 0; i < length; i++)
    printf("%2x ", data[i]);
  puts("");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_3dm_gx4");
  ros::NodeHandle n("~");

  std::string name = ros::this_node::getName();

  std::string port;
  if (n.hasParam("port"))
    n.getParam("port", port);
  else
    {
      ROS_ERROR("%s: must provide a port", name.c_str());
      return -1;
    }

  boost::asio::io_service io_service;
  boost::asio::serial_port serial_port(io_service);

  unsigned short reply_length = 4;
  unsigned char reply[reply_length];

  int baud;
  n.param("baud", baud, 115200);

  char mode[4] = {'\xD4','\xA3','\x47','\x01'};

  while (true)
    {
      try
        {
          serial_port.open(port);
        }
      catch (boost::system::system_error &error)
        {
          ROS_ERROR("%s: Failed to open port %s with error %s",
                    name.c_str(), port.c_str(), error.what());
          return EXIT_FAILURE;
        }

      if (!serial_port.is_open())
        {
          ROS_ERROR("%s: failed to open serial port %s",
                    name.c_str(), port.c_str());
          return EXIT_FAILURE;
        }

      typedef boost::asio::serial_port_base sb;

      sb::baud_rate baud_option(baud);
      sb::flow_control flow_control(sb::flow_control::none);
      sb::parity parity(sb::parity::none);
      sb::stop_bits stop_bits(sb::stop_bits::one);

      serial_port.set_option(baud_option);
      serial_port.set_option(flow_control);
      serial_port.set_option(parity);
      serial_port.set_option(stop_bits);

      mode[3] = '\x01';
      boost::asio::write(serial_port, boost::asio::buffer(mode, 4));
      boost::asio::read(serial_port, boost::asio::buffer(reply, reply_length));
      if (!validate_checksum(reply, reply_length))
        {
          ROS_DEBUG("%s: failed to stop continuous mode", name.c_str());
          serial_port.close();
          continue;
        }

      if (reply[1] != '\x01')
        {
          std::cout << "Failed to set to active mode" << std::endl;
          serial_port.close();
          continue;
        }

      break;
    }

  const char preset[4] = {'\xD6','\xC6','\x6B','\xCC'};
  boost::asio::write(serial_port, boost::asio::buffer(preset, 4));

  boost::asio::read(serial_port, boost::asio::buffer(reply, reply_length));
  if (!validate_checksum(reply, reply_length))
    {
      ROS_ERROR("%s: failed to set continuous mode preset", name.c_str());
      if (serial_port.is_open())
        serial_port.close();
      return EXIT_FAILURE;
    }

  // Set the mode to continous output
  mode[3] = '\x02';
  boost::asio::write(serial_port, boost::asio::buffer(mode, 4));
  boost::asio::read(serial_port, boost::asio::buffer(reply, reply_length));
  if (!validate_checksum(reply, reply_length))
    {
      ROS_ERROR("%s: failed to set mode to continuous output", name.c_str());
      if (serial_port.is_open())
        serial_port.close();
      return EXIT_FAILURE;
    }

  unsigned short data_length = 79;
  unsigned char data[data_length];

  sensor_msgs::Imu msg;
  ros::Publisher pub = n.advertise<sensor_msgs::Imu>("imu", 10);

  while (n.ok())
    {
      boost::asio::read(serial_port, boost::asio::buffer(data, data_length));
      if (!validate_checksum(data, data_length))
        {
          ROS_ERROR("%s: checksum failed on message", name.c_str());
          continue;
        }

      unsigned int k = 1;
      float acc[3];
      float ang_vel[3];
      float mag[3];
      float M[9];

      for (unsigned int i = 0; i < 3; i++, k += 4)
        acc[i] = extract_float(&(data[k]));

      for (unsigned int i = 0; i < 3; i++, k += 4)
        ang_vel[i] = extract_float(&(data[k]));

      for (unsigned int i = 0; i < 3; i++, k += 4)
        mag[i] = extract_float(&(data[k]));

      for (unsigned int i = 0; i < 9; i++, k += 4)
        M[i] = extract_float(&(data[k]));

      msg.angular_velocity.x = ang_vel[0];
      msg.angular_velocity.y = ang_vel[1];
      msg.angular_velocity.z = ang_vel[2];

      msg.linear_acceleration.x = acc[0];
      msg.linear_acceleration.y = acc[1];
      msg.linear_acceleration.z = acc[2];

      // Inline transpose to generate transform from B -> W
      btMatrix3x3 rot(M[0], M[3], M[6],
                      M[1], M[4], M[7],
                      M[2], M[5], M[8]);
      btQuaternion quat;
      rot.getRotation(quat);
      quat.normalize();
      tf::quaternionTFToMsg(quat, msg.orientation);

      pub.publish(msg);
    }

  return 0;
}
