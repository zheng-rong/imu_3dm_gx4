#include <csignal>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_utils/GeometryUtilsROS.h>
#include <GeographicLib/NormalGravity.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <imu_3dm_gx3/GPSFix.h>

#include <asio_serial_device/ASIOSerialDevice.h>

namespace gu = geometry_utils;
namespace gr = gu::ros;

#define DESCRIPTOR_BASE_COMMAND 0x01
#define DESCRIPTOR_3DM_COMMAND 0x0C
#define DESCRIPTOR_AHRS_DATA 0x80
#define DESCRIPTOR_GPS_DATA 0x81

class ImuPacket
{
public:
  ImuPacket(unsigned char descriptor_, unsigned char field_descriptor_)
  {
    descriptor = descriptor_;
    field_descriptor = field_descriptor_;

    header.resize(4);
    header[0] = 0x75;
    header[1] = 0x65;

    checksum.resize(2);
    payload.clear();
    field_data.clear();
  }
  ~ImuPacket() {}

  // 4 header (0x75, 0x65, descriptor, payload_length ) + 
  // n payload (field length, field_descriptor, field data) + 
  // 2 checksum
  void Assemble(std::vector<unsigned char>& buffer)
  {
    AssemblePayload();
    AssembleHeader();
    AssembleChecksum();

    buffer.clear();
    buffer.resize(2 + payload.size() + 4);
    for (unsigned int i = 0; i < 4; i++)
      buffer[i] = header[i];
    for (unsigned int i = 0; i < payload.size(); i++)
      buffer[i + 4] = payload[i];
    for (unsigned int i = 0; i < 2; i++)
      buffer[i + 4 + payload.size()] = checksum[i];
  }

  void AddFieldData(unsigned char c)
  {
    field_data.push_back(c);
  }
private:
  void AssembleHeader()
  {
    header[2] = descriptor;
    header[3] = payload.size();
  }

  void AssemblePayload()
  {
    payload.resize(2 + field_data.size());

    payload[0] = (unsigned char)(2 + field_data.size());
    payload[1] = field_descriptor;

    for (unsigned int i = 0; i < field_data.size(); i++)
      payload[2 + i] = field_data[i];
  }

  void AssembleChecksum()
  {
    checksum[0] = 0;
    checksum[1] = 0;

    for (unsigned int i = 0; i < 4; i++)
      {
        checksum[0] += header[i];
        checksum[1] += checksum[0];
      }

    for (unsigned int i = 0; i < payload.size(); i++)
      {
        checksum[0] += payload[i];
        checksum[1] += checksum[0];
      }
  }

  unsigned char descriptor, field_descriptor;

  std::vector<unsigned char> header, payload, checksum, field_data;
};







class ImuPacketParser
{
public:
  typedef enum
    {
      START1,
      START2,
      DESCRIPTOR,
      PAYLOAD_LENGTH,
      PAYLOAD,
      CHECKSUM1,
      CHECKSUM2
    } imu_packet_parser_state_t;

  ImuPacketParser()
  {
    state = START1;

    header.resize(4);
    header[0] = 0x75;
    header[1] = 0x65;

    payload.clear();
    checksum.resize(2);
  }
  ~ImuPacketParser() {}

  bool ParseChar(unsigned char c)
  {
    bool done_parsing = false;

    switch (state)
      {
      case START1:
        if (c == header[0])
          state = START2;
        break;
      case START2:
        if (c == header[1])
          state = DESCRIPTOR;
        else
          state = START1;
        break;
      case DESCRIPTOR:
        header[2] = c;
        state = PAYLOAD_LENGTH;
        break;
      case PAYLOAD_LENGTH:
        header[3] = c;
        payload.resize(c);
        payload_length = (unsigned int)c;
        payload_counter = payload_length;
        state = PAYLOAD;
        break;
      case PAYLOAD:
        payload[payload_length - payload_counter] = c;
        payload_counter--;

        if (payload_counter == 0)
          state = CHECKSUM1;
        break;
      case CHECKSUM1:
        checksum[0] = c;
        state = CHECKSUM2;
        break;
      case CHECKSUM2:
        checksum[1] = c;
        done_parsing = true;
        break;
      }

    bool received = false;
    if (done_parsing)
      {
        unsigned char checksum1 = 0;
        unsigned char checksum2 = 0;

        for (unsigned int i = 0; i < 4; i++)
          {
            checksum1 += header[i];
            checksum2 += checksum1;
          }

        for (unsigned int i = 0; i < payload.size(); i++)
          {
            checksum1 += payload[i];
            checksum2 += checksum1;
          }

        if ((checksum1 == checksum[0]) && (checksum2 == checksum[1]))
          received = true;

        state = START1;
      }

    return received;
  }

  unsigned char GetDescriptionSet()
  {
    return header[2];
  }

  float ExtractFloat(unsigned char* addr)
  {
    float tmp;

    *((unsigned char*)(&tmp) + 3) = *(addr);
    *((unsigned char*)(&tmp) + 2) = *(addr+1);
    *((unsigned char*)(&tmp) + 1) = *(addr+2);
    *((unsigned char*)(&tmp)) = *(addr+3);

    return tmp;
  }

  double ExtractDouble(unsigned char* addr)
  {
    double tmp;

    *((unsigned char*)(&tmp) + 7) = *(addr);
    *((unsigned char*)(&tmp) + 6) = *(addr+1);
    *((unsigned char*)(&tmp) + 5) = *(addr+2);
    *((unsigned char*)(&tmp) + 4) = *(addr+3);
    *((unsigned char*)(&tmp) + 3) = *(addr+4);
    *((unsigned char*)(&tmp) + 2) = *(addr+5);
    *((unsigned char*)(&tmp) + 1) = *(addr+6);
    *((unsigned char*)(&tmp)) = *(addr+7);

    return tmp;
  }

  unsigned int ExtractUnsignedInt(unsigned char* addr)
  {
    unsigned int tmp;

    *((unsigned char*)(&tmp) + 3) = *(addr);
    *((unsigned char*)(&tmp) + 2) = *(addr+1);
    *((unsigned char*)(&tmp) + 1) = *(addr+2);
    *((unsigned char*)(&tmp)) = *(addr+3);

    return tmp;
  }

  unsigned short ExtractUnsignedShort(unsigned char* addr)
  {
    unsigned short tmp;

    *((unsigned char*)(&tmp) + 1) = *(addr);
    *((unsigned char*)(&tmp)) = *(addr+1);

    return tmp;
  }

  void GetPayload(std::vector<unsigned char>& p)
  {
    p.clear();
    p.resize(payload.size());
    for (unsigned int i = 0; i < payload.size(); i++)
      p[i] = payload[i];
  }

  private:
  imu_packet_parser_state_t state;
  unsigned int payload_counter, payload_length;
  std::vector<unsigned char> header, payload, checksum;
};

ImuPacketParser p;

bool imu_time_set = false;
double imu_delta_t = 0.0;
bool imu_msg_ready = false;
sensor_msgs::Imu imu_msg;
nav_msgs::Odometry gps_msg;
bool gps_time_set = false;
double gps_delta_t = 0.0;
bool gps_msg_ready = false;
bool mag_msg_ready = false;

double gravity = 9.802;
bool gravity_set = false;

ros::Publisher imu_pub;
ros::Publisher gps_pub;
ros::Publisher mag_pub;
ros::Publisher gps_fix_pub;

imu_3dm_gx3::GPSFix gps_fix_msg;
geometry_msgs::Vector3Stamped mag_msg;

std::vector<double> M(9);

bool PPS_present = false;
unsigned char last_toggle = 0x00;
unsigned char this_toggle = 0x00;
bool PPS_initialized = false;

double origin_lat, origin_lon, origin_height;
bool has_device_info = false;

std::string model_name;

bool enable_gps = false;

void read_callback(const unsigned char* buf, size_t len)
{
  for (unsigned int i = 0; i < len; i++)
    {
      if (p.ParseChar(buf[i]))
        {
          std::vector<unsigned char> m;
          p.GetPayload(m);

          unsigned int i = 0;
          while (i < m.size())
            {
              unsigned char length = m[i++];
              unsigned char descriptor = m[i++];
              unsigned char data[length - 2];
              for (unsigned int j = 0; j < length - 2; j++)
                data[j] = m[i++];

              switch (p.GetDescriptionSet())
                {
                case DESCRIPTOR_BASE_COMMAND:
                  {
                    switch (descriptor)
                      {
                      case 0xF1: // Ack
                        //ROS_INFO("==: DESCRIPTOR_BASE_COMMAND ACK");
                        //std::cout << "Got Ack to cmd = " << (int)m[2] << ", ";
                        //std::cout << "code = " << (int)m[3] << std::endl;
                        break;
                      case 0x81: // Device info
                        {
                          // unsigned short firmware_version =
                          //   p.ExtractUnsignedShort(&(data[0]));
                          //ROS_INFO("==: Device Info processing...");

                          unsigned char buf[82];
                          for (unsigned int i = 0; i < 16; i++)
                            buf[i] = data[2 + i];
                          model_name.assign((char*)buf, 16);
                          
                          // Remove leading whitespace
                          std::stringstream trimmer;
                          trimmer << model_name;
                          trimmer >> model_name;
                          

                          has_device_info = true;
                        }
                        break;
                      }
                  }
                  break;
                case DESCRIPTOR_3DM_COMMAND:
                  {
                    switch (descriptor)
                      {
                      case 0xF1: // Ack
                        //ROS_INFO("==: DESCRIPTOR_3DM_COMMAND ACK");
                        //std::cout << "Got Ack to cmd = " << (int)m[2] << ", ";
                        //std::cout << "code = " << (int)m[3] << std::endl;
                        break;
                      }
                    break;
                  }
                case DESCRIPTOR_AHRS_DATA:
                  {
                    //ROS_INFO("==: AHRS DATA");
                    switch (descriptor)
                      {
                      case 0x04: // Scaled Accelerometer Vector
                        {
                          imu_msg.linear_acceleration.x = gravity*p.ExtractFloat(&(data[0]));
                          imu_msg.linear_acceleration.y = gravity*p.ExtractFloat(&(data[4]));
                          imu_msg.linear_acceleration.z = gravity*p.ExtractFloat(&(data[8]));
                        }
                        break;
                      case 0x05: // Scaled Gyro Vector
                        {
                          imu_msg.angular_velocity.x = p.ExtractFloat(&(data[0]));
                          imu_msg.angular_velocity.y = p.ExtractFloat(&(data[4]));
                          imu_msg.angular_velocity.z = p.ExtractFloat(&(data[8]));
                        }
                        break;
                      case 0x06: // Scaled Magnetometer Vector
                        {
                          mag_msg.vector.x = p.ExtractFloat(&(data[0]));
                          mag_msg.vector.y = p.ExtractFloat(&(data[4]));
                          mag_msg.vector.z = p.ExtractFloat(&(data[8]));

                          mag_msg_ready = true;
                        }
                        break;
                      case 0x0A: // Quaternion
                        {
                          float qw = p.ExtractFloat(&(data[0]));
                          float qx = p.ExtractFloat(&(data[4]));
                          float qy = p.ExtractFloat(&(data[8]));
                          float qz = p.ExtractFloat(&(data[12]));

                          imu_msg.orientation = gr::toQuatMsg(gu::Quat(qw, qx, qy, qz));
                        }
                        break;
                      case 0x0E: // Internal Timestamp, NOTE: assumes timestamp is last entry
                        {
                          unsigned int t = p.ExtractUnsignedInt(&(data[0]));
                          double tsec = t/62500.0;

                          if (!imu_time_set)
                            {
                              imu_delta_t = ros::Time::now().toSec() - tsec;
                              imu_time_set = true;
                            }

                          double drift = tsec + imu_delta_t - ros::Time::now().toSec();
                          if (fabs(drift) > 1e-2)
                            {
                              ROS_WARN("%s: imu time drifting by %f",
                                       ros::this_node::getName().c_str(), drift);
                              imu_delta_t = ros::Time::now().toSec() - tsec;
                            }

                          imu_msg.header.stamp = ros::Time(tsec + imu_delta_t);
                          mag_msg.header.stamp = imu_msg.header.stamp;

                          imu_msg_ready = true;
                        }
                        break;
                      case 0x12: // GPS correlation Timestamp from GX4-25, NOTE: assumes timestamp is last entry
                        {
                          double week_time = p.ExtractDouble(&(data[0]));
                          unsigned int week = p.ExtractUnsignedShort(&(data[8]));
                          unsigned int time_flag = p.ExtractUnsignedShort(&(data[10]));
                          double tsec = week_time;

                          this_toggle = time_flag & 0x02;

                          if ((time_flag & 0x01 == 0x01) && (time_flag & 0x04 == 0x04) && (!PPS_initialized))
                          {
                            PPS_present = true;
                            PPS_initialized = true;
                            ROS_INFO("%s: GPS PPS beacon time is Good and Initialized!", ros::this_node::getName().c_str());
                          }

                          if ((time_flag & 0x01 == 0x01) && ((this_toggle ^  last_toggle) == 0x02))
                          {
                            PPS_present = true;
                            last_toggle = this_toggle;
                            ROS_INFO("%s: GPS PPS time refreshed! %f", ros::this_node::getName().c_str(), tsec);
                          }


                          

                          if (!imu_time_set)
                          {
                            imu_delta_t = ros::Time::now().toSec() - tsec;
                            imu_time_set = true;

                            if(!PPS_present)
                              ROS_ERROR("%s: GPS PPS do NOT exist!", ros::this_node::getName().c_str());
                          }
/*
                          double drift = tsec + imu_delta_t - ros::Time::now().toSec();
                          if (fabs(drift) > 1e-2)
                          {
                            ROS_WARN("%s: imu time drifting by %f",
                                     ros::this_node::getName().c_str(), drift);
                            imu_delta_t = ros::Time::now().toSec() - tsec;
                          }

                          imu_msg.header.stamp = ros::Time(tsec + imu_delta_t);
                          mag_msg.header.stamp = imu_msg.header.stamp;
*/
                        imu_msg.header.stamp = ros::Time::now();
                        mag_msg.header.stamp = imu_msg.header.stamp;

                          imu_msg_ready = true;
                        }
                        break;
                      }
                    break;
                  }
                case DESCRIPTOR_GPS_DATA:
                  {
                    switch (descriptor)
                      {
                      case 0x03: // LLH Position
                        {
                          double lat = p.ExtractDouble(&(data[0]));
                          double lon = p.ExtractDouble(&(data[8]));
                          double h_ellipsoid = p.ExtractDouble(&(data[16]));
                          //double h_msl = p.ExtractDouble(&(data[24]));
                          float h_accuracy = p.ExtractFloat(&(data[32]));
                          float v_accuracy = p.ExtractFloat(&(data[36]));
                          unsigned short flags = p.ExtractUnsignedShort(&(data[40]));

                          if (flags > 0)
                            {
                              if (!gravity_set)
                                {
                                  const GeographicLib::NormalGravity& g =
                                    GeographicLib::NormalGravity::WGS84();
                                  gravity = g.SurfaceGravity(lat);
                                  gravity_set = true;
                                  ROS_INFO("%s: setting gravity = %f",
                                           ros::this_node::getName().c_str(), gravity);
                                }

                              // Convert to local cartesion
                              GeographicLib::LocalCartesian loc(origin_lat,
                                                                origin_lon,
                                                                origin_height);
                              double x, y, z;
                              loc.Forward(lat, lon, h_ellipsoid, x, y, z, M);

                              gps_msg.pose.pose.position.x = x;
                              gps_msg.pose.pose.position.y = y;
                              gps_msg.pose.pose.position.z = z;

                              gps_msg.pose.covariance[0] = h_accuracy*h_accuracy; // C(1, 1)
                              gps_msg.pose.covariance[7] = h_accuracy*h_accuracy; // C(2, 2)
                              gps_msg.pose.covariance[14] = v_accuracy*v_accuracy; // C(3, 3)
                            }
                        }
                        break;
                      case 0x05: // NED Velocity
                        {
                          float north = p.ExtractFloat(&(data[0]));
                          float east = p.ExtractFloat(&(data[4]));
                          float down = p.ExtractFloat(&(data[8]));
                          //float speed = p.ExtractFloat(&(data[12]));
                          //float ground_speed = p.ExtractFloat(&(data[16]));
                          float heading = p.ExtractFloat(&(data[20]));
                          float speed_accuracy = p.ExtractFloat(&(data[24]));
                          float heading_accuracy = p.ExtractFloat(&(data[28]));
                          unsigned short flags = p.ExtractUnsignedShort(&(data[32]));

                          if (flags > 0)
                            {
                              // Convert from END to ENU, then to local cartesian
                              float up = -down;
                              double vx = M[0]*east + M[1]*north + M[2]*up;
                              double vy = M[3]*east + M[4]*north + M[5]*up;
                              double vz = M[6]*east + M[7]*north + M[8]*up;

                              gps_msg.twist.twist.linear.x = vx;
                              gps_msg.twist.twist.linear.y = vy;
                              gps_msg.twist.twist.linear.z = vz;

                              // Invert heading to and subtract 90 deg
                              gps_msg.pose.pose.orientation =
                                gr::ZYXToQuatMsg(gu::Vec3(0.0, 0.0,
                                                               -heading*M_PI/180.0 - M_PI/2.0));

                              // TODO: Error models almost certainly inaccurate
                              gps_msg.pose.covariance[35] =
                                pow(heading_accuracy*M_PI/180.0, 2.0);

                              gps_msg.twist.covariance[0] = speed_accuracy*speed_accuracy;
                              gps_msg.twist.covariance[7] = speed_accuracy*speed_accuracy;
                              gps_msg.twist.covariance[14] = speed_accuracy*speed_accuracy;

                              //printf("NED = %f, %f, %f\n", north, east, down);
                              //printf("speed = %f\n", speed);
                              //printf("ground speed = %f\n", ground_speed);
                              //printf("heading = %f\n", heading);
                              // printf("accuracy (speed/heading) = %f, %f\n",
                              //        speed_accuracy, heading_accuracy);
                              // printf("flags = %u\n", flags);
                            }
                        }
                        break;
                      case 0x0B: // GPS Fix Information
                        {
                          unsigned char type = data[0];
                          unsigned char n_sv = data[1];
                          //unsigned short fix_flags = p.ExtractUnsignedShort(&(data[2]));
                          unsigned short valid_flags = p.ExtractUnsignedShort(&(data[4]));

                          if (valid_flags > 0)
                            {
                              switch (type)
                                {
                                case 0x00:
                                  gps_fix_msg.type = imu_3dm_gx3::GPSFix::TYPE_3D_FIX;
                                  break;
                                case 0x01:
                                  gps_fix_msg.type = imu_3dm_gx3::GPSFix::TYPE_2D_FIX;
                                  break;
                                case 0x02:
                                  gps_fix_msg.type = imu_3dm_gx3::GPSFix::TYPE_TIME_ONLY;
                                  break;
                                case 0x03:
                                  gps_fix_msg.type = imu_3dm_gx3::GPSFix::TYPE_NONE;
                                  break;
                                case 0x04:
                                  gps_fix_msg.type = imu_3dm_gx3::GPSFix::TYPE_INVALID;
                                  break;
                                }
                              gps_fix_msg.count = n_sv;
                            }
                        }
                        break;
                      case 0x08: // UTC Time
                        {
                          unsigned short year = p.ExtractUnsignedShort(&(data[0]));
                          unsigned char month = data[2];
                          unsigned char day = data[3];
                          unsigned char hour = data[4];
                          unsigned char minute = data[5];
                          unsigned char second = data[6];
                          unsigned int millisecond = p.ExtractUnsignedInt(&(data[7]));
                          unsigned short flags = p.ExtractUnsignedShort(&(data[11]));

                          if (flags > 0)
                            {
                              // Convert from time above to seconds
                              boost::gregorian::date d(year, month, day);
                              boost::posix_time::time_duration dur =
                                boost::posix_time::hours(hour) +
                                boost::posix_time::minutes(minute) +
                                boost::posix_time::seconds(second) +
                                boost::posix_time::milliseconds(millisecond);
                              boost::posix_time::ptime t(d, dur);

                              // ROS time version
                              ros::Time tr = ros::Time::fromBoost(t);
                              double tsec = tr.toSec();

                              if (!gps_time_set)
                                {
                                  gps_delta_t = ros::Time::now().toSec() - tsec;
                                  gps_time_set = true;
                                }

                              double drift = tsec + gps_delta_t - ros::Time::now().toSec();
                              if (fabs(drift) > 1e-2)
                                {
                                  ROS_WARN("%s: gps time drifting by %f",
                                           ros::this_node::getName().c_str(), drift);
                                  gps_delta_t = ros::Time::now().toSec() - tsec;
                                }

                              gps_msg.header.stamp = ros::Time(tsec + gps_delta_t);
                              gps_msg_ready = true;

                              gps_fix_msg.header.stamp = gps_msg.header.stamp;
                            }
                          else
                            {
                              gps_fix_msg.header.stamp = ros::Time(0);
                              gps_fix_msg.header.frame_id = std::string("invalid");
                            }
                          // Publish the fix message always
                          gps_fix_pub.publish(gps_fix_msg);
                        }
                        break;
                      }
                  }
                }
            }

          if (imu_msg_ready)
            {
              imu_pub.publish(imu_msg);
              if (mag_msg_ready)
                {
                  mag_pub.publish(mag_msg);
                  mag_msg_ready = false;
                }
              imu_msg_ready = false;
            }

          if (gps_msg_ready)
            {
              gps_pub.publish(gps_msg);
              gps_msg_ready = false;
            }
        }
    }
}

ASIOSerialDevice d;

void sigint_handler(int signum)
{
  if (d.Active())
    {
      std::vector<unsigned char> ms;
      // Disable AHRS Stream
      ImuPacket p1(DESCRIPTOR_3DM_COMMAND, 0x11);
      p1.AddFieldData(0x01);
      p1.AddFieldData(0x01);
      p1.AddFieldData(0x00);
      p1.Assemble(ms);
      d.Write(ms);

      if (enable_gps)
        {
          // Disable GPS Stream
          ImuPacket p2(DESCRIPTOR_3DM_COMMAND, 0x11);
          p2.AddFieldData(0x01);
          p2.AddFieldData(0x02);
          p2.AddFieldData(0x00);
          p2.Assemble(ms);
          d.Write(ms);
        }

      // Set to idle mode
      ImuPacket p3(DESCRIPTOR_BASE_COMMAND, 0x02);
      p3.Assemble(ms);
      d.Write(ms);
    }

  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_3dm_gx3", ros::init_options::NoSigintHandler);
  signal(SIGINT, sigint_handler);

  ros::NodeHandle n("~");

  std::string name = ros::this_node::getName();

  std::string port;
  if (!n.hasParam("port"))
    {
      ROS_ERROR("%s: must provide a port", name.c_str());
      return EXIT_FAILURE;
    }
  n.getParam("port", port);
  ROS_INFO("%s: got a port: %s", name.c_str(), port.c_str());

  int rate;
  if (!n.hasParam("rate"))
    {
      ROS_ERROR("%s: must provide a baud rate", name.c_str());
      return EXIT_FAILURE;
    }
  n.getParam("rate", rate);
  ROS_INFO("%s: got baud-rate: %i", name.c_str(), rate);

  try
    {
      d.Open(port, rate);
      ROS_INFO("%s: succeed to open the port - rate:%i", name.c_str(), rate);
    }
  catch (std::exception& e)
    {
      ROS_ERROR("%s: failed to open port:rate %s:%i", name.c_str(), port.c_str(), rate);
      return EXIT_FAILURE;
    }

  d.SetReadCallback(read_callback);

  d.Start();
  ROS_INFO("%s: start to read data", name.c_str());

  std::vector<unsigned char> ms;
  // Disable AHRS Stream
  ImuPacket p1(DESCRIPTOR_3DM_COMMAND, 0x11);
  p1.AddFieldData(0x01);
  p1.AddFieldData(0x01);
  p1.AddFieldData(0x00);
  p1.Assemble(ms);
  d.Write(ms);

  ROS_INFO("%s: Disable AHRS Stream", name.c_str());
  ros::Duration(0.5).sleep();
  
  /**/
  // Disable GPS Stream
  ImuPacket p2(DESCRIPTOR_3DM_COMMAND, 0x11);
  p2.AddFieldData(0x01);
  p2.AddFieldData(0x02);
  p2.AddFieldData(0x00);
  p2.Assemble(ms);
  d.Write(ms);

  ROS_INFO("%s: Disable GPS Stream", name.c_str());
  ros::Duration(0.5).sleep();
  
  // Request the device information
  ROS_INFO("%s: Request device info...", name.c_str());
  ImuPacket pinfo(DESCRIPTOR_BASE_COMMAND, 0x03);
  pinfo.Assemble(ms);
  d.Write(ms);

  ROS_INFO("%s: Sent Request and waiting for reply...", name.c_str());

  while (!has_device_info)
    ros::Duration(0.1).sleep();

  ROS_INFO("%s: Got device info", name.c_str());

  if (model_name.compare("3DM-GX3-35") == 0)
    {
      enable_gps = true;
    }
  else if (model_name.compare("3DM-GX3-25") == 0)
    {
      enable_gps = false;
    }
  else if (model_name.compare("3DM-GX4-25") == 0)
    {
      enable_gps = false;
      ROS_INFO("%s: Model: %s", name.c_str(), model_name.c_str());
    }
  else
    {
      ROS_INFO("%s: unknown device %s - disabling GPS",
               name.c_str(), model_name.c_str());
      enable_gps = false;
    }

  if (enable_gps)
    {
      n.param("origin/lat", origin_lat, 40.443469);
      n.param("origin/lon", origin_lon, -79.946361);
      n.param("origin/height", origin_height, 238.21);
    }

  imu_pub = n.advertise<sensor_msgs::Imu>("imu", 10, false);
  mag_pub = n.advertise<geometry_msgs::Vector3Stamped>("mag", 10, false);

  if (enable_gps)
    {
      gps_pub = n.advertise<nav_msgs::Odometry>("gps", 10, false);
      gps_fix_pub = n.advertise<imu_3dm_gx3::GPSFix>("gps_fix", 10, false);
    }

  ROS_INFO("%s: Configuring AHRS package...", name.c_str());
  // Configure the AHRS Message Format
  ImuPacket p3(DESCRIPTOR_3DM_COMMAND, 0x08);
  p3.AddFieldData(0x01); // Apply
  p3.AddFieldData(0x05); // Number of descriptors

  p3.AddFieldData(0x04); // Desc 1: Scaled Accelerometer Vector
  p3.AddFieldData(0x00); // rate_H Rate = 100 Hz, units: g
  p3.AddFieldData(0x0A);  // rate_L

  p3.AddFieldData(0x05); // Desc 2: Scaled Gyro Vector
  p3.AddFieldData(0x00); // Rate = 100 Hz, units: rad/s
  p3.AddFieldData(0x0A);

  p3.AddFieldData(0x06); // Desc 3: Scaled Magnetometer Vector
  p3.AddFieldData(0x00); // Rate = 10 Hz, units: Gauss
  p3.AddFieldData(0x64);

  p3.AddFieldData(0x0A); // Desc 4: Orientation, quaternion
  p3.AddFieldData(0x00); // Rate = 100 Hz
  p3.AddFieldData(0x0A);

  p3.AddFieldData(0x12); // Desc 5: GPS Correlation Timestamp  !==0x0E Internal Timestamp
  p3.AddFieldData(0x00); // Rate = 100 Hz, units: N/62500 (16 micro-sec) = sec
  p3.AddFieldData(0x0A);
  p3.Assemble(ms);
  d.Write(ms);

  if (enable_gps)
    {
      // Configure the GPS Message Format
      ImuPacket p4(DESCRIPTOR_3DM_COMMAND, 0x09);
      p4.AddFieldData(0x01); // Apply
      p4.AddFieldData(0x04); // Number of descriptors
      p4.AddFieldData(0x03); // Desc 1: LLH Position
      p4.AddFieldData(0x00); // Rate = 4 Hz, units: decimal degrees and meters
      p4.AddFieldData(0x01);
      p4.AddFieldData(0x05); // Desc 2: NED Velocity
      p4.AddFieldData(0x00); // Rate = 4 Hz, units: decimal degress and m/s
      p4.AddFieldData(0x01);
      p4.AddFieldData(0x0B); // Desc 3: GPS Fix Information
      p4.AddFieldData(0x00); // Rate = 4 Hz
      p4.AddFieldData(0x01);
      p4.AddFieldData(0x08); // Desc 3: UTC Time
      p4.AddFieldData(0x00); // Rate = 4 Hz
      p4.AddFieldData(0x01);
      p4.Assemble(ms);
      d.Write(ms);
    }

  ROS_INFO("%s: Enabling AHRS stream...", name.c_str());
  // Enable AHRS Stream
  ImuPacket p5(DESCRIPTOR_3DM_COMMAND, 0x11);
  p5.AddFieldData(0x01); // Apply
  p5.AddFieldData(0x01); // Device: AHRS
  p5.AddFieldData(0x01); // Enable
  p5.Assemble(ms);
  d.Write(ms);

  ROS_INFO("%s: All configuration done!", name.c_str());

  if (enable_gps)
    {
      // Enable GPS Stream
      ImuPacket p6(DESCRIPTOR_3DM_COMMAND, 0x11);
      p6.AddFieldData(0x01); // Apply
      p6.AddFieldData(0x02); // Device: GPS
      p6.AddFieldData(0x01); // Enable
      p6.Assemble(ms);
      d.Write(ms);
    }

  ros::spin();

  return EXIT_SUCCESS;
}
