// Generated by gencpp from file pub_imu/Daoyuan.msg
// DO NOT EDIT!


#ifndef PUB_IMU_MESSAGE_DAOYUAN_H
#define PUB_IMU_MESSAGE_DAOYUAN_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace pub_imu
{
template <class ContainerAllocator>
struct Daoyuan_
{
  typedef Daoyuan_<ContainerAllocator> Type;

  Daoyuan_()
    : header()
    , roll(0)
    , pitch(0)
    , yaw(0)
    , wx(0)
    , wy(0)
    , wz(0)
    , ax(0)
    , ay(0)
    , az(0)
    , lat(0)
    , lon(0)
    , hei(0)
    , vy(0)
    , vx(0)
    , vz(0)
    , status(0)
    , data1(0)
    , data2(0)
    , data3(0)
    , imu_time(0)
    , type(0)  {
    }
  Daoyuan_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , roll(0)
    , pitch(0)
    , yaw(0)
    , wx(0)
    , wy(0)
    , wz(0)
    , ax(0)
    , ay(0)
    , az(0)
    , lat(0)
    , lon(0)
    , hei(0)
    , vy(0)
    , vx(0)
    , vz(0)
    , status(0)
    , data1(0)
    , data2(0)
    , data3(0)
    , imu_time(0)
    , type(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef int16_t _roll_type;
  _roll_type roll;

   typedef int16_t _pitch_type;
  _pitch_type pitch;

   typedef int16_t _yaw_type;
  _yaw_type yaw;

   typedef int16_t _wx_type;
  _wx_type wx;

   typedef int16_t _wy_type;
  _wy_type wy;

   typedef int16_t _wz_type;
  _wz_type wz;

   typedef int16_t _ax_type;
  _ax_type ax;

   typedef int16_t _ay_type;
  _ay_type ay;

   typedef int16_t _az_type;
  _az_type az;

   typedef int32_t _lat_type;
  _lat_type lat;

   typedef int32_t _lon_type;
  _lon_type lon;

   typedef int32_t _hei_type;
  _hei_type hei;

   typedef int16_t _vy_type;
  _vy_type vy;

   typedef int16_t _vx_type;
  _vx_type vx;

   typedef int16_t _vz_type;
  _vz_type vz;

   typedef uint8_t _status_type;
  _status_type status;

   typedef int16_t _data1_type;
  _data1_type data1;

   typedef int16_t _data2_type;
  _data2_type data2;

   typedef int16_t _data3_type;
  _data3_type data3;

   typedef uint32_t _imu_time_type;
  _imu_time_type imu_time;

   typedef uint8_t _type_type;
  _type_type type;





  typedef boost::shared_ptr< ::pub_imu::Daoyuan_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pub_imu::Daoyuan_<ContainerAllocator> const> ConstPtr;

}; // struct Daoyuan_

typedef ::pub_imu::Daoyuan_<std::allocator<void> > Daoyuan;

typedef boost::shared_ptr< ::pub_imu::Daoyuan > DaoyuanPtr;
typedef boost::shared_ptr< ::pub_imu::Daoyuan const> DaoyuanConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pub_imu::Daoyuan_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pub_imu::Daoyuan_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pub_imu

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'pub_imu': ['/home/iairiv/Documents/velodyne_ws/src/pub_imu/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pub_imu::Daoyuan_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pub_imu::Daoyuan_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pub_imu::Daoyuan_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pub_imu::Daoyuan_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pub_imu::Daoyuan_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pub_imu::Daoyuan_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pub_imu::Daoyuan_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0790d32890eb3a2c7f329bf5c7d57719";
  }

  static const char* value(const ::pub_imu::Daoyuan_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0790d32890eb3a2cULL;
  static const uint64_t static_value2 = 0x7f329bf5c7d57719ULL;
};

template<class ContainerAllocator>
struct DataType< ::pub_imu::Daoyuan_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pub_imu/Daoyuan";
  }

  static const char* value(const ::pub_imu::Daoyuan_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pub_imu::Daoyuan_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
int16 roll\n\
int16 pitch\n\
int16 yaw\n\
int16 wx \n\
int16 wy\n\
int16 wz\n\
int16 ax\n\
int16 ay\n\
int16 az\n\
int32 lat\n\
int32 lon\n\
int32 hei\n\
int16 vy\n\
int16 vx\n\
int16 vz\n\
uint8 status\n\
int16 data1\n\
int16 data2\n\
int16 data3\n\
uint32 imu_time\n\
uint8 type\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::pub_imu::Daoyuan_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pub_imu::Daoyuan_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.roll);
      stream.next(m.pitch);
      stream.next(m.yaw);
      stream.next(m.wx);
      stream.next(m.wy);
      stream.next(m.wz);
      stream.next(m.ax);
      stream.next(m.ay);
      stream.next(m.az);
      stream.next(m.lat);
      stream.next(m.lon);
      stream.next(m.hei);
      stream.next(m.vy);
      stream.next(m.vx);
      stream.next(m.vz);
      stream.next(m.status);
      stream.next(m.data1);
      stream.next(m.data2);
      stream.next(m.data3);
      stream.next(m.imu_time);
      stream.next(m.type);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Daoyuan_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pub_imu::Daoyuan_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pub_imu::Daoyuan_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "roll: ";
    Printer<int16_t>::stream(s, indent + "  ", v.roll);
    s << indent << "pitch: ";
    Printer<int16_t>::stream(s, indent + "  ", v.pitch);
    s << indent << "yaw: ";
    Printer<int16_t>::stream(s, indent + "  ", v.yaw);
    s << indent << "wx: ";
    Printer<int16_t>::stream(s, indent + "  ", v.wx);
    s << indent << "wy: ";
    Printer<int16_t>::stream(s, indent + "  ", v.wy);
    s << indent << "wz: ";
    Printer<int16_t>::stream(s, indent + "  ", v.wz);
    s << indent << "ax: ";
    Printer<int16_t>::stream(s, indent + "  ", v.ax);
    s << indent << "ay: ";
    Printer<int16_t>::stream(s, indent + "  ", v.ay);
    s << indent << "az: ";
    Printer<int16_t>::stream(s, indent + "  ", v.az);
    s << indent << "lat: ";
    Printer<int32_t>::stream(s, indent + "  ", v.lat);
    s << indent << "lon: ";
    Printer<int32_t>::stream(s, indent + "  ", v.lon);
    s << indent << "hei: ";
    Printer<int32_t>::stream(s, indent + "  ", v.hei);
    s << indent << "vy: ";
    Printer<int16_t>::stream(s, indent + "  ", v.vy);
    s << indent << "vx: ";
    Printer<int16_t>::stream(s, indent + "  ", v.vx);
    s << indent << "vz: ";
    Printer<int16_t>::stream(s, indent + "  ", v.vz);
    s << indent << "status: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.status);
    s << indent << "data1: ";
    Printer<int16_t>::stream(s, indent + "  ", v.data1);
    s << indent << "data2: ";
    Printer<int16_t>::stream(s, indent + "  ", v.data2);
    s << indent << "data3: ";
    Printer<int16_t>::stream(s, indent + "  ", v.data3);
    s << indent << "imu_time: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.imu_time);
    s << indent << "type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.type);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PUB_IMU_MESSAGE_DAOYUAN_H