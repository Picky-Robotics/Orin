// Generated by gencpp from file mycobot_communication/MycobotSetAngles.msg
// DO NOT EDIT!


#ifndef MYCOBOT_COMMUNICATION_MESSAGE_MYCOBOTSETANGLES_H
#define MYCOBOT_COMMUNICATION_MESSAGE_MYCOBOTSETANGLES_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace mycobot_communication
{
template <class ContainerAllocator>
struct MycobotSetAngles_
{
  typedef MycobotSetAngles_<ContainerAllocator> Type;

  MycobotSetAngles_()
    : joint_1(0.0)
    , joint_2(0.0)
    , joint_3(0.0)
    , joint_4(0.0)
    , joint_5(0.0)
    , joint_6(0.0)
    , speed(0)  {
    }
  MycobotSetAngles_(const ContainerAllocator& _alloc)
    : joint_1(0.0)
    , joint_2(0.0)
    , joint_3(0.0)
    , joint_4(0.0)
    , joint_5(0.0)
    , joint_6(0.0)
    , speed(0)  {
  (void)_alloc;
    }



   typedef float _joint_1_type;
  _joint_1_type joint_1;

   typedef float _joint_2_type;
  _joint_2_type joint_2;

   typedef float _joint_3_type;
  _joint_3_type joint_3;

   typedef float _joint_4_type;
  _joint_4_type joint_4;

   typedef float _joint_5_type;
  _joint_5_type joint_5;

   typedef float _joint_6_type;
  _joint_6_type joint_6;

   typedef int8_t _speed_type;
  _speed_type speed;





  typedef boost::shared_ptr< ::mycobot_communication::MycobotSetAngles_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mycobot_communication::MycobotSetAngles_<ContainerAllocator> const> ConstPtr;

}; // struct MycobotSetAngles_

typedef ::mycobot_communication::MycobotSetAngles_<std::allocator<void> > MycobotSetAngles;

typedef boost::shared_ptr< ::mycobot_communication::MycobotSetAngles > MycobotSetAnglesPtr;
typedef boost::shared_ptr< ::mycobot_communication::MycobotSetAngles const> MycobotSetAnglesConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mycobot_communication::MycobotSetAngles_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mycobot_communication::MycobotSetAngles_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mycobot_communication::MycobotSetAngles_<ContainerAllocator1> & lhs, const ::mycobot_communication::MycobotSetAngles_<ContainerAllocator2> & rhs)
{
  return lhs.joint_1 == rhs.joint_1 &&
    lhs.joint_2 == rhs.joint_2 &&
    lhs.joint_3 == rhs.joint_3 &&
    lhs.joint_4 == rhs.joint_4 &&
    lhs.joint_5 == rhs.joint_5 &&
    lhs.joint_6 == rhs.joint_6 &&
    lhs.speed == rhs.speed;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mycobot_communication::MycobotSetAngles_<ContainerAllocator1> & lhs, const ::mycobot_communication::MycobotSetAngles_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mycobot_communication

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::mycobot_communication::MycobotSetAngles_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mycobot_communication::MycobotSetAngles_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mycobot_communication::MycobotSetAngles_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mycobot_communication::MycobotSetAngles_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mycobot_communication::MycobotSetAngles_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mycobot_communication::MycobotSetAngles_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mycobot_communication::MycobotSetAngles_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d77601489610418de50f0fa3e3e88a65";
  }

  static const char* value(const ::mycobot_communication::MycobotSetAngles_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd77601489610418dULL;
  static const uint64_t static_value2 = 0xe50f0fa3e3e88a65ULL;
};

template<class ContainerAllocator>
struct DataType< ::mycobot_communication::MycobotSetAngles_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mycobot_communication/MycobotSetAngles";
  }

  static const char* value(const ::mycobot_communication::MycobotSetAngles_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mycobot_communication::MycobotSetAngles_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 joint_1\n"
"float32 joint_2\n"
"float32 joint_3\n"
"float32 joint_4\n"
"float32 joint_5\n"
"float32 joint_6\n"
"\n"
"int8 speed\n"
;
  }

  static const char* value(const ::mycobot_communication::MycobotSetAngles_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mycobot_communication::MycobotSetAngles_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.joint_1);
      stream.next(m.joint_2);
      stream.next(m.joint_3);
      stream.next(m.joint_4);
      stream.next(m.joint_5);
      stream.next(m.joint_6);
      stream.next(m.speed);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MycobotSetAngles_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mycobot_communication::MycobotSetAngles_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mycobot_communication::MycobotSetAngles_<ContainerAllocator>& v)
  {
    s << indent << "joint_1: ";
    Printer<float>::stream(s, indent + "  ", v.joint_1);
    s << indent << "joint_2: ";
    Printer<float>::stream(s, indent + "  ", v.joint_2);
    s << indent << "joint_3: ";
    Printer<float>::stream(s, indent + "  ", v.joint_3);
    s << indent << "joint_4: ";
    Printer<float>::stream(s, indent + "  ", v.joint_4);
    s << indent << "joint_5: ";
    Printer<float>::stream(s, indent + "  ", v.joint_5);
    s << indent << "joint_6: ";
    Printer<float>::stream(s, indent + "  ", v.joint_6);
    s << indent << "speed: ";
    Printer<int8_t>::stream(s, indent + "  ", v.speed);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MYCOBOT_COMMUNICATION_MESSAGE_MYCOBOTSETANGLES_H
