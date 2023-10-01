// Generated by gencpp from file mecharm_communication/GripperStatusRequest.msg
// DO NOT EDIT!


#ifndef MECHARM_COMMUNICATION_MESSAGE_GRIPPERSTATUSREQUEST_H
#define MECHARM_COMMUNICATION_MESSAGE_GRIPPERSTATUSREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace mecharm_communication
{
template <class ContainerAllocator>
struct GripperStatusRequest_
{
  typedef GripperStatusRequest_<ContainerAllocator> Type;

  GripperStatusRequest_()
    : Status(false)  {
    }
  GripperStatusRequest_(const ContainerAllocator& _alloc)
    : Status(false)  {
  (void)_alloc;
    }



   typedef uint8_t _Status_type;
  _Status_type Status;





  typedef boost::shared_ptr< ::mecharm_communication::GripperStatusRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mecharm_communication::GripperStatusRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GripperStatusRequest_

typedef ::mecharm_communication::GripperStatusRequest_<std::allocator<void> > GripperStatusRequest;

typedef boost::shared_ptr< ::mecharm_communication::GripperStatusRequest > GripperStatusRequestPtr;
typedef boost::shared_ptr< ::mecharm_communication::GripperStatusRequest const> GripperStatusRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mecharm_communication::GripperStatusRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mecharm_communication::GripperStatusRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mecharm_communication::GripperStatusRequest_<ContainerAllocator1> & lhs, const ::mecharm_communication::GripperStatusRequest_<ContainerAllocator2> & rhs)
{
  return lhs.Status == rhs.Status;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mecharm_communication::GripperStatusRequest_<ContainerAllocator1> & lhs, const ::mecharm_communication::GripperStatusRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mecharm_communication

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::mecharm_communication::GripperStatusRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mecharm_communication::GripperStatusRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mecharm_communication::GripperStatusRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mecharm_communication::GripperStatusRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mecharm_communication::GripperStatusRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mecharm_communication::GripperStatusRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mecharm_communication::GripperStatusRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "513e93c68ef2f26ff494445b932bb052";
  }

  static const char* value(const ::mecharm_communication::GripperStatusRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x513e93c68ef2f26fULL;
  static const uint64_t static_value2 = 0xf494445b932bb052ULL;
};

template<class ContainerAllocator>
struct DataType< ::mecharm_communication::GripperStatusRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mecharm_communication/GripperStatusRequest";
  }

  static const char* value(const ::mecharm_communication::GripperStatusRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mecharm_communication::GripperStatusRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool Status\n"
"\n"
;
  }

  static const char* value(const ::mecharm_communication::GripperStatusRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mecharm_communication::GripperStatusRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.Status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GripperStatusRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mecharm_communication::GripperStatusRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mecharm_communication::GripperStatusRequest_<ContainerAllocator>& v)
  {
    s << indent << "Status: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.Status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MECHARM_COMMUNICATION_MESSAGE_GRIPPERSTATUSREQUEST_H
