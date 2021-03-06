// Generated by gencpp from file cruiser_msgs/cruiserLedOnOff.msg
// DO NOT EDIT!


#ifndef CRUISER_MSGS_MESSAGE_CRUISERLEDONOFF_H
#define CRUISER_MSGS_MESSAGE_CRUISERLEDONOFF_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace cruiser_msgs
{
template <class ContainerAllocator>
struct cruiserLedOnOff_
{
  typedef cruiserLedOnOff_<ContainerAllocator> Type;

  cruiserLedOnOff_()
    : onOff(0)  {
    }
  cruiserLedOnOff_(const ContainerAllocator& _alloc)
    : onOff(0)  {
  (void)_alloc;
    }



   typedef uint32_t _onOff_type;
  _onOff_type onOff;





  typedef boost::shared_ptr< ::cruiser_msgs::cruiserLedOnOff_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cruiser_msgs::cruiserLedOnOff_<ContainerAllocator> const> ConstPtr;

}; // struct cruiserLedOnOff_

typedef ::cruiser_msgs::cruiserLedOnOff_<std::allocator<void> > cruiserLedOnOff;

typedef boost::shared_ptr< ::cruiser_msgs::cruiserLedOnOff > cruiserLedOnOffPtr;
typedef boost::shared_ptr< ::cruiser_msgs::cruiserLedOnOff const> cruiserLedOnOffConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cruiser_msgs::cruiserLedOnOff_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cruiser_msgs::cruiserLedOnOff_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cruiser_msgs::cruiserLedOnOff_<ContainerAllocator1> & lhs, const ::cruiser_msgs::cruiserLedOnOff_<ContainerAllocator2> & rhs)
{
  return lhs.onOff == rhs.onOff;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cruiser_msgs::cruiserLedOnOff_<ContainerAllocator1> & lhs, const ::cruiser_msgs::cruiserLedOnOff_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cruiser_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::cruiser_msgs::cruiserLedOnOff_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cruiser_msgs::cruiserLedOnOff_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cruiser_msgs::cruiserLedOnOff_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cruiser_msgs::cruiserLedOnOff_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cruiser_msgs::cruiserLedOnOff_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cruiser_msgs::cruiserLedOnOff_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cruiser_msgs::cruiserLedOnOff_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5542655f2c2b872f0777fe5564fc094c";
  }

  static const char* value(const ::cruiser_msgs::cruiserLedOnOff_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5542655f2c2b872fULL;
  static const uint64_t static_value2 = 0x0777fe5564fc094cULL;
};

template<class ContainerAllocator>
struct DataType< ::cruiser_msgs::cruiserLedOnOff_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cruiser_msgs/cruiserLedOnOff";
  }

  static const char* value(const ::cruiser_msgs::cruiserLedOnOff_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cruiser_msgs::cruiserLedOnOff_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint32 onOff\n"
"\n"
;
  }

  static const char* value(const ::cruiser_msgs::cruiserLedOnOff_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cruiser_msgs::cruiserLedOnOff_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.onOff);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct cruiserLedOnOff_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cruiser_msgs::cruiserLedOnOff_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cruiser_msgs::cruiserLedOnOff_<ContainerAllocator>& v)
  {
    s << indent << "onOff: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.onOff);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CRUISER_MSGS_MESSAGE_CRUISERLEDONOFF_H
