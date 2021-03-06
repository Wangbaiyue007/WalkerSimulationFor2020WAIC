// Generated by gencpp from file cruiser_msgs/JointCtrl.msg
// DO NOT EDIT!


#ifndef CRUISER_MSGS_MESSAGE_JOINTCTRL_H
#define CRUISER_MSGS_MESSAGE_JOINTCTRL_H


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
struct JointCtrl_
{
  typedef JointCtrl_<ContainerAllocator> Type;

  JointCtrl_()
    : ctrlId()
    , ctrlStamp()
    , ctrlName()
    , jointIdx(0)
    , reserved()  {
    }
  JointCtrl_(const ContainerAllocator& _alloc)
    : ctrlId(_alloc)
    , ctrlStamp()
    , ctrlName(_alloc)
    , jointIdx(0)
    , reserved(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _ctrlId_type;
  _ctrlId_type ctrlId;

   typedef ros::Time _ctrlStamp_type;
  _ctrlStamp_type ctrlStamp;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _ctrlName_type;
  _ctrlName_type ctrlName;

   typedef int32_t _jointIdx_type;
  _jointIdx_type jointIdx;

   typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  _reserved_type;
  _reserved_type reserved;





  typedef boost::shared_ptr< ::cruiser_msgs::JointCtrl_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cruiser_msgs::JointCtrl_<ContainerAllocator> const> ConstPtr;

}; // struct JointCtrl_

typedef ::cruiser_msgs::JointCtrl_<std::allocator<void> > JointCtrl;

typedef boost::shared_ptr< ::cruiser_msgs::JointCtrl > JointCtrlPtr;
typedef boost::shared_ptr< ::cruiser_msgs::JointCtrl const> JointCtrlConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cruiser_msgs::JointCtrl_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cruiser_msgs::JointCtrl_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cruiser_msgs::JointCtrl_<ContainerAllocator1> & lhs, const ::cruiser_msgs::JointCtrl_<ContainerAllocator2> & rhs)
{
  return lhs.ctrlId == rhs.ctrlId &&
    lhs.ctrlStamp == rhs.ctrlStamp &&
    lhs.ctrlName == rhs.ctrlName &&
    lhs.jointIdx == rhs.jointIdx &&
    lhs.reserved == rhs.reserved;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cruiser_msgs::JointCtrl_<ContainerAllocator1> & lhs, const ::cruiser_msgs::JointCtrl_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cruiser_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::cruiser_msgs::JointCtrl_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cruiser_msgs::JointCtrl_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cruiser_msgs::JointCtrl_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cruiser_msgs::JointCtrl_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cruiser_msgs::JointCtrl_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cruiser_msgs::JointCtrl_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cruiser_msgs::JointCtrl_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8223324199b2c8c391007fad926392ef";
  }

  static const char* value(const ::cruiser_msgs::JointCtrl_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8223324199b2c8c3ULL;
  static const uint64_t static_value2 = 0x91007fad926392efULL;
};

template<class ContainerAllocator>
struct DataType< ::cruiser_msgs::JointCtrl_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cruiser_msgs/JointCtrl";
  }

  static const char* value(const ::cruiser_msgs::JointCtrl_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cruiser_msgs::JointCtrl_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string ctrlId\n"
"time ctrlStamp\n"
"string ctrlName\n"
"int32 jointIdx\n"
"uint8[] reserved\n"
;
  }

  static const char* value(const ::cruiser_msgs::JointCtrl_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cruiser_msgs::JointCtrl_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.ctrlId);
      stream.next(m.ctrlStamp);
      stream.next(m.ctrlName);
      stream.next(m.jointIdx);
      stream.next(m.reserved);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct JointCtrl_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cruiser_msgs::JointCtrl_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cruiser_msgs::JointCtrl_<ContainerAllocator>& v)
  {
    s << indent << "ctrlId: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.ctrlId);
    s << indent << "ctrlStamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.ctrlStamp);
    s << indent << "ctrlName: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.ctrlName);
    s << indent << "jointIdx: ";
    Printer<int32_t>::stream(s, indent + "  ", v.jointIdx);
    s << indent << "reserved[]" << std::endl;
    for (size_t i = 0; i < v.reserved.size(); ++i)
    {
      s << indent << "  reserved[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.reserved[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // CRUISER_MSGS_MESSAGE_JOINTCTRL_H
