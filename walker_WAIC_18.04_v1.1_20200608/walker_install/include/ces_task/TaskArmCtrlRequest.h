// Generated by gencpp from file ces_task/TaskArmCtrlRequest.msg
// DO NOT EDIT!


#ifndef CES_TASK_MESSAGE_TASKARMCTRLREQUEST_H
#define CES_TASK_MESSAGE_TASKARMCTRLREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ces_task
{
template <class ContainerAllocator>
struct TaskArmCtrlRequest_
{
  typedef TaskArmCtrlRequest_<ContainerAllocator> Type;

  TaskArmCtrlRequest_()
    : task_id()
    , useJointOTG()
    , useCartOTG()
    , securityDection()
    , collisionDetection()
    , demander()
    , executor()
    , cmd()  {
    }
  TaskArmCtrlRequest_(const ContainerAllocator& _alloc)
    : task_id(_alloc)
    , useJointOTG(_alloc)
    , useCartOTG(_alloc)
    , securityDection(_alloc)
    , collisionDetection(_alloc)
    , demander(_alloc)
    , executor(_alloc)
    , cmd(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _task_id_type;
  _task_id_type task_id;

   typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  _useJointOTG_type;
  _useJointOTG_type useJointOTG;

   typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  _useCartOTG_type;
  _useCartOTG_type useCartOTG;

   typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  _securityDection_type;
  _securityDection_type securityDection;

   typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  _collisionDetection_type;
  _collisionDetection_type collisionDetection;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _demander_type;
  _demander_type demander;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _executor_type;
  _executor_type executor;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _cmd_type;
  _cmd_type cmd;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(CMD_START)
  #undef CMD_START
#endif
#if defined(_WIN32) && defined(CMD_STOP)
  #undef CMD_STOP
#endif


  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  CMD_START;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  CMD_STOP;

  typedef boost::shared_ptr< ::ces_task::TaskArmCtrlRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ces_task::TaskArmCtrlRequest_<ContainerAllocator> const> ConstPtr;

}; // struct TaskArmCtrlRequest_

typedef ::ces_task::TaskArmCtrlRequest_<std::allocator<void> > TaskArmCtrlRequest;

typedef boost::shared_ptr< ::ces_task::TaskArmCtrlRequest > TaskArmCtrlRequestPtr;
typedef boost::shared_ptr< ::ces_task::TaskArmCtrlRequest const> TaskArmCtrlRequestConstPtr;

// constants requiring out of line definition

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmCtrlRequest_<ContainerAllocator>::CMD_START =
        
          "start"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmCtrlRequest_<ContainerAllocator>::CMD_STOP =
        
          "stop"
        
        ;
   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ces_task::TaskArmCtrlRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ces_task::TaskArmCtrlRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ces_task::TaskArmCtrlRequest_<ContainerAllocator1> & lhs, const ::ces_task::TaskArmCtrlRequest_<ContainerAllocator2> & rhs)
{
  return lhs.task_id == rhs.task_id &&
    lhs.useJointOTG == rhs.useJointOTG &&
    lhs.useCartOTG == rhs.useCartOTG &&
    lhs.securityDection == rhs.securityDection &&
    lhs.collisionDetection == rhs.collisionDetection &&
    lhs.demander == rhs.demander &&
    lhs.executor == rhs.executor &&
    lhs.cmd == rhs.cmd;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ces_task::TaskArmCtrlRequest_<ContainerAllocator1> & lhs, const ::ces_task::TaskArmCtrlRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ces_task

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::ces_task::TaskArmCtrlRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ces_task::TaskArmCtrlRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ces_task::TaskArmCtrlRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ces_task::TaskArmCtrlRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ces_task::TaskArmCtrlRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ces_task::TaskArmCtrlRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ces_task::TaskArmCtrlRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "eb2e25d94af76f3480b1d4d23ddf0c13";
  }

  static const char* value(const ::ces_task::TaskArmCtrlRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xeb2e25d94af76f34ULL;
  static const uint64_t static_value2 = 0x80b1d4d23ddf0c13ULL;
};

template<class ContainerAllocator>
struct DataType< ::ces_task::TaskArmCtrlRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ces_task/TaskArmCtrlRequest";
  }

  static const char* value(const ::ces_task::TaskArmCtrlRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ces_task::TaskArmCtrlRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"string task_id\n"
"\n"
"\n"
"bool[] useJointOTG\n"
"bool[] useCartOTG\n"
"bool[] securityDection\n"
"bool[] collisionDetection\n"
"\n"
"\n"
"\n"
"string demander\n"
"\n"
"\n"
"string executor\n"
"\n"
"\n"
"string CMD_START=start\n"
"string CMD_STOP=stop\n"
"string cmd\n"
"\n"
;
  }

  static const char* value(const ::ces_task::TaskArmCtrlRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ces_task::TaskArmCtrlRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.task_id);
      stream.next(m.useJointOTG);
      stream.next(m.useCartOTG);
      stream.next(m.securityDection);
      stream.next(m.collisionDetection);
      stream.next(m.demander);
      stream.next(m.executor);
      stream.next(m.cmd);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TaskArmCtrlRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ces_task::TaskArmCtrlRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ces_task::TaskArmCtrlRequest_<ContainerAllocator>& v)
  {
    s << indent << "task_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.task_id);
    s << indent << "useJointOTG[]" << std::endl;
    for (size_t i = 0; i < v.useJointOTG.size(); ++i)
    {
      s << indent << "  useJointOTG[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.useJointOTG[i]);
    }
    s << indent << "useCartOTG[]" << std::endl;
    for (size_t i = 0; i < v.useCartOTG.size(); ++i)
    {
      s << indent << "  useCartOTG[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.useCartOTG[i]);
    }
    s << indent << "securityDection[]" << std::endl;
    for (size_t i = 0; i < v.securityDection.size(); ++i)
    {
      s << indent << "  securityDection[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.securityDection[i]);
    }
    s << indent << "collisionDetection[]" << std::endl;
    for (size_t i = 0; i < v.collisionDetection.size(); ++i)
    {
      s << indent << "  collisionDetection[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.collisionDetection[i]);
    }
    s << indent << "demander: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.demander);
    s << indent << "executor: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.executor);
    s << indent << "cmd: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.cmd);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CES_TASK_MESSAGE_TASKARMCTRLREQUEST_H