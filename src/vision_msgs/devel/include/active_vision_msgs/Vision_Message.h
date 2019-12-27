// Generated by gencpp from file active_vision_msgs/Vision_Message.msg
// DO NOT EDIT!


#ifndef ACTIVE_VISION_MSGS_MESSAGE_VISION_MESSAGE_H
#define ACTIVE_VISION_MSGS_MESSAGE_VISION_MESSAGE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point32.h>

namespace active_vision_msgs
{
template <class ContainerAllocator>
struct Vision_Message_
{
  typedef Vision_Message_<ContainerAllocator> Type;

  Vision_Message_()
    : Frameid()
    , Pos()
    , Found(false)  {
    }
  Vision_Message_(const ContainerAllocator& _alloc)
    : Frameid(_alloc)
    , Pos(_alloc)
    , Found(false)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _Frameid_type;
  _Frameid_type Frameid;

   typedef  ::geometry_msgs::Point32_<ContainerAllocator>  _Pos_type;
  _Pos_type Pos;

   typedef uint8_t _Found_type;
  _Found_type Found;





  typedef boost::shared_ptr< ::active_vision_msgs::Vision_Message_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::active_vision_msgs::Vision_Message_<ContainerAllocator> const> ConstPtr;

}; // struct Vision_Message_

typedef ::active_vision_msgs::Vision_Message_<std::allocator<void> > Vision_Message;

typedef boost::shared_ptr< ::active_vision_msgs::Vision_Message > Vision_MessagePtr;
typedef boost::shared_ptr< ::active_vision_msgs::Vision_Message const> Vision_MessageConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::active_vision_msgs::Vision_Message_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::active_vision_msgs::Vision_Message_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace active_vision_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'active_vision_msgs': ['/home/pourya/Documents/Active_Perception/src/vision_msgs/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::active_vision_msgs::Vision_Message_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::active_vision_msgs::Vision_Message_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::active_vision_msgs::Vision_Message_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::active_vision_msgs::Vision_Message_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::active_vision_msgs::Vision_Message_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::active_vision_msgs::Vision_Message_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::active_vision_msgs::Vision_Message_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9e12f5951169920222965062c93aead0";
  }

  static const char* value(const ::active_vision_msgs::Vision_Message_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9e12f59511699202ULL;
  static const uint64_t static_value2 = 0x22965062c93aead0ULL;
};

template<class ContainerAllocator>
struct DataType< ::active_vision_msgs::Vision_Message_<ContainerAllocator> >
{
  static const char* value()
  {
    return "active_vision_msgs/Vision_Message";
  }

  static const char* value(const ::active_vision_msgs::Vision_Message_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::active_vision_msgs::Vision_Message_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string Frameid\n\
geometry_msgs/Point32 Pos\n\
bool Found\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point32\n\
# This contains the position of a point in free space(with 32 bits of precision).\n\
# It is recommeded to use Point wherever possible instead of Point32.  \n\
# \n\
# This recommendation is to promote interoperability.  \n\
#\n\
# This message is designed to take up less space when sending\n\
# lots of points at once, as in the case of a PointCloud.  \n\
\n\
float32 x\n\
float32 y\n\
float32 z\n\
";
  }

  static const char* value(const ::active_vision_msgs::Vision_Message_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::active_vision_msgs::Vision_Message_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.Frameid);
      stream.next(m.Pos);
      stream.next(m.Found);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Vision_Message_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::active_vision_msgs::Vision_Message_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::active_vision_msgs::Vision_Message_<ContainerAllocator>& v)
  {
    s << indent << "Frameid: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.Frameid);
    s << indent << "Pos: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point32_<ContainerAllocator> >::stream(s, indent + "  ", v.Pos);
    s << indent << "Found: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.Found);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ACTIVE_VISION_MSGS_MESSAGE_VISION_MESSAGE_H