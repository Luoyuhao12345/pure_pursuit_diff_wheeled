// Generated by gencpp from file my_planner/way_point.msg
// DO NOT EDIT!


#ifndef MY_PLANNER_MESSAGE_WAY_POINT_H
#define MY_PLANNER_MESSAGE_WAY_POINT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace my_planner
{
template <class ContainerAllocator>
struct way_point_
{
  typedef way_point_<ContainerAllocator> Type;

  way_point_()
    : x(0.0)
    , y(0.0)
    , yaw(0.0)
    , delta(0.0)  {
    }
  way_point_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , yaw(0.0)
    , delta(0.0)  {
  (void)_alloc;
    }



   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _yaw_type;
  _yaw_type yaw;

   typedef float _delta_type;
  _delta_type delta;





  typedef boost::shared_ptr< ::my_planner::way_point_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::my_planner::way_point_<ContainerAllocator> const> ConstPtr;

}; // struct way_point_

typedef ::my_planner::way_point_<std::allocator<void> > way_point;

typedef boost::shared_ptr< ::my_planner::way_point > way_pointPtr;
typedef boost::shared_ptr< ::my_planner::way_point const> way_pointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::my_planner::way_point_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::my_planner::way_point_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::my_planner::way_point_<ContainerAllocator1> & lhs, const ::my_planner::way_point_<ContainerAllocator2> & rhs)
{
  return lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.yaw == rhs.yaw &&
    lhs.delta == rhs.delta;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::my_planner::way_point_<ContainerAllocator1> & lhs, const ::my_planner::way_point_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace my_planner

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::my_planner::way_point_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::my_planner::way_point_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::my_planner::way_point_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::my_planner::way_point_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::my_planner::way_point_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::my_planner::way_point_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::my_planner::way_point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e8cd1dbab2b3ea429895fb8c9f90e478";
  }

  static const char* value(const ::my_planner::way_point_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe8cd1dbab2b3ea42ULL;
  static const uint64_t static_value2 = 0x9895fb8c9f90e478ULL;
};

template<class ContainerAllocator>
struct DataType< ::my_planner::way_point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "my_planner/way_point";
  }

  static const char* value(const ::my_planner::way_point_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::my_planner::way_point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 x\n"
"float32 y\n"
"float32 yaw\n"
"float32 delta\n"
;
  }

  static const char* value(const ::my_planner::way_point_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::my_planner::way_point_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.yaw);
      stream.next(m.delta);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct way_point_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::my_planner::way_point_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::my_planner::way_point_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "yaw: ";
    Printer<float>::stream(s, indent + "  ", v.yaw);
    s << indent << "delta: ";
    Printer<float>::stream(s, indent + "  ", v.delta);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MY_PLANNER_MESSAGE_WAY_POINT_H
