// Generated by gencpp from file crosbot_explore/FollowPath.msg
// DO NOT EDIT!


#ifndef CROSBOT_EXPLORE_MESSAGE_FOLLOWPATH_H
#define CROSBOT_EXPLORE_MESSAGE_FOLLOWPATH_H

#include <ros/service_traits.h>


#include <crosbot_explore/FollowPathRequest.h>
#include <crosbot_explore/FollowPathResponse.h>


namespace crosbot_explore
{

struct FollowPath
{

typedef FollowPathRequest Request;
typedef FollowPathResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct FollowPath
} // namespace crosbot_explore


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::crosbot_explore::FollowPath > {
  static const char* value()
  {
    return "58d6f138c7de7ef47c75d4b7e5df5472";
  }

  static const char* value(const ::crosbot_explore::FollowPath&) { return value(); }
};

template<>
struct DataType< ::crosbot_explore::FollowPath > {
  static const char* value()
  {
    return "crosbot_explore/FollowPath";
  }

  static const char* value(const ::crosbot_explore::FollowPath&) { return value(); }
};


// service_traits::MD5Sum< ::crosbot_explore::FollowPathRequest> should match 
// service_traits::MD5Sum< ::crosbot_explore::FollowPath > 
template<>
struct MD5Sum< ::crosbot_explore::FollowPathRequest>
{
  static const char* value()
  {
    return MD5Sum< ::crosbot_explore::FollowPath >::value();
  }
  static const char* value(const ::crosbot_explore::FollowPathRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::crosbot_explore::FollowPathRequest> should match 
// service_traits::DataType< ::crosbot_explore::FollowPath > 
template<>
struct DataType< ::crosbot_explore::FollowPathRequest>
{
  static const char* value()
  {
    return DataType< ::crosbot_explore::FollowPath >::value();
  }
  static const char* value(const ::crosbot_explore::FollowPathRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::crosbot_explore::FollowPathResponse> should match 
// service_traits::MD5Sum< ::crosbot_explore::FollowPath > 
template<>
struct MD5Sum< ::crosbot_explore::FollowPathResponse>
{
  static const char* value()
  {
    return MD5Sum< ::crosbot_explore::FollowPath >::value();
  }
  static const char* value(const ::crosbot_explore::FollowPathResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::crosbot_explore::FollowPathResponse> should match 
// service_traits::DataType< ::crosbot_explore::FollowPath > 
template<>
struct DataType< ::crosbot_explore::FollowPathResponse>
{
  static const char* value()
  {
    return DataType< ::crosbot_explore::FollowPath >::value();
  }
  static const char* value(const ::crosbot_explore::FollowPathResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // CROSBOT_EXPLORE_MESSAGE_FOLLOWPATH_H
