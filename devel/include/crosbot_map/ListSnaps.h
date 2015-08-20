// Generated by gencpp from file crosbot_map/ListSnaps.msg
// DO NOT EDIT!


#ifndef CROSBOT_MAP_MESSAGE_LISTSNAPS_H
#define CROSBOT_MAP_MESSAGE_LISTSNAPS_H

#include <ros/service_traits.h>


#include <crosbot_map/ListSnapsRequest.h>
#include <crosbot_map/ListSnapsResponse.h>


namespace crosbot_map
{

struct ListSnaps
{

typedef ListSnapsRequest Request;
typedef ListSnapsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ListSnaps
} // namespace crosbot_map


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::crosbot_map::ListSnaps > {
  static const char* value()
  {
    return "a9950552cda46ef9fb7f384a56cf8cd8";
  }

  static const char* value(const ::crosbot_map::ListSnaps&) { return value(); }
};

template<>
struct DataType< ::crosbot_map::ListSnaps > {
  static const char* value()
  {
    return "crosbot_map/ListSnaps";
  }

  static const char* value(const ::crosbot_map::ListSnaps&) { return value(); }
};


// service_traits::MD5Sum< ::crosbot_map::ListSnapsRequest> should match 
// service_traits::MD5Sum< ::crosbot_map::ListSnaps > 
template<>
struct MD5Sum< ::crosbot_map::ListSnapsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::crosbot_map::ListSnaps >::value();
  }
  static const char* value(const ::crosbot_map::ListSnapsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::crosbot_map::ListSnapsRequest> should match 
// service_traits::DataType< ::crosbot_map::ListSnaps > 
template<>
struct DataType< ::crosbot_map::ListSnapsRequest>
{
  static const char* value()
  {
    return DataType< ::crosbot_map::ListSnaps >::value();
  }
  static const char* value(const ::crosbot_map::ListSnapsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::crosbot_map::ListSnapsResponse> should match 
// service_traits::MD5Sum< ::crosbot_map::ListSnaps > 
template<>
struct MD5Sum< ::crosbot_map::ListSnapsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::crosbot_map::ListSnaps >::value();
  }
  static const char* value(const ::crosbot_map::ListSnapsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::crosbot_map::ListSnapsResponse> should match 
// service_traits::DataType< ::crosbot_map::ListSnaps > 
template<>
struct DataType< ::crosbot_map::ListSnapsResponse>
{
  static const char* value()
  {
    return DataType< ::crosbot_map::ListSnaps >::value();
  }
  static const char* value(const ::crosbot_map::ListSnapsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // CROSBOT_MAP_MESSAGE_LISTSNAPS_H
