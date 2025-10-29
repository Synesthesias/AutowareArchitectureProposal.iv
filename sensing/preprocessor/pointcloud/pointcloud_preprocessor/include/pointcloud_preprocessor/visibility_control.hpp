#pragma once

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define POINTCLOUD_PREPROCESSOR_EXPORT __attribute__ ((dllexport))
    #define POINTCLOUD_PREPROCESSOR_IMPORT __attribute__ ((dllimport))
  #else
    #define POINTCLOUD_PREPROCESSOR_EXPORT __declspec(dllexport)
    #define POINTCLOUD_PREPROCESSOR_IMPORT __declspec(dllimport)
  #endif
  #ifdef POINTCLOUD_PREPROCESSOR_BUILDING_LIBRARY
    #define POINTCLOUD_PREPROCESSOR_PUBLIC POINTCLOUD_PREPROCESSOR_EXPORT
  #else
    #define POINTCLOUD_PREPROCESSOR_PUBLIC POINTCLOUD_PREPROCESSOR_IMPORT
  #endif
  #define POINTCLOUD_PREPROCESSOR_PUBLIC_TYPE POINTCLOUD_PREPROCESSOR_PUBLIC
  #define POINTCLOUD_PREPROCESSOR_LOCAL
#else
  #define POINTCLOUD_PREPROCESSOR_EXPORT __attribute__ ((visibility("default")))
  #define POINTCLOUD_PREPROCESSOR_IMPORT
  #if __GNUC__ >= 4
    #define POINTCLOUD_PREPROCESSOR_PUBLIC __attribute__ ((visibility("default")))
    #define POINTCLOUD_PREPROCESSOR_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define POINTCLOUD_PREPROCESSOR_PUBLIC
    #define POINTCLOUD_PREPROCESSOR_LOCAL
  #endif
  #define POINTCLOUD_PREPROCESSOR_PUBLIC_TYPE
#endif
