#ifndef HOKUYO3D__HOKUYO3D_DRIVER_HPP_ 
#define HOKUYO3D__HOKUYO3D_DRIVER_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define HOKUYO3D_EXPORT __attribute__ ((dllexport))
    #define HOKUYO3D_IMPORT __attribute__ ((dllimport))
  #else
    #define HOKUYO3D_EXPORT __declspec(dllexport)
    #define HOKUYO3D_IMPORT __declspec(dllimport)
  #endif
  #ifdef HOKUYO3D_BUILDING_DLL
    #define HOKUYO3D_PUBLIC HOKUYO3D_EXPORT
  #else
    #define HOKUYO3D_PUBLIC HOKUYO3D_IMPORT
  #endif
  #define HOKUYO3D_PUBLIC_TYPE _PUBLIC
  #define HOKUYO3D_LOCAL
#else
  #define HOKUYO3D_EXPORT __attribute__ ((visibility("default")))
  #define HOKUYO3D_IMPORT
  #if __GNUC__ >= 4
    #define HOKUYO3D_PUBLIC __attribute__ ((visibility("default")))
    #define HOKUYO3D_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define HOKUYO3D_PUBLIC
    #define HOKUYO3D_LOCAL
  #endif
  #define HOKUYO3D_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

