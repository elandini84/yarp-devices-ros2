#ifndef CB_HW_INTERFACE__VISIBILITY_CONTROL_H_
#define CB_HW_INTERFACE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CB_HW_INTERFACE_EXPORT __attribute__ ((dllexport))
    #define CB_HW_INTERFACE_IMPORT __attribute__ ((dllimport))
  #else
    #define CB_HW_INTERFACE_EXPORT __declspec(dllexport)
    #define CB_HW_INTERFACE_IMPORT __declspec(dllimport)
  #endif
  #ifdef CB_HW_INTERFACE_BUILDING_LIBRARY
    #define CB_HW_INTERFACE_PUBLIC CB_HW_INTERFACE_EXPORT
  #else
    #define CB_HW_INTERFACE_PUBLIC CB_HW_INTERFACE_IMPORT
  #endif
  #define CB_HW_INTERFACE_PUBLIC_TYPE CB_HW_INTERFACE_PUBLIC
  #define CB_HW_INTERFACE_LOCAL
#else
  #define CB_HW_INTERFACE_EXPORT __attribute__ ((visibility("default")))
  #define CB_HW_INTERFACE_IMPORT
  #if __GNUC__ >= 4
    #define CB_HW_INTERFACE_PUBLIC __attribute__ ((visibility("default")))
    #define CB_HW_INTERFACE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CB_HW_INTERFACE_PUBLIC
    #define CB_HW_INTERFACE_LOCAL
  #endif
  #define CB_HW_INTERFACE_PUBLIC_TYPE
#endif

#endif  // CB_HW_INTERFACE__VISIBILITY_CONTROL_H_
