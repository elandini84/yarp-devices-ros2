#ifndef CB_HW_IFACE__VISIBILITY_CONTROL_H_
#define CB_HW_IFACE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CB_HW_IFACE_EXPORT __attribute__ ((dllexport))
    #define CB_HW_IFACE_IMPORT __attribute__ ((dllimport))
  #else
    #define CB_HW_IFACE_EXPORT __declspec(dllexport)
    #define CB_HW_IFACE_IMPORT __declspec(dllimport)
  #endif
  #ifdef CB_HW_IFACE_BUILDING_LIBRARY
    #define CB_HW_IFACE_PUBLIC CB_HW_IFACE_EXPORT
  #else
    #define CB_HW_IFACE_PUBLIC CB_HW_IFACE_IMPORT
  #endif
  #define CB_HW_IFACE_PUBLIC_TYPE CB_HW_IFACE_PUBLIC
  #define CB_HW_IFACE_LOCAL
#else
  #define CB_HW_IFACE_EXPORT __attribute__ ((visibility("default")))
  #define CB_HW_IFACE_IMPORT
  #if __GNUC__ >= 4
    #define CB_HW_IFACE_PUBLIC __attribute__ ((visibility("default")))
    #define CB_HW_IFACE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CB_HW_IFACE_PUBLIC
    #define CB_HW_IFACE_LOCAL
  #endif
  #define CB_HW_IFACE_PUBLIC_TYPE
#endif

#endif  // CB_HW_IFACE__VISIBILITY_CONTROL_H_
