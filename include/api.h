#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define Walking_controller_DLLIMPORT __declspec(dllimport)
#  define Walking_controller_DLLEXPORT __declspec(dllexport)
#  define Walking_controller_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define Walking_controller_DLLIMPORT __attribute__((visibility("default")))
#    define Walking_controller_DLLEXPORT __attribute__((visibility("default")))
#    define Walking_controller_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define Walking_controller_DLLIMPORT
#    define Walking_controller_DLLEXPORT
#    define Walking_controller_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef Walking_controller_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define Walking_controller_DLLAPI
#  define Walking_controller_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef Walking_controller_EXPORTS
#    define Walking_controller_DLLAPI Walking_controller_DLLEXPORT
#  else
#    define Walking_controller_DLLAPI Walking_controller_DLLIMPORT
#  endif // Walking_controller_EXPORTS
#  define Walking_controller_LOCAL Walking_controller_DLLLOCAL
#endif // Walking_controller_STATIC