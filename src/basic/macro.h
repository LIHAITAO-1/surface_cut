#ifndef macro_H
#define macro_H

#ifndef _API
#if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
#if defined(STATIC_LINKED)
#define _API
#else
#define _API __declspec(dllexport)
#endif
#else
#if defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
#define _API __attribute__((visibility("default")))
#else
#define _API
#endif
#endif
#endif

#define  ASSERT_MSG(condition, msg) \
    if((condition) == false) { logger().error(msg) ; exit(-1);}

#define  WARN_MSG(condition, msg) \
    if((condition) == false) { logger().warn(msg) ;}


#define  _Infinity  std::numeric_limits<double>::infinity()

#endif