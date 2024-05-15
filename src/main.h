//
// Created by xmyci on 21/03/2024.
//
#include <string>

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


#ifndef TETGEO_MAIN_H
#define TETGEO_MAIN_H

_API int dll_main(std::string path);

#endif //TETGEO_MAIN_H
