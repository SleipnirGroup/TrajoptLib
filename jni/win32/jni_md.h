// Copyright (c) TrajoptLib contributors

#ifndef TRAJOPTLIB_JNI_WIN32_JNI_MD_H_
#define TRAJOPTLIB_JNI_WIN32_JNI_MD_H_

#define JNIEXPORT __declspec(dllexport)
#define JNIIMPORT __declspec(dllimport)
#define JNICALL __stdcall

typedef long jint;
typedef __int64 jlong;
typedef signed char jbyte;

#endif  // TRAJOPTLIB_JNI_WIN32_JNI_MD_H_
