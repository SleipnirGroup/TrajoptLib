// Copyright (c) TrajoptLib contributors

#ifndef TRAJOPTLIB_JNI_DARWIN_JNI_MD_H_
#define TRAJOPTLIB_JNI_DARWIN_JNI_MD_H_

#ifndef __has_attribute
#define __has_attribute(x) 0
#endif

#ifndef JNIEXPORT
#if (defined(__GNUC__) &&                                            \
     ((__GNUC__ > 4) || (__GNUC__ == 4) && (__GNUC_MINOR__ > 2))) || \
    __has_attribute(visibility)
#ifdef ARM
#define JNIEXPORT __attribute__((externally_visible, visibility("default")))
#else
#define JNIEXPORT __attribute__((visibility("default")))
#endif

#else
#define JNIEXPORT
#endif
#endif

#if (defined(__GNUC__) &&                                            \
     ((__GNUC__ > 4) || (__GNUC__ == 4) && (__GNUC_MINOR__ > 2))) || \
    __has_attribute(visibility)
#ifdef ARM
#define JNIIMPORT __attribute__((externally_visible, visibility("default")))
#else
#define JNIIMPORT __attribute__((visibility("default")))
#endif
#else
#define JNIIMPORT
#endif

#define JNICALL

typedef int jint;
#ifdef _LP64
typedef long jlong;
#else
typedef long long jlong;
#endif

typedef signed char jbyte;

#endif  // TRAJOPTLIB_JNI_DARWIN_JNI_MD_H_
