// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ARM_VO__VISIBILITY_CONTROL_H_
#define ARM_VO__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ARM_VO_EXPORT __attribute__ ((dllexport))
    #define ARM_VO_IMPORT __attribute__ ((dllimport))
  #else
    #define ARM_VO_EXPORT __declspec(dllexport)
    #define ARM_VO_IMPORT __declspec(dllimport)
  #endif
  #ifdef ARM_VO_BUILDING_DLL
    #define ARM_VO_PUBLIC ARM_VO_EXPORT
  #else
    #define ARM_VO_PUBLIC ARM_VO_IMPORT
  #endif
  #define ARM_VO_PUBLIC_TYPE ARM_VO_PUBLIC
  #define ARM_VO_LOCAL
#else
  #define ARM_VO_EXPORT __attribute__ ((visibility("default")))
  #define ARM_VO_IMPORT
  #if __GNUC__ >= 4
    #define ARM_VO_PUBLIC __attribute__ ((visibility("default")))
    #define ARM_VO_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ARM_VO_PUBLIC
    #define ARM_VO_LOCAL
  #endif
  #define ARM_VO_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ARM_VO__VISIBILITY_CONTROL_H_