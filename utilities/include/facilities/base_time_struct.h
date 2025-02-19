#pragma once
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "Time.h"

inline std::ostream& operator<<(std::ostream& stream, const builtin_interfaces::msg::Time& time) {
  return stream << "sec:" << time.sec() << ", nanosec:" << time.nanosec();
}

namespace TimeToolKit {
#define RCL_S_TO_NS(seconds) (seconds * (1000 * 1000 * 1000))
#define RCL_MS_TO_NS(milliseconds) (milliseconds * (1000 * 1000))
#define RCL_NS_TO_S(nanoseconds) (nanoseconds / (1000 * 1000 * 1000))
#define RCL_NS_TO_MS(nanoseconds) (nanoseconds / (1000 * 1000))
enum time_source_type {
  SYSTEM_TIME, /*系统时间,会随着系统的时间调整而发生变更*/
  STEADY_TIME  /*系统启动相对时间,不会随着系统的时间调整而发生变更*/
};

inline int64_t TimeSpecSysCurrentMs(time_source_type type = STEADY_TIME) {
  if (type == STEADY_TIME) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  } else {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  }
}

inline std::int64_t ConvertTmToTimestamp(const std::tm& timeInfo) {
  std::chrono::system_clock::time_point timePoint = std::chrono::system_clock::from_time_t(std::mktime(const_cast<std::tm*>(&timeInfo)));
  std::chrono::nanoseconds duration = std::chrono::duration_cast<std::chrono::nanoseconds>(timePoint.time_since_epoch());
  return duration.count();
}

inline std::tm ConvertTimestampToTm(std::int64_t timestamp) {
  std::time_t time = static_cast<std::time_t>(timestamp / 1000000000);  // 将纳秒转换为秒
  std::tm* timeInfo = std::localtime(&time);
  return *timeInfo;
}

inline builtin_interfaces::msg::Time Now(time_source_type type = STEADY_TIME) {
  builtin_interfaces::msg::Time msg_time;
  if (type == STEADY_TIME) {
    auto currentSteadyTimePoint = std::chrono::steady_clock::now();
    auto nanoSeconds = std::chrono::time_point_cast<std::chrono::nanoseconds>(currentSteadyTimePoint).time_since_epoch().count();
    msg_time.sec() = static_cast<std::int32_t>(RCL_NS_TO_S(nanoSeconds));
    msg_time.nanosec() = static_cast<std::uint32_t>(nanoSeconds % (1000 * 1000 * 1000));
  } else {
    auto currentSteadyTimePoint = std::chrono::system_clock::now();
    auto nanoSeconds = std::chrono::time_point_cast<std::chrono::nanoseconds>(currentSteadyTimePoint).time_since_epoch().count();
    msg_time.sec() = static_cast<std::int32_t>(RCL_NS_TO_S(nanoSeconds));
    msg_time.nanosec() = static_cast<std::uint32_t>(nanoSeconds % (1000 * 1000 * 1000));
  }
  return msg_time;
}

//亳秒，只保留亳秒
inline int64_t MilliTimeStamp(time_source_type type = STEADY_TIME) {
  if (type == STEADY_TIME) {
    auto currentSteadyTimePoint = std::chrono::steady_clock::now();
    auto milliSeconds = std::chrono::time_point_cast<std::chrono::milliseconds>(currentSteadyTimePoint).time_since_epoch().count();
    return milliSeconds;
  } else {
    auto currentSteadyTimePoint = std::chrono::system_clock::now();
    auto milliSeconds = std::chrono::time_point_cast<std::chrono::milliseconds>(currentSteadyTimePoint).time_since_epoch().count();
    return milliSeconds;
  }
}

//纳秒，只保留纳秒
inline int64_t NanoTimeStamp(time_source_type type = STEADY_TIME) {
  if (type == STEADY_TIME) {
    auto currentSteadyTimePoint = std::chrono::steady_clock::now();
    auto nanoSeconds = std::chrono::time_point_cast<std::chrono::nanoseconds>(currentSteadyTimePoint).time_since_epoch().count();
    return nanoSeconds;
  } else {
    auto currentSteadyTimePoint = std::chrono::system_clock::now();
    auto nanoSeconds = std::chrono::time_point_cast<std::chrono::nanoseconds>(currentSteadyTimePoint).time_since_epoch().count();
    return nanoSeconds;
  }
};

// Time转毫秒，直接返回int64类型
inline int64_t toMilliseconds(const builtin_interfaces::msg::Time& rhs) {
  int64_t milliseconds = static_cast<int64_t>(rhs.sec()) * 1000;
  milliseconds += static_cast<int64_t>(rhs.nanosec()) / 1000000;
  return milliseconds;
};

// Time转纳秒，直接返回int64类型
inline int64_t toNanoseconds(const builtin_interfaces::msg::Time& rhs) {
  int64_t nanoseconds = static_cast<int64_t>(rhs.sec()) * 1000000000;
  nanoseconds += static_cast<int64_t>(rhs.nanosec());
  return nanoseconds;
};

//毫秒转Time,直接返回Time类型
inline builtin_interfaces::msg::Time fromMilliseconds(const int64_t& Milliseconds) {
  builtin_interfaces::msg::Time time;
  time.sec() = static_cast<int32_t>(Milliseconds / 1000);
  time.nanosec() = static_cast<uint32_t>((Milliseconds - time.sec() * 1000) * 1000000);
  return time;
};

//纳秒转Time,直接返回Time类型
inline builtin_interfaces::msg::Time fromNanoseconds(const int64_t& Nanoseconds) {
  builtin_interfaces::msg::Time time;
  time.sec() = static_cast<int32_t>(Nanoseconds / 1000000000);
  time.nanosec() = static_cast<uint32_t>((Nanoseconds - time.sec() * 1000000000));
  return time;
};
};  // namespace TimeToolKit
