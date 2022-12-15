/*
 * @Author: xindong324
 * @Date: 2022-01-31 12:17:28
 * @LastEditors: xindong324
 * @LastEditTime: 2022-09-21 15:58:18
 * @Description: file content
 */
#ifndef __UAV_UTILS_H
#define __UAV_UTILS_H

#include <ros/ros.h>

#include <my_px4ctrl/converters.h>
#include <my_px4ctrl/geometry_utils.h>

namespace myuav_utils
{

/* judge if value belongs to [low,high] */
template <typename T, typename T2>
bool
in_range(T value, const T2& low, const T2& high)
{
  ROS_ASSERT_MSG(low < high, "%f < %f?", low, high);
  return (low <= value) && (value <= high);
}

/* judge if value belongs to [-limit, limit] */
template <typename T, typename T2>
bool
in_range(T value, const T2& limit)
{
  ROS_ASSERT_MSG(limit > 0, "%f > 0?", limit);
  return in_range(value, -limit, limit);
}

template <typename T, typename T2>
void
limit_range(T& value, const T2& low, const T2& high)
{
  ROS_ASSERT_MSG(low < high, "%f < %f?", low, high);
  if (value < low)
  {
    value = low;
  }

  if (value > high)
  {
    value = high;
  }

  return;
}

template <typename T, typename T2>
void
limit_range(T& value, const T2& limit)
{
  ROS_ASSERT_MSG(limit > 0, "%f > 0?", limit);
  limit_range(value, -limit, limit);
}

typedef std::stringstream DebugSS_t;
} // end of namespace uav_utils

#endif
