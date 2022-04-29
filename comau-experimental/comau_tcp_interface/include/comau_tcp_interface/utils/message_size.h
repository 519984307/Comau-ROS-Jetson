/**
 * @file message_size.h
 * @author LMS ()
 * @brief
 * @version 0.1
 * @date  23-7-2020
 *
 * @copyright Copyright (c) 2020
 *
 */
#pragma once

#include <cstring>
#include <stdint.h>
#include <string>
#include <vector>

namespace comau_tcp_interface {
namespace utils {
/**
 * @brief The MessageSize class helping to meassure the size of message
 */
class MessageSize {
public:
  /**
   * @brief a general method for meassuring the size of arbitary datatypes
   *
   * @tparam T The type to serialize
   * @param val The input value
   * @return size_t Size in byte of the value
   */
  template <typename T> static size_t size(T val) {
    return sizeof(val);
  }

  template <typename T> static size_t size(std::vector<T> vec) {
    return sizeof(vec)*vec.size();
  }

  static size_t size(std::string &str){
    return str.size();
  }

};

} // namespace utils

} // namespace comau_tcp_interface
