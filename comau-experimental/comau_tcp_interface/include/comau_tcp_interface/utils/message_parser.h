/**
 * @file message_parser.h
 * @author LMS ()
 * @brief
 * @version 0.1
 * @date 14-02-2020
 *
 * @copyright Copyright (c) 2020 (refactor and some additions)
 *
 * Initial copyright 2019 FZI Forschungszentrum Informatik
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#pragma once

#include <assert.h>
#include <cmath>
#include <cstring>
#include <string>

#include "comau_tcp_interface/utils/custom_data_type.h"

namespace comau_tcp_interface {
namespace utils {

/**
 * @brief The MessageParser class handles the binary message parsing
 *
 */
class MessageParser {
public:
  /**
   * @brief Construct a new Message Parser object
   *
   * @param buffer
   * @param buffer_len
   */
  MessageParser(uint8_t* buffer, size_t buffer_len)
      : buffer_curr_pos_(buffer), buffer_end_(buffer + buffer_len), parent_(*this) {
    assert(buffer_curr_pos_ <= buffer_end_);
  }
  /**
   * @brief Destroy the Message Parser object
   *
   */
  ~MessageParser() {
    parent_.buffer_curr_pos_ = buffer_curr_pos_;
  }

  /**
   * @brief Parses the next bytes as given type without moving the buffer pointer.
   *
   * @tparam T Type to parse as
   * @return Value of the next bytes as type T
   */
  template <typename T> T peek() {
    if (buffer_curr_pos_ + sizeof(T) > buffer_end_)
      throw new std::string("Could not parse received package.");

    T value;
    std::memcpy(&value, buffer_curr_pos_, sizeof(T));
    return value;
  }
  /**
   * @brief Parses the next bytes as given type.
   *
   * @param value Reference to write the parsed value to
   */
  template <typename T> void parse(T& value) {
    value = peek<T>();
    buffer_curr_pos_ += sizeof(T);
  }
  /**
   * @brief Parses the next bytes as double
   *
   * @param value Reference to write the parsed value to
   */
  void parse(double& value) {
    uint64_t inner;
    parse<uint64_t>(inner);
    std::memcpy(&value, &inner, sizeof(double));
  }
  /**
   * @brief Parses the next bytes as float
   *
   * @param value Reference to write the parsed value to
   */
  void parse(float& value) {
    uint32_t inner;
    parse<uint32_t>(inner);
    std::memcpy(&value, &inner, sizeof(float));
  }
  /**
   * @brief Parses the next byte as a bool.
   *
   * @param value Reference to write the parsed value to
   */
  void parse(bool& value) {
    uint8_t inner;
    parse<uint8_t>(inner);
    value = (inner != 0);
  }
  /**
   * @brief Parses the next bytes as a vector of 6 doubles.
   *
   * @param value Reference to write the parsed value to
   */
  void parse(vector6d_t& value) {
    for (size_t i = 0; i < value.size(); ++i) {
      parse(value[i]);
    }
  }
  /**
   * @brief Parses the next bytes as a vector of 6 floats.
   *
   * @param value Reference to write the parsed value to
   */
  void parse(vector6f_t& value) {
    for (size_t i = 0; i < value.size(); ++i) {
      parse(value[i]);
    }
  }
  /**
   * @brief Parses the dynamic size trajectory.
   *
   * @param value Reference to write the parsed value to
   */
  void parse(trajectoryf_t &value) {
    for (size_t i = 0; i < value.size(); ++i) {
      parse(value[i]);
    }
  }
  /**
   * @brief Parses the dynamic size trajectory.
   *
   * @param value Reference to write the parsed value to
   */
  void parse(trajectoryd_t &value) {
    for (size_t i = 0; i < value.size(); ++i) {
      parse(value[i]);
    }
  }
  /**
   * @brief Parses a given number of bytes as a string.
   *
   * @param value Reference to write the parsed value to
   * @param length  Number of bytes to parse
   */
  void parse(std::string& value, size_t length) {
    value.assign(reinterpret_cast<char*>(buffer_curr_pos_), length);
    buffer_curr_pos_ += length;
  }
  /**
   * @brief Parses the remaining bytes as a string.
   *
   * @param value Reference to write the parsed value to
   */
  void parseRemainder(std::string& value) {
    parse(value, size_t(buffer_end_ - buffer_curr_pos_));
  }

private:
  uint8_t *buffer_curr_pos_, *buffer_end_;
  MessageParser& parent_;
};

} // namespace utils
} // namespace comau_tcp_interface
