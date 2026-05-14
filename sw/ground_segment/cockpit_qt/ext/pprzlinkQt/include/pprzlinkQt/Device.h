/*
 * Copyright 2019 garciafa
 * This file is part of PprzLinkCPP
 *
 * PprzLinkCPP is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * PprzLinkCPP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with ModemTester.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

/** \file Device.h
 *
 *
 */

#ifndef PPRZLINKCPP_DEVICE_H
#define PPRZLINKCPP_DEVICE_H

#include <cstdint>
#include <functional>
#include <deque>

/*
using check_free_space_t = std::function<int(void *, long *, uint16_t)>;
using put_byte_t = std::function<void(void *, long, uint8_t)>;
using put_buffer_t = std::function<void(void *, long, const uint8_t *, uint16_t)>;
using send_message_t = std::function<void(void *, long)>;
using char_available_t = std::function<int(void *)>;
using get_byte_t = std::function<uint8_t(void *)>;
using set_baudrate_t = std::function<void (void *, uint32_t baudrate)>;
*/

namespace pprzlink {

  using BytesBuffer = std::vector<uint8_t>;

  /**
   *
   */
  class Device {
  public:
    virtual size_t availableBytes() = 0;

    virtual BytesBuffer readAll() = 0;

    virtual void writeBuffer(BytesBuffer const &data) = 0;
  };
}
#endif //PPRZLINKCPP_DEVICE_H
