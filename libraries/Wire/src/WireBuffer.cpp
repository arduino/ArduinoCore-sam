/*
  WireBuffer.cpp - TWI/I2C library for Arduino & Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <stdint.h>
#include <stddef.h>
#include "variant.h"
#include "WireBuffer.h"

#if WIRE_INTERFACES_COUNT > 0
// Define default buffers as weak buffers
namespace WireBuffer {
  extern __attribute__((weak)) const Buffers buffers {
    WIRE_BUFFER_DEFAULT_SIZE, WIRE_BUFFER_DEFAULT_SIZE, WIRE_BUFFER_DEFAULT_SIZE
    , srvBuffer, rxBuffer, txBuffer
  };
  __attribute__((weak)) uint8_t srvBuffer[WIRE_BUFFER_DEFAULT_SIZE];
  __attribute__((weak)) uint8_t  rxBuffer[WIRE_BUFFER_DEFAULT_SIZE];
  __attribute__((weak)) uint8_t  txBuffer[WIRE_BUFFER_DEFAULT_SIZE];
}
#endif

#if WIRE_INTERFACES_COUNT > 1
// Define default buffers as weak buffers
namespace Wire1Buffer {
  extern __attribute__((weak)) const Buffers buffers {
    WIRE_BUFFER_DEFAULT_SIZE, WIRE_BUFFER_DEFAULT_SIZE, WIRE_BUFFER_DEFAULT_SIZE
    , srvBuffer, rxBuffer, txBuffer
  };
  __attribute__((weak)) uint8_t srvBuffer[WIRE_BUFFER_DEFAULT_SIZE];
  __attribute__((weak)) uint8_t  rxBuffer[WIRE_BUFFER_DEFAULT_SIZE];
  __attribute__((weak)) uint8_t  txBuffer[WIRE_BUFFER_DEFAULT_SIZE];
}
#endif
