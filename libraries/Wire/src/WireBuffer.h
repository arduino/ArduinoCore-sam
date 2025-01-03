/*
  WireBuffer.h - TWI/I2C library for Arduino & Wiring
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

#pragma once

#ifndef Wire_WireBuffer_h_
#define Wire_WireBuffer_h_

#include <stdint.h>
#include <stddef.h>
#include "variant.h"


// Use extra namespace to avoid collision with other symbols
namespace TwoWireBuffer {
  // Declare twi buffers
  typedef size_t bufferSize_t;
  constexpr bufferSize_t WIRE_BUFFER_DEFAULT_SIZE = 32;

  struct Buffers {
    bufferSize_t SRV_BUFFER_SIZE;
    bufferSize_t RX_BUFFER_SIZE;
    bufferSize_t TX_BUFFER_SIZE;
    uint8_t* const srvBuffer;
    uint8_t* const rxBuffer;
    uint8_t* const txBuffer;
  };
}

#define SET_TwoWire_BUFFERS(rxBufferSize, txBufferSize) \
    constexpr bufferSize_t srvBufferSize = (rxBufferSize) > (txBufferSize) ? (rxBufferSize) : (txBufferSize); \
    const Buffers buffers { (srvBufferSize), (rxBufferSize), (txBufferSize), srvBuffer, rxBuffer, txBuffer }; \
    uint8_t srvBuffer[srvBufferSize]; \
    uint8_t  rxBuffer[(rxBufferSize)];\
    uint8_t  txBuffer[(txBufferSize)];


#if WIRE_INTERFACES_COUNT > 0
  namespace WireBuffer {
    using namespace TwoWireBuffer;
    extern const Buffers buffers;
    extern uint8_t srvBuffer[];
    extern uint8_t  rxBuffer[];
    extern uint8_t  txBuffer[];
  }

  #define SET_Wire_BUFFERS(rxBufferSize, txBufferSize, enableMaster, enableSlave) \
        namespace WireBuffer {SET_TwoWire_BUFFERS(rxBufferSize, txBufferSize)}
#endif // WIRE_INTERFACES_COUNT > 0

#if WIRE_INTERFACES_COUNT > 1
  namespace Wire1Buffer {
    using namespace TwoWireBuffer;
    extern const Buffers buffers;
    extern uint8_t srvBuffer[];
    extern uint8_t  rxBuffer[];
    extern uint8_t  txBuffer[];
  }

  #define SET_Wire1_BUFFERS(rxBufferSize, txBufferSize, enableMaster, enableSlave) \
        namespace Wire1Buffer {SET_TwoWire_BUFFERS(rxBufferSize, txBufferSize)}
#endif // WIRE_INTERFACES_COUNT > 1

#endif /* Wire_WireBuffer_h_ */
