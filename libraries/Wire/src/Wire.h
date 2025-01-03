/*
 * TwoWire.h - TWI/I2C library for Arduino Due
 * Copyright (c) 2011 Cristian Maglie <c.maglie@arduino.cc>
 * All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef TwoWire_h
#define TwoWire_h

// Include Atmel CMSIS driver
#include <include/twi.h>
#include "Stream.h"
#include "variant.h"

// Forward declaration of TwoWireBuffer::Buffers
namespace TwoWireBuffer {
  struct Buffers;
}

 // WIRE_HAS_END means Wire has end()
#define WIRE_HAS_END 1

class TwoWire : public Stream {
public:
	TwoWire(const TwoWireBuffer::Buffers& _buffers,
	    Twi *twi, void(*begin_cb)(void), void(*end_cb)(void));
	void begin();
	void begin(uint8_t);
	void begin(int);
	void end();
	void setClock(uint32_t);
	void beginTransmission(uint8_t);
  inline void beginTransmission(int address) {
    beginTransmission(static_cast<uint8_t>(address));
  }
	uint8_t endTransmission(void);
  uint8_t endTransmission(uint8_t);
	uint8_t requestFrom(uint8_t, uint8_t, uint32_t, uint8_t, uint8_t);
  inline uint8_t requestFrom(uint8_t address, uint8_t quantity) {
    return requestFrom(static_cast<uint8_t>(address),
        static_cast<uint8_t>(quantity), static_cast<uint8_t>(true));
  }
  inline uint8_t requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop) {
    return requestFrom(static_cast<uint8_t>(address),
        static_cast<uint8_t>(quantity), static_cast<uint32_t>(0),
        static_cast<uint8_t>(0), static_cast<uint8_t>(sendStop));
  }
  inline uint8_t requestFrom(int address, int quantity) {
    return requestFrom(static_cast<uint8_t>(address),
        static_cast<uint8_t>(quantity), static_cast<uint8_t>(true));
  }
  inline uint8_t requestFrom(int address, int quantity, int sendStop) {
    return requestFrom(static_cast<uint8_t>(address),
        static_cast<uint8_t>(quantity), static_cast<uint8_t>(sendStop));
  }
	size_t write(uint8_t) override;
	size_t write(const uint8_t *, size_t) override;
	int available(void) override;
	int read(void) override;
	int peek(void) override;
	void flush(void) override;
	void onReceive(void(*)(int));
	void onRequest(void(*)(void));

  inline size_t write(unsigned long n) { return write(static_cast<uint8_t>(n)); }
  inline size_t write(long n) { return write(static_cast<uint8_t>(n)); }
  inline size_t write(unsigned int n) { return write(static_cast<uint8_t>(n)); }
  inline size_t write(int n) { return write(static_cast<uint8_t>(n)); }
  using Print::write;

	void onService(void);

private:
	// Container of rxBuffer, txBuffer and srvBuffer
	const TwoWireBuffer::Buffers& buffers;

	// RX Buffer
  inline uint8_t* rxBuffer() const;
	uint8_t rxBufferIndex;
	uint8_t rxBufferLength;

	// TX Buffer
  inline uint8_t* txBuffer() const;
	uint8_t txAddress;
	uint8_t txBufferLength;

	// Service buffer
	inline uint8_t* srvBuffer() const;
	uint8_t srvBufferIndex;
	uint8_t srvBufferLength;

	// Callback user functions
	void (*onRequestCallback)(void);
	void (*onReceiveCallback)(int);

	// Called before initialization
	void (*const onBeginCallback)(void);

	// Called after deinitialization
	void (*const onEndCallback)(void);

	// TWI instance
	Twi * const twi;

	// TWI state
	enum TwoWireStatus {
		UNINITIALIZED,
		MASTER_IDLE,
		MASTER_SEND,
		MASTER_RECV,
		SLAVE_IDLE,
		SLAVE_RECV,
		SLAVE_SEND
	};
	TwoWireStatus status;

	// TWI clock frequency
	static constexpr uint32_t TWI_CLOCK = 100000;
	uint32_t twiClock;

	// Timeouts (
	static constexpr uint32_t RECV_TIMEOUT = 100000;
	static constexpr uint32_t XMIT_TIMEOUT = 100000;
};

#if WIRE_INTERFACES_COUNT > 0
extern TwoWire Wire;
#endif
#if WIRE_INTERFACES_COUNT > 1
extern TwoWire Wire1;
#endif

#endif

