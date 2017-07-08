/*
  Copyright (c) 2012 Arduino.  
  Copyright (c) 2017 Boris Barbour
  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef __USBAPI__
#define __USBAPI__

#if defined __cplusplus

#include "Stream.h"

//================================================================================
//================================================================================
//	USB

class USBDevice_
{
public:
	USBDevice_();
	bool configured();

	bool attach();
	bool detach();	// Serial port goes down too...
	void poll();
};
extern USBDevice_ USBDevice;

// Best to use a power of 2 to enable simplified modulo calculations
// (the compiler will automatically use &). USB2 packets can contain
// up to 1kb (isochronous mode), but usually less. The FIFOs seem to
// be 512 bytes on the Due/SAM.
#define CDC_SERIAL_BUFFER_SIZE	512

// This could go into a separate header and file, but it's small and
// users may need access to the size definition. Note that this is
// (confusingly) distinct from the general RingBuffer declared in
// RingBuffer.h. The implementation has been changed. Instead of
// continuously taking the modulus of head and tail, we now have
// ever-increasing longs, whose modulus is taken only to address the
// buffer. This may add a small overhead, but ensures that the tail
// overrunning the head can be detected even with interrupt and DMA
// applications. As a minor side-effect, the buffer can hold one more
// byte, since head==tail (empty) can now be distinguished from
// head==tail+size (full). The use of 64-bit uints ensures that they
// never overflow in the lifetime of the universe; 32-bit uints might
// do so in a matter of hours or days at top rates.
struct ring_buffer
{
	uint8_t buffer[CDC_SERIAL_BUFFER_SIZE];
	volatile uint64_t head;
	volatile uint64_t tail;
};

//================================================================================
//================================================================================
//	Serial over CDC (Serial1 is the physical port)

class Serial_ : public Stream
{
public:
        // The ring buffer implementation is public to allow user DMA access.
	ring_buffer cdc_rx_buffer = { { 0 }, 0, 0};
	// Standard arduino only schedules interrupts between "loop"
	// iterations, so this is the default and the user will be
	// responsible for scheduling reception of data via "accept"
	// directly or indirectly by calling one of the "read"
	// functions, which call "accept" (the read functions are not
	// used during DMA applications). Even when interrupts are
	// enabled, if the receive buffer doesn't have space when the
	// interrupt service routine is called, it can be necessary to
	// call "accept" manually to complete the transfer and prevent
	// blocking.
	void enableInterrupts();
	void disableInterrupts();
	void begin(uint32_t baud_count);
	void begin(uint32_t baud_count, uint8_t config);
	void end(void);

	virtual int available(void);
	virtual int availableForWrite(void);
	virtual void accept(void);
	virtual int peek(void);
	virtual int read(void);
	virtual int read(uint8_t *d, size_t t);
	virtual void flush(void);
	virtual size_t write(uint8_t);
	virtual size_t write(const uint8_t *buffer, size_t size);
	using Print::write; // pull in write(str) from Print
	operator bool();

	// This method allows processing "SEND_BREAK" requests sent by
	// the USB host. Those requests indicate that the host wants to
	// send a BREAK signal and are accompanied by a single uint16_t
	// value, specifying the duration of the break. The value 0
	// means to end any current break, while the value 0xffff means
	// to start an indefinite break.
	// readBreak() will return the value of the most recent break
	// request, but will return it at most once, returning -1 when
	// readBreak() is called again (until another break request is
	// received, which is again returned once).
	// This also mean that if two break requests are received
	// without readBreak() being called in between, the value of the
	// first request is lost.
	// Note that the value returned is a long, so it can return
	// 0-0xffff as well as -1.
	int32_t readBreak();

	// These return the settings specified by the USB host for the
	// serial port. These aren't really used, but are offered here
	// in case a sketch wants to act on these settings.
	uint32_t baud();
	uint8_t stopbits();
	uint8_t paritytype();
	uint8_t numbits();
	bool dtr();
	bool rts();
	enum {
		ONE_STOP_BIT = 0,
		ONE_AND_HALF_STOP_BIT = 1,
		TWO_STOP_BITS = 2,
	};
	enum {
		NO_PARITY = 0,
		ODD_PARITY = 1,
		EVEN_PARITY = 2,
		MARK_PARITY = 3,
		SPACE_PARITY = 4,
	};
};
extern Serial_ SerialUSB;

//================================================================================
//================================================================================
//	Low level API

typedef struct
{
	uint8_t bmRequestType;
	uint8_t bRequest;
	uint8_t wValueL;
	uint8_t wValueH;
	uint16_t wIndex;
	uint16_t wLength;
} USBSetup;

//================================================================================
//================================================================================
//	MSC 'Driver'

int		MSC_GetInterface(uint8_t* interfaceNum);
int		MSC_GetDescriptor(int i);
bool	MSC_Setup(USBSetup& setup);
bool	MSC_Data(uint8_t rx,uint8_t tx);

//================================================================================
//================================================================================
//	CSC 'Driver'

int		CDC_GetInterface(uint8_t* interfaceNum);
int		CDC_GetOtherInterface(uint8_t* interfaceNum);
int		CDC_GetDescriptor(int i);
bool	CDC_Setup(USBSetup& setup);

//================================================================================
//================================================================================

#define TRANSFER_RELEASE	0x40
#define TRANSFER_ZERO		0x20

void USBD_InitControl(int end);
int USBD_SendControl(uint8_t flags, const void* d, uint32_t len);
int USBD_RecvControl(void* d, uint32_t len);
uint8_t USBD_SendInterfaces(void);
bool USBD_ClassInterfaceRequest(USBSetup& setup);


uint32_t USBD_Available(uint32_t ep);
uint32_t USBD_SendSpace(uint32_t ep);
uint32_t USBD_Send(uint32_t ep, const void* d, uint32_t len);
uint32_t USBD_Recv(uint32_t ep, void* data, uint32_t len);		// non-blocking
uint32_t USBD_Recv(uint32_t ep);							// non-blocking
void USBD_Flush(uint32_t ep);
uint32_t USBD_Connected(void);

#endif
#endif
