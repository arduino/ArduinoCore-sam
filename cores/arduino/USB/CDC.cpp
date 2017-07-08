/* Copyright (c) 2011, Peter Barrett
** Copyright (c) 2017, Boris Barbour
**
** Permission to use, copy, modify, and/or distribute this software for
** any purpose with or without fee is hereby granted, provided that the
** above copyright notice and this permission notice appear in all copies.
**
** THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
** WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR
** BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES
** OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
** WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
** ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
** SOFTWARE.
*/

#include "Arduino.h"
#include "USBAPI.h"
#include "Reset.h"
#include "Print.h"

#ifdef CDC_ENABLED

/* For information purpose only since RTS is not always handled by the terminal application */
#define CDC_LINESTATE_DTR		0x01 // Data Terminal Ready
#define CDC_LINESTATE_RTS		0x02 // Ready to Send

#define CDC_LINESTATE_READY		(CDC_LINESTATE_RTS | CDC_LINESTATE_DTR)

typedef struct
{
	uint32_t	dwDTERate;
	uint8_t		bCharFormat;
	uint8_t 	bParityType;
	uint8_t 	bDataBits;
	uint8_t		lineState;
} LineInfo;

static volatile LineInfo _usbLineInfo = { 
    57600, // dWDTERate
    0x00,  // bCharFormat
    0x00,  // bParityType
    0x08,  // bDataBits
    0x00   // lineState
};

static volatile int32_t breakValue = -1;

_Pragma("pack(1)")
static const CDCDescriptor _cdcInterface =
{
	D_IAD(0,2,CDC_COMMUNICATION_INTERFACE_CLASS,CDC_ABSTRACT_CONTROL_MODEL,1),

	//	CDC communication interface
	D_INTERFACE(CDC_ACM_INTERFACE,1,CDC_COMMUNICATION_INTERFACE_CLASS,CDC_ABSTRACT_CONTROL_MODEL,0),
	D_CDCCS(CDC_HEADER,0x10,0x01),								// Header (1.10 bcd)
	D_CDCCS(CDC_CALL_MANAGEMENT,1,1),							// Device handles call management (not)
	D_CDCCS4(CDC_ABSTRACT_CONTROL_MANAGEMENT,6),				// SET_LINE_CODING, GET_LINE_CODING, SET_CONTROL_LINE_STATE supported
	D_CDCCS(CDC_UNION,CDC_ACM_INTERFACE,CDC_DATA_INTERFACE),	// Communication interface is master, data interface is slave 0
	D_ENDPOINT(USB_ENDPOINT_IN (CDC_ENDPOINT_ACM),USB_ENDPOINT_TYPE_INTERRUPT,0x10, 0x10),

	//	CDC data interface
	D_INTERFACE(CDC_DATA_INTERFACE,2,CDC_DATA_INTERFACE_CLASS,0,0),
	D_ENDPOINT(USB_ENDPOINT_OUT(CDC_ENDPOINT_OUT),USB_ENDPOINT_TYPE_BULK,512,0),
	D_ENDPOINT(USB_ENDPOINT_IN (CDC_ENDPOINT_IN ),USB_ENDPOINT_TYPE_BULK,512,0)
};
static const CDCDescriptor _cdcOtherInterface =
{
	D_IAD(0,2,CDC_COMMUNICATION_INTERFACE_CLASS,CDC_ABSTRACT_CONTROL_MODEL,1),

	//	CDC communication interface
	D_INTERFACE(CDC_ACM_INTERFACE,1,CDC_COMMUNICATION_INTERFACE_CLASS,CDC_ABSTRACT_CONTROL_MODEL,0),
	D_CDCCS(CDC_HEADER,0x10,0x01),								// Header (1.10 bcd)
	D_CDCCS(CDC_CALL_MANAGEMENT,1,1),							// Device handles call management (not)
	D_CDCCS4(CDC_ABSTRACT_CONTROL_MANAGEMENT,6),				// SET_LINE_CODING, GET_LINE_CODING, SET_CONTROL_LINE_STATE supported
	D_CDCCS(CDC_UNION,CDC_ACM_INTERFACE,CDC_DATA_INTERFACE),	// Communication interface is master, data interface is slave 0
	D_ENDPOINT(USB_ENDPOINT_IN (CDC_ENDPOINT_ACM),USB_ENDPOINT_TYPE_INTERRUPT,0x10, 0x10),

	//	CDC data interface
	D_INTERFACE(CDC_DATA_INTERFACE,2,CDC_DATA_INTERFACE_CLASS,0,0),
	D_ENDPOINT(USB_ENDPOINT_OUT(CDC_ENDPOINT_OUT),USB_ENDPOINT_TYPE_BULK,64,0),
	D_ENDPOINT(USB_ENDPOINT_IN (CDC_ENDPOINT_IN ),USB_ENDPOINT_TYPE_BULK,64,0)
};
_Pragma("pack()")

int WEAK CDC_GetInterface(uint8_t* interfaceNum)
{
	interfaceNum[0] += 2;	// uses 2
	return USBD_SendControl(0,&_cdcInterface,sizeof(_cdcInterface));
}

int WEAK CDC_GetOtherInterface(uint8_t* interfaceNum)
{
	interfaceNum[0] += 2;	// uses 2
	return USBD_SendControl(0,&_cdcOtherInterface,sizeof(_cdcOtherInterface));
}

bool WEAK CDC_Setup(USBSetup& setup)
{
	uint8_t r = setup.bRequest;
	uint8_t requestType = setup.bmRequestType;

	if (REQUEST_DEVICETOHOST_CLASS_INTERFACE == requestType)
	{
		if (CDC_GET_LINE_CODING == r)
		{
			USBD_SendControl(0,(void*)&_usbLineInfo,7);
			return true;
		}
	}

	if (REQUEST_HOSTTODEVICE_CLASS_INTERFACE == requestType)
	{
		if (CDC_SET_LINE_CODING == r)
		{
			USBD_RecvControl((void*)&_usbLineInfo,7);
			return true;
		}

		if (CDC_SET_CONTROL_LINE_STATE == r)
		{
			_usbLineInfo.lineState = setup.wValueL;
			// auto-reset into the bootloader is triggered when the port, already
			// open at 1200 bps, is closed.
			if (1200 == _usbLineInfo.dwDTERate)
			{
				// We check DTR state to determine if host port is open (bit 0 of lineState).
				if ((_usbLineInfo.lineState & 0x01) == 0)
					initiateReset(250);
				else
					cancelReset();
			}
			return true;
		}

		if (CDC_SEND_BREAK == r)
		{
			breakValue = ((uint16_t)setup.wValueH << 8) | setup.wValueL;
			return true;
		}
	}
	return false;
}

int _serialPeek = -1;
void Serial_::begin(uint32_t baud_count)
{
	// suppress "unused parameter" warning
	(void)baud_count;
}

void Serial_::begin(uint32_t baud_count, uint8_t config)
{
	// suppress "unused parameter" warning
	(void)baud_count;
	(void)config;
}

void Serial_::end(void)
{
}

void Serial_::accept(void)
{
        // Use fifocon to synchronise. Leave if there is no data.
        if (!Is_udd_fifocon(CDC_RX)) return;
        // This rearms interrupt, but FIFO must be released before it
        // can retrigger. Moved here from the interrupt service
        // routine because we may come to this function directly.
	if (Is_udd_out_received(CDC_RX)) udd_ack_out_received(CDC_RX);
	ring_buffer *buffer = &cdc_rx_buffer;
	uint32_t b = CDC_SERIAL_BUFFER_SIZE;
	uint32_t u = UDD_FifoByteCount(CDC_RX);
	uint32_t s = b - (uint32_t)(buffer->head - buffer->tail);
	uint32_t r = min(s, u);
	while(r) {
	        // May only be able to fill to the end of the buffer in first call.
	        uint32_t h = (buffer->head)%b;
	        uint32_t g = min(r, b-h);
	        UDD_Recv(CDC_RX, &(buffer->buffer[h]), g);
		r -= g;
		buffer->head += g;
	}
	// Don't release FIFO if not all data was transferred.
	if (!UDD_FifoByteCount(CDC_RX)) UDD_ReleaseRX(CDC_RX);
}

void Serial_::enableInterrupts()
{
        udd_enable_out_received_interrupt(CDC_RX);
	udd_enable_endpoint_interrupt(CDC_RX); 
}

void Serial_::disableInterrupts()
{
        udd_disable_out_received_interrupt(CDC_RX);
	udd_disable_endpoint_interrupt(CDC_RX);
}

int Serial_::available(void)
{
	ring_buffer *buffer = &cdc_rx_buffer;
	return (unsigned int)(buffer->head - buffer->tail);
}

int Serial_::availableForWrite(void)
{
	// return the number of bytes left in the current bank,
	// always EP size - 1, because bank is flushed on every write
	return (EPX_SIZE - 1);
}

int Serial_::peek(void)
{
	ring_buffer *buffer = &cdc_rx_buffer;

	if (buffer->head == buffer->tail)
	{
		return -1;
	}
	else
	{
	        uint32_t b = CDC_SERIAL_BUFFER_SIZE;
		return buffer->buffer[(buffer->tail)%b];
	}
}

int Serial_::read(void)
{
	ring_buffer *buffer = &cdc_rx_buffer;

	// Give "accept" a chance to catch up if data is ready.
	// Interrupt shouldn't be able to fire in this condition.
        //if (Is_udd_fifocon(CDC_RX)) 
	  accept();

	// if the head isn't ahead of the tail, we don't have any characters
	if (buffer->head == buffer->tail)
	{
		return -1;
	}
	else
	{
	        uint32_t b = CDC_SERIAL_BUFFER_SIZE;
		unsigned char c = buffer->buffer[(buffer->tail)%b];
		buffer->tail++;
		return c;
	}
}

int Serial_::read(uint8_t *d, size_t s) 
{
  	ring_buffer *buffer = &cdc_rx_buffer;
	uint32_t b = CDC_SERIAL_BUFFER_SIZE;
	uint32_t a = (uint32_t) (buffer->head - buffer->tail);
	// Number of bytes to read is the smaller of those available and those requested.
	uint32_t r = min(a, s);
	uint32_t k = r;
	// May reach end of buffer before completing transfer.
	while(r) {
	  uint32_t tm = (buffer->tail)%b;
	  uint32_t g = min(r, b-tm);
	  for (int i = 0 ; i < g; i++) { 
	    d[i] = buffer->buffer[tm + i];
	  }
	  d += g;
	  r -= g;
	  buffer->tail += g;
	}
	// Give "accept" a chance to catch up if data is ready.
	// Interrupt shouldn't be able to fire in this condition.
	//        if (Is_udd_fifocon(CDC_RX)) {
	  if ((a-k) < b) accept();
	  //}
	return k;
}


void Serial_::flush(void)
{
	USBD_Flush(CDC_TX);
}

size_t Serial_::write(const uint8_t *buffer, size_t size)
{
	/* only try to send bytes if the high-level CDC connection itself
	 is open (not just the pipe) - the OS should set lineState when the port
	 is opened and clear lineState when the port is closed.
	 bytes sent before the user opens the connection or after
	 the connection is closed are lost - just like with a UART. */

	// TODO - ZE - check behavior on different OSes and test what happens if an
	// open connection isn't broken cleanly (cable is yanked out, host dies
	// or locks up, or host virtual serial port hangs)
	if (_usbLineInfo.lineState > 0)
	{
		int r = USBD_Send(CDC_TX, buffer, size);

		if (r > 0)
		{
			return r;
		} else
		{
			setWriteError();
			return 0;
		}
	}
	setWriteError();
	return 0;
}

size_t Serial_::write(uint8_t c) {
	return write(&c, 1);
}

// This operator is a convenient way for a sketch to check whether the
// port has actually been configured and opened by the host (as opposed
// to just being connected to the host).  It can be used, for example, in
// setup() before printing to ensure that an application on the host is
// actually ready to receive and display the data.
// We add a short delay before returning to fix a bug observed by Federico
// where the port is configured (lineState != 0) but not quite opened.
Serial_::operator bool()
{
	// this is here to avoid spurious opening after upload
	if (millis() < 500)
		return false;

	bool result = false;

	if (_usbLineInfo.lineState > 0)
	{
		result = true;
	}

	delay(10);
	return result;
}

int32_t Serial_::readBreak() {
	uint8_t enableInterrupts = ((__get_PRIMASK() & 0x1) == 0 && (__get_FAULTMASK() & 0x1) == 0);

	// disable interrupts,
	// to avoid clearing a breakValue that might occur 
	// while processing the current break value
	__disable_irq();

	int ret = breakValue;

	breakValue = -1;

	if (enableInterrupts)
	{
		// re-enable the interrupts
		__enable_irq();
	}

	return ret;
}

unsigned long Serial_::baud() {
	return _usbLineInfo.dwDTERate;
}

uint8_t Serial_::stopbits() {
	return _usbLineInfo.bCharFormat;
}

uint8_t Serial_::paritytype() {
	return _usbLineInfo.bParityType;
}

uint8_t Serial_::numbits() {
	return _usbLineInfo.bDataBits;
}

bool Serial_::dtr() {
	return _usbLineInfo.lineState & 0x1;
}

bool Serial_::rts() {
	return _usbLineInfo.lineState & 0x2;
}

Serial_ SerialUSB;

#endif
