/* Copyright (c) 2011, Peter Barrett
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

#ifdef CDC_ENABLED

#ifndef __SAM4S4A__
#define CDC_SERIAL_BUFFER_SIZE	512
#else
//Moved to USBAPI.h so user sketch can see it.
//#define CDC_SERIAL_BUFFER_SIZE	1024 
#endif//SAM4S4A

/* For information purpose only since RTS is not always handled by the terminal application */
#define CDC_LINESTATE_DTR		0x01 // Data Terminal Ready
#define CDC_LINESTATE_RTS		0x02 // Ready to Send

#define CDC_LINESTATE_READY		(CDC_LINESTATE_RTS | CDC_LINESTATE_DTR)
//static volatile int doug=0; //debug
struct ring_buffer
{
	uint8_t buffer[CDC_SERIAL_BUFFER_SIZE];
	volatile uint32_t head;
	volatile uint32_t tail;
};

ring_buffer cdc_rx_buffer = { { 0 }, 0, 0};

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
#ifndef __SAM4S4A__
	D_ENDPOINT(USB_ENDPOINT_OUT(CDC_ENDPOINT_OUT),USB_ENDPOINT_TYPE_BULK,512,0),
	D_ENDPOINT(USB_ENDPOINT_IN (CDC_ENDPOINT_IN ),USB_ENDPOINT_TYPE_BULK,512,0)
#else
	//SAM3S has 64 on endpoint 4 and 5 (data sheet says 512, but that's only for ISOCHRONOS)
	D_ENDPOINT(USB_ENDPOINT_OUT(CDC_ENDPOINT_OUT),USB_ENDPOINT_TYPE_BULK,64,0),
	D_ENDPOINT(USB_ENDPOINT_IN (CDC_ENDPOINT_IN ),USB_ENDPOINT_TYPE_BULK,64,0)
#endif
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

bool WEAK CDC_Setup(Setup& setup)
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
	static uint32_t guard = 0;

	// synchronized access to guard
	do {
		if (__LDREXW(&guard) != 0) {
			__CLREX();
			return;  // busy
		}
	} while (__STREXW(1, &guard) != 0); // retry until write succeed
	ring_buffer *buffer = &cdc_rx_buffer;
	#ifdef __SAM4S4A__
	//SAM3S receives OUT token data in fifo and holds it there until ACK'd. RXBYTECNT indicates
	//The number of bytes available in the fifo. When data is popped from the fifo, the hardware
	//leaves RXBYTECNT unaltered (it does not decrement). Therefore, all bytes must be read from
	//the fifo at once, and then the fifo must be cleared. So we have to check the ring buffer
	//to see if there is enough room to accommodate the bytes available in the fifo. If not, then
	//we can't read the fifo, and we have to block further writes from the host. In this situation,
	//the RX interrupt will keep retriggering until there is room in the ring buffer to accept the data.
	//Because of this, we have to disable CDC_RX interrupt to allow user code to read the data (see Serial_:read()).
	//Short version: Must read entire fifo on the SAM3S or nothing at all. 
	uint32_t len;
	uint32_t bytes_free = CDC_SERIAL_BUFFER_SIZE - this->available(); // bytes free in ring buffer
	uint32_t fifo_count = USBD_Available(CDC_RX); //bytes available to be read in SAM3S fifo
	if((fifo_count) && (bytes_free > fifo_count)) {
		//bytes available in fifo, and enough space in ring buffer. Read the available bytes into the ring buffer.
		uint8_t data_tmp[64];//can't read more than 64 bytes on CDC_RX
		len = USBD_Recv(CDC_RX,&data_tmp,fifo_count); //read all available bytes and clear fifo
		//Now copy data into ring buffer
		for(int i=0; i<len; i++)
		{
			buffer->buffer[buffer->head] = data_tmp[i];
			buffer->head = (uint32_t)(buffer->head+1) % CDC_SERIAL_BUFFER_SIZE;
		}
	}

	#else
	uint32_t i = (uint32_t)(buffer->head+1) % CDC_SERIAL_BUFFER_SIZE;
	// if we should be storing the received character into the location
	// just before the tail (meaning that the head would advance to the
	// current location of the tail), we're about to overflow the buffer
	// and so we don't write the character or advance the head.
	while (i != buffer->tail) {
		uint32_t c;
		if (!USBD_Available(CDC_RX)) {
			udd_ack_fifocon(CDC_RX);
			break;
		}
		c = USBD_Recv(CDC_RX);
		// c = UDD_Recv8(CDC_RX & 0xF);
		buffer->buffer[buffer->head] = c;
		buffer->head = i;

		i = (i + 1) % CDC_SERIAL_BUFFER_SIZE;
	}
	#endif
	
	// release the guard
	guard = 0;
}

int Serial_::available(void)
{
	ring_buffer *buffer = &cdc_rx_buffer;
	return (unsigned int)(CDC_SERIAL_BUFFER_SIZE + buffer->head - buffer->tail) % CDC_SERIAL_BUFFER_SIZE;
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
		return buffer->buffer[buffer->tail];
	}
}

#ifdef __SAM4S4A__
//extended Serial class in USBAPI.h to provide this bulk read method.
//User code must supply a pointer to receive the data. The number of
//bytes actually read is returned. The idea is to quere SerialUSB.available()
//and pass it in as len to this method to read all available bytes from the 
//ring buffer in one shot. This should realize full SAM3S transmit bandwidth.

//Here is an example sketch to demonstrate:
/*
	char rx_bytes[CDC_SERIAL_BUFFER_SIZE];
	int len;
	String bytes_read;

	void setup() {
	  SerialUSB.begin(9600); 
	}

	void loop() {
	  if (SerialUSB.available() > 0) {
	    bytes_read = "";
	    len = SerialUSB.available();
	    len = SerialUSB.read((uint8_t*)&rx_bytes,len);
	    for (int i=0;i<len;i++){
	      bytes_read = bytes_read + rx_bytes[i];
	    }
	    SerialUSB.print("you typed ");
	    SerialUSB.println(bytes_read);
	  }
	}
*/
int Serial_::read(uint8_t* d, uint32_t len)
{
	ring_buffer* buffer = &cdc_rx_buffer;
	
	// if the head isn't ahead of the tail, we don't have any characters
	if (buffer->head == buffer->tail)
	{
		return 0;
	}
	else
	{
		uint8_t* ptr_dest = d;
		uint32_t bytes_avail = this->available(); //number of bytes available in ring buffer
		uint32_t fifo_count = USBD_Available(CDC_RX); //bytes available to be read in SAM3S fifo
		//Don't try to read more bytes than are available in the ring buffer
		if (len > bytes_avail) {
			len = bytes_avail;
		}
		
		//Now read the bytes from the ring buffer into the user's requested location
		uint32_t i;
		for (i = 0; i < len; ++i) {
			*ptr_dest++ = buffer->buffer[buffer->tail];
			buffer->tail = (uint32_t)(buffer->tail+1) % CDC_SERIAL_BUFFER_SIZE;
		}
		
		//Now see if there is more data available from the host
		if (fifo_count) {
			//read fifo into ring buffer
			accept();
		} else {
			//All bytes read from fifo and buffer. Turn calling of accept() back over to interrupt.
			if (!this->available()){
				udd_enable_endpoint_interrupt(CDC_RX);
			}
		}
		
	}
	return len;
}
#endif //SAM4S4A



int Serial_::read(void)
{
	ring_buffer *buffer = &cdc_rx_buffer;
	
	// if the head isn't ahead of the tail, we don't have any characters
	if (buffer->head == buffer->tail)
	{
		return -1;
	}
	else
	{
		unsigned char c = buffer->buffer[buffer->tail];
		buffer->tail = (unsigned int)(buffer->tail + 1) % CDC_SERIAL_BUFFER_SIZE;
		#ifdef __SAM4S4A__
		//The first call of accept() is triggered by the CDC_RX interrupt. The interrupt is desabled 
		//and the data is read into the ring buffer. This allows user code to call Serial_::read() to 
		//drain the ring buffer. Otherwise, the interrupts would continue to pre-empt user code, 
		//preventing user code from reading the data, and finally the buffer would fill and deadlock would result.
		//Each read() checks again for data available in the fifo. If there is data there and the ring buffer has
		//room, accept() will be called again to draw the fifo data into the ring buffer. Finally when there
		//is no more data in the fifo (nothing coming in from the host) the interrupt is re-enabled to wake up if 
		//the host sends more data later, and the process is started all over again. So the main problem here is 
		//that the host can fill the fifo in 64 byte chunks, but Serial_:read() can only read one byte at a time. 
		//This means the host is blocked from sending unless the ring buffer capacity is increased.
		//There is no implementation here that can read the entire buffer. Is there? This cuts read bandwidth significantly.
		//Seems like user code should be able to call Serial_::available() to determine the byte count, and pull it out in one
		//call. ex: Serial_::read(&data,count). Such a method could return the actual number of bytes read just in case.
		uint32_t bytes_free = CDC_SERIAL_BUFFER_SIZE - this->available(); //bytes free in ring buffer
		uint32_t fifo_count = USBD_Available(CDC_RX); //bytes available to be read in SAM3S fifo
		if (fifo_count) {
			if (bytes_free > fifo_count) {
				//read fifo into ring buffer
				accept();
			}
		} else {
			//All bytes read from fifo and buffer. Turn calling of accept() back over to interrupt.
			if (!this->available()){
				udd_enable_endpoint_interrupt(CDC_RX);
			}
		}
		#else	
		if (USBD_Available(CDC_RX))
			accept();
		#endif
		
		return c;
	}
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

Serial_ SerialUSB;

#ifdef __SAM4S4A__
//Allow USBCore.cpp to fetch the configuration descriptor
const CDCDescriptor* CDC_GetDescriptor(void){
	return &_cdcInterface;
}
#endif //SAM4S4A

#endif //CDC_ENABLED


