// Copyright (c) 2010, Peter Barrett
/*
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
#include <stdio.h>


//#define TRACE_CORE(x)	x
#define TRACE_CORE(x)

#ifdef __SAM4S4A__
/* functions used by USBCore.cpp, implemented in variant.cpp*/
static volatile uint32_t doug;
static volatile uint32_t ul_send_fifo_ptr[MAX_ENDPOINTS];
static volatile uint32_t ul_recv_fifo_ptr[MAX_ENDPOINTS];
extern void (*gpf_isr)(void); //see uotghs.c
static uint32_t UDD_Send(uint32_t ep, const void* data, uint32_t len);
static void UDD_Recv(uint32_t ep, uint8_t* data, uint32_t len);
static uint8_t UDD_Recv8(uint32_t ep);
static uint32_t UDD_ReceivedSetupInt(void);
static void UDD_ClearSetupInt(void);
static void UDD_WaitIN(void);
static void UDD_WaitOUT(void);
static void UDD_ClearOUT(void);
static void UDD_ClearIN(void);
static void UDD_Send8(uint32_t ep,  uint8_t data );
static void UDD_SetAddress(uint32_t addr);
static void UDD_InitEP( uint32_t ul_ep_nb, uint32_t ul_ep_cfg );
static void UDD_InitEndpoints(const uint32_t* eps_table, const uint32_t ul_eps_table_size);
static uint32_t UDD_FifoByteCount(uint32_t ep);
static void UDD_ReleaseRX(uint32_t ep);
static void UDD_Stall(void);
static uint32_t UDD_GetFrameNumber(void);
static uint32_t UDD_Init(void);
static void UDD_SetStack(void (*pf_isr)(void));
static void UDD_ReleaseTX(uint32_t ep);
static void UDD_Attach(void);
static void UDD_Detach(void);
void sysclk_enable_usb(void);
struct pll_config {
	uint32_t ctrl;
};
int USBD_GetConfiguration(uint8_t* data,uint32_t len);

#ifdef CDC_ENABLED
static volatile bool cdc_tx_banks_idle=true; //true if both tx banks are idle.
static volatile int cdc_rx_bank_number=0; //track which bank to ack
#endif

#endif //__SAM4S4A__

static const uint32_t EndPoints[] =
{
	EP_TYPE_CONTROL,

#ifdef CDC_ENABLED
	EP_TYPE_INTERRUPT_IN,           // CDC_ENDPOINT_ACM 
	EP_TYPE_BULK_OUT,               // CDC_ENDPOINT_OUT
	EP_TYPE_BULK_IN,                // CDC_ENDPOINT_IN
#endif

#ifdef HID_ENABLED
	EP_TYPE_INTERRUPT_IN_HID        // HID_ENDPOINT_INT
#endif
};

/** Pulse generation counters to keep track of the number of milliseconds remaining for each pulse type */
#define TX_RX_LED_PULSE_MS 100
volatile uint8_t TxLEDPulse; /**< Milliseconds remaining for data Tx LED pulse */
volatile uint8_t RxLEDPulse; /**< Milliseconds remaining for data Rx LED pulse */
static char isRemoteWakeUpEnabled = 0;
static char isEndpointHalt = 0;
//==================================================================
//==================================================================

extern const uint16_t STRING_LANGUAGE[];
extern const uint8_t STRING_PRODUCT[];
extern const uint8_t STRING_MANUFACTURER[];
extern const DeviceDescriptor USB_DeviceDescriptor;
extern const DeviceDescriptor USB_DeviceDescriptorA;

const uint16_t STRING_LANGUAGE[2] = {
	(3<<8) | (2+2),
	0x0409	// English
};


#ifdef __SAM4S4A__
//Use Arduino Due instead of product from boards.txt
#undef USB_PRODUCT
#endif

#ifndef USB_PRODUCT
// Use a hardcoded product name if none is provided
#if USB_PID == USB_PID_DUE
#define USB_PRODUCT "USB Bow board" // daniel Arduino Due
#else
#define USB_PRODUCT "USB IO Board"
#endif
#endif

const uint8_t STRING_PRODUCT[] = USB_PRODUCT;

#if USB_VID == 0x2341
#  if defined(USB_MANUFACTURER)
#    undef USB_MANUFACTURER
#  endif
#  define USB_MANUFACTURER "Arduino LLC"
#elif !defined(USB_MANUFACTURER)
// Fall through to unknown if no manufacturer name was provided in a macro
#  define USB_MANUFACTURER "Unknown"
#endif

const uint8_t STRING_MANUFACTURER[12] = USB_MANUFACTURER;

#ifdef CDC_ENABLED
#define DEVICE_CLASS 0x02
#else
#define DEVICE_CLASS 0x00
#endif

//	DEVICE DESCRIPTOR
#ifdef CDC_ENABLED 
#ifdef __SAM4S4A__ //SAM3S modification
//Since IAD is present, the device should be class EF.
//see here https://msdn.microsoft.com/en-us/library/windows/hardware/ff540054(v=vs.85).aspx
const DeviceDescriptor USB_DeviceDescriptor =
	D_DEVICE(0xEF,0x02,0x01,64,USB_VID,USB_PID,0x100,IMANUFACTURER,IPRODUCT,0,1);
#else 
const DeviceDescriptor USB_DeviceDescriptor =
	D_DEVICE(0x00,0x00,0x00,64,USB_VID,USB_PID,0x100,IMANUFACTURER,IPRODUCT,0,1);
#endif //__SAM4S4A__
#else	
const DeviceDescriptor USB_DeviceDescriptor =
	D_DEVICE(0x00,0x00,0x00,64,USB_VID,USB_PID,0x100,IMANUFACTURER,IPRODUCT,0,1);
#endif //CDC_ENABLED

//SAM3S never saw this come into play. Only used if get configuration descriptor request has 8 byte length. 
//Not sure how or why that should invoke DEVICE_CLASS=0x02.
const DeviceDescriptor USB_DeviceDescriptorA =
	D_DEVICE(DEVICE_CLASS,0x00,0x00,64,USB_VID,USB_PID,0x100,IMANUFACTURER,IPRODUCT,0,1);

//SAM3S stalls the request for this data because it cannot run at high speed.
const DeviceDescriptor USB_DeviceQualifier =
	D_QUALIFIER(0x00,0x00,0x00,64,1);

//! 7.1.20 Test Mode Support
static const unsigned char test_packet_buffer[] = {
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,                // JKJKJKJK * 9
    0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,                     // JJKKJJKK * 8
    0xEE,0xEE,0xEE,0xEE,0xEE,0xEE,0xEE,0xEE,                     // JJJJKKKK * 8
    0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, // JJJJJJJKKKKKKK * 8
    0x7F,0xBF,0xDF,0xEF,0xF7,0xFB,0xFD,                          // JJJJJJJK * 8
    0xFC,0x7E,0xBF,0xDF,0xEF,0xF7,0xFB,0xFD,0x7E                 // {JKKKKKKK * 10}, JK
};

//==================================================================
//==================================================================

volatile uint32_t _usbConfiguration = 0;
volatile uint32_t _usbInitialized = 0;
uint32_t _usbSetInterface = 0;
uint32_t _cdcComposite = 0;

//==================================================================
//==================================================================

#define USB_RECV_TIMEOUT
class LockEP
{
	irqflags_t flags;
public:
	LockEP(uint32_t ep) : flags(cpu_irq_save())
	{
	}
	~LockEP()
	{
		cpu_irq_restore(flags);
	}
};

//	Number of bytes, assumes a rx endpoint
uint32_t USBD_Available(uint32_t ep)
{
	#ifndef __SAM4S4A__
	//SAM4S4A Handle diable/enable CDC_RX interrupts elsewhere as necessary 
	LockEP lock(ep);
	#endif
	return UDD_FifoByteCount(ep & 0xF);
}

//	Non Blocking receive
//	Return number of bytes read
uint32_t USBD_Recv(uint32_t ep, void* d, uint32_t len)
{
	if (!_usbConfiguration)
		return -1;
		
	
	#ifndef __SAM4S4A__
	//SAM4S4A Handle diable/enable CDC_RX interrupts elsewhere as necessary 
	LockEP lock(ep);
	#endif

	uint32_t n = UDD_FifoByteCount(ep & 0xF);
	len = min(n,len);
	n = len;
	uint8_t* dst = (uint8_t*)d;
	
	
	while (n--)
		*dst++ = UDD_Recv8(ep & 0xF);
	#ifdef __SAM4S4A__
	//SAM3S fifo byte count (RXBYTECNT) is not decremented by the hardware when fifo is read.
	//You should always check RXBYTECNT and pass it in as len to this function, or RXBYTECNT
	//will not be decremented, and the read will not be ack'd. You will have trouble otherwise 
	//because 1) there will be less data than RXBYTECNT remaining in the fifo, and 2) the fifo 
	//will be blocked because the read has not been ACK'd.
	if(len==UDD_FifoByteCount(ep)) //release empty buffer
		UDD_ReleaseRX(ep);
	#else
	if (len && !UDD_FifoByteCount(ep & 0xF)) // release empty buffer
		UDD_ReleaseRX(ep & 0xF);
	#endif //SAM4S4A
	return len;
}


//SAM3S does not decrement fifo byte count (RXBYTECNT) in hardware when the fifo is read.
//So reading one byte and hoping for a decrement is asking for trouble. It will only work
//if there is exactly one byte in the fifo. 
//	Recv 1 byte if ready
uint32_t USBD_Recv(uint32_t ep)
{
	uint8_t c;
	if (USBD_Recv(ep & 0xF, &c, 1) != 1)
		return -1;
	else
		return c;
}


//	Space in send EP
//uint32_t USBD_SendSpace(uint32_t ep)
//{
	//LockEP lock(ep);
////	if (!UDD_ReadWriteAllowed(ep & 0xF))
    ////{
        ////printf("pb "); // UOTGHS->UOTGHS_DEVEPTISR[%d]=0x%X\n\r", ep, UOTGHS->UOTGHS_DEVEPTISR[ep]);
		////return 0;
    ////}

    //if(ep==0) return 64 - UDD_FifoByteCount(ep & 0xF);  // EP0_SIZE  jcb
    //else return 512 - UDD_FifoByteCount(ep & 0xF);  // EPX_SIZE  jcb
//}

//	Blocking Send of data to an endpoint
uint32_t USBD_Send(uint32_t ep, const void* d, uint32_t len)
{
    uint32_t n;
	int r = len;
	const uint8_t* data = (const uint8_t*)d;
	
#ifdef __SAM4S4A__
	//SAM3S employs this handy routine to send to EP0 during setup.
    if ((!_usbConfiguration) && (ep!=EP0))
    {
    	TRACE_CORE(printf("pb conf\n\r");)
		return -1;
    }
#else
    if (!_usbConfiguration)
    {
    	TRACE_CORE(printf("pb conf\n\r");)
		return -1;
    }
#endif

	while (len)
	{
		#ifdef __SAM4S4A__
		//All endpoints (except ISO) are 64 on SAM3S and we are not using ISO here.
		n=64;
		#else
        if(ep==0) n = EP0_SIZE;
        else n =  EPX_SIZE;
		#endif
		if (n > len)
			n = len;
		len -= n;
		UDD_Send(ep & 0xF, data, n);
		data += n;
    }
	

	
	//TXLED1;					// light the TX LED
	//TxLEDPulse = TX_RX_LED_PULSE_MS;
	return r;
}

//SAM3S: I hate these globals and seek to avoid them. If any EP0
//transfers are more than 64 bytes, they MUST be avoided because I am 
//not using UDD_Send to queue up data like the original did.
int _cmark;
int _cend;

void USBD_InitControl(int end)
{
	_cmark = 0;
	_cend = end;
}

//	Clipped by _cmark/_cend
int USBD_SendControl(uint8_t flags, const void* d, uint32_t len)
{
	const uint8_t* data = (const uint8_t*)d;
	uint32_t length = len;
	uint32_t sent = 0;
	uint32_t pos = 0;

	TRACE_CORE(printf("=> USBD_SendControl TOTAL len=%lu\r\n", len);)

	if (_cmark < _cend)
	{
		while (len > 0)
		{
			sent = UDD_Send(EP0, data + pos, len);
			TRACE_CORE(printf("=> USBD_SendControl sent=%lu\r\n", sent);)
			pos += sent;
			len -= sent;
		}
	}
	_cmark += length;
	return length;
}

// Send a USB descriptor string. The string is stored as a
// plain ASCII string but is sent out as UTF-16 with the
// correct 2-byte prefix
static bool USB_SendStringDescriptor(const uint8_t *string, int wLength) {
	uint16_t buff[64];
	int l = 1;
	wLength-=2;
	while (*string && wLength>0) {
		buff[l++] = (uint8_t)(*string++);
		wLength-=2;
	}
	buff[0] = (3<<8) | (l*2);
	return USBD_SendControl(0, (uint8_t*)buff, l*2);
}

//	Does not timeout or cross fifo boundaries
//	Will only work for transfers <= 64 bytes
//	TODO
int USBD_RecvControl(void* d, uint32_t len)
{
	UDD_WaitOUT();
	UDD_Recv(EP0, (uint8_t*)d, len);
	UDD_ClearOUT();

	return len;
}

//	Handle CLASS_INTERFACE requests
bool USBD_ClassInterfaceRequest(Setup& setup)
{
	uint8_t i = setup.wIndex;

	TRACE_CORE(printf("=> USBD_ClassInterfaceRequest\r\n");)

#ifdef CDC_ENABLED
	if (CDC_ACM_INTERFACE == i)
	{
		return CDC_Setup(setup);
	}
#endif

#ifdef HID_ENABLED
	if (HID_INTERFACE == i)
	{
		return HID_Setup(setup);
	}
#endif

	return false;
}

int USBD_SendInterfaces(void)
{
	int total = 0;
	uint8_t interfaces = 0;

#ifdef CDC_ENABLED
	total = CDC_GetInterface(&interfaces);
#endif

#ifdef HID_ENABLED
	total += HID_GetInterface(&interfaces);
#endif

	total = total; // Get rid of compiler warning
	TRACE_CORE(printf("=> USBD_SendInterfaces, total=%d interfaces=%d\r\n", total, interfaces);)
	return interfaces;
}

int USBD_SendOtherInterfaces(void)
{
	int total = 0;
	uint8_t interfaces = 0;

#ifdef CDC_ENABLED
	total = CDC_GetOtherInterface(&interfaces);
#endif

#ifdef HID_ENABLED
	total += HID_GetInterface(&interfaces);
#endif

	total = total; // Get rid of compiler warning
	TRACE_CORE(printf("=> USBD_SendInterfaces, total=%d interfaces=%d\r\n", total, interfaces);)
	return interfaces;
}

//	Construct a dynamic configuration descriptor
//	This really needs dynamic endpoint allocation etc
//	TODO
static bool USBD_SendConfiguration(int maxlen)
{
#ifdef __SAM4S4A__
	//Since configuration is greater than 64 bytes, and UDD_Send is no longer queuing, 
	//SAM3S uses completely different (and much simpler) means of getting the descriptor.
	//I did have to make an assumption about the upper limit on descriptor size though.
	//See if you can follow the original Arduino code after the #else. It's crazy.
	int len;
	uint8_t config[512]; //not sure how big a config descriptor can be (CDC is 75)
	len = USBD_GetConfiguration((uint8_t*)&config,maxlen);
	USBD_Send(EP0, (uint8_t*)&config, len);
#else 

	//	Count and measure interfaces
	USBD_InitControl(0);
	//TRACE_CORE(printf("=> USBD_SendConfiguration _cmark1=%d\r\n", _cmark);)
	int interfaces = USBD_SendInterfaces();
	//TRACE_CORE(printf("=> USBD_SendConfiguration _cmark2=%d\r\n", _cmark);)
	//TRACE_CORE(printf("=> USBD_SendConfiguration sizeof=%d\r\n", sizeof(ConfigDescriptor));)

_Pragma("pack(1)")
	ConfigDescriptor config = D_CONFIG(_cmark + sizeof(ConfigDescriptor),interfaces);
_Pragma("pack()")
	//TRACE_CORE(printf("=> USBD_SendConfiguration clen=%d\r\n", config.clen);)

	//TRACE_CORE(printf("=> USBD_SendConfiguration maxlen=%d\r\n", maxlen);)

	//	Now send them
	USBD_InitControl(maxlen);
	USBD_SendControl(0,&config,sizeof(ConfigDescriptor));
	USBD_SendInterfaces();
#endif //__SAM4S4A__
	return true;
}

static bool USBD_SendOtherConfiguration(int maxlen)
{
	//	Count and measure interfaces
	USBD_InitControl(0);
	//TRACE_CORE(printf("=> USBD_SendConfiguration _cmark1=%d\r\n", _cmark);)
	int interfaces = USBD_SendOtherInterfaces();
	//TRACE_CORE(printf("=> USBD_SendConfiguration _cmark2=%d\r\n", _cmark);)
	//TRACE_CORE(printf("=> USBD_SendConfiguration sizeof=%d\r\n", sizeof(ConfigDescriptor));)

_Pragma("pack(1)")
	ConfigDescriptor config = D_OTHERCONFIG(_cmark + sizeof(ConfigDescriptor),interfaces);
_Pragma("pack()")
	//TRACE_CORE(printf("=> USBD_SendConfiguration clen=%d\r\n", config.clen);)

	//TRACE_CORE(printf("=> USBD_SendConfiguration maxlen=%d\r\n", maxlen);)

	//	Now send them
	USBD_InitControl(maxlen);
	USBD_SendControl(0,&config,sizeof(ConfigDescriptor));
	USBD_SendOtherInterfaces();
	return true;
}

static bool USBD_SendDescriptor(Setup& setup)
{
	uint8_t t = setup.wValueH;
	uint8_t desc_length = 0;
	const uint8_t* desc_addr = 0;
	

	if (USB_CONFIGURATION_DESCRIPTOR_TYPE == t)
	{

		TRACE_CORE(printf("=> USBD_SendDescriptor : USB_CONFIGURATION_DESCRIPTOR_TYPE length=%d\r\n", setup.wLength);)
		return USBD_SendConfiguration(setup.wLength);
		
	}

	USBD_InitControl(setup.wLength);
#ifdef HID_ENABLED
	if (HID_REPORT_DESCRIPTOR_TYPE == t)
	{
		TRACE_CORE(puts("=> USBD_SendDescriptor : HID_REPORT_DESCRIPTOR_TYPE\r\n");)
		return HID_GetDescriptor(t);
	}
#endif

	if (USB_DEVICE_DESCRIPTOR_TYPE == t)
	{
		TRACE_CORE(puts("=> USBD_SendDescriptor : USB_DEVICE_DESCRIPTOR_TYPE\r\n");)
		if (setup.wLength == 8)
		{
			_cdcComposite = 1;
		}
		desc_addr = _cdcComposite ?  (const uint8_t*)&USB_DeviceDescriptorA : (const uint8_t*)&USB_DeviceDescriptor;
        if( *desc_addr > setup.wLength ) {
            desc_length = setup.wLength;
        }
	
	}
	else if (USB_STRING_DESCRIPTOR_TYPE == t)
	{

		TRACE_CORE(puts("=> USBD_SendDescriptor : USB_STRING_DESCRIPTOR_TYPE\r\n");)
		if (setup.wValueL == 0) {
			desc_addr = (const uint8_t*)&STRING_LANGUAGE;
		}
		else if (setup.wValueL == IPRODUCT) {
			return USB_SendStringDescriptor(STRING_PRODUCT, setup.wLength);
		}
		else if (setup.wValueL == IMANUFACTURER) {
			return USB_SendStringDescriptor(STRING_MANUFACTURER, setup.wLength);
		}
		else {
			return false;
		}
		if( *desc_addr > setup.wLength ) {
			desc_length = setup.wLength;
		}
	}
	else if (USB_DEVICE_QUALIFIER == t)
	{
#ifdef __SAM4S4A__
		//Is this a request to see if we can run at high speed? YES. 
		//SAM3S is full speed only and must STALL this.
		UDD_Stall();
		return true;
#else
		// Device qualifier descriptor requested
		desc_addr = (const uint8_t*)&USB_DeviceQualifier;
        if( *desc_addr > setup.wLength ) {
            desc_length = setup.wLength;
        }
#endif
    }
    else if (USB_OTHER_SPEED_CONFIGURATION == t)
    {
		// Other configuration descriptor requested
		return USBD_SendOtherConfiguration(setup.wLength);
    }
    else
    {
        //printf("Device ERROR");
    }

	if (desc_addr == 0)
	{
		return false;
	}

	if (desc_length == 0)
	{
		desc_length = *desc_addr;
	}
	
	TRACE_CORE(printf("=> USBD_SendDescriptor : desc_addr=%p desc_length=%d\r\n", desc_addr, desc_length);)
	USBD_SendControl(0, desc_addr, desc_length);

	return true;
}


static void USB_SendZlp( void )
{
	#ifdef __SAM4S4A__
	//Send zero length packet.
	//while(Is_udd_transmit_ready(EP0)){
	if(Is_udd_transmit_ready(EP0)) {
			while(!Is_udd_in_sent(EP0) || Is_udd_transmit_ready(EP0)) {
				if(Is_udd_suspend()) {
					return;
			}
		}
	}

	udd_set_transmit_ready(EP0);
	while(!Is_udd_in_sent(EP0)) {}
	udd_ack_in_sent(EP0);
	#else
    while( UOTGHS_DEVEPTISR_TXINI != (UOTGHS->UOTGHS_DEVEPTISR[0] & UOTGHS_DEVEPTISR_TXINI ) )
    {
        if((UOTGHS->UOTGHS_DEVISR & UOTGHS_DEVISR_SUSP) == UOTGHS_DEVISR_SUSP)
        {
            return;
        }
    }
    UOTGHS->UOTGHS_DEVEPTICR[0] = UOTGHS_DEVEPTICR_TXINIC;
	#endif //__SAM4S4A__
}

#ifndef __SAM4S4A__ //High speed not avail on SAM3S
static void Test_Mode_Support( uint8_t wIndex )
{
    uint8_t i;
	uint8_t *ptr_dest = (uint8_t *) &udd_get_endpoint_fifo_access8(2);

	switch( wIndex )
	{
		case 4:
			//Test mode Test_Packet:
			//Upon command, a port must repetitively transmit the following test packet until
			//the exit action is taken. This enables the testing of rise and fall times, eye
			//patterns, jitter, and any other dynamic waveform specifications.
			//The test packet is made up by concatenating the following strings.
			//(Note: For J/K NRZI data, and for NRZ data, the bit on the left is the first one
			//transmitted. "S" indicates that a bit stuff occurs, which inserts an "extra" NRZI data bit.
			//"* N" is used to indicate N occurrences of a string of bits or symbols.)
			//A port in Test_Packet mode must send this packet repetitively. The inter-packet timing
			//must be no less than the minimum allowable inter-packet gap as defined in Section 7.1.18 and
			//no greater than 125 us.

			// Send ZLP
			USB_SendZlp();

			UOTGHS->UOTGHS_DEVDMA[0].UOTGHS_DEVDMACONTROL = 0; // raz
			UOTGHS->UOTGHS_DEVDMA[1].UOTGHS_DEVDMACONTROL = 0; // raz

			// Configure endpoint 2, 64 bytes, direction IN, type BULK, 1 bank
			UOTGHS->UOTGHS_DEVEPTCFG[2] = UOTGHS_DEVEPTCFG_EPSIZE_64_BYTE
												 | UOTGHS_DEVEPTCFG_EPDIR_IN
												 | UOTGHS_DEVEPTCFG_EPTYPE_BLK
												 | UOTGHS_DEVEPTCFG_EPBK_1_BANK;
			// Check if the configuration is ok
			UOTGHS->UOTGHS_DEVEPTCFG[2] |= UOTGHS_DEVEPTCFG_ALLOC;
			while((UOTGHS->UOTGHS_DEVEPTISR[2]&UOTGHS_DEVEPTISR_CFGOK)==0) {}
			UOTGHS->UOTGHS_DEVEPT |= UOTGHS_DEVEPT_EPEN2;
			// Write FIFO
			for( i=0; i<sizeof(test_packet_buffer); i++)
			{
				ptr_dest[i] = test_packet_buffer[i];;
			}
			// Tst PACKET
			UOTGHS->UOTGHS_DEVCTRL |= UOTGHS_DEVCTRL_TSTPCKT;
			// Send packet
			UOTGHS->UOTGHS_DEVEPTICR[2] = UOTGHS_DEVEPTICR_TXINIC;
			UOTGHS->UOTGHS_DEVEPTIDR[2] = UOTGHS_DEVEPTIDR_FIFOCONC;
			for(;;);
//      break;

		case 1:
			//Test mode Test_J:
			//Upon command, a port's transceiver must enter the high-speed J state and remain in that
			//state until the exit action is taken. This enables the testing of the high output drive
			//level on the D+ line.
			// Send a ZLP
			USB_SendZlp();
			UOTGHS->UOTGHS_DEVCTRL |= UOTGHS_DEVCTRL_TSTJ;
			for(;;);
//      break;

		case 2:
			//Test mode Test_K:
			//Upon command, a port's transceiver must enter the high-speed K state and remain in
			//that state until the exit action is taken. This enables the testing of the high output drive
			//level on the D- line.
			// Send a ZLP
			USB_SendZlp();
			UOTGHS->UOTGHS_DEVCTRL |= UOTGHS_DEVCTRL_TSTK;
			for(;;);
//		break;

		case 3:
			//Test mode Test_SE0_NAK:
			//Upon command, a port's transceiver must enter the high-speed receive mode
			//and remain in that mode until the exit action is taken. This enables the testing
			//of output impedance, low level output voltage, and loading characteristics.
			//In addition, while in this mode, upstream facing ports (and only upstream facing ports)
			//must respond to any IN token packet with a NAK handshake (only if the packet CRC is
			//determined to be correct) within the normal allowed device response time. This enables testing of
			//the device squelch level circuitry and, additionally, provides a general purpose stimulus/response
			//test for basic functional testing.

			// Send a ZLP
			USB_SendZlp();
			UOTGHS->UOTGHS_DEVIDR = UOTGHS_DEVIDR_SUSPEC
							   | UOTGHS_DEVIDR_MSOFEC
							   | UOTGHS_DEVIDR_SOFEC
							   | UOTGHS_DEVIDR_EORSTEC
							   | UOTGHS_DEVIDR_WAKEUPEC
							   | UOTGHS_DEVIDR_EORSMEC
							   | UOTGHS_DEVIDR_UPRSMEC
							   | UOTGHS_DEVIDR_PEP_0
							   | UOTGHS_DEVIDR_PEP_1
							   | UOTGHS_DEVIDR_PEP_2
							   | UOTGHS_DEVIDR_PEP_3
							   | UOTGHS_DEVIDR_PEP_4
							   | UOTGHS_DEVIDR_PEP_5
							   | UOTGHS_DEVIDR_PEP_6
							   | UOTGHS_DEVIDR_DMA_1
							   | UOTGHS_DEVIDR_DMA_2
							   | UOTGHS_DEVIDR_DMA_3
							   | UOTGHS_DEVIDR_DMA_4
							   | UOTGHS_DEVIDR_DMA_5
							   | UOTGHS_DEVIDR_DMA_6;
			for(;;);
//		break;
	}
}
#endif //__SAM4S4A__


//unsigned int iii=0;
//	Endpoint 0 interrupt
static void USB_ISR(void)
{


#ifdef __SAM4S4A__
	//saw these in testing, and will retrigger if not ack'd
	if (Is_udd_suspend()){
		udd_ack_suspend();
	}
	if (Is_udd_any_wakeup()){
		udd_ack_wakeups();
	}

#ifdef CDC_ENABLED	
	if (Is_udd_endpoint_interrupt(CDC_TX)){
		if (Is_udd_in_sent(CDC_TX)) { 
			//CDC_TX is dual bank. Ack and decrement bank status.
			udd_ack_in_sent(CDC_TX);
			cdc_tx_banks_idle=true;
		}
	}

#endif //CDC_ENABLED

#endif //SAM4S4A

//    printf("ISR=0x%X\n\r", UOTGHS->UOTGHS_DEVISR); // jcb
//    if( iii++ > 1500 ) while(1); // jcb
    // End of bus reset
    if (Is_udd_reset())
    {
		TRACE_CORE(printf(">>> End of Reset\r\n");)

		// Reset USB address to 0
		udd_configure_address(0);
		udd_enable_address();

		// Configure EP 0
        UDD_InitEP(0, EP_TYPE_CONTROL);
		#ifndef __SAM4S4A__
		//not available or necessary on SAM3S
		udd_enable_setup_received_interrupt(0);
		#endif
		
		udd_enable_endpoint_interrupt(0);

        _usbConfiguration = 0;
		udd_ack_reset();
    }

#ifdef CDC_ENABLED
  	if (Is_udd_endpoint_interrupt(CDC_RX))
	{
		#ifndef __SAM4S4A__
		udd_ack_out_received(CDC_RX);
		// Handle received bytes
		if (USBD_Available(CDC_RX))
			SerialUSB.accept();
		#else
		// Handle received bytes and disable interrupt so lower priority Serial_::read() can read buffer
		if (USBD_Available(CDC_RX)) {
			udd_disable_endpoint_interrupt(CDC_RX);
			SerialUSB.accept();
		}
		#endif //SAM4S4A
	}

	if (Is_udd_sof())
	{
		udd_ack_sof();
	//	USBD_Flush(CDC_TX); // jcb
	}
#endif

	// EP 0 Interrupt
	if (Is_udd_endpoint_interrupt(0) )
	{
		if (!UDD_ReceivedSetupInt())
		{
#ifdef __SAM4S4A__
			//EP0 is not dual bank, always uses bank0
			if(Is_udd_bank0_received(EP0) && udd_byte_count(EP0)==0){
				//Host sent ZLP in Status Stage to ACK receipt of IN data. This ACKs ZLP.
				udd_ack_bank0_received(EP0);
			}
#endif
			return;
		}
		Setup setup;
		UDD_Recv(EP0, (uint8_t*)&setup, 8);

#ifndef __SAM4S4A__
		UDD_ClearSetupInt();
		uint8_t requestType = setup.bmRequestType;
#else
		//SAM3S datasheet page 929 says dir must be set before RXSETUP is cleared
		uint8_t requestType = setup.bmRequestType;
		if (requestType & REQUEST_DEVICETOHOST){
			udd_configure_endpoint_direction(EP0, 1);
		} else {
			udd_configure_endpoint_direction(EP0, 0);
		}	
		UDD_ClearSetupInt();
#endif //__SAM4S4A__
		if (requestType & REQUEST_DEVICETOHOST)
		{
			TRACE_CORE(puts(">>> EP0 Int: IN Request\r\n");)
			UDD_WaitIN(); //Wait for any previous IN transactions to complete?
		}
		else
		{
			TRACE_CORE(puts(">>> EP0 Int: OUT Request\r\n");)
			#ifdef __SAM4S4A__
			//The only time we want to run UDD_ClearIN before processing OUT tokens is in the SET_ADDRESS case.
			//So we call UDD_ClearIN within UDD_SetAddress instead of here.
			#else
			UDD_ClearIN(); 
			#endif
		}

		bool ok = true;
		if (REQUEST_STANDARD == (requestType & REQUEST_TYPE))
		{
			// Standard Requests
			uint8_t r = setup.bRequest;
			if (GET_STATUS == r)
			{
                if( setup.bmRequestType == 0 )  // device
                {
                    // Send the device status
     				TRACE_CORE(puts(">>> EP0 Int: GET_STATUS\r\n");)
                    // Check current configuration for power mode (if device is configured)
                    // TODO
                    // Check if remote wake-up is enabled
                    // TODO
                    UDD_Send8(EP0, 0); // TODO
	    			UDD_Send8(EP0, 0);
                }
                // if( setup.bmRequestType == 2 ) // Endpoint:
                else
                {
                    // Send the endpoint status
                    // Check if the endpoint if currently halted
                    if( isEndpointHalt == 1 )
    				UDD_Send8(EP0, 1); // TODO
                    else
    				UDD_Send8(EP0, 0); // TODO
	    			UDD_Send8(EP0, 0);
                }
			}
			else if (CLEAR_FEATURE == r)
			{
               // Check which is the selected feature
                if( setup.wValueL == 1) // DEVICEREMOTEWAKEUP
                {
                    // Enable remote wake-up and send a ZLP
                    if( isRemoteWakeUpEnabled == 1 )
	    			UDD_Send8(EP0, 1);
                    else
	    			UDD_Send8(EP0, 0);
                    UDD_Send8(EP0, 0);
                }
                else // if( setup.wValueL == 0) // ENDPOINTHALT
                {
                    isEndpointHalt = 0;  // TODO
    				UDD_Send8(EP0, 0);
	    			UDD_Send8(EP0, 0);
                }

 			}
			else if (SET_FEATURE == r)
			{
                // Check which is the selected feature
                if( setup.wValueL == 1) // DEVICEREMOTEWAKEUP
                {
                    // Enable remote wake-up and send a ZLP
                    isRemoteWakeUpEnabled = 1;
	    			UDD_Send8(EP0, 0);
                }
                if( setup.wValueL == 0) // ENDPOINTHALT
                {
                    // Halt endpoint
                    isEndpointHalt = 1;
                    //USBD_Halt(USBGenericRequest_GetEndpointNumber(pRequest));
	    			UDD_Send8(EP0, 0);
                }
				#ifndef __SAM4S4A__
				//TODO SAM3S should STALL this instead of ignore
                if( setup.wValueL == 2) // TEST_MODE
                {
                    // 7.1.20 Test Mode Support, 9.4.9 SetFeature
                    if( (setup.bmRequestType == 0 /*USBGenericRequest_DEVICE*/) &&
                        ((setup.wIndex & 0x000F) == 0) )
                    {
                        // the lower byte of wIndex must be zero
                        // the most significant byte of wIndex is used to specify the specific test mode

                        UOTGHS->UOTGHS_DEVIDR &= ~UOTGHS_DEVIDR_SUSPEC;
                        UOTGHS->UOTGHS_DEVCTRL |= UOTGHS_DEVCTRL_SPDCONF_HIGH_SPEED; // remove suspend ?

                        Test_Mode_Support( (setup.wIndex & 0xFF00)>>8 );
                    }
                }
				#endif //__SAM4S4A__
			}
			else if (SET_ADDRESS == r)
			{

				TRACE_CORE(puts(">>> EP0 Int: SET_ADDRESS\r\n");)
				UDD_WaitIN();
				UDD_SetAddress(setup.wValueL);
			}
			else if (GET_DESCRIPTOR == r)
			{
				TRACE_CORE(puts(">>> EP0 Int: GET_DESCRIPTOR\r\n");)
				ok = USBD_SendDescriptor(setup);
			}
			else if (SET_DESCRIPTOR == r)
			{
				TRACE_CORE(puts(">>> EP0 Int: SET_DESCRIPTOR\r\n");)
				ok = false;
			}
			else if (GET_CONFIGURATION == r)
			{
				TRACE_CORE(puts(">>> EP0 Int: GET_CONFIGURATION\r\n");)
				UDD_Send8(EP0, _usbConfiguration);
			}
			else if (SET_CONFIGURATION == r)
			{
				if (REQUEST_DEVICE == (requestType & REQUEST_RECIPIENT))
				{
					TRACE_CORE(printf(">>> EP0 Int: SET_CONFIGURATION REQUEST_DEVICE %d\r\n", setup.wValueL);)
					UDD_InitEndpoints(EndPoints, (sizeof(EndPoints) / sizeof(EndPoints[0])));
					_usbConfiguration = setup.wValueL;
					
#ifdef CDC_ENABLED
					
					// Enable interrupt for CDC reception from host (OUT packet)
					#ifndef __SAM4S4A__
					//not available or necessary for the SAM3S
					udd_enable_out_received_interrupt(CDC_RX);
					#else
					udd_enable_endpoint_interrupt(CDC_TX);
					#endif
					udd_enable_endpoint_interrupt(CDC_RX);
#endif
				}
				else
				{
					TRACE_CORE(puts(">>> EP0 Int: SET_CONFIGURATION failed!\r\n");)
					ok = false;
				}
			}
			else if (GET_INTERFACE == r)
			{
				TRACE_CORE(puts(">>> EP0 Int: GET_INTERFACE\r\n");)
				UDD_Send8(EP0, _usbSetInterface);
			}
			else if (SET_INTERFACE == r)
			{
                _usbSetInterface = setup.wValueL;
				TRACE_CORE(puts(">>> EP0 Int: SET_INTERFACE\r\n");)
			}
		}
		else
		{

			TRACE_CORE(puts(">>> EP0 Int: ClassInterfaceRequest\r\n");)
			UDD_WaitIN(); // Workaround: need tempo here, else CDC serial won't open correctly
			USBD_InitControl(setup.wLength); // Max length of transfer
			ok = USBD_ClassInterfaceRequest(setup);
		}

		if (ok)
		{
			TRACE_CORE(puts(">>> EP0 Int: Send packet\r\n");)
			#ifdef __SAM4S4A__	
			//ACK status stage after processing OUT tokens unless SET_ADDRESS
			if (!(requestType & REQUEST_DEVICETOHOST) && !(setup.bRequest==SET_ADDRESS)) 
			{
				UDD_ClearIN();
			}
			#else
			UDD_ClearIN();	
			#endif
		}
		else
		{
			TRACE_CORE(puts(">>> EP0 Int: Stall\r\n");)
			UDD_Stall();
		}
	}
}

void USBD_Flush(uint32_t ep)
{
	if (UDD_FifoByteCount(ep))
		UDD_ReleaseTX(ep);
}

//	VBUS or counting frames
//	Any frame counting?
uint32_t USBD_Connected(void)
{
	uint8_t f = UDD_GetFrameNumber();

	delay(3);

	return f != UDD_GetFrameNumber();
}


//=======================================================================
//=======================================================================

USBDevice_ USBDevice;

USBDevice_::USBDevice_()
{
	UDD_SetStack(&USB_ISR);

	if (UDD_Init() == 0UL)
	{
		_usbInitialized=1UL;
	}
}

bool USBDevice_::attach(void)
{
  if (_usbInitialized != 0UL)
  {
    UDD_Attach();
	_usbConfiguration = 0;
	return true;
  }
  else
  {
    return false;
  }
}

bool USBDevice_::detach(void)
{
	if (_usbInitialized != 0UL)
	{
		UDD_Detach();
		return true;
	}
	else
	{
		return false;
	}
}

//	Check for interrupts
//	TODO: VBUS detection
bool USBDevice_::configured()
{
	return _usbConfiguration;
}

void USBDevice_::poll()
{
}

#ifdef __SAM4S4A__
//This is a rewrite of all the functions in uotghs_device.c. Everything had to move from
//uotghs to udp because the SAM3S has a full speed USB transceiver. Might be best to get 
//these out of here and into their own file? Or into variable.cpp if possible.
uint32_t UDD_Send(uint32_t ep, const void* data, uint32_t len)
{
	const uint8_t *ptr_src = (const uint8_t* )data; //not sure why compiler complains without cast, although fine in uotghs_device.c.
	uint32_t i;
	uint32_t n;
	
	n=64; //All endpoints (except ISO) are 64 on SAM3S and we are not using ISO here.
	if (len > n)
		len = n; //Caller can make repeated calls, checking return val each time.

	//Note EP0 is single bank, CDC_TX is dual bank.
	if (ep==CDC_TX){
		//prevent interrupt from acking other bank before this bank is written
		udd_disable_endpoint_interrupt(ep);
		//CDC_TX is dual bank. Only wait if thought to be idle.
		if ((cdc_tx_banks_idle)) { 
			if(Is_udd_transmit_ready(ep)) {
				while(!Is_udd_in_sent(ep) || Is_udd_transmit_ready(ep)) {}
				udd_ack_in_sent(ep);
			} else if (Is_udd_in_sent(ep)){
				udd_ack_in_sent(ep);
			}
		}
		 
	} else {
		//default to single bank operation (always works)
		if(Is_udd_transmit_ready(ep)) {
			while(!Is_udd_in_sent(ep) || Is_udd_transmit_ready(ep)) {}
			udd_ack_in_sent(ep);
		} else if (Is_udd_in_sent(ep)){
			udd_ack_in_sent(ep);
		}
	}

	//write to the bank
	for (i = 0; i < len; ++i)
		udd_endpoint_fifo_write(ep & 0x7, *ptr_src++);
		
	
	if (ep==CDC_TX) {
		if (cdc_tx_banks_idle) {
			//if banks are idle, don't wait for TXPKTRDY to clear
		} else {
			//Other bank is busy. Wait for TXPKTRDY to clear on it, then set TXPKTRDY on the current bank
			if(Is_udd_transmit_ready(ep)) {
				while(!Is_udd_in_sent(ep) || Is_udd_transmit_ready(ep)) {}
				udd_ack_in_sent(ep);
			} else if (Is_udd_in_sent(ep)){
				udd_ack_in_sent(ep);
			}
		}
		udd_set_transmit_ready(ep);
		cdc_tx_banks_idle=false;
		//listen for transition back to idle
		udd_enable_endpoint_interrupt(ep);
	} else {
		//Default to single bank. Always works
		udd_set_transmit_ready(ep);
		while(!Is_udd_in_sent(ep)) {}
		udd_ack_in_sent(ep & 0x7);
	}

	return len;
}

void UDD_Recv(uint32_t ep, uint8_t* data, uint32_t len)
{
	uint8_t *ptr_dest = data;
	uint32_t i;

	for (i = 0; i < len; ++i)
		*ptr_dest++ = udd_endpoint_fifo_read(ep & 0x7);
}

uint8_t UDD_Recv8(uint32_t ep)
{
	return udd_endpoint_fifo_read(ep & 0x7);
}

uint32_t UDD_ReceivedSetupInt(void)
{
	return Is_udd_setup_received(EP0);
}

void UDD_ClearSetupInt(void)
{
	udd_ack_setup_received(EP0);
}

void UDD_WaitIN(void)
{
	if(Is_udd_transmit_ready(EP0)) {
		while(!Is_udd_in_sent(EP0) || Is_udd_transmit_ready(EP0)) {}
		udd_ack_in_sent(EP0);
	} else if (Is_udd_in_sent(EP0)){
		udd_ack_in_sent(EP0);
	}
}

void UDD_WaitOUT(void)
{
	//this is always called for EP0, so single bank ok
	while (!Is_udd_bank0_received(EP0))
		;
}

void UDD_ClearOUT(void)
{
	//this is always called for EP0, so single bank ok
	udd_ack_bank0_received(EP0);
}

void UDD_ClearIN(void)
{
	udd_set_transmit_ready(EP0);
	while(!Is_udd_in_sent(EP0)) {}
	udd_ack_in_sent(EP0);
}

void UDD_Send8(uint32_t ep,  uint8_t data )
{
	//while (Is_udd_transmit_ready(EP0))
	//	;
	if(Is_udd_transmit_ready(ep)) {
		while(!Is_udd_in_sent(ep) || Is_udd_transmit_ready(ep)) {}
		udd_ack_in_sent(ep);
	} else if (Is_udd_in_sent(ep)){
		udd_ack_in_sent(ep);
	}
	udd_endpoint_fifo_write(ep & 0x7, data);
	udd_set_transmit_ready(ep & 0x7);
	while(!Is_udd_in_sent(ep)) {}
	udd_ack_in_sent(ep & 0x7);
}

void UDD_SetAddress(uint32_t addr)
{
	//page 911 of SAM3S datasheet says TXCOMP must be set 
	//by HW and cleared by FW before address is set.
	//To make this happen a ZLP has to be sent. This 
	//ZLP acks the status packet that followed the 
	//address OUT packet.
	UDD_ClearIN();
	//Now set the address
	udd_configure_address(addr);
	udd_enable_address();
	udd_enable_address_state();
}

void UDD_InitEP( uint32_t ul_ep_nb, uint32_t ul_ep_cfg )
{
	ul_ep_nb = ul_ep_nb & 0x7; // EP range is 0..7, hence mask is 0x7.

	udd_reset_endpoint(ul_ep_nb); //necessary? 
	// Configure EP
	udd_configure_endpoint_type(ul_ep_nb, ul_ep_cfg);
	// Enable EP
	udd_enable_endpoint(ul_ep_nb);
	
	//Pretty crazy to put this in a generic InitEP function, 
	//but ok since this function is only called during reset to init EP0.
	udd_disable_address_state();
	udd_disable_configured_state();
}

void UDD_InitEndpoints(const uint32_t* eps_table, const uint32_t ul_eps_table_size)
{
	//Completely ignore SAM3X specific eps table and just enable endpoints as defined in USBDesc.h
#ifdef CDC_ENABLED
	udd_reset_endpoint(CDC_ENDPOINT_ACM);
	udd_reset_endpoint(CDC_ENDPOINT_OUT);
	udd_reset_endpoint(CDC_ENDPOINT_IN);	
	
	udd_configure_endpoint_type(CDC_ENDPOINT_ACM,UDP_CSR_EPTYPE_INT_IN);
	udd_configure_endpoint_type(CDC_ENDPOINT_OUT,UDP_CSR_EPTYPE_BULK_OUT);
	udd_configure_endpoint_type(CDC_ENDPOINT_IN,UDP_CSR_EPTYPE_BULK_IN);

	udd_enable_endpoint(CDC_ENDPOINT_ACM);
	udd_enable_endpoint(CDC_ENDPOINT_OUT);
	udd_enable_endpoint(CDC_ENDPOINT_IN);	
	cdc_tx_banks_idle=true; //true if both tx banks are idle.
	cdc_rx_bank_number=0; //track which bank to ack
#endif	

}

uint32_t UDD_FifoByteCount(uint32_t ep)
{
	return udd_byte_count(ep & 0x7);
}


void UDD_ReleaseRX(uint32_t ep)
{
	if (ep==EP0){
		//EP0 is single bank
		udd_ack_bank0_received(ep);
	} else if (ep==CDC_RX) {
		//CDC_RX is dual bank... need to track
		if (Is_udd_all_banks_received(ep)) {
			// The only way is to use cdc_rx_bank_number
		} else if (Is_udd_bank0_received(ep)) {
			// Must be bank0
			cdc_rx_bank_number = 0;
		} else {
			// Must be bank1
			cdc_rx_bank_number = 1;
		}
		if (cdc_rx_bank_number == 0) {
			udd_ack_bank0_received(ep);
			if (udd_get_endpoint_bank_max_nbr(ep) > 1) {
				cdc_rx_bank_number = 1;
			}
		} else {
			udd_ack_bank1_received(ep);
			cdc_rx_bank_number = 0;
		}	
	}	
}

void UDD_Stall(void)
{
	//udd_enable_endpoint(EP0);  
	udd_enable_stall_handshake(EP0);
	while(!Is_udd_stall(EP0)) {}
	udd_disable_stall_handshake(EP0); //clear FORCESTALL
	udd_ack_stall(EP0); //clear STALLSENT
}

uint32_t UDD_GetFrameNumber(void)
{
	return udd_frame_number();
}

uint32_t UDD_Init(void)
{
	doug=0;
	irqflags_t flags;
	flags = cpu_irq_save();
	// Enables the USB Clock
	udd_enable_periph_ck();
	// ===== Target frequency (USB Clock)
	// - USB clock source: PLLB
	// - USB clock divider: 2 (divided by 2)
	// - PLLB output: XTAL * 16 / 2
	// - USB clock: 12 * 16 / 2 / 2 = 48MHz
	pmc_enable_pllbck(15, 48, 2); //mult-1,startup count,div
	pmc_switch_udpck_to_pllbck(1); //div-1.
	pmc_enable_udpck();
	// Cortex, uses NVIC, no need to register IRQ handler
	NVIC_SetPriority((IRQn_Type) ID_UDP, UDD_USB_INT_LEVEL);
	NVIC_EnableIRQ((IRQn_Type) ID_UDP);

	// Always authorize asynchronous USB interrupts to exit of sleep mode
	pmc_set_fast_startup_input(PMC_FSMR_USBAL);
	
	// Enable USB hardware
	udd_disable_transceiver();
	udd_enable_transceiver();
	cpu_irq_restore(flags);

	return 0UL ;
}

//void UDP_Handler() {
//	USB_ISR();
//}

void UDD_SetStack(void (*pf_isr)(void))
{
	gpf_isr = pf_isr;
}

void UDD_ReleaseTX(uint32_t ep)
{
	udd_set_transmit_ready(ep & 0x7);
	while(!Is_udd_in_sent(ep)) {}
	udd_ack_in_sent(ep & 0x7);
}

void UDD_Attach(void)
{

	irqflags_t flags = cpu_irq_save();
	udd_enable_periph_ck();
	
	// Authorize attach if VBus is present
	udd_enable_transceiver();
	udd_attach_device();

	// Enable USB line events
	udd_enable_endpoint_interrupt(EP0);
	//udd_enable_sof_interrupt();
	//udd_enable_suspend_interrupt();
	//udd_enable_wake_up_interrupt();
	//udd_enable_resume_interrupt();
	//udd_enable_ext_resume_interrupt();

	cpu_irq_restore(flags);
}

void UDD_Detach(void)
{
	udd_detach_device();
}

//This routine assembles the configuration descriptor.
//Descriptor data is returned at address of data with 
//length returned as integer return value.
int USBD_GetConfiguration(uint8_t* data,uint32_t len)
{

	uint32_t total_length = 0;
	uint32_t num_interfaces = 0;
	uint32_t i;
	uint32_t j;
	uint8_t* src_data;

	total_length=sizeof(ConfigDescriptor);
#ifdef CDC_ENABLED
	total_length=total_length + sizeof(CDCDescriptor);
	const CDCDescriptor* cdc_ptr = CDC_GetDescriptor();
	IADDescriptor iad = (*cdc_ptr).iad;
	num_interfaces = num_interfaces + iad.interfaceCount;
#endif	
	
	
_Pragma("pack(1)")
	ConfigDescriptor config = D_CONFIG(total_length,num_interfaces);
_Pragma("pack()")


	//clip data by len
	if (len > total_length) {
		len = total_length;
	}
	
	//copy the config header first
	src_data = (uint8_t *) &config;
	for (i = 0; (i < sizeof(ConfigDescriptor)) && (i < len); ++i)
		*data++ = *src_data++;
	j=i;


#ifdef CDC_ENABLED		
	//copy the CDC data 
	src_data = (uint8_t *)cdc_ptr;	
	for (i = j; (i < (sizeof(CDCDescriptor) + j)) && (i < len); ++i)
		*data++ = *src_data++;
	j=i;//this is here in case more interfaces (like HID) are added in the future.
#endif
	
	return len; //this is the length at *data
}

#endif //__SAM4S4A__




