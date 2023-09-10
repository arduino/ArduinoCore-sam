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

#ifndef __USBCORE_H__
#define __USBCORE_H__

//	Standard requests
#define GET_STATUS					0
#define CLEAR_FEATURE				1
#define SET_FEATURE					3
#define SET_ADDRESS					5
#define GET_DESCRIPTOR				6
#define SET_DESCRIPTOR				7
#define GET_CONFIGURATION			8
#define SET_CONFIGURATION			9
#define GET_INTERFACE				10
#define SET_INTERFACE				11


// bmRequestType
#define REQUEST_HOSTTODEVICE		0x00
#define REQUEST_DEVICETOHOST		0x80
#define REQUEST_DIRECTION			0x80

#define REQUEST_STANDARD			0x00
#define REQUEST_CLASS				0x20
#define REQUEST_VENDOR				0x40
#define REQUEST_TYPE				0x60

#define REQUEST_DEVICE				0x00
#define REQUEST_INTERFACE			0x01
#define REQUEST_ENDPOINT			0x02
#define REQUEST_OTHER				0x03
#define REQUEST_RECIPIENT			0x1F

#define REQUEST_DEVICETOHOST_CLASS_INTERFACE  (REQUEST_DEVICETOHOST + REQUEST_CLASS + REQUEST_INTERFACE)
#define REQUEST_HOSTTODEVICE_CLASS_INTERFACE  (REQUEST_HOSTTODEVICE + REQUEST_CLASS + REQUEST_INTERFACE)

//	Class requests

#define CDC_SET_LINE_CODING			0x20
#define CDC_GET_LINE_CODING			0x21
#define CDC_SET_CONTROL_LINE_STATE	0x22

#define MSC_RESET					0xFF
#define MSC_GET_MAX_LUN				0xFE

#define HID_GET_REPORT				0x01
#define HID_GET_IDLE				0x02
#define HID_GET_PROTOCOL			0x03
#define HID_SET_REPORT				0x09
#define HID_SET_IDLE				0x0A
#define HID_SET_PROTOCOL			0x0B

//	Descriptors

#define USB_DEVICE_DESC_SIZE 18
#define USB_CONFIGUARTION_DESC_SIZE 9
#define USB_INTERFACE_DESC_SIZE 9
#define USB_ENDPOINT_DESC_SIZE 7

#define USB_DEVICE_DESCRIPTOR_TYPE             1
#define USB_CONFIGURATION_DESCRIPTOR_TYPE      2
#define USB_STRING_DESCRIPTOR_TYPE             3
#define USB_INTERFACE_DESCRIPTOR_TYPE          4
#define USB_ENDPOINT_DESCRIPTOR_TYPE           5
#define USB_DEVICE_QUALIFIER                   6
#define USB_OTHER_SPEED_CONFIGURATION          7

#define USB_DEVICE_CLASS_COMMUNICATIONS        0x02
#define USB_DEVICE_CLASS_HUMAN_INTERFACE       0x03
#define USB_DEVICE_CLASS_STORAGE               0x08
#define USB_DEVICE_CLASS_VENDOR_SPECIFIC       0xFF

#define USB_CONFIG_POWERED_MASK                0x40
#define USB_CONFIG_BUS_POWERED                 0x80
#define USB_CONFIG_SELF_POWERED                0xC0
#define USB_CONFIG_REMOTE_WAKEUP               0x20

// bMaxPower in Configuration Descriptor
#define USB_CONFIG_POWER_MA(mA)                ((mA)/2)

// bEndpointAddress in Endpoint Descriptor
#define USB_ENDPOINT_DIRECTION_MASK            0x80
#define USB_ENDPOINT_OUT(addr)                 ((addr) | 0x00)
#define USB_ENDPOINT_IN(addr)                  ((addr) | 0x80)

#define USB_ENDPOINT_TYPE_MASK                 0x03
#define USB_ENDPOINT_TYPE_CONTROL              0x00
#define USB_ENDPOINT_TYPE_ISOCHRONOUS          0x01
#define USB_ENDPOINT_TYPE_BULK                 0x02
#define USB_ENDPOINT_TYPE_INTERRUPT            0x03

#define TOBYTES(x) ((x) & 0xFF),(((x) >> 8) & 0xFF)

#define CDC_V1_10                               0x0110
#define CDC_COMMUNICATION_INTERFACE_CLASS       0x02


#define CDC_CALL_MANAGEMENT                     0x01
#define CDC_ABSTRACT_CONTROL_MODEL              0x02
#define CDC_HEADER                              0x00
#define CDC_ABSTRACT_CONTROL_MANAGEMENT         0x02
#define CDC_UNION                               0x06
#define CDC_CS_INTERFACE                        0x24
#define CDC_CS_ENDPOINT                         0x25
#define CDC_DATA_INTERFACE_CLASS                0x0A

#define MSC_SUBCLASS_SCSI						0x06
#define MSC_PROTOCOL_BULK_ONLY					0x50

#define HID_HID_DESCRIPTOR_TYPE					0x21
#define HID_REPORT_DESCRIPTOR_TYPE				0x22
#define HID_PHYSICAL_DESCRIPTOR_TYPE			0x23

_Pragma("pack(1)")

//	Device
typedef struct {
	uint8_t len;				// 18
	uint8_t dtype;				// 1 USB_DEVICE_DESCRIPTOR_TYPE
	uint16_t usbVersion;		// 0x200
	uint8_t	deviceClass;
	uint8_t	deviceSubClass;
	uint8_t	deviceProtocol;
	uint8_t	packetSize0;		// Packet 0
	uint16_t	idVendor;
	uint16_t	idProduct;
	uint16_t	deviceVersion;	// 0x100
	uint8_t	iManufacturer;
	uint8_t	iProduct;
	uint8_t	iSerialNumber;
	uint8_t	bNumConfigurations;
} DeviceDescriptor;

//	Config
typedef struct {
	uint8_t	len;			// 9
	uint8_t	dtype;			// 2
	uint16_t clen;			// total length
	uint8_t	numInterfaces;
	uint8_t	config;
	uint8_t	iconfig;
	uint8_t	attributes;
	uint8_t	maxPower;
} ConfigDescriptor;

//	String

//	Interface
typedef struct
{
	uint8_t len;		// 9
	uint8_t dtype;		// 4
	uint8_t number;
	uint8_t alternate;
	uint8_t numEndpoints;
	uint8_t interfaceClass;
	uint8_t interfaceSubClass;
	uint8_t protocol;
	uint8_t iInterface;
} InterfaceDescriptor;

//	Endpoint
typedef struct
{
	uint8_t len;		// 7
	uint8_t dtype;		// 5
	uint8_t addr;
	uint8_t attr;
	uint16_t packetSize;
	uint8_t interval;
} EndpointDescriptor;

// Interface Association Descriptor
// Used to bind 2 interfaces together in CDC compostite device
typedef struct
{
	uint8_t len;				// 8
	uint8_t dtype;				// 11
	uint8_t firstInterface;
	uint8_t interfaceCount;
	uint8_t functionClass;
	uint8_t funtionSubClass;
	uint8_t functionProtocol;
	uint8_t iInterface;
} IADDescriptor;

//	CDC CS interface descriptor
typedef struct
{
	uint8_t len;		// 5
	uint8_t dtype;		// 0x24
	uint8_t subtype;
	uint8_t d0;
	uint8_t d1;
} CDCCSInterfaceDescriptor;

typedef struct
{
	uint8_t len;		// 4
	uint8_t dtype;		// 0x24
	uint8_t subtype;
	uint8_t d0;
} CDCCSInterfaceDescriptor4;

typedef struct
{
    uint8_t	len;
    uint8_t 	dtype;		// 0x24
    uint8_t 	subtype;	// 1
    uint8_t 	bmCapabilities;
    uint8_t 	bDataInterface;
} CMFunctionalDescriptor;

typedef struct
{
    uint8_t	len;
    uint8_t 	dtype;		// 0x24
    uint8_t 	subtype;	// 1
    uint8_t 	bmCapabilities;
} ACMFunctionalDescriptor;

typedef struct
{
	//	IAD
	IADDescriptor				iad;	// Only needed on compound device

	//	Control
	InterfaceDescriptor			cif;
	CDCCSInterfaceDescriptor	header;
	CMFunctionalDescriptor		callManagement;			// Call Management
	ACMFunctionalDescriptor		controlManagement;		// ACM
	CDCCSInterfaceDescriptor	functionalDescriptor;	// CDC_UNION
	EndpointDescriptor			cifin;

	//	Data
	InterfaceDescriptor			dif;
	EndpointDescriptor			in;
	EndpointDescriptor			out;
} CDCDescriptor;

typedef struct
{
	InterfaceDescriptor			msc;
	EndpointDescriptor			in;
	EndpointDescriptor			out;
} MSCDescriptor;

typedef struct
{
	uint8_t len;			// 9
	uint8_t dtype;			// 0x21
	uint8_t addr;
	uint8_t	versionL;		// 0x101
	uint8_t	versionH;		// 0x101
	uint8_t	country;
	uint8_t	desctype;		// 0x22 report
	uint8_t	descLenL;
	uint8_t	descLenH;
} HIDDescDescriptor;

typedef struct
{
	InterfaceDescriptor		hid;
	HIDDescDescriptor		desc;
	EndpointDescriptor		in;
} HIDDescriptor;

_Pragma("pack()")

#define D_DEVICE(_class,_subClass,_proto,_packetSize0,_vid,_pid,_version,_im,_ip,_is,_configs) \
	{ 18, 1, 0x200, _class,_subClass,_proto,_packetSize0,_vid,_pid,_version,_im,_ip,_is,_configs }

#define D_CONFIG(_totalLength,_interfaces) \
	{ 9, 2, _totalLength,_interfaces, 1, 0, USB_CONFIG_SELF_POWERED, USB_CONFIG_POWER_MA(500) }

#define D_OTHERCONFIG(_totalLength,_interfaces) \
	{ 9, 7, _totalLength,_interfaces, 1, 0, USB_CONFIG_SELF_POWERED, USB_CONFIG_POWER_MA(500) }

#define D_INTERFACE(_n,_numEndpoints,_class,_subClass,_protocol) \
	{ 9, 4, _n, 0, _numEndpoints, _class,_subClass, _protocol, 0 }

#define D_ENDPOINT(_addr,_attr,_packetSize, _interval) \
	{ 7, 5, _addr,_attr,_packetSize, _interval }

#define D_QUALIFIER(_class,_subClass,_proto,_packetSize0,_configs) \
	{ 10, 6, 0x200, _class,_subClass,_proto,_packetSize0,_configs }

#define D_IAD(_firstInterface, _count, _class, _subClass, _protocol) \
	{ 8, 11, _firstInterface, _count, _class, _subClass, _protocol, 0 }

#define D_HIDREPORT(_descriptorLength) \
	{ 9, 0x21, 0x1, 0x1, 0, 1, 0x22, _descriptorLength, 0 }

#define D_CDCCS(_subtype,_d0,_d1)	{ 5, 0x24, _subtype, _d0, _d1 }
#define D_CDCCS4(_subtype,_d0)		{ 4, 0x24, _subtype, _d0 }

#endif

#ifdef __SAM4S4A__
/** Atmel helper functions from compiler.h used in udp_device.h*/
typedef uint32_t                U32;  //!< 32-bit unsigned integer.
#if (defined __GNUC__) || (defined __CC_ARM)
#   define ctz(u)              __builtin_ctz(u)
#else
#   define ctz(u)              ((u) & (1ul <<  0) ?  0 : \
                                (u) & (1ul <<  1) ?  1 : \
                                (u) & (1ul <<  2) ?  2 : \
                                (u) & (1ul <<  3) ?  3 : \
                                (u) & (1ul <<  4) ?  4 : \
                                (u) & (1ul <<  5) ?  5 : \
                                (u) & (1ul <<  6) ?  6 : \
                                (u) & (1ul <<  7) ?  7 : \
                                (u) & (1ul <<  8) ?  8 : \
                                (u) & (1ul <<  9) ?  9 : \
                                (u) & (1ul << 10) ? 10 : \
                                (u) & (1ul << 11) ? 11 : \
                                (u) & (1ul << 12) ? 12 : \
                                (u) & (1ul << 13) ? 13 : \
                                (u) & (1ul << 14) ? 14 : \
                                (u) & (1ul << 15) ? 15 : \
                                (u) & (1ul << 16) ? 16 : \
                                (u) & (1ul << 17) ? 17 : \
                                (u) & (1ul << 18) ? 18 : \
                                (u) & (1ul << 19) ? 19 : \
                                (u) & (1ul << 20) ? 20 : \
                                (u) & (1ul << 21) ? 21 : \
                                (u) & (1ul << 22) ? 22 : \
                                (u) & (1ul << 23) ? 23 : \
                                (u) & (1ul << 24) ? 24 : \
                                (u) & (1ul << 25) ? 25 : \
                                (u) & (1ul << 26) ? 26 : \
                                (u) & (1ul << 27) ? 27 : \
                                (u) & (1ul << 28) ? 28 : \
                                (u) & (1ul << 29) ? 29 : \
                                (u) & (1ul << 30) ? 30 : \
                                (u) & (1ul << 31) ? 31 : \
                                32)
#endif
#define Rd_bits( value, mask)        ((value) & (mask))
#define Rd_bitfield( value, mask)           (Rd_bits( value, mask) >> ctz(mask))
#define Tst_bits( value, mask)  (Rd_bits(value, mask) != 0)
#define Wr_bitfield(lvalue, mask, bitfield) (Wr_bits(lvalue, mask, (U32)(bitfield) << ctz(mask)))
#define Wr_bits(lvalue, mask, bits)  ((lvalue) = ((lvalue) & ~(mask)) |\
                                                 ((bits  ) &  (mask)))
#define Set_bits(lvalue, mask)  ((lvalue) |=  (mask))
#define Clr_bits(lvalue, mask)  ((lvalue) &= ~(mask))
#endif //SAM4S4A

