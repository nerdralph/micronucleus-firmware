/* Name: usbdrv.c
 * Project: V-USB, virtual USB port for Atmel's(r) AVR(r) microcontrollers
 * Author: Christian Starkjohann
 * Creation Date: 2004-12-29
 *

 * Copyright: (c) 2005 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 */

/* This copy of usbdrv.c was optimized to reduce the memory footprint with micronucleus V2
 *
 * Changes:
 *     a) Replies to USB SETUP IN Packets are now only possible from Flash
 *       * Commented out routines to copy from SRAM
 *       * remove msgflag variable and all handling involving it
 *
 * code cleanup and optimization by Ralph Doncaster 2020-08
 */

#define MNHACK_NO_DATASECTION

#include "usbdrv.h"
#include "oddebug.h"

/*
General Description:
This module implements the C-part of the USB driver. See usbdrv.h for a
documentation of the entire driver.
*/

/* ------------------------------------------------------------------------- */

/* raw USB registers / interface to assembler code: */
uchar usbRxBuf[USB_BUFSIZE];  /* raw RX buffer: PID, 8 bytes data, 2 bytes CRC */
uchar       usbDeviceAddr;      /* assigned during enumeration, defaults to 0 */
uchar       usbNewDeviceAddr;   /* device ID which should be set after status phase */
uchar       usbConfiguration;   /* currently selected configuration. Administered by driver, but not used */
//volatile schar usbRxLen;        /* = 0; number of bytes in usbRxBuf; 0 means free, -1 for flow control */
register schar usbRxLen asm("r10");        /* = 0; usbRxBuf byte count; 0 means free, -1 for flow control */
uchar       usbCurrentTok;      /* last token received or endpoint number for last OUT token if != 0 */
uchar       usbRxToken;         /* token for data we received; or endpont number for last OUT */
register uchar usbTxLen asm("r17");   /* number of bytes to transmit with next IN token or handshake token */
uchar       usbTxBuf[USB_BUFSIZE];/* data to transmit with next IN, free if usbTxLen contains handshake token */

//usbMsgPtr_t         usbMsgPtr;      /* data to tx next -- flash address */
register usbMsgPtr_t         usbMsgPtr asm("r8");      /* data to tx next -- flash address */

/* USB status registers / not shared with asm code */
// static usbMsgLen_t  usbMsgLen; /* remaining number of bytes */
//  usbMsgLen_t  usbMsgLen; /* remaining number of bytes */
register usbMsgLen_t  usbMsgLen asm("r16"); /* remaining number of bytes */

#define USB_FLG_USE_USER_RW     (1<<7)

/*
optimizing hints:
- do not post/pre inc/dec integer values in operations
- assign value of USB_READ_FLASH() to register variables and don't use side effects in arg
- use narrow scope for variables which should be in X/Y/Z register
- assign char sized expressions to variables to force 8 bit arithmetics
*/

/* --------------------------- Device Descriptor --------------------------- */

#if USB_CFG_DESCR_PROPS_DEVICE == 0
#undef USB_CFG_DESCR_PROPS_DEVICE
#define USB_CFG_DESCR_PROPS_DEVICE  sizeof(usbDescriptorDevice)
//PROGMEM const char usbDescriptorDevice[] = {    /* USB device descriptor */
const __flash char usbDescriptorDevice[] = {    /* USB device descriptor */
    18,         /* sizeof(usbDescriptorDevice): length of descriptor in bytes */
    USBDESCR_DEVICE,        /* descriptor type */
    0x10, 0x01,             /* USB version supported */
    USB_CFG_DEVICE_CLASS,
    USB_CFG_DEVICE_SUBCLASS,
    0,                      /* protocol */
    8,                      /* max packet size */
    /* the following two casts affect the first byte of the constant only, but
     * that's sufficient to avoid a warning with the default values.
     */
    (char)USB_CFG_VENDOR_ID,/* 2 bytes */
    (char)USB_CFG_DEVICE_ID,/* 2 bytes */
    USB_CFG_DEVICE_VERSION, /* 2 bytes */
    0,                      /* manufacturer string index */
    0,                      /* product string index */
    0,                      /* serial number string index */
    1,                      /* number of configurations */
};
#endif

/* ----------------------- Configuration Descriptor ------------------------ */

#if USB_CFG_DESCR_PROPS_CONFIGURATION == 0
#undef USB_CFG_DESCR_PROPS_CONFIGURATION
#define USB_CFG_DESCR_PROPS_CONFIGURATION   sizeof(usbDescriptorConfiguration)
//PROGMEM const char usbDescriptorConfiguration[] = {    /* USB configuration descriptor */
const __flash char usbDescriptorConfiguration[] = {    /* USB configuration descriptor */
    9,          /* sizeof(usbDescriptorConfiguration): length of descriptor in bytes */
    USBDESCR_CONFIG,    /* descriptor type */
    9, 0,      /* total length of data returned (including inlined descriptors) */
    0,          /* number of interfaces in this configuration */
    1,          /* index of this configuration */
    0,          /* configuration name string index */
    (1 << 7),                           /* attributes bus powered */
    USB_CFG_MAX_BUS_POWER/2,            /* max USB current in 2mA units */
#if 0
/* interface descriptor follows inline: */
    9,          /* sizeof(usbDescrInterface): length of descriptor in bytes */
    USBDESCR_INTERFACE, /* descriptor type */
    0,          /* index of this interface */
    0,          /* alternate setting for this interface */
    0,          /* endpoints excl 0: number of endpoint descriptors to follow */
    USB_CFG_INTERFACE_CLASS,
    USB_CFG_INTERFACE_SUBCLASS,
    USB_CFG_INTERFACE_PROTOCOL,
    0,          /* string index for interface */
#endif
};
#endif

/* ------------------------------------------------------------------------- */

USB_PUBLIC void usbInit(void)
{
#ifdef MNHACK_NO_DATASECTION
    usbTxLen = USBPID_NAK;
    usbMsgLen = USB_NO_MSG;
#endif

#if USB_INTR_CFG_SET != 0
    USB_INTR_CFG |= USB_INTR_CFG_SET;
#endif
#if USB_INTR_CFG_CLR != 0
    USB_INTR_CFG &= ~(USB_INTR_CFG_CLR);
#endif
#if (defined(GIMSK) || defined(EIMSK)) // GICR contains other bits, which must be kept
    USB_INTR_ENABLE |= (1 << USB_INTR_ENABLE_BIT);
#else
    USB_INTR_ENABLE = (1 << USB_INTR_ENABLE_BIT); // We only want one interrupt to be enabled, so keeping the other bits makes no sense.
#endif
}

