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
 */

 // code cleanup and optimization by Ralph Doncaster 2020-08

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
uchar usbRxBuf[2*USB_BUFSIZE];  /* raw RX buffer: PID, 8 bytes data, 2 bytes CRC */
// uchar       usbInputBufOffset;  /* offset in usbRxBuf used for low level receiving */
uchar       usbDeviceAddr;      /* assigned during enumeration, defaults to 0 */
uchar       usbNewDeviceAddr;   /* device ID which should be set after status phase */
uchar       usbConfiguration;   /* currently selected configuration. Administered by driver, but not used */
volatile schar usbRxLen;        /* = 0; number of bytes in usbRxBuf; 0 means free, -1 for flow control */
uchar       usbCurrentTok;      /* last token received or endpoint number for last OUT token if != 0 */
uchar       usbRxToken;         /* token for data we received; or endpont number for last OUT */
#ifdef MNHACK_NO_DATASECTION
  volatile uchar usbTxLen;   /* number of bytes to transmit with next IN token or handshake token */
#else
  volatile uchar usbTxLen = USBPID_NAK;   /* number of bytes to transmit with next IN token or handshake token */
#endif
uchar       usbTxBuf[USB_BUFSIZE];/* data to transmit with next IN, free if usbTxLen contains handshake token */

/* USB status registers / not shared with asm code */
usbMsgPtr_t         usbMsgPtr;      /* data to transmit next -- ROM or RAM address */
#ifdef MNHACK_NO_DATASECTION
  static usbMsgLen_t  usbMsgLen; /* remaining number of bytes */
#else
  static usbMsgLen_t  usbMsgLen = USB_NO_MSG; /* remaining number of bytes */
#endif

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
PROGMEM const char usbDescriptorDevice[] = {    /* USB device descriptor */
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
PROGMEM const char usbDescriptorConfiguration[] = {    /* USB configuration descriptor */
    9,          /* sizeof(usbDescriptorConfiguration): length of descriptor in bytes */
    USBDESCR_CONFIG,    /* descriptor type */
    18, 0,      /* total length of data returned (including inlined descriptors) */
    1,          /* number of interfaces in this configuration */
    1,          /* index of this configuration */
    0,          /* configuration name string index */
    (1 << 7),                           /* attributes bus powered */
    USB_CFG_MAX_BUS_POWER/2,            /* max USB current in 2mA units */
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
};
#endif

/* ------------------------------------------------------------------------- */

static inline void  usbResetDataToggling(void)
{
}

static inline void  usbResetStall(void)
{
}

/* ------------------ utilities for code following below ------------------- */

/* Use defines for the switch statement so that we can choose between an
 * if()else if() and a switch/case based implementation. switch() is more
 * efficient for a LARGE set of sequential choices, if() is better in all other
 * cases.
 */
#if USB_CFG_USE_SWITCH_STATEMENT
#   define SWITCH_START(cmd)       switch(cmd){{
#   define SWITCH_CASE(value)      }break; case (value):{
#   define SWITCH_CASE2(v1,v2)     }break; case (v1): case(v2):{
#   define SWITCH_CASE3(v1,v2,v3)  }break; case (v1): case(v2): case(v3):{
#   define SWITCH_DEFAULT          }break; default:{
#   define SWITCH_END              }}
#else
#   define SWITCH_START(cmd)       {uchar _cmd = cmd; if(0){
#   define SWITCH_CASE(value)      }else if(_cmd == (value)){
#   define SWITCH_CASE2(v1,v2)     }else if(_cmd == (v1) || _cmd == (v2)){
#   define SWITCH_CASE3(v1,v2,v3)  }else if(_cmd == (v1) || _cmd == (v2) || _cmd == (v3)){
#   define SWITCH_DEFAULT          }else{
#   define SWITCH_END              }}
#endif

#ifndef USB_RX_USER_HOOK
#define USB_RX_USER_HOOK(data, len)
#endif
#ifndef USB_SET_ADDRESS_HOOK
#define USB_SET_ADDRESS_HOOK()
#endif

/* ------------------------------------------------------------------------- */

/* We use if() instead of #if in the macro below because #if can't be used
 * in macros and the compiler optimizes constant conditions anyway.
 * This may cause problems with undefined symbols if compiled without
 * optimizing!
 */
// no ram descriptor possible here
#define GET_DESCRIPTOR(cfgProp, staticName)         \
    if(cfgProp){                                    \
        len = USB_PROP_LENGTH(cfgProp);             \
        usbMsgPtr = (usbMsgPtr_t)(staticName);      \
    }

/* usbDriverDescriptor() is similar to usbFunctionDescriptor(), but used
 * internally for all types of descriptors.
 */
static inline usbMsgLen_t usbDriverDescriptor(usbRequest_t *rq)
{
    usbMsgLen_t len = 0;

    SWITCH_START(rq->wValue.bytes[1])
    SWITCH_CASE(USBDESCR_DEVICE)    /* 1 */
        GET_DESCRIPTOR(USB_CFG_DESCR_PROPS_DEVICE, usbDescriptorDevice)
    SWITCH_CASE(USBDESCR_CONFIG)    /* 2 */
        GET_DESCRIPTOR(USB_CFG_DESCR_PROPS_CONFIGURATION, usbDescriptorConfiguration)
    SWITCH_DEFAULT
    SWITCH_END

    return len;
}

/* ------------------------------------------------------------------------- */

/* usbDriverSetup() is similar to usbFunctionSetup(), but it's used for
 * standard requests instead of class and custom requests.
 */
static inline usbMsgLen_t usbDriverSetup(usbRequest_t *rq)
{
    usbMsgLen_t len = 0;
    uchar   *dataPtr = usbTxBuf + 9;    /* there are 2 bytes free space at the end of the buffer */
    uchar   value = rq->wValue.bytes[0];

    dataPtr[0] = 0; /* default reply common to USBRQ_GET_STATUS and USBRQ_GET_INTERFACE */
    SWITCH_START(rq->bRequest)
    SWITCH_CASE(USBRQ_GET_STATUS)           /* 0 */
        dataPtr[1] = 0;
        len = 2;
    SWITCH_CASE(USBRQ_SET_ADDRESS)          /* 5 */
        usbNewDeviceAddr = value;
        USB_SET_ADDRESS_HOOK();
    SWITCH_CASE(USBRQ_GET_DESCRIPTOR)       /* 6 */
        len = usbDriverDescriptor(rq);
        goto skipMsgPtrAssignment;
    SWITCH_CASE(USBRQ_GET_CONFIGURATION)    /* 8 */
        dataPtr = &usbConfiguration;  /* send current configuration value */
        len = 1;
    SWITCH_CASE(USBRQ_SET_CONFIGURATION)    /* 9 */
        usbConfiguration = value;
        usbResetStall();
    SWITCH_CASE(USBRQ_GET_INTERFACE)        /* 10 */
        len = 1;
    SWITCH_DEFAULT                          /* 7=SET_DESCRIPTOR, 12=SYNC_FRAME */
        /* Should we add an optional hook here? */
    SWITCH_END
    usbMsgPtr = (usbMsgPtr_t)dataPtr;
skipMsgPtrAssignment:
    return len;
}

/* ------------------------------------------------------------------------- */

/* usbProcessRx() is called for every message received by the interrupt
 * routine. It distinguishes between SETUP and DATA packets and processes
 * them accordingly.
 */
static inline void usbProcessRx(uchar *data, uchar len)
{
usbRequest_t    *rq = (void *)data;

/* usbRxToken can be:
 * 0x2d 00101101 (USBPID_SETUP for setup data)
 * 0xe1 11100001 (USBPID_OUT: data phase of setup transfer)
 * 0...0x0f for OUT on endpoint X
 */
    DBG2(0x10 + (usbRxToken & 0xf), data, len + 2); /* SETUP=1d, SETUP-DATA=11, OUTx=1x */
    USB_RX_USER_HOOK(data, len)
    if(usbRxToken == (uchar)USBPID_SETUP){
        if(len != 8)    /* Setup size must be always 8 bytes. Ignore otherwise. */
            return;
        usbMsgLen_t replyLen;
        usbTxBuf[0] = USBPID_DATA0;         /* initialize data toggling */
        usbTxLen = USBPID_NAK;              /* abort pending transmit */
        uchar type = rq->bmRequestType & USBRQ_TYPE_MASK;
        if(type != USBRQ_TYPE_STANDARD){    /* standard requests are handled by driver */
            replyLen = usbFunctionSetup(data); // for USBRQ_TYPE_CLASS or USBRQ_TYPE_VENDOR
        }else{
            replyLen = usbDriverSetup(rq);
        }
        if(sizeof(replyLen) < sizeof(rq->wLength.word)){ /* help compiler with optimizing */
            if(!rq->wLength.bytes[1] && replyLen > rq->wLength.bytes[0])    /* limit length to max */
                replyLen = rq->wLength.bytes[0];
        }else{
            if(replyLen > rq->wLength.word)     /* limit length to max */
                replyLen = rq->wLength.word;
        }
        usbMsgLen = replyLen;
    }else{  /* usbRxToken must be USBPID_OUT, which means data phase of setup (control-out) */
    }
}

/* ------------------------------------------------------------------------- */

/* This function is similar to usbFunctionRead(), but it's also called for
 * data handled automatically by the driver (e.g. descriptor reads).
 */
static uchar usbDeviceRead(uchar *data, uchar len)
{
    if(len > 0){    /* don't bother app with 0 sized reads */
        uchar i = len;
        usbMsgPtr_t r = usbMsgPtr;
        do{
            uchar c = USB_READ_FLASH(r);    /* assign to char size variable to enforce byte ops */
            *data++ = c;
            r++;
        }while(--i);
        usbMsgPtr = r;
    }
    return len;
}

/* ------------------------------------------------------------------------- */

/* usbBuildTxBlock() is called when we have data to transmit and the
 * interrupt routine's transmit buffer is empty.
 */
static inline void usbBuildTxBlock(void)
{
usbMsgLen_t wantLen;
uchar       len;

    wantLen = usbMsgLen;
    if(wantLen > 8)
        wantLen = 8;
    usbMsgLen -= wantLen;
    usbTxBuf[0] ^= USBPID_DATA0 ^ USBPID_DATA1; /* DATA toggling */
    len = usbDeviceRead(usbTxBuf + 1, wantLen);
    if(len <= 8){           /* valid data packet */
        usbCrc16Append(&usbTxBuf[1], len);
        len += 4;           /* length including sync byte */
        if(len < 12)        /* a partial package identifies end of message */
            usbMsgLen = USB_NO_MSG;
    }else{
        len = USBPID_STALL;   /* stall the endpoint */
        usbMsgLen = USB_NO_MSG;
    }
    usbTxLen = len;
    DBG2(0x20, usbTxBuf, len-1);
}

/* ------------------------------------------------------------------------- */

#ifdef USB_RESET_HOOK
static inline void usbHandleResetHook(uchar notResetState)
{

static uchar    wasReset;
uchar           isReset = !notResetState;

    if(wasReset != isReset){
        USB_RESET_HOOK(isReset);
        wasReset = isReset;
    }
}
#else
//    notResetState = notResetState;  // avoid compiler warning -> leads to another warning :-(
#endif
/* ------------------------------------------------------------------------- */
// Replaced for micronucleus V2
//USB_PUBLIC void usbPoll(void)

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
    usbResetDataToggling();
}

/* ------------------------------------------------------------------------- */
