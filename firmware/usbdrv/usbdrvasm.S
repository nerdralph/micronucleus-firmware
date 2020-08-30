/* Name: usbdrvasm.S
 * Project: V-USB, virtual USB port for Atmel's(r) AVR(r) microcontrollers
 * Author: Christian Starkjohann
 * Creation Date: 2007-06-13
 * Copyright: (c) 2007 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 * code cleanup and optimization by Ralph Doncaster 2020
 */

/*
General Description:
This module is the assembler part of the USB driver. This file contains
general code (preprocessor acrobatics and CRC computation) and then includes
the file appropriate for the given clock rate.
*/

#define __SFR_OFFSET 0      /* used by avr-libc's register definitions */
// #include "usbportability.h"
#include "usbdrv.h"         /* for common defs */

/* register names */
#define x1      r23
#define x2      r24
#define shift   r18
#define cnt     r19
#define x3      r20
#define x4      r21
#define x5		r22
#define bitcnt  x5
#define phase   x4
#define leap    x4


#ifndef USB_INTR_VECTOR /* default to hardware interrupt INT0 */
#   define USB_INTR_VECTOR  INT0_vect       // this is the "new" define for the vector
#endif

    .text
    .global USB_INTR_VECTOR
    .type   USB_INTR_VECTOR, @function
    .global usbCrc16Append
    .global usbProcessRx
    .global usbBuildTxBlock
    .global usbFunctionSetup            ; in main.c 

#if USB_INTR_PENDING < 0x40 /* This is an I/O address, use in and out */
#   define  USB_LOAD_PENDING(reg)   in reg, USB_INTR_PENDING
#   define  USB_STORE_PENDING(reg)  out USB_INTR_PENDING, reg
#else   /* It's a memory address, use lds and sts */
#   define  USB_LOAD_PENDING(reg)   lds reg, USB_INTR_PENDING
#   define  USB_STORE_PENDING(reg)  sts USB_INTR_PENDING, reg
#endif

#define usbTxLen1   usbTxStatus1
#define usbTxBuf1   (usbTxStatus1 + 1)
#define usbTxLen3   usbTxStatus3
#define usbTxBuf3   (usbTxStatus3 + 1)

.equ o_bmRequestType, 0
.equ o_bRequest,    1
.equ o_wValue,      2
.equ o_wIndex,      4

; void usbProcessRx(uchar *rq, uchar len)
; r25:24 = data, r22 = len
#define rxToken r23
usbProcessRx:
    lds rxToken, usbRxToken
    cpi rxToken, USBPID_SETUP
    brne done
    ldi r18, USBPID_DATA0               ; setup data toggling 
    sts usbTxBuf, r18
    movw ZL, r24

    ldd r18, Z+o_bmRequestType
    andi r18, USBRQ_TYPE_MASK
    cpi r18, USBRQ_TYPE_STANDARD
    breq rq_std
    ; for USBRQ_TYPE_CLASS or USBRQ_TYPE_VENDOR 
    rcall usbFunctionSetup
    rjmp saveLen
rq_std:
    ; handle driver requests
    clr r24                             ; default length
    ldd r18, Z+o_bRequest
    cpi r18, USBRQ_SET_ADDRESS          ; 5
    brne 1f
    ldd r19, Z+o_wValue
    sts usbNewDeviceAddr, r19
1:  cpi r18, USBRQ_GET_DESCRIPTOR
    brne setMsgPtr
    ldd r19, Z+o_wValue + 1             ; descriptor type
    cpi r19, USBDESCR_DEVICE
    brne 2f
    ldi r20, lo8(usbDescriptorDevice)
    ldi r21, hi8(usbDescriptorDevice)
    ldi r24, 18                         ; length
    rjmp setMsgPtr
2:  cpi r19, USBDESCR_CONFIG
    brne saveLen
    ldi r20, lo8(usbDescriptorConfiguration)
    ldi r21, hi8(usbDescriptorConfiguration)
    ldi r24, 9                          ; length
setMsgPtr:
    ;sts usbMsgPtr, r20
    ;sts usbMsgPtr+1, r21
    movw r8, r20                        ; r8 fixed usbMsgPtr

saveLen:
    sts usbMsgLen, r24
done:
ret
#undef rxToken

#if 0
; void usbBuildTxBlock(void)
#define wantLen r25 
uBuildTxBlock:
    lds wantLen, usbMsgLen
    mov r0, wantLen
    cpi wantLen, 8                      ; 8 bytes max data packet
    brlo 1f
    ldi wantLen, 8
1:  sub r0, wantLen
    sts usbMsgLen, r0

    ; read Tx data from flash
    ldi XL, lo8(usbTxBuf + 1)
    ldi XL, hi8(usbTxBuf + 1)
    ldi ZL, lo8(usbMsgBuf + 1)
    ldi ZL, hi8(usbMsgBuf + 1)
ret
#endif

;----------------------------------------------------------------------------
; Utility functions
;----------------------------------------------------------------------------

/* Register assignments for usbCrc16 on gcc */
/* Calling conventions on gcc:
 * First parameter passed in r24/r25, second in r22/23 and so on.
 * Callee must preserve r1-r17, r28/r29
 */
; size optimized CRC
; extern unsigned usbCrc16(unsigned char *argPtr, unsigned char argLen);
; note: argLen can be zero!
#   define argLen   r22 /* argument 2 */
#   define argPtrL  r24 /* argument 1 */
#   define argPtrH  r25 /* argument 1 */

#   define resCrcL  r24 /* result */
#   define resCrcH  r25 /* result */

; temp variables
#   define ptrL     XL
#   define ptrH     XH
#   define ptr      x
#   define byte     r18
#   define bitCnt   r19
#   define polyL    r20
#   define polyH    r21
#   define scratch  r23

usbCrc16Append:
    movw    ptrL, argPtrL
crc16Direct:                            ; ptr arg in X
    ldi     resCrcL, 0
    ldi     resCrcH, 0
    ldi     polyL, lo8(0xa001)
    ldi     polyH, hi8(0xa001)
    com     argLen      ; argLen = -argLen - 1: modified loop to ensure that carry is set
    ldi     bitCnt, 0   ; loop counter with starnd condition = end condition
    rjmp    usbCrcLoopEntry
usbCrcByteLoop:
    ld      byte, ptr+
    eor     resCrcL, byte
;    sec
usbCrcBitLoop:
    ror     resCrcH     ; carry is always set here (see brcs jumps to here)
    ror     resCrcL
    brcs    usbCrcNoXor
    eor     resCrcL, polyL
    eor     resCrcH, polyH
usbCrcNoXor:
    subi    bitCnt, 224 ; (8 * 224) % 256 = 0; this loop iterates 8 times
    brcs    usbCrcBitLoop
usbCrcLoopEntry:
    subi    argLen, -1
    brcs    usbCrcByteLoop
;    subi    argLen, 1
;    brne    usbCrcByteLoop
    st      ptr+, resCrcL
    st      ptr+, resCrcH
    ret
; Thanks to Reimar Doeffinger for optimizing this CRC routine!

#undef argLen
#undef argPtrL
#undef argPtrH
#undef resCrcL
#undef resCrcH
#undef ptrL
#undef ptrH
#undef ptr
#undef byte
#undef bitCnt
#undef polyL
#undef polyH


;----------------------------------------------------------------------------
; Now include the clock rate specific code
;----------------------------------------------------------------------------

#ifndef USB_CFG_CLOCK_KHZ
#   define USB_CFG_CLOCK_KHZ (F_CPU/1000)
#endif

#if USB_CFG_CLOCK_KHZ == 12000
#   include "usbdrvasm12.inc"
#elif USB_CFG_CLOCK_KHZ == 16000
#   include "usbdrvasm16.inc"
#elif USB_CFG_CLOCK_KHZ == 16500
#   include "usbdrvasm165.inc"
#else
#   error "USB_CFG_CLOCK_KHZ is not one of the supported rates!"
#endif
