; Ralph Doncaster 2020
; optimized OSCCAL tuning from low-speed USB SOF every 1ms

#include "macros.inc"

#define __SFR_OFFSET 0                  /* start SFRs at 0x00 not 0x20 */
#include "usbdrv/usbdrv.h"

/*
; defines for standalone debugging 
#define OSCCAL 0x31
#define USBIN 0x16
#define USBPLUS 4
#define USBMINUS 3
*/

#define countH r25
#define countL r24
#define scratch r23
#define shift r22

; tuneOsccal should be called after USB reset
; needs to see 5 consecutive EOF/SOF transistions
GLABEL tuneOsccal
    rcall countFrame                    ; ignore 1st count
    rcall countFrame
    rcall tuneOnce
    rcall tuneOnce
    ; fall through to tuneOnce for third time

tuneOnce:
    ; assume 1 OSCCAL step = 0.667% fequency changea so 100% = 150
    subi countH, 150                    ; countH - 150 = change to OSCCAL 
    ;in scratch, OSCCAL
    LOAD scratch, OSCCAL
    sub scratch, countH
    ;out OSCCAL, scratch
    STORE OSCCAL, scratch
    ; fall through to countFrame for next tuning

; countFrame counts the time to the next EOF/SOF
; end-of-frame has an idle guard band followed by SE0
; idle is DPLUS low & DMINUS high. save DPLUS history in shift
; countL is used as a fractional accumulator with countH LSbit = 0.667%
.equ fMhz, (F_CPU/1000000)
.equ fraction, (2*384/fMhz + 1) / 2     ; (256/.667)/fMhz rounded
countFrame:
    ldi shift, 0xFF
    ldi countL, fraction                ; countFrame overhead adjustment
    clr countH
waitSOF:                                ; 10-cycle loop
    in scratch, USBIN
    lsr shift
    adiw countL, fraction
    bst scratch, USBPLUS 
    bld shift, 7
    sbrs scratch, USBMINUS
    tst shift
    brne waitSOF                        ; not idle last 7 loops
ret

