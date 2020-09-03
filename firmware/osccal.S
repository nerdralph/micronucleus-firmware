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
; needs to see 6 consecutive EOF/SOF transistions
GLABEL tuneOsccal
    rcall tuneOnce
    rcall tuneOnce
    ; fall through to tuneOnce for third time

tuneOnce:
    rcall countFrame                    ; ignore 1st count
    rcall countFrame
    ; assume 1 OSCCAL step = 0.667% fequency changea so 100% = 150
    sbci countH, 150                    ; countH - 150 = change to OSCCAL 
    in scratch, OSCCAL    
    sub scratch, countH
    out OSCCAL, scratch
ret

; countFrame counts the time to the next EOF/SOF
; end-of-frame has an idle guard band followed by SE0
; idle is DPLUS low & DMINUS high. save DPLUS history in shift
; countL is used as a fractional accumulator with countH LSbit = 0.667%
countFrame:
    ldi scratch, 0xFF
    clr countL
    clr countH
waitSOF:                                ; 10-cycle loop
    in scratch, USBIN
    lsr shift
    .equ fMhz, (F_CPU/1000000)
    adiw countL, 256/fMhz + 1           ; add 1 for rounding
    bst scratch, USBPLUS 
    bld shift, 7
    sbrs scratch, USBMINUS
    tst scratch
    brne waitSOF                        ; not idle last 7 loops
ret

