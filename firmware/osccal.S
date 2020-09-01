; Ralph Doncaster 2020
; optimized OSCCAL tuning from low-speed USB SOF every 1ms

#include "macros.inc"

// #include "usbdrv/usbdrv.h"

#define OSCCAL 0x31
#define USBIN 0x15
#define USBPLUS 1
#define USBMINUS 1

#define countH r25
#define countL r24
#define scratch r23
#define shift r23

; tuneOsccal should be called after USB reset
; needs to see 6 consecutive EOF/SOF transistions
GLABEL tuneOsccal
    rcall tuneOnce
    rcall tuneOnce
    rjmp tuneOnce

tuneOnce:
    rcall countFrame                    ; ignore 1st count
    rcall countFrame
    .equ goal, (F_CPU/10000)            ; expected count for 1ms frame
    subi countL, lo8(goal)
    sbci countH, hi8(goal)
    ; at 16.5Mhz, a 10% variation will be a difference of 165
    ; assume 1 OSCCAL step = 1%, so divide by 16
    ldi scratch, 4
1:  asr countH
    ror countL
    dec scratch
    brne 1b
    ; round here using C?
    in scratch, OSCCAL    
    sub scratch, countL
    out OSCCAL, scratch
    nop                                 ; probably not really needed
ret

; countFrame counts the time to the next EOF/SOF
; end-of-frame has an idle guard band followed by SE0
; idle is DPLUS low & DMINUS high. save DPLUS history in shift
countFrame:
    ldi scratch, 0xFF
    clr countL
    clr countH
waitSOF:                                ; 10-cycle loop
    in scratch, USBIN
    lsr shift
    adiw countL, 1
    bst scratch, USBPLUS 
    bld shift, 7
    sbrs scratch, USBMINUS
    tst scratch
    brne waitSOF                        ; not idle last 7 loops
ret
