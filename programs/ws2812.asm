; ===========================================================================
; ProtocolForge Firmware: WS2812B 800 kHz RGB LED Driver
; ===========================================================================
; Description:
;   Transmits high-precision NRZ pulse-width modulated data to WS2812B / NeoPixel
;   addressable RGB LEDs.
;   - Bit period: 1.25 us (800 kHz) divided into 10 clock cycles (125 ns/cycle)
;   - Bit '0': 3 cycles High, 7 cycles Low
;   - Bit '1': 7 cycles High, 3 cycles Low
;
; Configuration:
;   - Clock divider: 50 MHz / 8 MHz = 6.25 (DIV_INT = 6, DIV_FRAC = 64)
;   - Side-set: 1 bit controlling LED data output pin
; ===========================================================================

.program ws2812
.side_set 1

.wrap_target
bit_loop:
    out x, 1            side 0 [2]  ; Shift 1 bit from OSR into X; drive Low for 3 cycles
    jmp !x do_zero      side 1 [3]  ; Drive High for 4 cycles; if bit is 0, branch to finish High
do_one:
    nop                 side 1 [2]  ; Bit 1: hold High for 3 more cycles (total 7 cycles High)
    jmp bit_loop        side 0      ; Drive Low and loop back
do_zero:
    nop                 side 0 [2]  ; Bit 0: hold Low for 3 more cycles (total 7 cycles Low)
.wrap
