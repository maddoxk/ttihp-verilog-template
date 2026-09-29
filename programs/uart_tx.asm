; ===========================================================================
; ProtocolForge Firmware: UART Transmitter (8N1)
; ===========================================================================
; Description:
;   Transmits 8-bit UART data with 1 start bit (0), 8 data bits (LSB first),
;   and 1 stop bit (1).
;
; Configuration:
;   - Clock divider set to baud rate (e.g. 50 MHz / 115200 = 434)
;   - SET_BASE / OUT_BASE mapped to TX pin
;   - Side-set: 1 bit controlling TX pin (default idle High = 1)
; ===========================================================================

.program uart_tx
.side_set 1

.wrap_target
    pull block          side 1      ; Wait for character in TX FIFO; keep TX pin High (idle)
    set x, 7            side 0      ; Assert Start bit (Low) for 1 bit period; set 8-bit counter
data_loop:
    out pins, 1                     ; Shift out 1 data bit (LSB first)
    jmp x-- data_loop               ; Loop through 8 data bits
    nop                 side 1      ; Assert Stop bit (High) for 1 bit period
.wrap
