; ===========================================================================
; ProtocolForge Firmware: UART Receiver (8N1)
; ===========================================================================
; Description:
;   Receives 8-bit UART data with start bit detection and center sampling.
;
; Configuration:
;   - Clock divider set to 8x oversampling or bit rate
;   - IN_BASE mapped to RX pin
;   - JMP_PIN mapped to RX pin (for stop bit check)
; ===========================================================================

.program uart_rx

.wrap_target
start:
    wait 0 pin 0                    ; Wait for line to go Low (Start bit detected)
    set x, 7            [3]         ; Delay 0.5 bit period to align to center of bit cell
rx_loop:
    in pins, 1          [6]         ; Sample 1 bit in center; wait 1 full bit period
    jmp x-- rx_loop                 ; Loop for all 8 data bits
    jmp pin good_stop               ; Verify Stop bit is High
    jmp start                       ; Framing error: drop packet and resynchronize
good_stop:
    push block                      ; Push received 8-bit byte to RX FIFO
.wrap
