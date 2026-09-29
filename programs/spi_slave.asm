; ===========================================================================
; ProtocolForge Firmware: SPI Slave (Mode 0: CPOL=0, CPHA=0)
; ===========================================================================
; Description:
;   SPI slave responding to external master. Synchronizes to external SCK
;   and CS_n inputs, shifting out MISO data while capturing MOSI data.
;
; Configuration:
;   - IN_BASE mapped to [SCK (pin 0), MOSI (pin 1), CS_n (pin 2)]
;   - OUT_BASE mapped to MISO pin
; ===========================================================================

.program spi_slave

.wrap_target
wait_cs:
    wait 0 pin 2                    ; Wait for CS_n active Low from external master
    pull ifempty noblock            ; Fetch transmit word if available in TX FIFO
    set x, 7                        ; 8-bit word transfer
slave_loop:
    wait 1 pin 0                    ; Wait for SCK rising edge
    in pins, 1                      ; Sample MOSI input into ISR
    wait 0 pin 0                    ; Wait for SCK falling edge
    out pins, 1                     ; Drive next MISO bit out
    jmp x-- slave_loop              ; Loop for all 8 bits
    push block                      ; Push received word to RX FIFO
.wrap
