; ===========================================================================
; ProtocolForge Firmware: SPI Master (Mode 0: CPOL=0, CPHA=0)
; ===========================================================================
; Description:
;   Full-duplex SPI master with CS assertion/deassertion.
;   Shifts data on MOSI while sampling MISO.
;
; Configuration:
;   - Side-set 2 bits: [CS_n, SCK]
;       side 0b10: CS_n = 1 (inactive), SCK = 0 (idle)
;       side 0b00: CS_n = 0 (active),   SCK = 0 (falling/shift phase)
;       side 0b01: CS_n = 0 (active),   SCK = 1 (rising/sample phase)
;   - OUT_BASE mapped to MOSI pin
;   - IN_BASE mapped to MISO pin
; ===========================================================================

.program spi_master
.side_set 2

.wrap_target
    pull block          side 0b10   ; Wait for TX data word; CS_n inactive (1), SCK low (0)
    set x, 7            side 0b00   ; Assert CS_n Low; prepare for 8-bit transfer
bit_loop:
    out pins, 1         side 0b00   ; Drive MOSI bit with SCK Low
    in pins, 1          side 0b01   ; SCK rises High: sample MISO into ISR
    jmp x-- bit_loop    side 0b00   ; SCK falls Low: loop for 8 bits
    push block          side 0b10   ; Deassert CS_n High; push received word to RX FIFO
.wrap
