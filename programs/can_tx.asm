; ===========================================================================
; ProtocolForge Firmware: CAN 2.0A Frame Transmitter (1 Mbps)
; ===========================================================================
; Description:
;   Transmits CAN 2.0A standard data frames with autonomous hardware bit-stuffing
;   (complement after 5 identical bits), arbitration collision detection (`jmp arb_lost`),
;   and inline streaming CRC-15 calculation.
;
; Configuration:
;   - Hardware BSU: BSU_CTRL = BSU_CAN (stuffs 5 identical bits)
;   - Hardware CRC: CRC_CTRL = CRC_POLY_CAN15 | CRC_SNOOP_TX
;   - Hardware OD: PIN_OD enabled on CAN TX pin (0=Dominant, 1=Recessive)
;   - Clock divider: 50 MHz / 1 MHz = 50 ticks/bit (DIV_INT = 50)
; ===========================================================================

.program can_tx

.wrap_target
sof:
    pull block                      ; Wait for frame header (Identifier + RTR + DLC)
    set crc, 0                      ; Reset CRC-15 coprocessor to 0x0000
    set pins, 0                     ; Drive Dominant 0 (Start of Frame)
    time t+50
    wait time
header_loop:
    out pins, 1                     ; Shift out header bit
    jmp arb_lost lost               ; Abort immediately if arbitration lost!
    time dl+50                      ; Precise 1 MHz bit deadline
    wait time
    jmp x-- header_loop             ; Loop over arbitration/control field
data_loop:
    pull ifempty block              ; Fetch payload data word from TX FIFO
    out pins, 16                    ; Stream payload data (hardware CRC-15 auto-snoops!)
    time dl+50
    wait time
    jmp y-- data_loop               ; Loop over payload bytes
send_crc:
    mov osr, crc                    ; Load hardware computed CRC-15 into OSR
    out pins, 15                    ; Stream 15-bit CRC (hardware bit-stuffs automatically!)
    time dl+50
    wait time
crc_delim:
    set pins, 1                     ; CRC Delimiter (Recessive 1)
    time dl+50
    wait time
ack_slot:
    set pins, 1                     ; Release bus for ACK slot (Recessive 1)
    time dl+50
    wait time
    in pins, 1                      ; Sample ACK bit from receiver (0 = Dominant ACK)
    push block                      ; Return ACK status to RX FIFO
eof:
    set pins, 1                     ; End of Frame (7 Recessive bits)
    set x, 6                        ; 7 bit times
eof_loop:
    time dl+50
    wait time
    jmp x-- eof_loop
    jmp sof                         ; Transmission complete; ready for next frame
lost:
    set pins, 1                     ; Release bus immediately to avoid corrupting winning master
    mov isr, status
    push block                      ; Alert host of arbitration collision
.wrap
