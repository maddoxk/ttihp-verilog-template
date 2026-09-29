; ===========================================================================
; ProtocolForge Firmware: Dallas / Maxim 1-Wire Master Driver
; ===========================================================================
; Description:
;   Dallas 1-Wire master transceiver. Implements bus reset pulse, presence
;   detect sampling, Write-0 / Write-1 time slots, Read time slots, and
;   hardware CRC-8 Dallas/Maxim calculation.
;
; Configuration:
;   - Hardware Open-Drain: PIN_OD enabled on pin 0 (0=Drive Low, 1=Release High)
;   - Clock divider: 1 us tick (DIV_INT = 50 at 50 MHz)
;   - Hardware CRC: CRC_CTRL = CRC_POLY_DALLAS8 | CRC_SNOOP_TX
; ===========================================================================

.program onewire

.wrap_target
reset_presence:
    set crc, 0                      ; Reset CRC-8 accumulator (poly 0x31, init 0x00)
    set x, 3                        ; 4 x 120 us = 480 us Reset pulse
reset_loop:
    time t+120
    wait time
    jmp x-- reset_loop
    set pins, 1                     ; Release bus to external pull-up
    time dl+70                      ; Wait 70 us to sample Presence Pulse
    wait time
    in pins, 1                      ; Sample Presence pulse (0 = slave present)
    push block                      ; Return presence status to host
    set x, 3                        ; 4 x 100 us = 400 us remainder of presence window
presence_wait:
    time dl+100
    wait time
    jmp x-- presence_wait

slot_loop:
    pull block                      ; Fetch command/data byte from TX FIFO
    set y, 7                        ; 8 bits per byte
bit_slot:
    out x, 1                        ; Extract next data bit into X
    jmp !x write_zero               ; If bit is 0, branch to long Low pulse
write_one:
    set pins, 0                     ; Master pulls bus Low for Write-1
    time t+2                        ; Short 2 us Low pulse
    wait time
    set pins, 1                     ; Release bus High
    time dl+58                      ; Hold High for remainder of 60 us slot
    wait time
    jmp next_bit
write_zero:
    set pins, 0                     ; Master pulls bus Low for Write-0
    time t+60                       ; Long 60 us Low pulse
    wait time
    set pins, 1                     ; Release bus High
    time dl+2                       ; 2 us recovery time
    wait time
next_bit:
    jmp y-- bit_slot                ; Loop over 8 bits
.wrap
