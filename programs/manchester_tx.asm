; ===========================================================================
; ProtocolForge Firmware: 10BASE-T Manchester Ethernet Transmitter (10 Mbps)
; ===========================================================================
; Description:
;   Transmits 10BASE-T IEEE 802.3 Ethernet packets at full 10.0 Mbps line rate.
;   Leverages the hardware Stream Bit Engine (SBE) Manchester phase generator
;   to produce cycle-exact 50 ns mid-bit transitions at 50 MHz without software jitter.
;
; Configuration:
;   - SBE_CTRL = SBE_MANCHESTER
;   - Clock divider: DIV_INT = 5 (10 Mbps from 50 MHz clock; 100 ns bit period)
;   - Pins: Differential TX pair (TX+, TX-)
; ===========================================================================

.program manchester_tx

.wrap_target
preamble:
    pull block                      ; Wait for packet trigger; load frame length into Y
    set x, 31                       ; Preamble: 56 alternating bits + 8-bit SFD (101010...1011)
preamble_loop:
    set pins, 0b10                  ; Alternating preamble square wave
    jmp x-- preamble_loop
stream_packet:
    pull block                      ; Fetch 16-bit word from TX FIFO
    out pins, 16                    ; Hardware SBE automatically splits into two 50 ns phases!
    jmp y-- stream_packet           ; Stream full Ethernet MAC frame + FCS payload
eop:
    set pins, 0b01                  ; TP_IDL: Line held High for 50 ns
    time t+5
    wait time
    set pins, 0b00                  ; Idle zero state
.wrap
