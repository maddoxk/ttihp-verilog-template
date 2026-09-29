; ===========================================================================
; ProtocolForge Firmware: Hardware Transition Sniffer & Logic Analyzer
; ===========================================================================
; Description:
;   Captures digital pin transitions with cycle-exact 20 ns timestamps using
;   the hardware sub-cycle CAPTURE register and streams {pin_levels, timestamp}
;   into the RX FIFO.
;
; Configuration:
;   - Core 2 (Sniffer / Aux Engine)
;   - IN_BASE mapped to monitored GPIO inputs (ui[7:0])
;   - Glitch filter: INFILT enabled for debouncing / deglitching
; ===========================================================================

.program sniffer

.wrap_target
wait_transition:
    wait edge                       ; Block until edge transition occurs on monitored input pin
    in capture, 16                  ; Snapshot 20 ns sub-cycle timestamp register into ISR
    in pins, 8                      ; Sample 8-channel GPIO logic levels
    push noblock                    ; Stream logic packet to host RX FIFO without stalling
.wrap
