; ===========================================================================
; ProtocolForge Firmware: Low-Speed USB 1.1 Transmitter (1.5 Mbps)
; ===========================================================================
; Description:
;   Transmits Low-Speed USB 1.1 packets at wire-speed (1.5 Mbps) using dedicated
;   hardware accelerators:
;     1. SBE (Stream Bit Engine): Real-time NRZI modulation
;     2. BSU (Bit-Stuffing Unit): Zero-jitter insertion of '0' after 6 consecutive '1's
;     3. CRC Coprocessor: Inline USB CRC-16 generation (poly 0x8005)
;     4. Single-Ended Zero (SE0) End-of-Packet (EOP) framing
;
; Configuration:
;   - SBE_CTRL = SBE_NRZI
;   - BSU_CTRL = BSU_USB
;   - CRC_CTRL = CRC_POLY_USB16 | CRC_SNOOP_TX
;   - Clock divider: 50 MHz / 1.5 MHz = 33.33 ticks/bit (DIV_INT=33, DIV_FRAC=85)
;   - Pin 0 = D+, Pin 1 = D-
;       Differential '1' (J-state, idle): D-=1, D+=0 (pins = 0b01)
;       Differential '0' (K-state):       D-=0, D+=1 (pins = 0b10)
;       SE0 (End of Packet):              D-=0, D+=0 (pins = 0b00)
; ===========================================================================

.program usb_ls_tx

.wrap_target
sync:
    pull block                      ; Wait for packet transmission trigger
    set crc, 1                      ; Preset CRC-16 accumulator to 0xFFFF
    set pins, 0b01                  ; Idle J-state (D-=1, D+=0)
payload_loop:
    pull block                      ; Fetch 16-bit payload word from TX FIFO
    out pins, 16                    ; Stream 16 bits (hardware NRZI, BSU, CRC active!)
    jmp y-- payload_loop            ; Loop over packet payload
send_crc:
    mov osr, crc                    ; Load hardware computed CRC-16 into OSR
    out pins, 16                    ; Stream CRC (automatically bit-stuffed & NRZI encoded!)
eop:
    set pins, 0b00                  ; Emit SE0 (End of Packet: D+ and D- both Low)
    time t+66                       ; Hold SE0 for 2 bit periods (~1.33 us)
    wait time
    set pins, 0b01                  ; Return to J-state (Idle) for 1 bit period
    time dl+33
    wait time
.wrap
