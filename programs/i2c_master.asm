; ===========================================================================
; ProtocolForge Firmware: I2C Master
; ===========================================================================
; Description:
;   I2C master with open-drain SDA/SCL drivers, START/STOP framing,
;   ACK/NACK handling, clock-stretching interlock, and arbitration monitoring.
;
; Configuration:
;   - Hardware Open-Drain: PIN_OD enabled on SDA and SCL pins
;   - Side-set 2 bits: [SCL, SDA]
;       0 = Drive Low, 1 = Release High (Open-Drain)
;       side 0b11: SCL=1, SDA=1 (Bus Idle)
;       side 0b10: SCL=1, SDA=0 (START condition)
;       side 0b00: SCL=0, SDA=0 (Clock Low)
;       side 0b01: SCL=0, SDA=1 (SDA released for ACK)
;   - IN_BASE mapped to SDA (pin 0) and SCL (pin 1)
;   - OUT_BASE mapped to SDA (pin 0)
; ===========================================================================

.program i2c_master
.side_set 2

.wrap_target
start:
    pull block          side 0b11   ; Wait for command; bus idle (SCL=1, SDA=1)
    nop                 side 0b10   ; START condition: SDA pulled Low while SCL High
    set x, 7            side 0b00   ; SCL pulled Low; 8 data bits
data_loop:
    out pins, 1         side 0b00   ; Drive SDA bit while SCL Low
    jmp arb_lost lost   side 0b00   ; Arbitration loss abort: another master driving dominant
    nop                 side 0b10   ; Release SCL High
    wait 1 pin 1        side 0b10   ; Clock Stretching: wait for slave to release SCL
    jmp x-- data_loop   side 0b00   ; Pull SCL Low and loop for 8 bits
ack_phase:
    nop                 side 0b01   ; Release SDA High (1) for slave ACK slot
    in pins, 1          side 0b11   ; SCL rises High: sample ACK (0=ACK, 1=NACK)
    wait 1 pin 1        side 0b11   ; Clock stretching wait on ACK
    nop                 side 0b00   ; Pull SCL Low
stop:
    nop                 side 0b10   ; SCL High, SDA Low
    nop                 side 0b11   ; STOP condition: SDA transitions Low -> High while SCL High
    push block          side 0b11   ; Return ACK status to RX FIFO
    jmp start           side 0b11   ; Ready for next transaction
lost:
    set pins, 0b11      side 0b11   ; Release SDA and SCL lines immediately
    mov isr, status     side 0b11   ; Report collision to host
    push block          side 0b11
.wrap
