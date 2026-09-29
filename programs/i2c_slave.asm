; ===========================================================================
; ProtocolForge Firmware: I2C Slave
; ===========================================================================
; Description:
;   I2C slave device. Detects START condition, receives 8-bit device address,
;   matches against slave address stored in Y register, generates ACK (SDA=0),
;   and captures payload data bytes.
;
; Configuration:
;   - PIN_OD enabled on SDA and SCL
;   - IN_BASE: pin 0 = SDA, pin 1 = SCL
;   - SET_BASE: pin 0 = SDA
;   - Y register pre-loaded with slave address (e.g. 0x50 << 1)
; ===========================================================================

.program i2c_slave

.wrap_target
wait_start:
    wait 1 pin 1                    ; Wait for SCL High
    wait 0 pin 0                    ; Detect START: SDA falling edge while SCL is High
    set x, 7                        ; 8 bits: 7-bit Address + R/W bit
addr_loop:
    wait 1 pin 1                    ; SCL High: sample SDA bit
    in pins, 1
    wait 0 pin 1                    ; SCL Low: bit interval end
    jmp x-- addr_loop
    push block                      ; Stream received address byte to RX FIFO
match:
    set pins, 0                     ; Address Match: Drive ACK (SDA pulled Low)
    wait 1 pin 1                    ; Clock ACK bit
    wait 0 pin 1
    set pins, 1                     ; Release SDA
    push block                      ; Send matched address to host
data_phase:
    set x, 7                        ; Receive 8-bit data payload
data_loop:
    wait 1 pin 1
    in pins, 1
    wait 0 pin 1
    jmp x-- data_loop
    set pins, 0                     ; Drive ACK for received data
    wait 1 pin 1
    wait 0 pin 1
    set pins, 1                     ; Release SDA
    push block                      ; Stream received data to RX FIFO
    jmp wait_start
no_match:
    mov isr, null                   ; Discard unmatched address and resynchronize
.wrap
