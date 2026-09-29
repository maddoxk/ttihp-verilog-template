/*
 * ProtocolForge Hardware Auto-Baud & Edge Capture Unit (ABU)
 * Tracks MIN_PULSE (shortest pulse width in 20ns cycles) and timestamps edges.
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module forge_abu (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        pin_in,         // Monitored GPIO input (e.g. ui_in[0])
    input  wire [15:0] current_time,   // System 16-bit timebase (20ns resolution)

    // Host / Register interface
    input  wire        clear,          // Reset min_pulse to 0xFFFF
    input  wire        write_en,       // Host write to min_pulse
    input  wire [15:0] write_data,
    output wire [15:0] min_pulse,      // Shortest observed pulse width

    // Edge capture
    output reg  [15:0] capture_val,    // Most recent edge timestamp
    output reg         edge_detected,  // 1-cycle strobe on any edge
    output reg  [15:0] sniff_packet,   // Format: {2'b00, pin_level, delta_t[12:0]}
    output reg         sniff_valid     // 1-cycle strobe when packet is ready
);

    reg        prev_pin;
    reg [15:0] pulse_cnt;
    reg [15:0] min_pulse_reg;
    reg [15:0] last_edge_time;

    assign min_pulse = min_pulse_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_pin       <= 1'b1;
            pulse_cnt      <= 16'd0;
            min_pulse_reg  <= 16'hFFFF;
            last_edge_time <= 16'd0;
            capture_val    <= 16'd0;
            edge_detected  <= 1'b0;
            sniff_packet   <= 16'd0;
            sniff_valid    <= 1'b0;
        end else if (clear) begin
            pulse_cnt      <= 16'd0;
            min_pulse_reg  <= 16'hFFFF;
            edge_detected  <= 1'b0;
            sniff_valid    <= 1'b0;
        end else if (write_en) begin
            min_pulse_reg  <= write_data;
            edge_detected  <= 1'b0;
            sniff_valid    <= 1'b0;
        end else begin
            if (pin_in != prev_pin) begin
                prev_pin       <= pin_in;
                capture_val    <= current_time;
                edge_detected  <= 1'b1;

                // Track minimum pulse duration (filter glitches < 2 cycles = 40ns)
                if (pulse_cnt > 16'd2 && pulse_cnt < min_pulse_reg) begin
                    min_pulse_reg <= pulse_cnt;
                end
                pulse_cnt <= 16'd1;

                // Sniffer streaming packet: {2'b00, pin_level, delta_t[12:0]}
                sniff_packet <= {2'b00, pin_in, (current_time[12:0] - last_edge_time[12:0])};
                sniff_valid  <= 1'b1;
                last_edge_time <= current_time;
            end else begin
                edge_detected <= 1'b0;
                sniff_valid   <= 1'b0;
                if (pulse_cnt != 16'hFFFF) begin
                    pulse_cnt <= pulse_cnt + 16'd1;
                end
            end
        end
    end

endmodule
