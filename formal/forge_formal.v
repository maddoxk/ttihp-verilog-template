/*
 * ProtocolForge Formal Verification Properties (BMC via Yosys SAT)
 *
 * Covers:
 * 1. forge_fifo_formal: No underflow, no overflow, FIFO data ordering preserved.
 * 2. forge_sbe_bsu_formal:
 *    - CAN mode: output pin never stays in identical state > 5 consecutive ticks.
 *    - USB mode: output pin never emits > 6 consecutive 1s without a stuffed 0.
 * 3. forge_scheduler_formal: Wrap-around safety of !((T - DL) >> 15).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none
`include "forge_defs.vh"

// =============================================================================
// 1. FORMAL VERIFICATION: FORGE_FIFO
// =============================================================================
module forge_fifo_formal (
    input wire        clk,
    input wire        rst_n,
    input wire        clear,
    input wire        push,
    input wire [15:0] wdata,
    input wire        pop
);
    localparam AW = 3;
    localparam DEPTH = 1 << AW; // 8 words

    wire [15:0] rdata;
    wire        full;
    wire        empty;
    wire [AW:0] level;

    forge_fifo #(
        .DATA_WIDTH(16),
        .ADDR_WIDTH(AW)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .clear(clear),
        .wdata(wdata),
        .push(push),
        .full(full),
        .rdata(rdata),
        .pop(pop),
        .empty(empty),
        .level(level)
    );

    // Past validity and synchronous reset assumptions
    reg f_past_valid;
    initial f_past_valid = 0;
    always @(posedge clk) begin
        f_past_valid <= 1;
        if (f_past_valid) begin
            assume(rst_n == 1'b1);
            assume(clear == 1'b0);
        end
    end

    // Formal token tracking to prove FIFO data ordering
    reg        f_token_active;
    reg [15:0] f_token_data;
    reg [AW:0] f_token_ahead;

    wire push_en = push && (!full || pop);
    wire pop_en  = pop  && !empty;

    always @(posedge clk) begin
        if (!rst_n || clear) begin
            f_token_active <= 1'b0;
            f_token_data   <= 16'd0;
            f_token_ahead  <= 0;
        end else begin
            if (!f_token_active && push_en) begin
                f_token_active <= 1'b1;
                f_token_data   <= wdata;
                // If popping in the same cycle, item at head is removed
                f_token_ahead  <= pop_en ? (level - 1'b1) : level;
            end else if (f_token_active) begin
                if (pop_en) begin
                    if (f_token_ahead == 0) begin
                        assert(rdata == f_token_data);
                        f_token_active <= 1'b0;
                    end else begin
                        f_token_ahead <= f_token_ahead - 1'b1;
                    end
                end
            end
        end
    end

    always @(posedge clk) begin
        if (rst_n && !clear) begin
            // Safety 1: Occupancy bounds
            assert(level <= DEPTH);
            assert(empty == (level == 0));
            assert(full  == (level == DEPTH));

            // Safety 2: Underflow and overflow protection
            if (f_past_valid && $past(rst_n) && !$past(clear)) begin
                if ($past(empty) && !$past(push)) begin
                    assert(empty);
                    assert(level == 0);
                end
                if ($past(full) && !$past(pop)) begin
                    assert(full);
                    assert(level == DEPTH);
                end
                assert(level == $past(level) || level == $past(level) + 1 || level == $past(level) - 1);
            end
        end
    end

endmodule


// =============================================================================
// 2. FORMAL VERIFICATION: FORGE_SBE_BSU
// =============================================================================
module forge_sbe_bsu_formal (
    input wire       clk,
    input wire       rst_n,
    input wire       clear,
    input wire       mode_sel,    // 0 = CAN mode, 1 = USB mode
    input wire       tx_bit_in    // Arbitrary streaming payload bits from firmware
);
    wire [1:0] bsu_mode = mode_sel ? `FORGE_BSU_USB : `FORGE_BSU_CAN;
    wire [2:0] sbe_mode = `FORGE_SBE_NRZ;
    wire       tx_tick  = 1'b1;   // Run bit-ticks continuously

    wire tx_stall_osr;
    wire tx_bit_crc;
    wire tx_crc_en;
    wire tx_pin_out;

    wire rx_bit_out;
    wire rx_valid;
    wire rx_stuff_err;

    forge_sbe_bsu dut (
        .clk(clk),
        .rst_n(rst_n),
        .clear(clear),
        .sbe_mode(sbe_mode),
        .bsu_mode(bsu_mode),
        .tx_tick(tx_tick),
        .tx_bit_in(tx_bit_in),
        .tx_stall_osr(tx_stall_osr),
        .tx_bit_crc(tx_bit_crc),
        .tx_crc_en(tx_crc_en),
        .tx_pin_out(tx_pin_out),
        .rx_tick(tx_tick),
        .rx_pin_in(tx_pin_out),   // Loopback TX pin to RX
        .rx_bit_out(rx_bit_out),
        .rx_valid(rx_valid),
        .rx_stuff_err(rx_stuff_err)
    );

    // Track consecutive identical bits on output
    reg [3:0] consecutive_identical;
    reg       prev_pin;

    // Track consecutive 1s on output
    reg [3:0] consecutive_ones;
    reg       started;

    reg f_past_valid;
    initial f_past_valid = 0;
    always @(posedge clk) begin
        f_past_valid <= 1;
        if (f_past_valid) begin
            assume(rst_n == 1'b1);
            assume(clear == 1'b0);
            assume(mode_sel == $past(mode_sel));
            if ($past(tx_stall_osr)) begin
                assume(tx_bit_in == $past(tx_bit_in));
            end
        end
    end

    always @(posedge clk) begin
        if (!rst_n || clear) begin
            consecutive_identical <= 4'd1;
            prev_pin              <= 1'b0;
            consecutive_ones      <= 4'd0;
            started               <= 1'b0;
        end else begin
            if (!started) begin
                started               <= 1'b1;
                prev_pin              <= tx_pin_out;
                consecutive_identical <= 4'd1;
                consecutive_ones      <= tx_pin_out ? 4'd1 : 4'd0;
            end else begin
                prev_pin <= tx_pin_out;
                if (tx_pin_out == prev_pin) begin
                    consecutive_identical <= consecutive_identical + 1'b1;
                end else begin
                    consecutive_identical <= 4'd1;
                end

                if (tx_pin_out == 1'b1) begin
                    consecutive_ones <= consecutive_ones + 1'b1;
                end else begin
                    consecutive_ones <= 4'd0;
                end
            end
        end
    end

    always @(posedge clk) begin
        if (f_past_valid && rst_n && !clear) begin
            // CAN Rule: Output never stays in identical state > 5 consecutive bit ticks
            if (bsu_mode == `FORGE_BSU_CAN) begin
                assert(consecutive_identical <= 4'd5);
            end

            // USB Rule: Output never emits > 6 consecutive 1s without a stuffed 0
            if (bsu_mode == `FORGE_BSU_USB) begin
                assert(consecutive_ones <= 4'd6);
            end

            // Loopback de-stuffer should never assert stuff error on well-formed stream
            assert(!rx_stuff_err);
        end
    end

endmodule


// =============================================================================
// 3. FORMAL VERIFICATION: FORGE_SCHEDULER (WRAP-AROUND SAFETY)
// =============================================================================
module forge_scheduler_formal (
    input wire        clk,
    input wire        rst_n,
    input wire [15:0] sched_delay // N cycles in future (1 <= N <= 1000)
);
    reg [15:0] t;
    reg [15:0] dl;
    reg [15:0] step_count;
    reg        scheduled;

    // Signed comparison from architecture specification
    wire signed [15:0] t_diff = t - dl;
    wire dl_reached = !t_diff[15];

    // Constrain delay to valid positive scheduling window [1 .. 1000]
    wire valid_delay = (sched_delay >= 16'd1) && (sched_delay <= 16'd1000);

    reg f_past_valid;
    initial f_past_valid = 0;
    always @(posedge clk) begin
        f_past_valid <= 1;
        if (f_past_valid) begin
            assume(rst_n == 1'b1);
            assume(sched_delay == $past(sched_delay));
        end
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            t          <= 16'hFFFA; // Start near 16-bit boundary to exercise wrap-around
            dl         <= 16'd0;
            step_count <= 16'd0;
            scheduled  <= 1'b0;
        end else begin
            t <= t + 16'd1;

            if (!scheduled && valid_delay) begin
                dl         <= t + sched_delay; // Can wrap around 0xFFFF -> 0x0000!
                step_count <= 16'd1;
                scheduled  <= 1'b1;
            end else if (scheduled) begin
                step_count <= step_count + 16'd1;
            end
        end
    end

    always @(posedge clk) begin
        if (rst_n && scheduled) begin
            // Before deadline (cycles elapsed < scheduled delay): must be FALSE
            if (step_count < sched_delay) begin
                assert(!dl_reached);
            end

            // At the exact deadline cycle: must trigger TRUE
            if (step_count == sched_delay) begin
                assert(dl_reached);
                assert(t == dl);
            end

            // For the post-deadline window: remains TRUE
            if (step_count >= sched_delay && step_count < sched_delay + 16'd100) begin
                assert(dl_reached);
            end
        end
    end

endmodule
