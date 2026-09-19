/*
 * Copyright (c) 2024-2026 Nuat Labs
 * Author: Nuat Labs Team
 * SPDX-License-Identifier: Apache-2.0
 *
 * Description:
 *   Nuat Labs High-Performance Configurable SPI Master Controller.
 *   Features:
 *     - Full support for SPI Modes 0, 1, 2, and 3 (CPOL / CPHA).
 *     - Configurable word length from 1 to 16 bits.
 *     - MSB-first or LSB-first bit order selection.
 *     - Programmable clock divider for baud rate generation.
 *     - Parallel-to-serial transmit shift register (PISO).
 *     - Serial-to-parallel receive shift register (SIPO).
 *     - Automatic and manual Chip Select (CS) management with dual-slave support.
 *     - Internal loopback mode for built-in self-test (BIST).
 *     - Standard 8-bit memory-mapped host register interface.
 */

`default_nettype none
`timescale 1ns / 1ps

module tt_um_nuatlabs_spi (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // Tiny Tapeout enable signal (active high)
    input  wire       clk,      // System clock
    input  wire       rst_n     // Active-low asynchronous/synchronous reset
);

    // =========================================================================
    // Pinout Mapping
    // =========================================================================
    // ui_in:
    //   ui_in[0]   : spi_miso      - Master In Slave Out (SPI input)
    //   ui_in[1]   : host_cs_n     - Host bus chip select (active low)
    //   ui_in[2]   : host_we       - Host bus write enable (1=write, 0=read)
    //   ui_in[5:3] : host_addr     - Host register address (3 bits, 0-7)
    //   ui_in[6]   : direct_start  - Direct hardware start strobe (rising edge)
    //   ui_in[7]   : unused_in     - Reserved
    //
    // uo_out:
    //   uo_out[0]  : spi_sclk      - SPI Serial Clock output
    //   uo_out[1]  : spi_mosi      - SPI Master Out Slave In output
    //   uo_out[2]  : spi_cs0_n     - SPI Chip Select 0 (active low, primary slave)
    //   uo_out[3]  : spi_busy      - SPI transfer busy flag
    //   uo_out[4]  : spi_done      - SPI transfer complete strobe (1 cycle)
    //   uo_out[5]  : spi_rx_ready  - Received data available flag
    //   uo_out[6]  : spi_cs1_n     - SPI Chip Select 1 (active low, secondary slave)
    //   uo_out[7]  : spi_irq       - Interrupt request (asserts when done, if enabled)
    //
    // uio:
    //   uio_in[7:0]  : Host write data bus
    //   uio_out[7:0] : Host read data bus
    //   uio_oe[7:0]  : Host data bus output enable (active high during host read)
    // =========================================================================

    wire        spi_miso     = ui_in[0];
    wire        host_cs_n    = ui_in[1];
    wire        host_we      = ui_in[2];
    wire [2:0]  host_addr    = ui_in[5:3];
    wire        direct_start = ui_in[6];

    // Prevent unused input warning for ena and ui_in[7]
    wire _unused_signals = &{ena, ui_in[7], 1'b0};

    // =========================================================================
    // Register Map Address Definitions
    // =========================================================================
    localparam [2:0] ADDR_CTRL      = 3'd0; // Control / Status
    localparam [2:0] ADDR_CLKDIV    = 3'd1; // Clock divider
    localparam [2:0] ADDR_WORDLEN   = 3'd2; // Word length (1-16 bits)
    localparam [2:0] ADDR_TX_DATA_L = 3'd3; // TX data low byte [7:0]
    localparam [2:0] ADDR_TX_DATA_H = 3'd4; // TX data high byte [15:8]
    localparam [2:0] ADDR_RX_DATA_L = 3'd5; // RX data low byte [7:0]
    localparam [2:0] ADDR_RX_DATA_H = 3'd6; // RX data high byte [15:8]
    localparam [2:0] ADDR_SLAVE_SEL = 3'd7; // Slave select & auxiliary control

    // =========================================================================
    // Internal Control & Configuration Registers
    // =========================================================================
    reg        reg_cpol;
    reg        reg_cpha;
    reg        reg_lsb_first;
    reg        reg_auto_cs;
    reg        reg_manual_cs;
    reg        reg_loopback;
    reg        reg_irq_en;
    reg [7:0]  reg_clkdiv;
    reg [4:0]  reg_wordlen;     // 1 to 16 bits (0 defaults to 8 bits)
    reg [15:0] reg_tx_data;
    reg [15:0] reg_rx_data;
    reg        reg_slave_sel;   // 0: slave 0, 1: slave 1

    // Status flags
    reg        reg_done_sticky;
    reg        reg_rx_ready;
    reg        reg_irq;

    // Edge detector for direct_start
    reg        direct_start_d;
    wire       direct_start_pulse = direct_start && !direct_start_d;

    // Start strobe generated either by software write to ADDR_CTRL[7] or direct_start
    reg        soft_start_pulse;
    wire       start_transfer = soft_start_pulse || direct_start_pulse;

    // Host read bus
    reg [7:0]  host_read_data;

    // Enable bidirectional output buffer only when host reads with chip select asserted
    wire host_read_active = (!host_cs_n) && (!host_we);
    assign uio_oe  = host_read_active ? 8'hFF : 8'h00;
    assign uio_out = host_read_data;

    // =========================================================================
    // SPI Core Signals
    // =========================================================================
    wire        core_sclk;
    wire        core_mosi;
    wire        core_cs_n;
    wire        core_busy;
    wire        core_done;
    wire [15:0] core_rx_data;

    // Effective MISO taking internal loopback into account
    wire effective_miso = reg_loopback ? core_mosi : spi_miso;

    // Effective CS per slave
    wire spi_cs0_out = (reg_slave_sel == 1'b0) ? (reg_auto_cs ? core_cs_n : reg_manual_cs) : 1'b1;
    wire spi_cs1_out = (reg_slave_sel == 1'b1) ? (reg_auto_cs ? core_cs_n : reg_manual_cs) : 1'b1;

    // Assign dedicated outputs
    assign uo_out[0] = core_sclk;
    assign uo_out[1] = core_mosi;
    assign uo_out[2] = spi_cs0_out;
    assign uo_out[3] = core_busy;
    assign uo_out[4] = core_done;
    assign uo_out[5] = reg_rx_ready;
    assign uo_out[6] = spi_cs1_out;
    assign uo_out[7] = reg_irq;

    // =========================================================================
    // Host Register Read/Write Logic & Status Management
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg_cpol        <= 1'b0;
            reg_cpha        <= 1'b0;
            reg_lsb_first   <= 1'b0;
            reg_auto_cs     <= 1'b1; // Default auto chip select
            reg_manual_cs   <= 1'b1; // Inactive high when manual
            reg_loopback    <= 1'b0;
            reg_irq_en      <= 1'b0;
            reg_clkdiv      <= 8'd1; // Default divider: f_clk / 4
            reg_wordlen     <= 5'd8; // Default 8-bit transfer
            reg_tx_data     <= 16'h0000;
            reg_rx_data     <= 16'h0000;
            reg_slave_sel   <= 1'b0;
            reg_done_sticky <= 1'b0;
            reg_rx_ready    <= 1'b0;
            reg_irq         <= 1'b0;
            soft_start_pulse<= 1'b0;
            direct_start_d  <= 1'b0;
        end else begin
            direct_start_d   <= direct_start;
            soft_start_pulse <= 1'b0; // Single-cycle strobe

            // Latch core completion
            if (core_done) begin
                reg_done_sticky <= 1'b1;
                reg_rx_ready    <= 1'b1;
                reg_rx_data     <= core_rx_data;
                if (reg_irq_en) begin
                    reg_irq     <= 1'b1;
                end
            end

            // Clear sticky status on transfer initiation
            if (start_transfer) begin
                reg_done_sticky <= 1'b0;
                reg_irq         <= 1'b0;
            end

            // Host Write Operations
            if (!host_cs_n && host_we) begin
                case (host_addr)
                    ADDR_CTRL: begin
                        reg_cpol      <= uio_in[0];
                        reg_cpha      <= uio_in[1];
                        reg_lsb_first <= uio_in[2];
                        reg_auto_cs   <= uio_in[3];
                        reg_manual_cs <= uio_in[4];
                        reg_loopback  <= uio_in[5];
                        reg_irq_en    <= uio_in[6];
                        if (uio_in[7] && !core_busy) begin
                            soft_start_pulse <= 1'b1;
                        end
                    end
                    ADDR_CLKDIV: begin
                        reg_clkdiv <= uio_in;
                    end
                    ADDR_WORDLEN: begin
                        // Enforce valid range: 1 to 16 (0 treated as 8)
                        if (uio_in[4:0] == 5'd0)
                            reg_wordlen <= 5'd8;
                        else if (uio_in[4:0] > 5'd16)
                            reg_wordlen <= 5'd16;
                        else
                            reg_wordlen <= uio_in[4:0];
                    end
                    ADDR_TX_DATA_L: begin
                        reg_tx_data[7:0] <= uio_in;
                    end
                    ADDR_TX_DATA_H: begin
                        reg_tx_data[15:8] <= uio_in;
                    end
                    ADDR_SLAVE_SEL: begin
                        reg_slave_sel <= uio_in[0];
                    end
                    default: ;
                endcase
            end

            // Host Read Clear behavior: reading ADDR_CTRL or ADDR_RX_DATA_L clears sticky done/irq
            if (!host_cs_n && !host_we) begin
                if (host_addr == ADDR_CTRL || host_addr == ADDR_RX_DATA_L) begin
                    reg_done_sticky <= 1'b0;
                    reg_irq         <= 1'b0;
                end
            end
        end
    end

    // Host Read Data Multiplexer
    always @(*) begin
        case (host_addr)
            ADDR_CTRL: begin
                host_read_data = {
                    reg_loopback,
                    reg_auto_cs,
                    reg_rx_ready,
                    reg_done_sticky,
                    core_busy,
                    reg_lsb_first,
                    reg_cpha,
                    reg_cpol
                };
            end
            ADDR_CLKDIV: begin
                host_read_data = reg_clkdiv;
            end
            ADDR_WORDLEN: begin
                host_read_data = {3'b000, reg_wordlen};
            end
            ADDR_TX_DATA_L: begin
                host_read_data = reg_tx_data[7:0];
            end
            ADDR_TX_DATA_H: begin
                host_read_data = reg_tx_data[15:8];
            end
            ADDR_RX_DATA_L: begin
                host_read_data = reg_rx_data[7:0];
            end
            ADDR_RX_DATA_H: begin
                host_read_data = reg_rx_data[15:8];
            end
            ADDR_SLAVE_SEL: begin
                host_read_data = {7'b0000000, reg_slave_sel};
            end
            default: begin
                host_read_data = 8'h00;
            end
        endcase
    end

    // =========================================================================
    // SPI Core Engine Instantiation
    // =========================================================================
    nuatlabs_spi_core u_spi_core (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (start_transfer),
        .cpol       (reg_cpol),
        .cpha       (reg_cpha),
        .lsb_first  (reg_lsb_first),
        .clk_div    (reg_clkdiv),
        .word_len   (reg_wordlen),
        .tx_data    (reg_tx_data),
        .miso       (effective_miso),
        .sclk       (core_sclk),
        .mosi       (core_mosi),
        .cs_n       (core_cs_n),
        .busy       (core_busy),
        .done       (core_done),
        .rx_data    (core_rx_data)
    );

endmodule


// =============================================================================
// Nuat Labs SPI Master Core Engine
// =============================================================================
// Handles:
//   - Precise timing for all 4 CPOL/CPHA modes.
//   - SCLK generation with configurable divider.
//   - Variable word length (1 to 16 bits).
//   - MSB-first / LSB-first bit sequencing.
//   - CS lead and trail guard times.
// =============================================================================
module nuatlabs_spi_core (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire        cpol,
    input  wire        cpha,
    input  wire        lsb_first,
    input  wire [7:0]  clk_div,
    input  wire [4:0]  word_len,
    input  wire [15:0] tx_data,
    input  wire        miso,
    output reg         sclk,
    output reg         mosi,
    output reg         cs_n,
    output reg         busy,
    output reg         done,
    output reg  [15:0] rx_data
);

    // State machine encodings
    localparam [2:0] ST_IDLE       = 3'd0;
    localparam [2:0] ST_LEAD_WAIT  = 3'd1;
    localparam [2:0] ST_HALF_1     = 3'd2;
    localparam [2:0] ST_HALF_2     = 3'd3;
    localparam [2:0] ST_TRAIL_WAIT = 3'd4;

    reg [2:0]  state;
    reg [7:0]  div_cnt;
    reg [4:0]  bit_idx;     // Counts from 0 up to active_len - 1
    reg [4:0]  active_len;
    reg        latched_cpol;
    reg        latched_cpha;
    reg        latched_lsb_first;
    reg [15:0] latched_tx_data;
    reg [15:0] rx_shift_reg;

    // Helper function to extract bit according to bit_idx, length, and lsb_first
    function get_tx_bit;
        input [15:0] data;
        input [4:0]  idx;
        input [4:0]  len;
        input        is_lsb_first;
        begin
            if (is_lsb_first) begin
                get_tx_bit = data[idx];
            end else begin
                // In MSB-first, bit 0 sent is data[len - 1]
                get_tx_bit = data[len - 1'b1 - idx];
            end
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state             <= ST_IDLE;
            sclk              <= 1'b0;
            mosi              <= 1'b0;
            cs_n              <= 1'b1;
            busy              <= 1'b0;
            done              <= 1'b0;
            rx_data           <= 16'h0000;
            rx_shift_reg      <= 16'h0000;
            div_cnt           <= 8'd0;
            bit_idx           <= 5'd0;
            active_len        <= 5'd8;
            latched_cpol      <= 1'b0;
            latched_cpha      <= 1'b0;
            latched_lsb_first <= 1'b0;
            latched_tx_data   <= 16'h0000;
        end else begin
            done <= 1'b0; // Single-cycle strobe default

            case (state)
                ST_IDLE: begin
                    busy <= 1'b0;
                    cs_n <= 1'b1;
                    sclk <= cpol; // SCLK remains at configured idle level
                    mosi <= 1'b0;

                    if (start) begin
                        busy              <= 1'b1;
                        latched_cpol      <= cpol;
                        latched_cpha      <= cpha;
                        latched_lsb_first <= lsb_first;
                        latched_tx_data   <= tx_data;
                        active_len        <= (word_len == 5'd0) ? 5'd8 :
                                             (word_len > 5'd16) ? 5'd16 : word_len;
                        bit_idx           <= 5'd0;
                        div_cnt           <= clk_div;
                        rx_shift_reg      <= 16'h0000;
                        cs_n              <= 1'b0; // Assert CS
                        sclk              <= cpol; // Ensure idle level

                        // In CPHA=0, data must be valid on MOSI prior to the first clock edge!
                        if (!cpha) begin
                            mosi <= get_tx_bit(tx_data, 5'd0,
                                               (word_len == 5'd0) ? 5'd8 :
                                               (word_len > 5'd16) ? 5'd16 : word_len,
                                               lsb_first);
                        end

                        state <= ST_LEAD_WAIT;
                    end
                end

                // Lead guard time between CS assertion and first SCLK edge
                ST_LEAD_WAIT: begin
                    if (div_cnt == 8'd0) begin
                        div_cnt <= clk_div;
                        sclk    <= ~latched_cpol; // First (leading) edge transition

                        // In CPHA=1, first bit is driven on the leading edge
                        if (latched_cpha) begin
                            mosi <= get_tx_bit(latched_tx_data, bit_idx, active_len, latched_lsb_first);
                        end else begin
                            // In CPHA=0, leading edge is the sample edge!
                            if (latched_lsb_first)
                                rx_shift_reg[bit_idx] <= miso;
                            else
                                rx_shift_reg[active_len - 1'b1 - bit_idx] <= miso;
                        end

                        state <= ST_HALF_1;
                    end else begin
                        div_cnt <= div_cnt - 1'b1;
                    end
                end

                // Half period 1 (Leading edge active)
                ST_HALF_1: begin
                    if (div_cnt == 8'd0) begin
                        div_cnt <= clk_div;
                        sclk    <= latched_cpol; // Second (trailing) edge transition

                        if (latched_cpha) begin
                            // In CPHA=1, trailing edge is the sample edge!
                            if (latched_lsb_first)
                                rx_shift_reg[bit_idx] <= miso;
                            else
                                rx_shift_reg[active_len - 1'b1 - bit_idx] <= miso;

                            // If not the last bit, prepare next bit index
                            if (bit_idx == active_len - 1'b1) begin
                                state <= ST_TRAIL_WAIT;
                            end else begin
                                bit_idx <= bit_idx + 1'b1;
                                state   <= ST_HALF_2;
                            end
                        end else begin
                            // In CPHA=0, trailing edge is the shift edge!
                            if (bit_idx == active_len - 1'b1) begin
                                state <= ST_TRAIL_WAIT;
                            end else begin
                                bit_idx <= bit_idx + 1'b1;
                                mosi    <= get_tx_bit(latched_tx_data, bit_idx + 1'b1, active_len, latched_lsb_first);
                                state   <= ST_HALF_2;
                            end
                        end
                    end else begin
                        div_cnt <= div_cnt - 1'b1;
                    end
                end

                // Half period 2 (Returning to idle level)
                ST_HALF_2: begin
                    if (div_cnt == 8'd0) begin
                        div_cnt <= clk_div;
                        sclk    <= ~latched_cpol; // Next leading edge

                        if (latched_cpha) begin
                            // Drive next bit on leading edge
                            mosi  <= get_tx_bit(latched_tx_data, bit_idx, active_len, latched_lsb_first);
                        end else begin
                            // Sample on leading edge
                            if (latched_lsb_first)
                                rx_shift_reg[bit_idx] <= miso;
                            else
                                rx_shift_reg[active_len - 1'b1 - bit_idx] <= miso;
                        end

                        state <= ST_HALF_1;
                    end else begin
                        div_cnt <= div_cnt - 1'b1;
                    end
                end

                // Trail guard time between last clock edge and CS deassertion
                ST_TRAIL_WAIT: begin
                    sclk <= latched_cpol; // Ensure idle clock level
                    if (div_cnt == 8'd0) begin
                        cs_n    <= 1'b1; // Deassert CS
                        busy    <= 1'b0;
                        done    <= 1'b1; // Pulse done
                        rx_data <= rx_shift_reg;
                        state   <= ST_IDLE;
                    end else begin
                        div_cnt <= div_cnt - 1'b1;
                    end
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
