/*
 * Copyright (c) 2024-2026 Nuat Labs
 * Author: Nuat Labs Team
 * SPDX-License-Identifier: Apache-2.0
 *
 * Nuat Labs Standalone Verification Testbench for SPI Master Controller.
 * Can be compiled and executed directly with Icarus Verilog:
 *   iverilog -Wall -g2012 -s tb_standalone -o sim.vvp src/project.v test/tb_standalone.v
 *   vvp sim.vvp
 */

`default_nettype none
`timescale 1ns / 1ps

module tb_standalone ();

    // Testbench signals
    reg        clk;
    reg        rst_n;
    reg        ena;
    reg  [7:0] ui_in;
    reg  [7:0] uio_in;
    wire [7:0] uo_out;
    wire [7:0] uio_out;
    wire [7:0] uio_oe;

    // Aliases for convenience
    wire spi_sclk   = uo_out[0];
    wire spi_mosi   = uo_out[1];
    wire spi_cs0_n  = uo_out[2];
    wire spi_busy   = uo_out[3];
    wire spi_done   = uo_out[4];
    wire spi_rx_rdy = uo_out[5];
    wire spi_cs1_n  = uo_out[6];
    wire spi_irq    = uo_out[7];

    // Instantiate Device Under Test (DUT)
    tt_um_nuatlabs_spi dut (
        .ui_in   (ui_in),
        .uo_out  (uo_out),
        .uio_in  (uio_in),
        .uio_out (uio_out),
        .uio_oe  (uio_oe),
        .ena     (ena),
        .clk     (clk),
        .rst_n   (rst_n)
    );

    // Clock Generation: 50 MHz (20 ns period)
    always #10 clk = ~clk;

    // Test metrics
    integer tests_passed = 0;
    integer tests_failed = 0;

    // Host bus write task
    task host_write(input [2:0] addr, input [7:0] data);
        begin
            @(negedge clk);
            ui_in  = (ui_in & 8'h01) | (1'b0 << 1) | (1'b1 << 2) | (addr << 3); // cs_n=0, we=1
            uio_in = data;
            @(posedge clk);
            @(negedge clk);
            ui_in  = (ui_in & 8'h01) | (1'b1 << 1) | (1'b0 << 2); // cs_n=1, we=0
            uio_in = 8'h00;
        end
    endtask

    // Host bus read task
    task host_read(input [2:0] addr, output [7:0] data);
        begin
            @(negedge clk);
            ui_in  = (ui_in & 8'h01) | (1'b0 << 1) | (1'b0 << 2) | (addr << 3); // cs_n=0, we=0
            #1;
            if (uio_oe !== 8'hFF) begin
                $display("[FAIL] uio_oe not active during host read! Got: %b", uio_oe);
                tests_failed = tests_failed + 1;
            end
            data = uio_out;
            @(posedge clk);
            @(negedge clk);
            ui_in  = (ui_in & 8'h01) | (1'b1 << 1); // cs_n=1
        end
    endtask

    // Wait until SPI transfer completes
    task wait_done();
        integer timeout;
        begin
            timeout = 2000;
            // First wait for transfer to start (busy goes high)
            while (!spi_busy && timeout > 0) begin
                @(posedge clk);
                timeout = timeout - 1;
            end
            // Then wait for transfer to complete (busy goes low)
            timeout = 2000;
            while (spi_busy && timeout > 0) begin
                @(posedge clk);
                timeout = timeout - 1;
            end
            if (timeout == 0) begin
                $display("[FAIL] Timeout waiting for SPI transfer completion!");
                tests_failed = tests_failed + 1;
            end
            @(posedge clk);
        end
    endtask

    // External SPI slave emulation task
    task run_slave_exchange(
        input        cpol,
        input        cpha,
        input [4:0]  word_len,
        input [15:0] slave_tx_data,
        output reg [15:0] slave_rx_data
    );
        integer i;
        reg last_sclk;
        reg curr_sclk;
        begin
            slave_rx_data = 16'h0000;
            // Wait for CS active low
            while (spi_cs0_n && spi_cs1_n) @(posedge clk);

            // If CPHA=0, drive first bit immediately upon CS assertion
            if (!cpha) begin
                ui_in[0] = slave_tx_data[word_len - 1];
            end

            last_sclk = cpol;
            i = 0;

            while (i < word_len) begin
                @(posedge clk);
                curr_sclk = spi_sclk;
                if (curr_sclk !== last_sclk) begin
                    if (!cpha) begin
                        // Leading edge is sample edge
                        if (curr_sclk !== cpol) begin
                            slave_rx_data = {slave_rx_data[14:0], spi_mosi};
                            i = i + 1;
                        end else begin
                            // Trailing edge is shift edge
                            if (i < word_len) begin
                                ui_in[0] = slave_tx_data[word_len - 1 - i];
                            end
                        end
                    end else begin
                        // CPHA=1: Leading edge is shift, Trailing edge is sample
                        if (curr_sclk !== cpol) begin
                            if (i < word_len) begin
                                ui_in[0] = slave_tx_data[word_len - 1 - i];
                            end
                        end else begin
                            slave_rx_data = {slave_rx_data[14:0], spi_mosi};
                            i = i + 1;
                        end
                    end
                    last_sclk = curr_sclk;
                end
            end
        end
    endtask

    reg [7:0]  rdata;
    reg [7:0]  rdata_h;
    reg [15:0] slave_rx;

    initial begin
        $display("==================================================================");
        $display("  Nuat Labs SPI Master Controller - Standalone Verification Suite  ");
        $display("==================================================================");

        // Signal initialization
        clk    = 0;
        rst_n  = 0;
        ena    = 1;
        ui_in  = 8'h02; // cs_n = 1
        uio_in = 8'h00;

        // Reset pulse
        #50;
        @(negedge clk);
        rst_n = 1;
        #50;

        // ---------------------------------------------------------------------
        // Test 1: Reset Defaults
        // ---------------------------------------------------------------------
        $display("[TEST 1] Verifying Reset Defaults...");
        if (spi_sclk === 1'b0 && spi_mosi === 1'b0 && spi_cs0_n === 1'b1 && spi_busy === 1'b0) begin
            $display("[PASS] Reset signal defaults valid.");
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] Invalid reset signal state: sclk=%b mosi=%b cs0_n=%b busy=%b",
                     spi_sclk, spi_mosi, spi_cs0_n, spi_busy);
            tests_failed = tests_failed + 1;
        end

        // ---------------------------------------------------------------------
        // Test 2: Register Read/Write
        // ---------------------------------------------------------------------
        $display("[TEST 2] Testing Register Read/Write Access...");
        host_write(3'd1, 8'h04); // clk_div = 4
        host_read(3'd1, rdata);
        if (rdata === 8'h04) begin
            $display("[PASS] Clock divider register read/write OK.");
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] Clock divider mismatch: Expected 0x04, Got 0x%02X", rdata);
            tests_failed = tests_failed + 1;
        end

        host_write(3'd2, 8'h10); // word_len = 16
        host_read(3'd2, rdata);
        if (rdata === 8'h10) begin
            $display("[PASS] Word length register read/write OK.");
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] Word length mismatch: Expected 0x10, Got 0x%02X", rdata);
            tests_failed = tests_failed + 1;
        end

        // ---------------------------------------------------------------------
        // Test 3: Mode 0 (CPOL=0, CPHA=0) 8-bit Transfer
        // ---------------------------------------------------------------------
        $display("[TEST 3] Testing SPI Mode 0 (CPOL=0, CPHA=0)...");
        host_write(3'd1, 8'h01); // clk_div = 1
        host_write(3'd2, 8'h08); // word_len = 8
        host_write(3'd3, 8'hA5); // TX data = 0xA5
        host_write(3'd0, 8'h08); // Auto CS, Mode 0

        fork
            begin
                run_slave_exchange(1'b0, 1'b0, 5'd8, 16'h005A, slave_rx);
            end
            begin
                #20;
                host_write(3'd0, 8'h88); // Trigger start
                wait_done();
            end
        join

        host_read(3'd5, rdata);
        if (slave_rx[7:0] === 8'hA5 && rdata === 8'h5A) begin
            $display("[PASS] Mode 0: Master sent 0xA5 (Slave received 0x%02X), Slave sent 0x5A (Master received 0x%02X)",
                     slave_rx[7:0], rdata);
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] Mode 0 mismatch: Slave got 0x%02X, Master got 0x%02X", slave_rx[7:0], rdata);
            tests_failed = tests_failed + 1;
        end

        // ---------------------------------------------------------------------
        // Test 4: Mode 1 (CPOL=0, CPHA=1) 8-bit Transfer
        // ---------------------------------------------------------------------
        $display("[TEST 4] Testing SPI Mode 1 (CPOL=0, CPHA=1)...");
        host_write(3'd3, 8'hC3); // TX data = 0xC3
        host_write(3'd0, 8'h0A); // Auto CS, Mode 1 (CPHA=1)

        fork
            begin
                run_slave_exchange(1'b0, 1'b1, 5'd8, 16'h003C, slave_rx);
            end
            begin
                #20;
                host_write(3'd0, 8'h8A); // Trigger start
                wait_done();
            end
        join

        host_read(3'd5, rdata);
        if (slave_rx[7:0] === 8'hC3 && rdata === 8'h3C) begin
            $display("[PASS] Mode 1: Master sent 0xC3 (Slave received 0x%02X), Slave sent 0x3C (Master received 0x%02X)",
                     slave_rx[7:0], rdata);
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] Mode 1 mismatch: Slave got 0x%02X, Master got 0x%02X", slave_rx[7:0], rdata);
            tests_failed = tests_failed + 1;
        end

        // ---------------------------------------------------------------------
        // Test 5: Mode 2 (CPOL=1, CPHA=0) 8-bit Transfer
        // ---------------------------------------------------------------------
        $display("[TEST 5] Testing SPI Mode 2 (CPOL=1, CPHA=0)...");
        host_write(3'd3, 8'h96); // TX data = 0x96
        host_write(3'd0, 8'h09); // Auto CS, Mode 2 (CPOL=1)

        fork
            begin
                run_slave_exchange(1'b1, 1'b0, 5'd8, 16'h0069, slave_rx);
            end
            begin
                #20;
                host_write(3'd0, 8'h89); // Trigger start
                wait_done();
            end
        join

        host_read(3'd5, rdata);
        if (slave_rx[7:0] === 8'h96 && rdata === 8'h69) begin
            $display("[PASS] Mode 2: Master sent 0x96 (Slave received 0x%02X), Slave sent 0x69 (Master received 0x%02X)",
                     slave_rx[7:0], rdata);
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] Mode 2 mismatch: Slave got 0x%02X, Master got 0x%02X", slave_rx[7:0], rdata);
            tests_failed = tests_failed + 1;
        end

        // ---------------------------------------------------------------------
        // Test 6: Mode 3 (CPOL=1, CPHA=1) 8-bit Transfer
        // ---------------------------------------------------------------------
        $display("[TEST 6] Testing SPI Mode 3 (CPOL=1, CPHA=1)...");
        host_write(3'd3, 8'hF0); // TX data = 0xF0
        host_write(3'd0, 8'h0B); // Auto CS, Mode 3 (CPOL=1, CPHA=1)

        fork
            begin
                run_slave_exchange(1'b1, 1'b1, 5'd8, 16'h000F, slave_rx);
            end
            begin
                #20;
                host_write(3'd0, 8'h8B); // Trigger start
                wait_done();
            end
        join

        host_read(3'd5, rdata);
        if (slave_rx[7:0] === 8'hF0 && rdata === 8'h0F) begin
            $display("[PASS] Mode 3: Master sent 0xF0 (Slave received 0x%02X), Slave sent 0x0F (Master received 0x%02X)",
                     slave_rx[7:0], rdata);
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] Mode 3 mismatch: Slave got 0x%02X, Master got 0x%02X", slave_rx[7:0], rdata);
            tests_failed = tests_failed + 1;
        end

        // ---------------------------------------------------------------------
        // Test 7: Configurable Word Length (16-bit Transfer)
        // ---------------------------------------------------------------------
        $display("[TEST 7] Testing 16-bit Word Length Transfer...");
        host_write(3'd2, 8'd16);  // word_len = 16
        host_write(3'd3, 8'h34);  // TX low = 0x34
        host_write(3'd4, 8'h12);  // TX high = 0x12 (Total: 0x1234)
        host_write(3'd0, 8'h08);  // Auto CS, Mode 0

        fork
            begin
                run_slave_exchange(1'b0, 1'b0, 5'd16, 16'hABCD, slave_rx);
            end
            begin
                #20;
                host_write(3'd0, 8'h88); // Start
                wait_done();
            end
        join

        host_read(3'd5, rdata);
        host_read(3'd6, rdata_h);
        if (slave_rx === 16'h1234 && {rdata_h, rdata} === 16'hABCD) begin
            $display("[PASS] 16-bit: Master sent 0x1234 (Slave got 0x%04X), Slave sent 0xABCD (Master got 0x%04X)",
                     slave_rx, {rdata_h, rdata});
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] 16-bit mismatch: Slave got 0x%04X, Master got 0x%04X", slave_rx, {rdata_h, rdata});
            tests_failed = tests_failed + 1;
        end

        // ---------------------------------------------------------------------
        // Test 8: Internal Loopback Mode (BIST)
        // ---------------------------------------------------------------------
        $display("[TEST 8] Testing Internal Loopback Mode (BIST)...");
        host_write(3'd2, 8'd8);
        host_write(3'd3, 8'h77);
        // Loopback enabled (bit 5), Auto CS (bit 3), Start (bit 7)
        host_write(3'd0, 8'h28 | 8'h80);
        wait_done();
        host_read(3'd5, rdata);
        if (rdata === 8'h77) begin
            $display("[PASS] Internal Loopback BIST OK: Transmitted 0x77, Received 0x%02X", rdata);
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] Internal Loopback mismatch: Expected 0x77, Got 0x%02X", rdata);
            tests_failed = tests_failed + 1;
        end

        // ---------------------------------------------------------------------
        // Test 9: Dual Slave Select (CS0 and CS1)
        // ---------------------------------------------------------------------
        $display("[TEST 9] Testing Dual Slave Select...");
        // Select slave 1
        host_write(3'd7, 8'h01);
        host_write(3'd0, 8'h28 | 8'h80); // Loopback start
        @(posedge clk);
        #1;
        if (spi_cs0_n === 1'b1 && spi_cs1_n === 1'b0) begin
            $display("[PASS] Slave 1 selected: CS0_n=1 (inactive), CS1_n=0 (active).");
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] Dual CS error: CS0_n=%b, CS1_n=%b", spi_cs0_n, spi_cs1_n);
            tests_failed = tests_failed + 1;
        end
        wait_done();

        // ---------------------------------------------------------------------
        // Test 10: Direct Hardware Start Trigger (ui_in[6])
        // ---------------------------------------------------------------------
        $display("[TEST 10] Testing Direct Hardware Start Trigger...");
        host_write(3'd7, 8'h00); // Select slave 0
        host_write(3'd3, 8'h99); // TX = 0x99
        host_write(3'd0, 8'h28); // Loopback mode, no soft start
        @(negedge clk);
        ui_in[6] = 1'b1; // Pulse direct start
        @(posedge clk);
        @(negedge clk);
        ui_in[6] = 1'b0;
        wait_done();
        host_read(3'd5, rdata);
        if (rdata === 8'h99) begin
            $display("[PASS] Direct hardware trigger successfully started transfer (Got 0x%02X).", rdata);
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] Direct trigger mismatch: Expected 0x99, Got 0x%02X", rdata);
            tests_failed = tests_failed + 1;
        end

        // Final Summary
        $display("==================================================================");
        $display("  NUAT LABS VERIFICATION SUMMARY");
        $display("  Tests Passed: %0d", tests_passed);
        $display("  Tests Failed: %0d", tests_failed);
        $display("==================================================================");

        if (tests_failed == 0) begin
            $display(">>> ALL NUAT LABS SPI CONTROLLER CHECKS PASSED SUCCESSFULLY! <<<");
            $finish(0);
        end else begin
            $display(">>> SOME CHECKS FAILED! <<<");
            $finish(1);
        end
    end

endmodule
