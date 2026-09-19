/*
 * Copyright (c) 2024-2026 Nuat Labs
 * Author: Nuat Labs Team
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none
`timescale 1ns / 1ps

/* This testbench instantiates the Nuat Labs SPI top module
   and provides signal hooks for cocotb test suite.
*/
module tb ();

  // Dump waveform signals to FST/VCD for analysis
  initial begin
    $dumpfile("tb.fst");
    $dumpvars(0, tb);
    #1;
  end

  // Testbench signals
  reg clk;
  reg rst_n;
  reg ena;
  reg [7:0] ui_in;
  reg [7:0] uio_in;
  wire [7:0] uo_out;
  wire [7:0] uio_out;
  wire [7:0] uio_oe;

  // Instantiate Nuat Labs SPI Controller top module
  tt_um_nuatlabs_spi user_project (
      .ui_in  (ui_in),    // Dedicated inputs
      .uo_out (uo_out),   // Dedicated outputs
      .uio_in (uio_in),   // IOs: Input path
      .uio_out(uio_out),  // IOs: Output path
      .uio_oe (uio_oe),   // IOs: Enable path (active high: 0=input, 1=output)
      .ena    (ena),      // enable - goes high when design is selected
      .clk    (clk),      // clock
      .rst_n  (rst_n)     // reset_n - low to reset
  );

endmodule
