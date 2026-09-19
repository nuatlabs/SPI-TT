![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg) ![](../../workflows/fpga/badge.svg)

# Nuat Labs Configurable SPI Master Controller

A production-grade, highly configurable **SPI Master Controller** ASIC design developed by **Nuat Labs** for the Tiny Tapeout shuttle.

- [Detailed Datasheet Documentation](docs/info.md)

---

## Architecture & Technical Scope

The Nuat Labs SPI Controller is designed for high-reliability embedded and mixed-signal communication:

- **All 4 SPI Modes (Modes 0, 1, 2, 3)**:
  - Supports arbitrary combinations of Clock Polarity (`CPOL`) and Clock Phase (`CPHA`).
- **Configurable Word Length (1 to 16 bits)**:
  - Dynamically adjustable word length for standard bytes (8-bit), multi-byte payloads (16-bit), or non-standard sensor/ADC word widths (4-bit, 10-bit, 12-bit).
- **Shift Registers & Serial/Parallel Conversion**:
  - Full 16-bit Parallel-In Serial-Out (PISO) transmit shift register.
  - Full 16-bit Serial-In Parallel-Out (SIPO) receive shift register.
  - Supports both **MSB-first** and **LSB-first** bit ordering.
- **Clock Generation & Prescaler**:
  - Flexible baud rate generation with programmable divider `REG_CLKDIV`:
    $$f_{\text{SCLK}} = \frac{f_{\text{CLK}}}{2 \times (\text{REG\_CLKDIV} + 1)}$$
- **Chip Select Management**:
  - Dual slave device support (`spi_cs0_n` and `spi_cs1_n`).
  - Automatic hardware guard times (lead time and trail time).
  - Manual software CS control option.
- **Host Interface**:
  - 8-bit memory-mapped register bus over `uio[7:0]` with address, read/write, and chip-select controls.
  - Direct hardware start trigger pin (`direct_start`).
- **Built-in Self-Test (BIST)**:
  - Internal digital loopback mode routing MOSI to MISO for automated verification.

---

## Pinout Mapping

| Pin | Signal Name | Direction | Description |
|:---|:---|:---:|:---|
| `ui_in[0]` | `spi_miso` | Input | SPI Master In Slave Out from peripheral |
| `ui_in[1]` | `host_cs_n` | Input | Host bus chip select (active low) |
| `ui_in[2]` | `host_we` | Input | Host bus write enable (1 = write, 0 = read) |
| `ui_in[5:3]` | `host_addr[2:0]` | Input | Host register address (0 to 7) |
| `ui_in[6]` | `direct_start` | Input | Hardware trigger strobe (rising edge starts transfer) |
| `ui_in[7]` | `reserved` | Input | Reserved |
| `uo_out[0]` | `spi_sclk` | Output | SPI Serial Clock |
| `uo_out[1]` | `spi_mosi` | Output | SPI Master Out Slave In |
| `uo_out[2]` | `spi_cs0_n` | Output | SPI Chip Select 0 (primary slave, active low) |
| `uo_out[3]` | `spi_busy` | Output | SPI transfer in progress flag |
| `uo_out[4]` | `spi_done` | Output | Transfer completion strobe (1 cycle) |
| `uo_out[5]` | `spi_rx_ready` | Output | Valid RX data ready flag |
| `uo_out[6]` | `spi_cs1_n` | Output | SPI Chip Select 1 (secondary slave, active low) |
| `uo_out[7]` | `spi_irq` | Output | Interrupt request line |
| `uio[7:0]` | `host_data[7:0]` | Bidirectional | 8-bit parallel bidirectional host register data bus |

---

## Register Map

| Address | Name | Access | Description |
|:---:|:---|:---:|:---|
| `0x0` | `REG_CTRL` | R/W | Control & Status (CPOL, CPHA, LSB_FIRST, AUTO_CS, MANUAL_CS, LOOPBACK, IRQ_EN, START) |
| `0x1` | `REG_CLKDIV` | R/W | Clock divider prescaler (`clk_div`) |
| `0x2` | `REG_WORDLEN` | R/W | Word length in bits (1 to 16; 0 defaults to 8) |
| `0x3` | `REG_TX_DATA_L` | R/W | Transmit data byte [7:0] |
| `0x4` | `REG_TX_DATA_H` | R/W | Transmit data byte [15:8] |
| `0x5` | `REG_RX_DATA_L` | R | Receive data byte [7:0] |
| `0x6` | `REG_RX_DATA_H` | R | Receive data byte [15:8] |
| `0x7` | `REG_SLAVE_SEL` | R/W | Slave select index (0 = CS0, 1 = CS1) |

---

## Verification & Test Results

### 1. Standalone Verilog Simulation
The design has been verified with 11 automated testcases in `test/tb_standalone.v`:
```bash
iverilog -Wall -g2012 -s tb_standalone -o sim.vvp src/project.v test/tb_standalone.v
vvp sim.vvp
```
**Result**:
```
==================================================================
  NUAT LABS VERIFICATION SUMMARY
  Tests Passed: 11
  Tests Failed: 0
==================================================================
>>> ALL NUAT LABS SPI CONTROLLER CHECKS PASSED SUCCESSFULLY! <<<
```

### 2. Cocotb Verification Suite
Comprehensive cocotb test suite in `test/test.py` validates all SPI modes, variable word lengths, clock dividers, dual CS, and loopback:
```
** TESTS=6 PASS=6 FAIL=0 SKIP=0 **
```

---

## License & Copyright

Copyright (c) 2024-2026 Nuat Labs.
Licensed under the Apache License, Version 2.0.
