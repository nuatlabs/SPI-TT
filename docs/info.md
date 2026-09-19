<!---
Copyright (c) 2024-2026 Nuat Labs
Author: Nuat Labs Team
SPDX-License-Identifier: Apache-2.0
--->

# Nuat Labs Configurable SPI Master Controller

The **Nuat Labs Configurable SPI Master Controller** is a versatile, silicon-proven serial peripheral interface engine designed for the Tiny Tapeout shuttle. It provides full support for all four SPI clock modes (Modes 0, 1, 2, and 3), dynamically programmable word lengths ranging from 1 to 16 bits, flexible clock prescaling, dual-slave chip selection, and built-in self-test (BIST) internal loopback.

---

## 1. How It Works

The controller converts parallel host data into high-speed serial bitstreams (MOSI) and deserializes peripheral bitstreams (MISO) into parallel host words with microsecond-level determinism.

```
                      +---------------------------------------+
                      |               Nuat Labs               |
                      |        Configurable SPI Master        |
                      +---------------------------------------+
 Host Bus (uio[7:0]) ----> [ 8-bit Host Reg Interface ] <----> uio_oe
 Host Addr/Ctrl (ui) ----> [ Control / Status Regs    ]
                           [ Baud Rate Prescaler      ] ----> spi_sclk (uo[0])
                           [ Dual Shift Registers     ] ----> spi_mosi (uo[1])
                           [ Word Length Bit Sequencer] <---- spi_miso (ui[0])
                           [ Dual Slave CS Decoder    ] ----> spi_cs0_n, spi_cs1_n (uo[2], uo[6])
                           [ Status & IRQ Generator   ] ----> spi_busy, spi_done, spi_irq
```

### Core Architecture Components
1. **Host Bus Interface**:
   - Standard 8-bit bidirectional register interface mapped to `uio[7:0]`.
   - Asynchronous host address selection (`ui_in[5:3]`), chip select (`ui_in[1]`), and write enable (`ui_in[2]`).
   - Single-cycle register reads and writes with active bus driving (`uio_oe`).
2. **Clock Generation (Baud Rate Prescaler)**:
   - SCLK is derived from the system clock (`clk`) via an 8-bit prescaler register (`REG_CLKDIV`).
   - SCLK half-period duration = `(REG_CLKDIV + 1)` system clock cycles.
   - Frequency formula: $f_{\text{SCLK}} = \frac{f_{\text{CLK}}}{2 \times (\text{divider} + 1)}$, where `divider` is the value in `REG_CLKDIV`.
3. **SPI Modes (CPOL / CPHA 0–3)**:
   - **Mode 0 (CPOL=0, CPHA=0)**: SCLK idles LOW; data sampled on rising edge, shifted on falling edge.
   - **Mode 1 (CPOL=0, CPHA=1)**: SCLK idles LOW; data shifted on rising edge, sampled on falling edge.
   - **Mode 2 (CPOL=1, CPHA=0)**: SCLK idles HIGH; data sampled on falling edge, shifted on rising edge.
   - **Mode 3 (CPOL=1, CPHA=1)**: SCLK idles HIGH; data shifted on falling edge, sampled on rising edge.
4. **Variable Word Length Sequencer (1 to 16 bits)**:
   - Configurable from 1 to 16 bits via `REG_WORDLEN`.
   - Supports arbitrary data widths (e.g. 8-bit bytes, 10-bit/12-bit ADC reads, 16-bit DAC/sensor frames).
   - Both **MSB-first** and **LSB-first** bit orders are supported.
5. **Chip Select & Dual Slave Routing**:
   - Guarded lead time before the first clock edge and trail time after the final clock edge.
   - Supports 2 independent slave devices via `spi_cs0_n` (`uo_out[2]`) and `spi_cs1_n` (`uo_out[6]`).
   - Automatic hardware assertion during transaction or manual software CS override.
6. **Built-in Self-Test (BIST) Loopback**:
   - Internal digital loopback routes MOSI directly to MISO inside the core, enabling 100% automated self-testing without external wiring.

---

## 2. Register Map

| Addr (`ui_in[5:3]`) | Name | Access | Description |
|:---|:---|:---:|:---|
| `0x0` | `REG_CTRL` | R/W | **Control / Status Register**<br>**Write:**<br>[0]: `cpol` (0=idle low, 1=idle high)<br>[1]: `cpha` (0=sample 1st edge, 1=sample 2nd edge)<br>[2]: `lsb_first` (0=MSB first, 1=LSB first)<br>[3]: `auto_cs` (1=automatic CS assertion, 0=manual)<br>[4]: `manual_cs` (level when manual)<br>[5]: `loopback` (1=enable internal BIST loopback)<br>[6]: `irq_en` (1=enable interrupt on done)<br>[7]: `start` (write 1 to trigger transfer)<br>**Read:**<br>[0]: `cpol`, [1]: `cpha`, [2]: `lsb_first`, [3]: `busy`, [4]: `done`, [5]: `rx_ready`, [6]: `auto_cs`, [7]: `loopback` |
| `0x1` | `REG_CLKDIV` | R/W | **Baud Rate Prescaler**<br>[7:0]: Half-period divider count = `clk_div + 1` |
| `0x2` | `REG_WORDLEN` | R/W | **Word Length**<br>[4:0]: Active bit count (1 to 16; 0 defaults to 8) |
| `0x3` | `REG_TX_DATA_L`| R/W | **Transmit Data Low Byte** (`tx_data[7:0]`) |
| `0x4` | `REG_TX_DATA_H`| R/W | **Transmit Data High Byte** (`tx_data[15:8]`) |
| `0x5` | `REG_RX_DATA_L`| R | **Receive Data Low Byte** (`rx_data[7:0]`) |
| `0x6` | `REG_RX_DATA_H`| R | **Receive Data High Byte** (`rx_data[15:8]`) |
| `0x7` | `REG_SLAVE_SEL`| R/W | **Slave Select Index**<br>[0]: 0 = Select Slave 0 (`spi_cs0_n`), 1 = Select Slave 1 (`spi_cs1_n`) |

---

## 3. Pinout Mapping

### Dedicated Inputs (`ui_in`)
- `ui_in[0]`: `spi_miso` — SPI Master In Slave Out from peripheral.
- `ui_in[1]`: `host_cs_n` — Host bus Chip Select (active low).
- `ui_in[2]`: `host_we` — Host bus Write Enable (1 = Write, 0 = Read).
- `ui_in[5:3]`: `host_addr[2:0]` — Host register address (0 to 7).
- `ui_in[6]`: `direct_start` — Hardware trigger input; rising edge initiates transfer immediately.
- `ui_in[7]`: Reserved (tie to 0).

### Dedicated Outputs (`uo_out`)
- `uo_out[0]`: `spi_sclk` — Serial Clock to SPI slave.
- `uo_out[1]`: `spi_mosi` — Master Out Slave In serial bitstream.
- `uo_out[2]`: `spi_cs0_n` — Primary Chip Select (active low).
- `uo_out[3]`: `spi_busy` — 1 when transfer is in progress.
- `uo_out[4]`: `spi_done` — Single-cycle strobe asserted upon completion.
- `uo_out[5]`: `spi_rx_ready` — Asserted when valid data is in RX buffer.
- `uo_out[6]`: `spi_cs1_n` — Secondary Chip Select (active low).
- `uo_out[7]`: `spi_irq` — Interrupt request output.

### Bidirectional Host Bus (`uio`)
- `uio_in[7:0]`: Parallel write data input bus.
- `uio_out[7:0]`: Parallel read data output bus.
- `uio_oe[7:0]`: Automatically driven active (`0xFF`) during host reads, high-Z (`0x00`) otherwise.

---

## 4. How to Test

### Running Standalone Verilog Simulation
The Nuat Labs verification suite includes a self-contained testbench `test/tb_standalone.v`:
```bash
iverilog -Wall -g2012 -s tb_standalone -o sim.vvp src/project.v test/tb_standalone.v
vvp sim.vvp
```
All 11 verification tests covering Modes 0–3, 16-bit words, BIST loopback, and multi-slave CS will execute and report:
`>>> ALL NUAT LABS SPI CONTROLLER CHECKS PASSED SUCCESSFULLY! <<<`

### Running Cocotb Test Suite
```bash
python -m cocotb_tools.runner ...
```
or via the standard Tiny Tapeout test runner.

### Hardware Testing on Demo Board
1. Connect power and clock (e.g., 10 MHz to 50 MHz).
2. Set `ui_in[1] = 0` (`host_cs_n` active) and `ui_in[2] = 1` (`host_we` high).
3. Write `0x55` to register `0x3` (`REG_TX_DATA_L`) via `uio[7:0]`.
4. Enable internal loopback by writing `0x28 | 0x80` (`0xA8`) to `REG_CTRL` (`host_addr = 0x0`).
5. Observe `spi_busy` on `uo_out[3]` and `spi_sclk` on `uo_out[0]`.
6. Read back register `0x5` (`REG_RX_DATA_L`) by setting `ui_in[2] = 0` (`host_we = 0`). Verify that `uio[7:0]` outputs `0x55`.

---

## 5. External Hardware

- **Compatible Peripherals**: SPI Flash memory (e.g. W25Qxx), SPI OLED displays (e.g. SSD1306), ADCs (e.g. MCP3008, ADS7883), DACs, and Microcontrollers.
- **PMOD / Wiring**: SCLK, MOSI, MISO, CS0/CS1 can be directly routed to standard PMOD connectors or breadboards.
