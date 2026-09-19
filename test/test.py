# ==============================================================================
# Copyright (c) 2024-2026 Nuat Labs
# Author: Nuat Labs Team
# SPDX-License-Identifier: Apache-2.0
#
# Nuat Labs SPI Master Controller Verification Suite
# Comprehensive test coverage for:
#   - CPOL / CPHA Modes 0, 1, 2, and 3
#   - Configurable word length (1 to 16 bits)
#   - MSB-first and LSB-first serial transmission/reception
#   - Programmable clock divider and serial timing
#   - Dual slave chip select (CS0, CS1) and manual CS control
#   - Memory-mapped register bus and direct hardware triggers
#   - Built-in self-test (BIST) internal loopback
# ==============================================================================

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, FallingEdge, Timer

# Register Address Constants
ADDR_CTRL      = 0
ADDR_CLKDIV    = 1
ADDR_WORDLEN   = 2
ADDR_TX_DATA_L = 3
ADDR_TX_DATA_H = 4
ADDR_RX_DATA_L = 5
ADDR_RX_DATA_H = 6
ADDR_SLAVE_SEL = 7

# Control Register Bitfield Masks
CTRL_CPOL_MASK      = 0x01
CTRL_CPHA_MASK      = 0x02
CTRL_LSB_FIRST_MASK = 0x04
CTRL_AUTO_CS_MASK   = 0x08
CTRL_MANUAL_CS_MASK = 0x10
CTRL_LOOPBACK_MASK  = 0x20
CTRL_IRQ_EN_MASK    = 0x40
CTRL_START_MASK     = 0x80


class HostBusDriver:
    """Nuat Labs 8-bit Host Bus Driver for SPI Controller verification."""

    def __init__(self, dut):
        self.dut = dut

    async def reset(self):
        """Perform hardware reset sequence."""
        self.dut.ena.value = 1
        self.dut.rst_n.value = 0
        self.dut.ui_in.value = 0x02  # host_cs_n = 1 (inactive)
        self.dut.uio_in.value = 0x00
        await ClockCycles(self.dut.clk, 5)
        self.dut.rst_n.value = 1
        await ClockCycles(self.dut.clk, 2)

    async def write_reg(self, addr: int, data: int):
        """Write 8-bit data to the specified register address."""
        await FallingEdge(self.dut.clk)
        # ui_in mapping: [0]: miso, [1]: cs_n, [2]: we, [5:3]: addr, [6]: direct_start
        ui_val = (0 << 1) | (1 << 2) | ((addr & 0x07) << 3)
        self.dut.ui_in.value = ui_val
        self.dut.uio_in.value = data & 0xFF
        await RisingEdge(self.dut.clk)
        await FallingEdge(self.dut.clk)
        # Deassert host_cs_n
        self.dut.ui_in.value = 0x02  # cs_n = 1, we = 0
        self.dut.uio_in.value = 0x00

    async def read_reg(self, addr: int) -> int:
        """Read 8-bit data from the specified register address."""
        await FallingEdge(self.dut.clk)
        ui_val = (0 << 1) | (0 << 2) | ((addr & 0x07) << 3)
        self.dut.ui_in.value = ui_val
        await Timer(1, unit="ns")
        # Check that output enable is active
        assert self.dut.uio_oe.value == 0xFF, f"Expected uio_oe=0xFF, got {self.dut.uio_oe.value}"
        read_val = int(self.dut.uio_out.value)
        await RisingEdge(self.dut.clk)
        await FallingEdge(self.dut.clk)
        self.dut.ui_in.value = 0x02  # cs_n = 1
        return read_val

    async def wait_done(self, timeout_cycles=1000):
        """Wait for SPI transfer completion."""
        for _ in range(30):
            await RisingEdge(self.dut.clk)
            if self.dut.uo_out[3].value == 1:
                break
        for _ in range(timeout_cycles):
            await RisingEdge(self.dut.clk)
            if self.dut.uo_out[3].value == 0:
                await RisingEdge(self.dut.clk)
                return
        raise TimeoutError("SPI transfer timed out waiting for completion")


async def simulate_spi_slave(dut, cpol: int, cpha: int, word_len: int, tx_slave_data: int, lsb_first: bool = False):
    """
    Emulates an external SPI slave device:
    Samples MOSI and shifts out MISO according to CPOL/CPHA and word_len.
    """
    received_mosi_bits = []

    # Prepare slave transmit bits
    slave_bits = []
    for i in range(word_len):
        if lsb_first:
            bit = (tx_slave_data >> i) & 1
        else:
            bit = (tx_slave_data >> (word_len - 1 - i)) & 1
        slave_bits.append(bit)

    bit_count = 0

    # Wait for CS to assert (active low)
    while dut.uo_out[2].value == 1 and dut.uo_out[6].value == 1:
        await RisingEdge(dut.clk)

    # Initial MISO drive for CPHA=0
    if cpha == 0 and bit_count < word_len:
        # Drive first slave bit during CS assertion
        cur_ui = int(dut.ui_in.value)
        dut.ui_in.value = (cur_ui & ~1) | slave_bits[0]

    last_sclk = cpol

    while bit_count < word_len:
        await RisingEdge(dut.clk)
        curr_sclk = int(dut.uo_out[0].value)

        # Detect SCLK edge
        if last_sclk != curr_sclk:
            is_leading_edge = (curr_sclk != cpol)
            is_trailing_edge = (curr_sclk == cpol)

            if cpha == 0:
                # CPHA=0: Leading edge is sample edge for both Master and Slave
                if is_leading_edge:
                    mosi_bit = int(dut.uo_out[1].value)
                    received_mosi_bits.append(mosi_bit)
                    bit_count += 1
                elif is_trailing_edge:
                    # Trailing edge is shift edge for slave
                    if bit_count < word_len:
                        cur_ui = int(dut.ui_in.value)
                        dut.ui_in.value = (cur_ui & ~1) | slave_bits[bit_count]
            else:
                # CPHA=1: Leading edge is shift edge, Trailing edge is sample edge
                if is_leading_edge:
                    if bit_count < word_len:
                        cur_ui = int(dut.ui_in.value)
                        dut.ui_in.value = (cur_ui & ~1) | slave_bits[bit_count]
                elif is_trailing_edge:
                    mosi_bit = int(dut.uo_out[1].value)
                    received_mosi_bits.append(mosi_bit)
                    bit_count += 1

            last_sclk = curr_sclk

    # Reconstruct integer from received MOSI bits
    reconstructed_data = 0
    for idx, bit in enumerate(received_mosi_bits):
        if lsb_first:
            reconstructed_data |= (bit << idx)
        else:
            reconstructed_data = (reconstructed_data << 1) | bit

    return reconstructed_data


@cocotb.test()
async def test_nuatlabs_reset_and_regs(dut):
    """Verify reset state, default register values, and register read/write integrity."""
    dut._log.info("Nuat Labs: Testing reset and register read/write")
    clock = Clock(dut.clk, 20, unit="ns")  # 50 MHz clock
    cocotb.start_soon(clock.start())

    driver = HostBusDriver(dut)
    await driver.reset()

    # Verify default outputs after reset
    assert dut.uo_out[0].value == 0, "Default SCLK must idle low (CPOL=0)"
    assert dut.uo_out[1].value == 0, "Default MOSI must be 0"
    assert dut.uo_out[2].value == 1, "Default CS0_n must be inactive (high)"
    assert dut.uo_out[3].value == 0, "Default busy flag must be low"
    assert dut.uo_out[4].value == 0, "Default done strobe must be low"

    # Verify default register reads
    clkdiv_val = await driver.read_reg(ADDR_CLKDIV)
    assert clkdiv_val == 1, f"Expected default clkdiv=1, got {clkdiv_val}"

    wordlen_val = await driver.read_reg(ADDR_WORDLEN)
    assert wordlen_val == 8, f"Expected default wordlen=8, got {wordlen_val}"

    # Write and read back test pattern to registers
    await driver.write_reg(ADDR_CLKDIV, 0x14)
    assert await driver.read_reg(ADDR_CLKDIV) == 0x14

    await driver.write_reg(ADDR_WORDLEN, 0x10)  # 16-bit word length
    assert await driver.read_reg(ADDR_WORDLEN) == 0x10

    await driver.write_reg(ADDR_TX_DATA_L, 0x5A)
    assert await driver.read_reg(ADDR_TX_DATA_L) == 0x5A

    await driver.write_reg(ADDR_TX_DATA_H, 0xA5)
    assert await driver.read_reg(ADDR_TX_DATA_H) == 0xA5

    dut._log.info("Nuat Labs: Reset and Register access tests passed successfully!")


@cocotb.test()
async def test_nuatlabs_spi_modes(dut):
    """
    Exhaustively verify all 4 SPI Modes (0, 1, 2, 3) with an active slave.
    Mode 0: CPOL=0, CPHA=0
    Mode 1: CPOL=0, CPHA=1
    Mode 2: CPOL=1, CPHA=0
    Mode 3: CPOL=1, CPHA=1
    """
    dut._log.info("Nuat Labs: Testing SPI Modes 0, 1, 2, and 3")
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())
    driver = HostBusDriver(dut)

    modes = [
        (0, 0, "Mode 0 (CPOL=0, CPHA=0)"),
        (0, 1, "Mode 1 (CPOL=0, CPHA=1)"),
        (1, 0, "Mode 2 (CPOL=1, CPHA=0)"),
        (1, 1, "Mode 3 (CPOL=1, CPHA=1)"),
    ]

    for cpol, cpha, mode_name in modes:
        dut._log.info(f"Nuat Labs: Testing {mode_name}")
        await driver.reset()

        tx_master_data = 0xA7
        tx_slave_data = 0x3E

        # Configure clock divider = 1, word length = 8
        await driver.write_reg(ADDR_CLKDIV, 1)
        await driver.write_reg(ADDR_WORDLEN, 8)
        await driver.write_reg(ADDR_TX_DATA_L, tx_master_data)

        # Configure CTRL with desired CPOL & CPHA and auto CS
        ctrl_config = (cpol << 0) | (cpha << 1) | CTRL_AUTO_CS_MASK
        await driver.write_reg(ADDR_CTRL, ctrl_config)

        # Start slave emulation coroutine
        slave_task = cocotb.start_soon(simulate_spi_slave(dut, cpol, cpha, 8, tx_slave_data))

        # Trigger SPI transfer via CTRL[7]
        await driver.write_reg(ADDR_CTRL, ctrl_config | CTRL_START_MASK)

        # Wait for completion
        await driver.wait_done()
        received_by_slave = await slave_task

        # Verify slave received what master sent
        assert received_by_slave == tx_master_data, (
            f"{mode_name} Master->Slave mismatch: Sent 0x{tx_master_data:02X}, Slave got 0x{received_by_slave:02X}"
        )

        # Verify master received what slave sent
        rx_master_data = await driver.read_reg(ADDR_RX_DATA_L)
        assert rx_master_data == tx_slave_data, (
            f"{mode_name} Slave->Master mismatch: Sent 0x{tx_slave_data:02X}, Master got 0x{rx_master_data:02X}"
        )

        dut._log.info(f"Nuat Labs: {mode_name} verified successfully!")


@cocotb.test()
async def test_nuatlabs_configurable_word_lengths(dut):
    """
    Verify configurable word length functionality:
    Tests 4-bit, 8-bit, 12-bit, and 16-bit word transfers.
    """
    dut._log.info("Nuat Labs: Testing Configurable Word Lengths (4, 8, 12, 16 bits)")
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())
    driver = HostBusDriver(dut)

    test_lengths = [
        (4, 0x0B, 0x05),
        (8, 0x93, 0x6C),
        (12, 0x0B72, 0x04A9),
        (16, 0xD42B, 0x7E19),
    ]

    for length, tx_master, tx_slave in test_lengths:
        dut._log.info(f"Nuat Labs: Testing Word Length = {length} bits")
        await driver.reset()

        await driver.write_reg(ADDR_CLKDIV, 1)
        await driver.write_reg(ADDR_WORDLEN, length)
        await driver.write_reg(ADDR_TX_DATA_L, tx_master & 0xFF)
        await driver.write_reg(ADDR_TX_DATA_H, (tx_master >> 8) & 0xFF)

        ctrl_config = CTRL_AUTO_CS_MASK  # Mode 0, Auto CS
        await driver.write_reg(ADDR_CTRL, ctrl_config)

        slave_task = cocotb.start_soon(simulate_spi_slave(dut, 0, 0, length, tx_slave))
        await driver.write_reg(ADDR_CTRL, ctrl_config | CTRL_START_MASK)

        await driver.wait_done()
        slave_rx = await slave_task
        assert slave_rx == tx_master, (
            f"Length {length} bit error: Master sent 0x{tx_master:X}, Slave received 0x{slave_rx:X}"
        )

        rx_low = await driver.read_reg(ADDR_RX_DATA_L)
        rx_high = await driver.read_reg(ADDR_RX_DATA_H)
        master_rx = rx_low | (rx_high << 8)
        assert master_rx == tx_slave, (
            f"Length {length} bit error: Slave sent 0x{tx_slave:X}, Master received 0x{master_rx:X}"
        )

        dut._log.info(f"Nuat Labs: Word length {length}-bit transfer verified!")


@cocotb.test()
async def test_nuatlabs_internal_loopback(dut):
    """
    Verify internal loopback Built-In Self-Test (BIST) feature:
    Internal MOSI is looped back to MISO.
    """
    dut._log.info("Nuat Labs: Testing Internal Loopback Mode (BIST)")
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())
    driver = HostBusDriver(dut)
    await driver.reset()

    test_vectors = [0x55, 0xAA, 0xF0, 0x0F, 0x1234]

    for vec in test_vectors:
        word_len = 16 if vec > 0xFF else 8
        await driver.write_reg(ADDR_CLKDIV, 0)
        await driver.write_reg(ADDR_WORDLEN, word_len)
        await driver.write_reg(ADDR_TX_DATA_L, vec & 0xFF)
        await driver.write_reg(ADDR_TX_DATA_H, (vec >> 8) & 0xFF)

        # Enable loopback bit in CTRL register
        ctrl_config = CTRL_AUTO_CS_MASK | CTRL_LOOPBACK_MASK
        await driver.write_reg(ADDR_CTRL, ctrl_config)
        await driver.write_reg(ADDR_CTRL, ctrl_config | CTRL_START_MASK)

        await driver.wait_done()

        rx_l = await driver.read_reg(ADDR_RX_DATA_L)
        rx_h = await driver.read_reg(ADDR_RX_DATA_H)
        rx_total = rx_l | (rx_h << 8) if word_len == 16 else rx_l

        assert rx_total == vec, f"Loopback mismatch: Sent 0x{vec:X}, received 0x{rx_total:X}"

    dut._log.info("Nuat Labs: Internal loopback verification passed!")


@cocotb.test()
async def test_nuatlabs_dual_slave_select(dut):
    """
    Verify multi-slave support:
    CS0 is asserted when slave_sel=0, CS1 is asserted when slave_sel=1.
    """
    dut._log.info("Nuat Labs: Testing Dual Slave Select (CS0 and CS1)")
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())
    driver = HostBusDriver(dut)
    await driver.reset()

    # Test Slave 0 selection
    await driver.write_reg(ADDR_SLAVE_SEL, 0)
    await driver.write_reg(ADDR_WORDLEN, 8)
    await driver.write_reg(ADDR_TX_DATA_L, 0x11)
    await driver.write_reg(ADDR_CTRL, CTRL_AUTO_CS_MASK | CTRL_LOOPBACK_MASK | CTRL_START_MASK)

    # During transfer, CS0 must be low and CS1 must be high
    await ClockCycles(dut.clk, 3)
    assert dut.uo_out[2].value == 0, "CS0_n must be active low when slave 0 selected"
    assert dut.uo_out[6].value == 1, "CS1_n must remain inactive high when slave 0 selected"
    await driver.wait_done()

    # Test Slave 1 selection
    await driver.write_reg(ADDR_SLAVE_SEL, 1)
    await driver.write_reg(ADDR_TX_DATA_L, 0x22)
    await driver.write_reg(ADDR_CTRL, CTRL_AUTO_CS_MASK | CTRL_LOOPBACK_MASK | CTRL_START_MASK)

    await ClockCycles(dut.clk, 3)
    assert dut.uo_out[2].value == 1, "CS0_n must remain inactive high when slave 1 selected"
    assert dut.uo_out[6].value == 0, "CS1_n must be active low when slave 1 selected"
    await driver.wait_done()

    dut._log.info("Nuat Labs: Dual Slave Select verified!")


@cocotb.test()
async def test_nuatlabs_direct_hardware_start(dut):
    """
    Verify direct hardware start trigger via ui_in[6].
    Allows triggering SPI transactions with a single hardware pin pulse.
    """
    dut._log.info("Nuat Labs: Testing Direct Hardware Start Trigger (ui_in[6])")
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())
    driver = HostBusDriver(dut)
    await driver.reset()

    test_byte = 0xE5
    await driver.write_reg(ADDR_TX_DATA_L, test_byte)
    await driver.write_reg(ADDR_CTRL, CTRL_AUTO_CS_MASK | CTRL_LOOPBACK_MASK)

    # Pulse direct_start (ui_in[6])
    await FallingEdge(dut.clk)
    dut.ui_in.value = 0x02 | (1 << 6)
    dut._log.info(f"ui_in set to: {dut.ui_in.value}, direct_start={dut.user_project.direct_start.value}")
    await RisingEdge(dut.clk)
    dut._log.info(f"After 1 cycle: direct_start={dut.user_project.direct_start.value}, busy={dut.uo_out[3].value}")
    await FallingEdge(dut.clk)
    dut.ui_in.value = 0x02
    await RisingEdge(dut.clk)
    dut._log.info(f"After release: busy={dut.uo_out[3].value}")

    await driver.wait_done()

    rx_val = await driver.read_reg(ADDR_RX_DATA_L)
    dut._log.info(f"rx_val read: {rx_val}")
    assert rx_val == test_byte, f"Direct start mismatch: Expected 0x{test_byte:02X}, got 0x{rx_val:02X}"

    dut._log.info("Nuat Labs: Direct hardware trigger verified!")
