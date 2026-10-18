# Timing constraints for the Oric core on NeptUNOplus.
#
# PLL input: CLOCK_50, 50 MHz.
# PLL outputs (pll.v):
#   clk[0] clk_24   24 MHz  machine (ULA, CPUs, VIAs, FDC), user_io, video
#   clk[1] clk_72   72 MHz  SDRAM controller, data_io, audio (I2S, S/PDIF)
# All PLL outputs come from the same PLL, so their crossings are timed.
# I/O pins are left unconstrained.

create_clock -name {CLOCK_50} -period 20.000 [get_ports {CLOCK_50}]
create_clock -name {SPI_SCK}  -period 41.666 -waveform { 20.8 41.666 } [get_ports {SPI_SCK}]

derive_pll_clocks
derive_clock_uncertainty

set clk_24 [get_clocks {*pll1|clk[0]}]

# SPI from the ARM (user_io, data_io, OSD) is asynchronous to the core
set_clock_groups -asynchronous -group [get_clocks {SPI_SCK}] \
	-group [get_clocks {*pll1|clk[*]}]

# SDRAM read data -> CPU (clk_72 -> clk_24). The ULA puts the CPU address on
# ram_ad/rom_ad when PHI2 rises (c16). The request logic in Oric.sv sees the
# change on the next clk_72 edge, and sdram.sv restarts its 6 state cycle on
# the rising edge of clkref (PHI2), so port1_q holds the new data at most
# about 14 clk_72 later (c21). sdram_ad[0], the byte select of ram_q, changes
# with the request. The T65 and the 65C816 only take the data on their enable,
# ENA_1MHZ_N = c(23), at the end of PHI2, and port1_q does not change again
# until the ULA switches to the video address at c0. cpu_di_last samples on
# every clk_24 while PHI2 is high, but only its value at c23 is ever used.
# One clk_24 period is therefore safe.
set sdram_q [get_registers {*sdram:sdram|port1_q[*] *sdram_ad[0]}]
set_multicycle_path -from $sdram_q -to $clk_24 -setup -start 3
set_multicycle_path -from $sdram_q -to $clk_24 -hold  -start 2
