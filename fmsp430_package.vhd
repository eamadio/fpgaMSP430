------------------------------------------------------------------------------
--! Copyright (C) 2017 , Emmanuel Amadio
--
--! Redistribution and use in source and binary forms, with or without
--! modification, are permitted provided that the following conditions
--! are met:
--!     * Redistributions of source code must retain the above copyright
--!       notice, this list of conditions and the following disclaimer.
--!     * Redistributions in binary form must reproduce the above copyright
--!       notice, this list of conditions and the following disclaimer in the
--!       documentation and/or other materials provided with the distribution.
--!     * Neither the name of the authors nor the names of its contributors
--!       may be used to endorse or promote products derived from this software
--!       without specific prior written permission.
--
--! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
--! AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
--! IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
--! ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
--! LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
--! OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
--! SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
--! INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
--! CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
--! ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
--! THE POSSIBILITY OF SUCH DAMAGE
--
------------------------------------------------------------------------------
--
--! @file fmsp430_package.vhd
--! 
--! @brief fpgaMSP430 package
--
--! @author Emmanuel Amadio,   emmanuel.amadio@gmail.com
--
------------------------------------------------------------------------------
--! @version 1
--! @date: 2017-04-21
------------------------------------------------------------------------------
library ieee;
	use ieee.std_logic_1164.all;	--! standard unresolved logic UX01ZWLH-
	use ieee.numeric_std.all;		--! for the signed, unsigned types and arithmetic ops
	use work.fmsp_functions.all;

package fmsp430_package is

component  fmsp430 is
generic (
	INST_NR					: integer := 0;			--! Current fmsp instance number     (for multicore systems)
	TOTAL_NR					: integer := 0;			--! Total number of fmsp instances-1 (for multicore systems)
	PMEM_SIZE				: integer := 32768;		--! Program Memory Size
	DMEM_SIZE				: integer := 16384;		--! Data Memory Size
	PER_SIZE					: integer := 16384;		--! Peripheral Memory Size
	MULTIPLIER				: boolean := false;		--! Include/Exclude Hardware Multiplier
	USER_VERSION			: integer := 0;			--! Custom user version number
	DEBUG_EN					: boolean := false;		--! Include/Exclude Serial Debug interface
	WATCHDOG					: boolean := false;		--! Include/Exclude Watchdog timer
	DMA_IF_EN				: boolean := false;		--! Include/Exclude DMA interface support
	NMI_EN					: boolean := false;		--! Include/Exclude Non-Maskable-Interrupt support
	IRQ_NR					: integer := 16;			--! Number of IRQs
	SYNC_NMI_EN				: boolean := true;		--! 
	SYNC_CPU_EN				: boolean := true;		--! 
	SYNC_DBG_EN				: boolean := true;		--! 
	SYNC_DBG_UART_RXD		: boolean := true;		--! Synchronize RXD inputs
	DBG_UART					: boolean := false;		--	Enable UART (8N1) debug interface
	DBG_I2C					: boolean := true;		--	Enable I2C debug interface
	DBG_I2C_BROADCAST_EN	: boolean := false;		--! Enable the I2C broadcast address
	DBG_RST_BRK_EN			: boolean := false;		--! CPU break on PUC reset
	DBG_HWBRK_0_EN			: boolean := false;		--	Include hardware breakpoints unit 
	DBG_HWBRK_1_EN			: boolean := false;		--	Include hardware breakpoints unit 
	DBG_HWBRK_2_EN			: boolean := false;		--	Include hardware breakpoints unit 
	DBG_HWBRK_3_EN			: boolean := false;		--	Include hardware breakpoints unit 
	DBG_HWBRK_RANGE		: boolean := true;		--! Enable/Disable the hardware breakpoint RANGE mode
	DBG_UART_AUTO_SYNC	: boolean := true;		--! Debug UART interface auto data synchronization
	DBG_UART_BAUD			: integer := 9600;		--! Debug UART interface data rate
	DBG_DCO_FREQ			: integer := 20000000	--! Debug DCO_CLK frequency
);
port (
	mclk					: in	std_logic;										-- Main system clock
	-- INPUTs
	lfxt_clk				: in	std_logic;										-- Low frequency oscillator (typ 32kHz)
	reset_n				: in	std_logic;										-- Reset Pin (active low, asynchronous and non-glitchy)
	cpu_en				: in	std_logic;										-- Enable CPU code execution (asynchronous and non-glitchy)
	nmi					: in	std_logic;										-- Non-maskable interrupt (asynchronous and non-glitchy)
	-- Debug interface
	dbg_en				: in	std_logic;										-- Debug interface enable (asynchronous and non-glitchy)
	dbg_i2c_addr		: in	std_logic_vector(6 downto 0);				-- Debug interface: I2C Address
	dbg_i2c_broadcast	: in	std_logic_vector(6 downto 0);				-- Debug interface: I2C Broadcast Address (for multicore systems)
	dbg_i2c_scl			: in	std_logic;										-- Debug interface: I2C SCL
	dbg_i2c_sda_in		: in	std_logic;										-- Debug interface: I2C SDA IN
	dbg_i2c_sda_out	: out	std_logic := '1';								-- Debug interface: I2C SDA OUT
	dbg_uart_rxd		: in	std_logic;										-- Debug interface: UART RXD (asynchronous)
	dbg_uart_txd		: out	std_logic := '1';								-- Debug interface: UART TXD
	-- DMA access
	dma_addr				: in	std_logic_vector(15 downto 1);			-- Direct Memory Access address
	dma_dout				: out	std_logic_vector(15 downto 0);			-- Direct Memory Access data output
	dma_din				: in	std_logic_vector(15 downto 0);			-- Direct Memory Access data input
	dma_en				: in	std_logic;										-- Direct Memory Access enable (high active)
	dma_we				: in	std_logic_vector(1 downto 0);				-- Direct Memory Access write byte enable (high active)
	dma_priority		: in	std_logic;										-- Direct Memory Access priority (0:low / 1:high)
	dma_ready			: out	std_logic;										-- Direct Memory Access is complete
	dma_resp				: out	std_logic;										-- Direct Memory Access response (0:Okay / 1:Error)
	-- Data memory
	dmem_addr			: out	std_logic_vector(f_log2(DMEM_SIZE)-2 downto 0);	-- Data Memory address
	dmem_dout			: in	std_logic_vector(15 downto 0);			-- Data Memory data output
	dmem_din				: out	std_logic_vector(15 downto 0);			-- Data Memory data input
	dmem_wen				: out	std_logic_vector(1 downto 0);				-- Data Memory write byte enable (low active)
	dmem_cen				: out	std_logic;										-- Data Memory chip enable (low active)
	-- Program memory
	pmem_addr			: out	std_logic_vector(f_log2(PMEM_SIZE)-2 downto 0);	-- Program Memory address
	pmem_dout			: in	std_logic_vector(15 downto 0);			-- Program Memory data output
	pmem_din				: out	std_logic_vector(15 downto 0);			-- Program Memory data input (optional)
	pmem_wen				: out	std_logic_vector(1 downto 0);				-- Program Memory write enable (low active) (optional)
	pmem_cen				: out	std_logic;										-- Program Memory chip enable (low active)
	-- Peripheral interface
	per_irq				: in	std_logic_vector(IRQ_NR-3 downto 0);	-- Maskable interrupts (14, 30 or 62)
	per_irq_acc			: out	std_logic_vector(IRQ_NR-3 downto 0);	-- Interrupt request accepted (one-hot signal)
	per_rst				: out	std_logic;										-- Main system reset
	per_freeze			: out	std_logic;										-- Freeze peripherals
	per_aclk_en			: out	std_logic;										-- FPGA ONLY: ACLK enable
	per_smclk_en		: out	std_logic;										-- FPGA ONLY: SMCLK enable
	-- Peripheral memory
	per_addr				: out	std_logic_vector(13 downto 0);			-- Peripheral address
	per_dout				: in	std_logic_vector(15 downto 0);			-- Peripheral data output
	per_din				: out	std_logic_vector(15 downto 0);			-- Peripheral data input
	per_we				: out	std_logic_vector(1 downto 0);				-- Peripheral write byte enable (high active)
	per_en				: out	std_logic										-- Peripheral enable (high active)
);
end component fmsp430;

end fmsp430_package; --! fmsp_package

