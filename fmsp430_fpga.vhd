------------------------------------------------------------------------------
-- Copyright (C) 2011 Authors
--
-- This source file may be used and distributed without restriction provided
-- that this copyright statement is not removed from the file and that any
-- derivative work contains the original copyright notice and the associated
-- disclaimer.
--
-- This source file is free software; you can redistribute it and/or modify
-- it under the terms of the GNU Lesser General Public License as published
-- by the Free Software Foundation; either version 2.1 of the License, or
-- (at your option) any later version.
--
-- This source is distributed in the hope that it will be useful, but WITHOUT
-- ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
-- FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public
-- License for more details.
--
-- You should have received a copy of the GNU Lesser General Public License
-- along with this source; if not, write to the Free Software Foundation,
-- Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
--
------------------------------------------------------------------------------
--
-- *File Name: fpgaMSP430_fpga.v
--
-- *Module Description:
--                      fpgaMSP430 FPGA Top-level for the DE0 Nano Soc
--
-- *Author(s):
--              - Olivier Girard,    olgirard@gmail.com
--
------------------------------------------------------------------------------
library ieee;
	use ieee.std_logic_1164.all;	-- standard unresolved logic UX01ZWLH-
	use ieee.numeric_std.all;		-- for the signed, unsigned types and arithmetic ops
	use work.fmsp430_package.all;
	use work.fmsp_per_package.all;
	use work.fmsp_functions.all;
	use work.fpga_package.all;

entity fmsp430_fpga is 
	port (
		-- USER CLOCKS
		FPGA_CLK		: in	std_logic;
		PIN_RESET_N	: in	std_logic;
		-- USER INTERFACE (FPGA)
		KEY			: in	std_logic_vector(3 downto 0);
		SW				: in	std_logic_vector(3 downto 0);
		LED			: out	std_logic_vector(7 downto 0);
		PIN_INCLK	: in	std_logic;
		PIN_TACLK	: in	std_logic;
		-- TIMER INTERFACE (FPGA)
		PIN_CCIXA	: in	std_logic_vector(2 downto 0);
		PIN_CCIXB	: in	std_logic_vector(2 downto 0);
		PIN_TAOUT	: out	std_logic_vector(2 downto 0);
		-- I2C DEBUG INTERFACE
		PIN_SCL		: inout	std_logic;
		PIN_SDA		: inout	std_logic
	);
end entity fmsp430_fpga;

architecture RTL of fmsp430_fpga is 

	constant	INST_NR					: integer := 0;			-- Current fmsp instance number     (for multicore systems)
	constant	TOTAL_NR					: integer := 0;			-- Total number of fmsp instances-1 (for multicore systems)
	constant	PMEM_SIZE				: integer := 16384;		-- Program Memory Size
	constant	DMEM_SIZE				: integer := 16384;		-- Data Memory Size
	constant	PER_SIZE					: integer := 16384;		-- Peripheral Memory Size
	constant	MULTIPLIER				: boolean := true;		-- Include/Exclude Hardware Multiplier
	constant	USER_VERSION			: integer := 0;			-- Custom user version number
	constant	DEBUG_EN					: boolean := true;		-- Include/Exclude Serial Debug interface
	constant	WATCHDOG					: boolean := true;		-- Include/Exclude Watchdog timer
	constant	CPUOFF_EN				: boolean := false;		-- Wakeup condition from DMA interface
	constant	DMA_IF_EN				: boolean := true;		-- Include/Exclude DMA interface support
	constant	NMI_EN					: boolean := true;		-- Include/Exclude Non-Maskable-Interrupt support
	constant	IRQ_NR					: integer := 16;			-- Number of IRQs
	constant	SYNC_NMI_EN				: boolean := true;		-- 
	constant	SYNC_CPU_EN				: boolean := true;		-- 
	constant	SYNC_DBG_EN				: boolean := true;		-- 
	constant	SYNC_DBG_UART_RXD		: boolean := true;		-- Synchronize RXD inputs
	constant	DBG_I2C_BROADCAST_EN	: boolean := true;		-- Enable the I2C broadcast address
	constant	DBG_RST_BRK_EN			: boolean := true;		-- CPU break on PUC reset
	constant	DBG_HWBRK_0_EN			: boolean := false;		--	Include hardware breakpoints unit 
	constant	DBG_HWBRK_1_EN			: boolean := false;		--	Include hardware breakpoints unit 
	constant	DBG_HWBRK_2_EN			: boolean := false;		--	Include hardware breakpoints unit 
	constant	DBG_HWBRK_3_EN			: boolean := false;		--	Include hardware breakpoints unit 
	constant	DBG_HWBRK_RANGE		: boolean := false;		-- Enable/Disable the hardware breakpoint RANGE mode
	constant	DBG_UART					: boolean := false;		--	Enable UART (8N1) debug interface
	constant	DBG_UART_AUTO_SYNC	: boolean := true;		-- Debug UART interface auto data synchronization
	constant	DBG_UART_BAUD			: integer := 9600;		-- Debug UART interface data rate
	constant	DBG_DCO_FREQ			: integer := 20000000;	-- Debug mclk frequency
	constant	DBG_I2C					: boolean := true;		--	Enable I2C debug interface

--=============================================================================
-- 1)  INTERNAL WIRES/REGISTERS/PARAMETERS DECLARATION
--=============================================================================

-- mclk Program memory bus
	signal	pmem_addr	: std_logic_vector(f_log2(pMEM_SIZE)-2 downto 0);
	signal	pmem_dout	: std_logic_vector(15 downto 0);
	signal	pmem_din		: std_logic_vector(15 downto 0);
	signal	pmem_cen		: std_logic;
	signal	pmem_wen		: std_logic_vector(1 downto 0);
-- mclk Data memory bus
	signal	dmem_addr	: std_logic_vector(f_log2(DMEM_SIZE)-2 downto 0);
	signal	dmem_dout	: std_logic_vector(15 downto 0);
	signal	dmem_din		: std_logic_vector(15 downto 0);
	signal	dmem_cen		: std_logic;
	signal	dmem_wen		: std_logic_vector(1 downto 0);
-- mclk Peripheral memory bus
	signal	per_addr		: std_logic_vector(13 downto 0);
	signal	per_dout		: std_logic_vector(15 downto 0);
	signal	per_din		: std_logic_vector(15 downto 0);
	signal	per_en		: std_logic;
	signal	per_we		: std_logic_vector(1 downto 0);

-- fpgaMSP430 IRQs
	signal	nmi			: std_logic;
	signal	irq_bus		: std_logic_vector(IRQ_NR-3 downto 0);
	signal	irq_acc		: std_logic_vector(IRQ_NR-3 downto 0);

-- fpgaMSP430 debug interface
	signal	dbg_freeze			: std_logic;
	signal	dbg_i2c_addr		: std_logic_vector(6 downto 0);
	signal	dbg_i2c_broadcast	: std_logic_vector(6 downto 0);
	signal	dbg_i2c_scl			: std_logic;
	signal	dbg_i2c_sda_in		: std_logic;
	signal	dbg_i2c_sda_out	: std_logic;
	signal	dbg_uart_txd		: std_logic;
	signal	dbg_uart_rxd		: std_logic;

-- fpgaMSP430 clocks and resets
	signal	mclk				: std_logic;
	signal	lfxt_clk			: std_logic;
	signal	aclk_en			: std_logic;
	signal	smclk_en			: std_logic;
	signal	reset_n			: std_logic;
	signal	puc_rst			: std_logic;
	signal	mrst				: std_logic;

-- LED / KEY / SW
	signal	irq_key					: std_logic;
	signal	irq_sw					: std_logic;
	signal	per_dout_led_key_sw	: std_logic_vector(15 downto 0);

-- Timer A
	signal	irq_ta0				: std_logic;
	signal	irq_ta1				: std_logic;
	signal	per_dout_tA			: std_logic_vector(15 downto 0);
	signal	reset_in_n			: std_logic;
	signal	reset_dly_chain	: std_logic_vector(7 downto 0);
	signal	lfxt_clk_cnt		: unsigned(8 downto 0);
	signal	ta_outx				: std_logic_vector(2 downto 0);
	signal	ta_outx_en			: std_logic_vector(2 downto 0);

begin


	--=============================================================================
	-- 2)  CLOCK AND RESET GENERATION
	--=============================================================================

	mclk			<= FPGA_CLK;
	reset_in_n	<= PIN_RESET_N;

	-- Release system reset a few clock cyles after the FPGA power-on-reset
	RESET_DELAY : process (mclk,reset_in_n)
	begin
		if (reset_in_n = '0') then
			reset_dly_chain		<=	x"00";
		elsif rising_edge(mclk) then
			reset_dly_chain	<= '1' & reset_dly_chain(7 downto 1);
		end if;
	end process RESET_DELAY;
	reset_n	<= reset_dly_chain(0);

	-- Generate a slow reference clock LFXT_CLK (10us period)
	LFXT_CLK_COUNTER : process (mclk,reset_n)
	begin
		if (reset_n = '0') then
			lfxt_clk_cnt		<=	"000000000";
		elsif rising_edge(mclk) then
			lfxt_clk_cnt	<= lfxt_clk_cnt + 1;
		end if;
	end process LFXT_CLK_COUNTER;
	lfxt_clk	<= lfxt_clk_cnt(8);


--=============================================================================
-- 3)  fpgaMSP430
--=============================================================================

	fmsp430_0 : fmsp430
	generic map(
		INST_NR					=>	INST_NR,
		TOTAL_NR					=>	TOTAL_NR,
		PMEM_SIZE				=>	PMEM_SIZE,
		DMEM_SIZE				=>	DMEM_SIZE,
		PER_SIZE					=>	PER_SIZE,
		MULTIPLIER				=>	MULTIPLIER,
		USER_VERSION			=>	USER_VERSION,
		DEBUG_EN					=>	DEBUG_EN,
		WATCHDOG					=>	WATCHDOG,
		DMA_IF_EN				=>	DMA_IF_EN,
		NMI_EN					=>	NMI_EN,
		IRQ_NR					=>	IRQ_NR,
		SYNC_NMI_EN				=>	SYNC_NMI_EN,
		SYNC_CPU_EN				=>	SYNC_CPU_EN,
		SYNC_DBG_EN				=>	SYNC_DBG_EN,
		SYNC_DBG_UART_RXD		=>	SYNC_DBG_UART_RXD,		-- Synchronize RXD inputs
		DBG_UART					=>	DBG_UART,					--	Enable UART (8N1) debug interface
		DBG_I2C					=>	DBG_I2C,						--	Enable I2C debug interface
		DBG_I2C_BROADCAST_EN	=>	DBG_I2C_BROADCAST_EN,	-- Enable the I2C broadcast address
		DBG_RST_BRK_EN			=>	DBG_RST_BRK_EN,			-- CPU break on PUC reset
		DBG_HWBRK_0_EN			=>	DBG_HWBRK_0_EN,			--	Include hardware breakpoints unit 
		DBG_HWBRK_1_EN			=>	DBG_HWBRK_1_EN,			--	Include hardware breakpoints unit 
		DBG_HWBRK_2_EN			=>	DBG_HWBRK_2_EN,			--	Include hardware breakpoints unit 
		DBG_HWBRK_3_EN			=>	DBG_HWBRK_3_EN,			--	Include hardware breakpoints unit 
		DBG_HWBRK_RANGE		=>	DBG_HWBRK_RANGE,			-- Enable/Disable the hardware breakpoint RANGE mode
		DBG_UART_AUTO_SYNC	=>	DBG_UART_AUTO_SYNC,		-- Debug UART interface auto data synchronization
		DBG_UART_BAUD			=>	DBG_UART_BAUD,				-- Debug UART interface data rate
		DBG_DCO_FREQ			=>	DBG_DCO_FREQ				-- Debug mclk frequency
	)
	port map(
		mclk					=>	mclk,						-- Main system clock
		-- INPUTs
		lfxt_clk				=>	lfxt_clk,				-- Low frequency oscillator (typ 32kHz)
		reset_n				=>	reset_n,					-- Reset Pin (low active, asynchronous and non-glitchy)
		cpu_en				=>	'1',						-- Enable CPU code execution (asynchronous and non-glitchy)
		nmi					=>	nmi,						-- Non-maskable interrupt (asynchronous)
		-- Debug interface
		dbg_en				=>	'1',						-- Debug interface enable (asynchronous and non-glitchy)
		dbg_i2c_sda_out	=>	dbg_i2c_sda_out,		-- Debug interface: I2C SDA OUT
		dbg_i2c_addr		=>	dbg_i2c_addr,			-- Debug interface: I2C Address
		dbg_i2c_broadcast	=>	dbg_i2c_broadcast,	-- Debug interface: I2C Broadcast Address (for multicore systems)
		dbg_i2c_scl			=>	dbg_i2c_scl,			-- Debug interface: I2C SCL
		dbg_i2c_sda_in		=>	dbg_i2c_sda_in,		-- Debug interface: I2C SDA IN
		dbg_uart_rxd		=>	dbg_uart_rxd,			-- Debug interface: UART RXD (asynchronous)
		dbg_uart_txd		=>	dbg_uart_txd,			-- Debug interface: UART TXD
		-- DMA access
		dma_addr				=>	"000000000000000",	-- Direct Memory Access address
		dma_dout				=>	open,						-- Direct Memory Access data output
		dma_din				=>	x"0000",					-- Direct Memory Access data input
		dma_we				=>	"00",						-- Direct Memory Access write byte enable (high active)
		dma_en				=>	'0',						-- Direct Memory Access enable (high active)
		dma_priority		=>	'0',						-- Direct Memory Access priority (0:low / 1:high)
		dma_ready			=>	open,						-- Direct Memory Access is complete
		dma_resp				=>	open,						-- Direct Memory Access response (0:Okay / 1:Error)
		-- Data memory
		dmem_addr			=>	dmem_addr,				-- Data Memory address
		dmem_dout			=>	dmem_dout,				-- Data Memory data output
		dmem_din				=>	dmem_din,				-- Data Memory data input
		dmem_wen				=>	dmem_wen,				-- Data Memory write enable (low active)
		dmem_cen				=>	dmem_cen,				-- Data Memory chip enable (low active)
		-- Program memory
		pmem_addr			=>	pmem_addr,				-- Program Memory address
		pmem_dout			=>	pmem_dout,				-- Program Memory data output
		pmem_din				=>	pmem_din,				-- Program Memory data input (optional)
		pmem_wen				=>	pmem_wen,					-- Program Memory write enable (low active) (optional)
		pmem_cen				=>	pmem_cen,				-- Program Memory chip enable (low active)
		-- Peripheral interface
		per_irq				=>	irq_bus,					-- Maskable interrupts
		per_irq_acc			=>	irq_acc,					-- Interrupt request accepted (one-hot signal)
		per_rst				=>	puc_rst,					-- Main system reset
		per_freeze			=>	dbg_freeze,				-- Freeze peripherals
		per_aclk_en			=>	aclk_en,					-- FPGA ONLY: ACLK enable
		per_smclk_en		=>	smclk_en,				-- FPGA ONLY: SMCLK enable
		-- Peripheral memory
		per_addr				=>	per_addr,				-- Peripheral address
		per_dout				=>	per_dout,				-- Peripheral data output
		per_din				=>	per_din,					-- Peripheral data input
		per_we				=>	per_we,					-- Peripheral write enable (high active)
		per_en				=>	per_en					-- Peripheral enable (high active)
	);

	--=============================================================================
	-- 4)  fpgaMSP430 PERIPHERALS
	--=============================================================================

	-------------------------------
	-- LED / KEY / SW interface
	-------------------------------
	de0_nano_soc_led_key_sw_0 : fmsp_de0_nano_soc_led_key_sw
	port map(
		-- INPUTs
		mclk		=>	mclk,						-- Main system clock
		puc_rst	=>	puc_rst,					-- Main system reset
		key		=>	KEY,						-- key/button inputs
		sw			=>	SW,						-- switches inputs
		per_addr	=>	per_addr,				-- Peripheral address
		per_din	=>	per_din,					-- Peripheral data input
		per_en	=>	per_en,					-- Peripheral enable (high active)
		per_we	=>	per_we,					-- Peripheral write enable (high active)
		-- OUTPUTs
		irq_key	=>	irq_key,					-- Key/Button interrupt
		irq_sw	=>	irq_sw,					-- Switch interrupt
		led		=>	LED,						-- LED output control
		per_dout	=>	per_dout_led_key_sw	-- Peripheral data output
	);

-------------------------------
-- Timer A
-------------------------------

	timerA_0 : fmsp_timerA
	port map(
		mclk			=>	mclk,				-- Main system clock
		mrst			=>	mrst,				-- Main system reset
		-- INPUTs
		aclk_en		=>	aclk_en,			-- ACLK enable (from CPU)
		smclk_en		=>	smclk_en,		-- SMCLK enable (from CPU)
		dbg_freeze	=>	dbg_freeze,		-- Freeze Timer A counter
		inclk			=>	PIN_INCLK,		-- INCLK external timer clock (SLOW)
		taclk			=>	PIN_TACLK,		-- TACLK external timer clock (SLOW)
		irq_ta0_acc	=>	irq_acc(9),		-- Interrupt request TACCR0 accepted
		per_addr		=>	per_addr,		-- Peripheral address
		per_din		=>	per_din,			-- Peripheral data input
		per_en		=>	per_en,			-- Peripheral enable (high active)
		per_we		=>	per_we,			-- Peripheral write enable (high active)
		ta_cci0a		=>	PIN_CCIXA(0),	-- Timer A capture 0 input A
		ta_cci0b		=>	PIN_CCIXB(0),	-- Timer A capture 0 input B
		ta_cci1a		=>	PIN_CCIXA(1),	-- Timer A capture 1 input A
		ta_cci1b		=>	PIN_CCIXB(1),	-- Timer A capture 1 input B
		ta_cci2a		=>	PIN_CCIXA(2),	-- Timer A capture 2 input A
		ta_cci2b		=>	PIN_CCIXB(2),	-- Timer A capture 2 input B
		-- OUTPUTs
		irq_ta0		=>	irq_ta0,			-- Timer A interrupt: TACCR0
		irq_ta1		=>	irq_ta1,			-- Timer A interrupt: TAIV, TACCR1, TACCR2
		per_dout		=>	per_dout_tA,	-- Peripheral data output
		ta_out0		=>	ta_outx(0),		-- Timer A output 0
		ta_out0_en	=>	ta_outx_en(0),	-- Timer A output 0 enable
		ta_out1		=>	ta_outx(1),		-- Timer A output 1
		ta_out1_en	=>	ta_outx_en(1),	-- Timer A output 1 enable
		ta_out2		=>	ta_outx(2),		-- Timer A output 2
		ta_out2_en	=>	ta_outx_en(2)	-- Timer A output 2 enable
	);
	PIN_TAOUT(0)	<=	'Z' when (ta_outx_en(0) = '0') else ta_outx(0);
	PIN_TAOUT(1)	<=	'Z' when (ta_outx_en(1) = '0') else ta_outx(1);
	PIN_TAOUT(2)	<=	'Z' when (ta_outx_en(2) = '0') else ta_outx(2);

-------------------------------
-- Combine peripheral
-- data buses
-------------------------------

	per_dout	<=		per_dout_led_key_sw
					or per_dout_tA;


-------------------------------
-- Assign interrupts
-------------------------------
	nmi		<= '0';
	irq_bus	<= 	'0'		-- Vector 13  (0xFFFA)
					&	'0'		-- Vector 12  (0xFFF8)
					&	'0'		-- Vector 11  (0xFFF6)
					&	'0'		-- Vector 10  (0xFFF4) - Watchdog -
					&	irq_ta0	-- Vector  9  (0xFFF2)
					&	irq_ta1	-- Vector  8  (0xFFF0)
					&	'0'		-- Vector  7  (0xFFEE)
					&	'0'		-- Vector  6  (0xFFEC)
					&	'0'		-- Vector  5  (0xFFEA)
					&	'0'		-- Vector  4  (0xFFE8)
					&	irq_key	-- Vector  3  (0xFFE6)
					&	irq_sw	-- Vector  2  (0xFFE4)
					&	'0'		-- Vector  1  (0xFFE2)
					&	'0';		-- Vector  0  (0xFFE0)


--=============================================================================
-- 5)  PROGRAM AND DATA MEMORIES
--=============================================================================

	pmem_0 : ram_16x8k
	port map(
		address	=>	pmem_addr,
		byteena	=>	not(pmem_wen),
		clken		=>	not(pmem_cen),
		clock		=>	mclk,
		data		=>	pmem_din,
		wren		=>	not(pmem_wen(0) and pmem_wen(1)),
		q			=>	pmem_dout
	);

	dmem_0 : ram_16x8k
	port map(
		address	=>	dmem_addr,
		byteena	=>	not(dmem_wen),
		clken		=> not(dmem_cen),
		clock		=>	mclk,
		data		=>	dmem_din,
		wren		=>	not(dmem_wen(0) and dmem_wen(1)),
		q			=>	dmem_dout
	);

	--=============================================================================
	-- 6)  DEBUG INTERFACE
	--=============================================================================

	dbg_i2c_addr		<=  STD_LOGIC_VECTOR(TO_UNSIGNED(50,7));
	dbg_i2c_broadcast	<=  STD_LOGIC_VECTOR(TO_UNSIGNED(49,7));
	dbg_i2c_scl			<=  PIN_SCL;
	PIN_SDA				<=	'Z' when (dbg_i2c_sda_out = '1') else '0';
	dbg_i2c_sda_in		<=	PIN_SDA;

	dbg_uart_rxd		<=  '0';

--	io_buf_sda_0 : io_buf  
--	port map(
--		datain	=>	'0',
--		oe			=>	not(dbg_i2c_sda_out),
--		dataout	=>	dbg_i2c_sda_in,
--		dataio	=>	ARDUINO_IO(14)
--	);

end RTL;	-- fpgaMSP430_fpga

