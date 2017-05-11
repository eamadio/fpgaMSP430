------------------------------------------------------------------------------
-- Copyright (C) 2009 , Olivier Girard
--
-- Redistribution and use in source and binary forms, with or without
-- modification, are permitted provided that the following conditions
-- are met:
--     * Redistributions of source code must retain the above copyright
--       notice, this list of conditions and the following disclaimer.
--     * Redistributions in binary form must reproduce the above copyright
--       notice, this list of conditions and the following disclaimer in the
--       documentation and/or other materials provided with the distribution.
--     * Neither the name of the authors nor the names of its contributors
--       may be used to endorse or promote products derived from this software
--       without specific prior written permission.
--
-- THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
-- AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
-- IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
-- ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
-- LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
-- OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
-- SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
-- INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
-- CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
-- ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
-- THE POSSIBILITY OF SUCH DAMAGE
--
------------------------------------------------------------------------------
--
--! @file fmsp430.vhd
--! 
--! @brief fpgaMSP430 Top level file
--
--! @author Olivier Girard,    olgirard@gmail.com
--! @author Emmanuel Amadio,   emmanuel.amadio@gmail.com (VHDL Rewrite)
--
------------------------------------------------------------------------------
--! @version 1
--! @date: 2017-04-21
------------------------------------------------------------------------------

library ieee;
	use ieee.std_logic_1164.all;	-- standard unresolved logic UX01ZWLH-
	use ieee.numeric_std.all;		-- for the signed, unsigned types and arithmetic ops
	use ieee.math_real.all;
	use work.fmsp_misc_package.all;
	use work.fmsp_core_package.all;
	use work.fmsp_per_package.all;
	use work.fmsp_dbg_package.all;
	use work.fmsp_functions.all;

entity  fmsp430 is
generic (
	INST_NR					: integer := 0;			-- Current fmsp instance number     (for multicore systems)
	TOTAL_NR					: integer := 0;			-- Total number of fmsp instances-1 (for multicore systems)
	PMEM_SIZE				: integer := 32768;		-- Program Memory Size
	DMEM_SIZE				: integer := 16384;		-- Data Memory Size
	PER_SIZE					: integer := 16384;		-- Peripheral Memory Size
	MULTIPLIER				: boolean := false;		-- Include/Exclude Hardware Multiplier
	USER_VERSION			: integer := 0;			-- Custom user version number
	DEBUG_EN					: boolean := false;		-- Include/Exclude Serial Debug interface
	WATCHDOG					: boolean := false;		-- Include/Exclude Watchdog timer
	DMA_IF_EN				: boolean := false;		-- Include/Exclude DMA interface support
	NMI_EN					: boolean := false;		-- Include/Exclude Non-Maskable-Interrupt support
	IRQ_NR					: integer := 16;			-- Number of IRQs
	SYNC_NMI_EN				: boolean := true;		-- 
	SYNC_CPU_EN				: boolean := true;		-- 
	SYNC_DBG_EN				: boolean := true;		-- 
	SYNC_DBG_UART_RXD		: boolean := true;		-- Synchronize RXD inputs
	DBG_UART					: boolean := false;		--	Enable UART (8N1) debug interface
	DBG_I2C					: boolean := true;		--	Enable I2C debug interface
	DBG_I2C_BROADCAST_EN	: boolean := false;		-- Enable the I2C broadcast address
	DBG_RST_BRK_EN			: boolean := false;		-- CPU break on PUC reset
	DBG_HWBRK_0_EN			: boolean := false;		--	Include hardware breakpoints unit 
	DBG_HWBRK_1_EN			: boolean := false;		--	Include hardware breakpoints unit 
	DBG_HWBRK_2_EN			: boolean := false;		--	Include hardware breakpoints unit 
	DBG_HWBRK_3_EN			: boolean := false;		--	Include hardware breakpoints unit 
	DBG_HWBRK_RANGE		: boolean := true;		-- Enable/Disable the hardware breakpoint RANGE mode
	DBG_UART_AUTO_SYNC	: boolean := true;		-- Debug UART interface auto data synchronization
	DBG_UART_BAUD			: integer := 9600;		-- Debug UART interface data rate
	DBG_DCO_FREQ			: integer := 20000000
);
port (
	mclk					: in	std_logic;										-- Main system clock
	-- Inputs
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
	dmem_dout			: in	std_logic_vector(15 downto 0);						-- Data Memory data output
	dmem_din				: out	std_logic_vector(15 downto 0);						-- Data Memory data input
	dmem_wen				: out	std_logic_vector(1 downto 0);							-- Data Memory write byte enable (low active)
	dmem_cen				: out	std_logic;													-- Data Memory chip enable (low active)
	-- Program memory
	pmem_addr			: out	std_logic_vector(f_log2(PMEM_SIZE)-2 downto 0);	-- Program Memory address
	pmem_dout			: in	std_logic_vector(15 downto 0);						-- Program Memory data output
	pmem_din				: out	std_logic_vector(15 downto 0);						-- Program Memory data input (optional)
	pmem_wen				: out	std_logic_vector(1 downto 0);							-- Program Memory write enable (low active) (optional)
	pmem_cen				: out	std_logic;													-- Program Memory chip enable (low active)
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
end entity fmsp430;

architecture RTL of fmsp430 is 

	constant	C_INST_NR			: std_logic_vector(7 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(INST_NR,8));--x"00";
	constant	C_TOTAL_NR			: std_logic_vector(7 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(TOTAL_NR,8));--x"00";

--=============================================================================
-- 1)  INTERNAL WIRES/REGISTERS/PARAMETERS DECLARATION
--=============================================================================

	type core_wires_type is record
		cpu_en				: std_logic;
		lfxt_clk				: std_logic;
		cpuoff				: std_logic;
		oscoff				: std_logic;
		scg1					: std_logic;
		por					: std_logic;
		gie					: std_logic;
		cpu_id				: std_logic_vector(31 downto 0);
		nmi					: std_logic;
		nmi_acc				: std_logic;
		nmi_pnd				: std_logic;
		nmi_wkup				: std_logic;
		irq_acc				: std_logic_vector(IRQ_NR-3 downto 0);	-- Interrupt request accepted (one-hot signal)
		dma_dout				: std_logic_vector(15 downto 0);			-- Direct Memory Access data output
		dma_ready			: std_logic;									-- Direct Memory Access is complete
		dma_resp				: std_logic;									-- Direct Memory Access response (0:Okay / 1:Error)
		mrst					: std_logic;									-- Main system reset
	end record;

	type peripheral_wires_type is record
		addr		: std_logic_vector(13 downto 0);			-- Peripheral address
		din		: std_logic_vector(15 downto 0);			-- Peripheral data input
		en			: std_logic;									-- Peripheral enable (high active)
		we			: std_logic_vector(1 downto 0);			-- Peripheral write byte enable (high active)
		aclk_en	: std_logic;									-- FPGA ONLY: ACLK enable
		smclk_en	: std_logic;									-- FPGA ONLY: SMCLK enable
	end record;

	type dbg_wires_type is record
		en					: std_logic;
		rst				: std_logic;
		puc_pnd_set		: std_logic;
		decode_noirq	: std_logic;
		pc					: std_logic_vector(15 downto 0);
		halt_cmd			: std_logic;
		halt_st			: std_logic;
		irq				: std_logic;
		wkup				: std_logic;
		cpu_reset		: std_logic;
		freeze			: std_logic;
		mem_addr			: std_logic_vector(15 downto 0);
		mem_dout			: std_logic_vector(15 downto 0);
		mem_din			: std_logic_vector(15 downto 0);
		mem_wr			: std_logic_vector(1 downto 0);
		mem_en			: std_logic;
		reg_din			: std_logic_vector(15 downto 0);
		reg_wr			: std_logic;
		eu_mab			: std_logic_vector(15 downto 0);
		eu_mdb_in		: std_logic_vector(15 downto 0);
		eu_mdb_out		: std_logic_vector(15 downto 0);
		eu_mb_wr			: std_logic_vector(1 downto 0);
		eu_mb_en			: std_logic;
		fe_mab			: std_logic_vector(15 downto 0);
		fe_mdb_in		: std_logic_vector(15 downto 0);
		fe_mb_en			: std_logic;
		fe_pmem_wait	: std_logic;
	end record;

	type wdt_wires_type is record
		ie				: std_logic;
		nmies			: std_logic;
		ifg			: std_logic;
		irq			: std_logic;
		wkup			: std_logic;
		cpu_reset	: std_logic;
		ifg_sw_clr	: std_logic;
		ifg_sw_set	: std_logic;
	end record;

	signal	wires				: core_wires_type;
	signal	dbg_w				: dbg_wires_type := (	en					=> '0',
																	rst				=> '0',
																	puc_pnd_set		=> '0',
																	decode_noirq	=> '0',
																	pc					=> x"0000",
																	halt_cmd			=> '0',
																	halt_st			=> '0',
																	irq				=> '0',
																	wkup				=> '0',
																	cpu_reset		=> '0',
																	freeze			=> '0',
																	mem_addr			=> x"0000",
																	mem_dout			=> x"0000",
																	mem_din			=> x"0000",
																	mem_wr			=> "00",
																	mem_en			=> '0',
																	reg_din			=> x"0000",
																	reg_wr			=> '0',
																	eu_mab			=> x"0000",
																	eu_mdb_in		=> x"0000",
																	eu_mdb_out		=> x"0000",
																	eu_mb_wr			=> "00",
																	eu_mb_en			=> '0',
																	fe_mab			=> x"0000",
																	fe_mdb_in		=> x"0000",
																	fe_mb_en			=> '0',
																	fe_pmem_wait	=> '0'
																);
	signal	wdt_w				: wdt_wires_type;
	signal	per_w				: peripheral_wires_type;
	signal	per_dout_sfr	: std_logic_vector(15 downto 0);
	signal	per_dout_clk	: std_logic_vector(15 downto 0);
	signal	per_dout_wdt	: std_logic_vector(15 downto 0) := x"0000";
	signal	per_dout_mpy	: std_logic_vector(15 downto 0) := x"0000";
	signal	per_dout_or		: std_logic_vector(15 downto 0);

begin

	--=============================================================================
	-- 1)  CORE (<=> FETCH & DECODE & EXECUTE & MEMORY BACKBONE)
	--=============================================================================

   --! Synchronization
	sync_cell_dbg_en : fmsp_sync_cell
	generic map(
		SYNC_EN	=>	SYNC_DBG_EN 
	)
	port map(
		rst		=> wires.por,
		clk		=> mclk,
		data_in	=> dbg_en,
		data_out	=> dbg_w.en
	);
	sync_cell_lfxt_clk : fmsp_sync_cell
	generic map(
		SYNC_EN		=>	false 
	)
	port map(
		clk		=> mclk,
		rst		=> wires.por,
		data_in	=> lfxt_clk,
		data_out	=> wires.lfxt_clk
	);
	sync_cell_cpu_en : fmsp_sync_cell
	generic map(
		SYNC_EN	=>	SYNC_CPU_EN 
	)
	port map(
		clk		=> mclk,
		rst		=> wires.por,
		data_in	=> cpu_en,
		data_out	=> wires.cpu_en
	);
	sync_cell_nmi : fmsp_sync_cell
	generic map(
		SYNC_EN	=>	SYNC_NMI_EN 
	)
	port map(
		clk		=> mclk,
		rst		=> wires.mrst,
		data_in	=> nmi,
		data_out	=> wires.nmi
	);

	--=============================================================================
	-- 1)  CORE (<=> FETCH & DECODE & EXECUTE & MEMORY BACKBONE)
	--=============================================================================

	core_unit : fmsp_core
	generic map(
		PMEM_SIZE		=>	PMEM_SIZE,				-- Program Memory Size
		DMEM_SIZE		=>	DMEM_SIZE,				-- Data Memory Size
		PER_SIZE			=>	PER_SIZE,
		DMA_IF_EN		=>	DMA_IF_EN,				-- Wakeup condition from DMA interface
		IRQ_NR			=>	IRQ_NR					-- Number of IRQs
	)
	port map(
		mclk				=>	mclk,						-- Main system clock
		mrst				=>	wires.mrst,				-- Main system reset
		-- Debug Interface
		dbg_halt_cmd	=>	dbg_w.halt_cmd,		-- Debug interface Halt CPU command
		dbg_halt_st		=>	dbg_w.halt_st,			-- Halt/Run status from CPU
		dbg_reg_din		=>	dbg_w.reg_din,			-- Debug unit CPU register data input
		dbg_reg_wr		=>	dbg_w.reg_wr,			-- Debug unit CPU register write
		dbg_mem_addr	=>	dbg_w.mem_addr,		-- Debug address for rd/wr access
		dbg_mem_dout	=>	dbg_w.mem_dout,		-- Debug unit data output
		dbg_mem_din		=>	dbg_w.mem_din,			-- Debug unit Memory data input
		dbg_mem_en		=>	dbg_w.mem_en,			-- Debug unit memory enable
		dbg_mem_wr		=>	dbg_w.mem_wr,			-- Debug unit memory write
		eu_mem_addr		=>	dbg_w.eu_mab,			-- Execution-Unit Memory address bus
		eu_mem_en		=>	dbg_w.eu_mb_en,		-- Execution-Unit Memory bus enable
		eu_mem_wr		=>	dbg_w.eu_mb_wr,		-- Execution-Unit Memory bus write transfer
		fe_mem_din		=>	dbg_w.fe_mdb_in,		-- Frontend Memory data bus input
		decode_noirq	=>	dbg_w.decode_noirq,	-- Frontend decode instruction
		pc					=>	dbg_w.pc,				-- Program counter
		-- DMA access
		dma_addr			=>	dma_addr,				-- Direct Memory Access address
		dma_dout			=>	dma_dout,				-- Direct Memory Access data output
		dma_din			=>	dma_din,					-- Direct Memory Access data input
		dma_we			=>	dma_we,					-- Direct Memory Access write byte enable (high active)
		dma_en			=>	dma_en,					-- Direct Memory Access enable (high active)
		dma_priority	=>	dma_priority,			-- Direct Memory Access priority (0:low / 1:high)
		dma_ready		=>	dma_ready,				-- Direct Memory Access is complete
		dma_resp			=>	dma_resp,				-- Direct Memory Access response (0:Okay / 1:Error)
		-- Peripheral memory
		per_addr			=>	per_w.addr,				-- Peripheral address
		per_dout			=>	per_dout_or,			-- Peripheral data output
		per_din			=>	per_w.din,				-- Peripheral data input
		per_we			=>	per_w.we,				-- Peripheral write enable (high active)
		per_en			=>	per_w.en,				-- Peripheral enable (high active)
		-- Data memory
		dmem_addr		=>	dmem_addr,				-- Data Memory address
		dmem_dout		=>	dmem_dout,				-- Data Memory data output
		dmem_din			=>	dmem_din,				-- Data Memory data input
		dmem_wen			=>	dmem_wen,				-- Data Memory write enable (low active)
		dmem_cen			=>	dmem_cen,				-- Data Memory chip enable (low active)
		-- Program memory
		pmem_addr		=>	pmem_addr,				-- Program Memory address
		pmem_dout		=>	pmem_dout,				-- Program Memory data output
		pmem_din			=>	pmem_din,				-- Program Memory data input (optional)
		pmem_wen			=>	pmem_wen,				-- Program Memory write enable (low active) (optional)
		pmem_cen			=>	pmem_cen,					-- Program Memory chip enable (low active)
		-- Non Maskable Interrupt
		nmi_pnd			=>	wires.nmi_pnd,			-- Non-maskable interrupt pending
		nmi_wkup			=>	wires.nmi_wkup,		-- NMI Wakeup
		nmi_acc			=>	wires.nmi_acc,			-- Non-Maskable interrupt request accepted
		-- Watchdog Interrupt
		wdt_irq			=>	wdt_w.irq,				-- Watchdog-timer interrupt
		wdt_wkup			=>	wdt_w.wkup,				-- Watchdog Wakeup
		-- Maskable Interrupt
		irq				=>	per_irq,					-- Maskable interrupts
		irq_acc			=>	wires.irq_acc,			-- Interrupt request accepted
		--============
		cpu_en_s			=>	wires.cpu_en,			-- Enable CPU code execution (synchronous)
		cpuoff			=>	wires.cpuoff,			-- Turns off the CPU
		oscoff			=>	wires.oscoff,			-- Turns off LFXT1 clock input
		scg1				=>	wires.scg1				-- System clock generator 1. Turns off the SMCLK
	);

	--=============================================================================
	-- 2)  GLOBAL CLOCK & RESET MANAGEMENT
	--=============================================================================

	clock_module_0 : fmsp_clock_module
	generic map(
		INST_NR				=>	INST_NR,			-- Current fmsp instance number     (for multicore systems)
		TOTAL_NR				=>	TOTAL_NR,		-- Total number of fmsp instances-1 (for multicore systems)
		PMEM_SIZE			=>	PMEM_SIZE,		-- Program Memory Size
		DMEM_SIZE			=>	DMEM_SIZE,		-- Data Memory Size
		PER_SIZE				=>	PER_SIZE,		-- Peripheral Memory Size
		DEBUG_EN				=>	DEBUG_EN,		-- Include/Exclude Serial Debug interface
		WATCHDOG				=>	WATCHDOG,		-- Include/Exclude Watchdog timer
		DMA_IF_EN			=>	DMA_IF_EN,		-- Include/Exclude DMA interface support
		NMI_EN				=>	NMI_EN			-- Include/Exclude Non-Maskable-Interrupt support
	)
	port map(
		mclk					=>	mclk,						-- Main system clock
		-- INPUTs
		reset_n				=>	reset_n,					-- Reset Pin (low active, asynchronous)
		lfxt_clk				=>	wires.lfxt_clk,		-- Low frequency oscillator (typ 32kHz)
		cpu_en				=>	wires.cpu_en,			-- Enable CPU code execution (synchronous)
		cpuoff				=>	wires.cpuoff,			-- Turns off the CPU
		oscoff				=>	wires.oscoff,			-- Turns off LFXT1 clock input
		scg1					=>	wires.scg1,				-- System clock generator 1. Turns off the SMCLK
		wdt_reset			=>	wdt_w.cpu_reset,		-- Watchdog-timer reset
		-- OUTPUTs
		por					=>	wires.por,				-- Power-on reset
		puc_rst				=>	wires.mrst,				-- Main system reset
		-- Debug Interface
		dbg_rst				=>	dbg_w.rst,				-- Debug unit reset
		dbg_en				=>	dbg_w.en,				-- Debug interface enable (synchronous)
		puc_pnd_set			=>	dbg_w.puc_pnd_set,	-- PUC pending set for the serial debug interface
		dbg_cpu_reset		=>	dbg_w.cpu_reset,		-- Reset CPU from debug interface
		-- Peripheral interface
		smclk_en				=>	per_w.smclk_en,		-- SMCLK enable
		aclk_en				=>	per_w.aclk_en,			-- ACLK enable
		per_addr				=>	per_w.addr,				-- Peripheral address
		per_din				=>	per_w.din,				-- Peripheral data input
		per_en				=>	per_w.en,				-- Peripheral enable (high active)
		per_we				=>	per_w.we,				-- Peripheral write enable (high active)
		per_dout				=>	per_dout_clk			-- Peripheral data output
	);
	
	--=============================================================================
	-- 3)  SPECIAL FUNCTION REGISTERS
	--=============================================================================
	sfr_0 : fmsp_sfr
	generic map(
		INST_NR			=>	INST_NR,			-- Current fmsp instance number     (for multicore systems)
		TOTAL_NR			=>	TOTAL_NR,		-- Total number of fmsp instances-1 (for multicore systems)
		PMEM_SIZE		=>	PMEM_SIZE,		-- Program Memory Size
		DMEM_SIZE		=>	DMEM_SIZE,		-- Data Memory Size
		PER_SIZE			=>	PER_SIZE,		-- Peripheral Memory Size
		MULTIPLIER		=>	MULTIPLIER,		-- Include/Exclude Hardware Multiplier
		USER_VERSION	=>	USER_VERSION,	-- Custom user version number
		WATCHDOG			=>	WATCHDOG,		-- Include/Exclude Watchdog timer
		NMI_EN			=>	NMI_EN			-- Include/Exclude Non-Maskable-Interrupt support
	)
	port map(
		mclk				=>	mclk,					-- Main system clock
		mrst				=>	wires.mrst,			-- Main system reset
		cpu_id			=>	wires.cpu_id,		-- CPU ID
		-- Non Maskable Interrupt
		nmi				=>	wires.nmi,			-- Non-maskable interrupt (asynchronous)
		nmi_acc			=>	wires.nmi_acc,		-- Non-Maskable interrupt request accepted
		nmi_pnd			=>	wires.nmi_pnd,		-- NMI Pending
		nmi_wkup			=>	wires.nmi_wkup,	-- NMI Wakeup
		-- Watchdog Interface
		wdtie				=>	wdt_w.ie,			-- Watchdog-timer interrupt enable
		wdtifg			=>	wdt_w.ifg,			-- Watchdog-timer interrupt flag
		wdtifg_sw_clr	=>	wdt_w.ifg_sw_clr,	-- Watchdog-timer interrupt flag software clear
		wdtifg_sw_set	=>	wdt_w.ifg_sw_set,	-- Watchdog-timer interrupt flag software set
		wdtnmies			=>	wdt_w.nmies,		-- Watchdog-timer NMI edge selection
		-- Peripheral interface
		per_addr			=>	per_w.addr,			-- Peripheral address
		per_din			=>	per_w.din,			-- Peripheral data input
		per_en			=>	per_w.en,			-- Peripheral enable (high active)
		per_we			=>	per_w.we,			-- Peripheral write enable (high active)
		per_dout			=>	per_dout_sfr		-- Peripheral data output
	);


	--=============================================================================
	-- 4)  WATCHDOG TIMER
	--=============================================================================
	ADD_WATCHDOG : if WATCHDOG generate
		watchdog : fmsp_watchdog
		port map(
			mclk				=>	mclk,							-- Main system clock
			mrst				=>	wires.mrst,					-- Main system reset
			por				=>	wires.por,					-- Power-on reset
			-- INPUTs
			dbg_freeze		=>	dbg_w.freeze,				-- Freeze Watchdog counter
			wdtie				=>	wdt_w.ie,					-- Watchdog-timer interrupt enable
			wdtifg_irq_clr	=>	wires.irq_acc(IRQ_NR-6),	-- Clear Watchdog-timer interrupt flag
			wdtifg_sw_clr	=>	wdt_w.ifg_sw_clr,			-- Watchdog-timer interrupt flag software clear
			wdtifg_sw_set	=>	wdt_w.ifg_sw_set,			-- Watchdog-timer interrupt flag software set
			-- OUTPUTs
			wdt_reset		=>	wdt_w.cpu_reset,				-- Watchdog-timer reset
			wdt_irq			=>	wdt_w.irq,					-- Watchdog-timer interrupt
			wdt_wkup			=>	wdt_w.wkup,					-- Watchdog Wakeup
			wdtifg			=>	wdt_w.ifg,					-- Watchdog-timer interrupt flag
			wdtnmies			=>	wdt_w.nmies,				-- Watchdog-timer NMI edge selection
			-- Peripheral interface
			aclk_en			=>	per_w.aclk_en,				-- ACLK enable
			smclk_en			=>	per_w.smclk_en,			-- SMCLK enable
			per_addr			=>	per_w.addr,					-- Peripheral address
			per_din			=>	per_w.din,					-- Peripheral data input
			per_en			=>	per_w.en,					-- Peripheral enable (high active)
			per_we			=>	per_w.we,					-- Peripheral write enable (high active)
			per_dout			=>	per_dout_wdt				-- Peripheral data output
		);
	end generate ADD_WATCHDOG;

	--=============================================================================
	-- 5)  HARDWARE MULTIPLIER
	--=============================================================================
	ADD_MULTIPLIER : if MULTIPLIER generate
		multiplier : fmsp_multiplier
		port map(
			mclk			=>	mclk,				-- Main system clock
			mrst			=>	wires.mrst,		-- Main system reset
			-- Peripheral interface
			per_addr		=>	per_w.addr,		-- Peripheral address
			per_din		=>	per_w.din,		-- Peripheral data input
			per_en		=>	per_w.en,		-- Peripheral enable (high active)
			per_we		=>	per_w.we,		-- Peripheral write enable (high active)
			per_dout		=>	per_dout_mpy	-- Peripheral data output
		);
	end generate ADD_MULTIPLIER;

	--=============================================================================
	-- 6)  PERIPHERALS' OUTPUT BUS
	--=============================================================================

	per_dout_or	<=		per_dout		
						or	per_dout_clk
						or	per_dout_sfr
						or	per_dout_wdt
						or	per_dout_mpy;

	--=============================================================================
	-- 7)  DEBUG INTERFACE
	--=============================================================================

	dbg : fmsp_dbg
	generic map(
		DBG_DCO_FREQ			=>	DBG_DCO_FREQ,				-- Debug mclk frequency
		DBG_UART					=>	DBG_UART,					--	Enable UART (8N1) debug interface
		DBG_UART_AUTO_SYNC	=>	DBG_UART_AUTO_SYNC,		-- Debug UART interface auto data synchronization
		DBG_UART_BAUD			=>	DBG_UART_BAUD,				-- Debug UART interface data rate
		DBG_I2C					=>	DBG_I2C,						--	Enable I2C debug interface
		DBG_I2C_BROADCAST_EN	=>	DBG_I2C_BROADCAST_EN,	-- Enable the I2C broadcast address
		DBG_RST_BRK_EN			=>	DBG_RST_BRK_EN,			-- CPU break on PUC reset
		DBG_HWBRK_0_EN			=>	DBG_HWBRK_0_EN,			--	Include hardware breakpoints unit 
		DBG_HWBRK_1_EN			=>	DBG_HWBRK_1_EN,			--	Include hardware breakpoints unit 
		DBG_HWBRK_2_EN			=>	DBG_HWBRK_2_EN,			--	Include hardware breakpoints unit 
		DBG_HWBRK_3_EN			=>	DBG_HWBRK_3_EN,			--	Include hardware breakpoints unit 
		DBG_HWBRK_RANGE		=>	DBG_HWBRK_RANGE,			-- Enable/Disable the hardware breakpoint RANGE mode
		SYNC_DBG_UART_RXD		=>	SYNC_DBG_UART_RXD			-- Synchronize RXD inputs
	)
	port map(
		dbg_clk				=>	mclk,						-- Debug unit clock
		dbg_rst				=>	dbg_w.rst,				-- Debug unit reset
		-- INPUTs
		cpu_nr_inst			=>	C_INST_NR,				-- Current fmsp instance number
		cpu_nr_total		=>	C_TOTAL_NR,				-- Total number of fmsp instances-1
		cpu_id				=>	wires.cpu_id,			-- CPU ID
		cpu_en_s				=>	wires.cpu_en,			-- Enable CPU code execution (synchronous)
		dbg_en_s				=>	dbg_w.en,				-- Debug interface enable (synchronous)
		dbg_halt_st			=>	dbg_w.halt_st,			-- Halt/Run status from CPU
		-- I2C Interface
		dbg_i2c_addr		=>	dbg_i2c_addr,			-- Debug interface: I2C Address
		dbg_i2c_broadcast	=>	dbg_i2c_broadcast,	-- Debug interface: I2C Broadcast Address (for multicore systems)
		dbg_i2c_scl			=>	dbg_i2c_scl,			-- Debug interface: I2C SCL
		dbg_i2c_sda_in		=>	dbg_i2c_sda_in,		-- Debug interface: I2C SDA IN
		dbg_i2c_sda_out	=>	dbg_i2c_sda_out,		-- Debug interface: I2C SDA OUT
		-- UART Interface
		dbg_uart_rxd		=>	dbg_uart_rxd,			-- Debug interface: UART RXD (asynchronous)
		dbg_uart_txd		=>	dbg_uart_txd,			-- Debug interface: UART TXD
		-- Core interface
		dbg_mem_din			=>	dbg_w.mem_din,			-- Debug unit Memory data input
		dbg_mem_addr		=>	dbg_w.mem_addr,		-- Debug address for rd/wr access
		dbg_mem_dout		=>	dbg_w.mem_dout,		-- Debug unit data output
		dbg_mem_en			=>	dbg_w.mem_en,			-- Debug unit memory enable
		dbg_mem_wr			=>	dbg_w.mem_wr,			-- Debug unit memory write
		dbg_reg_din			=>	dbg_w.reg_din,			-- Debug unit CPU register data input
		decode_noirq		=>	dbg_w.decode_noirq,	-- Frontend decode instruction
		eu_mab				=>	dbg_w.eu_mab,			-- Execution-Unit Memory address bus
		eu_mb_en				=>	dbg_w.eu_mb_en,		-- Execution-Unit Memory bus enable
		eu_mb_wr				=>	dbg_w.eu_mb_wr,		-- Execution-Unit Memory bus write transfer
		fe_mdb_in			=>	dbg_w.fe_mdb_in,		-- Frontend Memory data bus input
		pc						=>	dbg_w.pc,				-- Program counter
		puc_pnd_set			=>	dbg_w.puc_pnd_set,	-- PUC pending set for the serial debug interface
		-- OUTPUTs
		dbg_cpu_reset		=>	dbg_w.cpu_reset,		-- Reset CPU from debug interface
		dbg_freeze			=>	dbg_w.freeze,			-- Freeze peripherals
		dbg_halt_cmd		=>	dbg_w.halt_cmd,		-- Halt CPU command
		dbg_reg_wr			=>	dbg_w.reg_wr			-- Debug unit CPU register write
	);

	-- Peripheral interface
	per_irq_acc		<=	wires.irq_acc;		-- Interrupt request accepted (one-hot signal)
	per_rst			<=	wires.mrst;			-- Main system reset
	per_freeze		<=	dbg_w.freeze;		-- Freeze peripherals
	per_aclk_en		<=	per_w.aclk_en;		-- ACLK enable
	per_smclk_en	<= per_w.smclk_en;	-- FPGA ONLY: SMCLK enable
	per_addr			<=	per_w.addr;			-- Peripheral address
	per_din			<=	per_w.din;			-- Peripheral data input
	per_en			<=	per_w.en;			-- Peripheral enable (high active)
	per_we			<=	per_w.we;			-- Peripheral write enable (high active)
end RTL;	-- fmsp430
