------------------------------------------------------------------------------
-- Copyright (C) 2009 , Emmanuel Amadio
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
--! @file fmsp_core.vhd
--! 
--! @brief fpgaMSP430 Core
--
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
	use work.fmsp_core_package.all;
	use work.fmsp_functions.all;

entity  fmsp_core is
generic (
	PMEM_SIZE				: integer := 32768;		-- Program Memory Size
	DMEM_SIZE				: integer := 16384;		-- Data Memory Size
	PER_SIZE					: integer := 16384;		-- Peripheral Memory Size
	DMA_IF_EN				: boolean := false;		-- Include/Exclude DMA interface support
	IRQ_NR					: integer := 16;			-- Number of IRQs
	CPUOFF_EN				: boolean := false		-- Wakeup condition from DMA interface
);
port (
	mclk					: in	std_logic;										-- Main system clock
	mrst					: in	std_logic;										-- Main system reset
	-- Debug Interface
	dbg_halt_cmd		: in	std_logic := '0';
	dbg_halt_st			: out	std_logic := '0';
	dbg_reg_din			: out	std_logic_vector(15 downto 0) := x"0000";
	dbg_reg_wr			: in	std_logic := '0';
	dbg_mem_addr		: in	std_logic_vector(15 downto 0);
	dbg_mem_dout		: in	std_logic_vector(15 downto 0) := x"0000";
	dbg_mem_din			: out	std_logic_vector(15 downto 0) := x"0000";
	dbg_mem_en			: in	std_logic := '0';
	dbg_mem_wr			: in	std_logic_vector(1 downto 0) := "00";
	-- Execution unit memory bus
	eu_mem_addr			: out	std_logic_vector(15 downto 0);					-- Execution-Unit Memory address bus
	eu_mem_en			: out	std_logic;											-- Execution-Unit Memory bus enable
	eu_mem_wr			: out	std_logic_vector(1 downto 0);					-- Execution-Unit Memory bus write transfer
	-- Frontend memory bus
	fe_mem_din			: out	std_logic_vector(15 downto 0);					-- Frontend Memory data bus input
	-- DMA access
	dma_addr				: in	std_logic_vector(15 downto 1);			-- Direct Memory Access address
	dma_dout				: out	std_logic_vector(15 downto 0);			-- Direct Memory Access data output
	dma_din				: in	std_logic_vector(15 downto 0);			-- Direct Memory Access data input
	dma_en				: in	std_logic;										-- Direct Memory Access enable (high active)
	dma_we				: in	std_logic_vector(1 downto 0);				-- Direct Memory Access write byte enable (high active)
	dma_priority		: in	std_logic;										-- Direct Memory Access priority (0:low / 1:high)
	dma_ready			: out	std_logic;										-- Direct Memory Access is complete
	dma_resp				: out	std_logic;										-- Direct Memory Access response (0:Okay / 1:Error)
	-- Peripheral memory
	per_addr				: out	std_logic_vector(13 downto 0);			-- Peripheral address
	per_dout				: in	std_logic_vector(15 downto 0);			-- Peripheral data output
	per_din				: out	std_logic_vector(15 downto 0);			-- Peripheral data input
	per_en				: out	std_logic;										-- Peripheral enable (high active)
	per_we				: out	std_logic_vector(1 downto 0);				-- Peripheral write byte enable (high active)
	-- Program memory
	pmem_addr			: out	std_logic_vector(f_log2(PMEM_SIZE)-2 downto 0);	-- Program Memory address
	pmem_dout			: in	std_logic_vector(15 downto 0);			-- Program Memory data output
	pmem_din				: out	std_logic_vector(15 downto 0);			-- Program Memory data input (optional)
	pmem_cen				: out	std_logic;										-- Program Memory chip enable (low active)
	pmem_wen				: out	std_logic_vector(1 downto 0);				-- Program Memory write enable (low active) (optional)
	-- Data memory
	dmem_addr			: out	std_logic_vector(f_log2(DMEM_SIZE)-2 downto 0);	-- Data Memory address
	dmem_dout			: in	std_logic_vector(15 downto 0);			-- Data Memory data output
	dmem_din				: out	std_logic_vector(15 downto 0);			-- Data Memory data input
	dmem_cen				: out	std_logic;										-- Data Memory chip enable (low active)
	dmem_wen				: out	std_logic_vector(1 downto 0);				-- Data Memory write byte enable (low active)
	-- Non maskable interrupt
	nmi_acc				: out	std_logic;
	nmi_pnd				: in	std_logic;
	nmi_wkup				: in	std_logic;
	-- Watchdog interrupt
	wdt_irq				: in	std_logic;
	wdt_wkup				: in	std_logic;
	-- Interrupts
	irq					: in	std_logic_vector(IRQ_NR-3 downto 0);	-- Maskable interrupts (14, 30 or 62)
	irq_acc				: out	std_logic_vector(IRQ_NR-3 downto 0);	-- Interrupt request accepted (one-hot signal)
	cpu_en_s				: in	std_logic;
	decode_noirq		: out	std_logic;
	pc						: out	std_logic_vector(15 downto 0);
	cpuoff				: out	std_logic;
	oscoff				: out	std_logic;
	scg1					: out	std_logic
);
end entity fmsp_core;

architecture RTL of fmsp_core is 

--=============================================================================
-- 1)  INTERNAL WIRES/REGISTERS/PARAMETERS DECLARATION
--=============================================================================
	type core_wires_type is record
		inst_ad			: std_logic_vector(7 downto 0);
		inst_as			: std_logic_vector(7 downto 0);
		inst_alu			: std_logic_vector(11 downto 0);
		inst_bw			: std_logic;
		inst_irq_rst	: std_logic;
		inst_mov			: std_logic;
		inst_dest		: std_logic_vector(15 downto 0);
		inst_dext		: std_logic_vector(15 downto 0);
		inst_sext		: std_logic_vector(15 downto 0);
		inst_so			: std_logic_vector(7 downto 0);
		inst_src			: std_logic_vector(15 downto 0);
		inst_type		: std_logic_vector(2 downto 0);
		inst_jmp			: std_logic_vector(7 downto 0);
		exec_state		: std_logic_vector(3 downto 0);
		exec_done		: std_logic;
		cpu_halt_st		: std_logic;
		cpu_halt_cmd	: std_logic;
		-- Execution unit memory bus
		eu_mem_addr			: std_logic_vector(15 downto 0);
		eu_mem_dout			: std_logic_vector(15 downto 0);
		eu_mem_din			: std_logic_vector(15 downto 0);
		eu_mem_wr			: std_logic_vector(1 downto 0);
		eu_mem_en			: std_logic;
		-- Frontend memory bus
		fe_mem_addr			: std_logic_vector(15 downto 0);
		fe_mem_din			: std_logic_vector(15 downto 0);
		fe_mem_en			: std_logic;
		fe_mem_wait			: std_logic;
		-- Program counter
		eu_pc_sw_wr			: std_logic;
		eu_pc_sw				: std_logic_vector(15 downto 0);
		fe_pc					: std_logic_vector(15 downto 0);
		fe_pc_nxt			: std_logic_vector(15 downto 0);
		cpuoff				: std_logic;
		oscoff				: std_logic;
		scg1					: std_logic;
		gie					: std_logic;
	end record;

	signal	wires		: core_wires_type;


begin

	--=============================================================================
	-- 3)  FRONTEND (<=> FETCH & DECODE)
	--=============================================================================

	frontend_0 : fmsp_frontend
	generic map(
		CPUOFF_EN			=>	CPUOFF_EN,				-- Wakeup condition from DMA interface
		DMA_IF_EN			=>	DMA_IF_EN,				-- Wakeup condition from DMA interface
		IRQ_NR				=>	IRQ_NR					-- Number of IRQs
	)
	port map(
		mclk					=>	mclk,						-- Main system clock
		mrst					=>	mrst,						-- Main system reset
		-- Memory bus
		mdb_in				=>	wires.fe_mem_din,				-- Frontend Memory data bus input
		mab					=>	wires.fe_mem_addr,			-- Frontend Memory address bus
		mb_en					=>	wires.fe_mem_en,				-- Frontend Memory bus enable
		fe_pmem_wait		=>	wires.fe_mem_wait,			-- Frontend wait for Instruction fetch
		-- INPUTs
		cpu_en_s				=>	cpu_en_s,				-- Enable CPU code execution (synchronous)
		cpu_halt_cmd		=>	wires.cpu_halt_cmd,			-- Halt CPU command
		cpuoff				=>	wires.cpuoff,					-- Turns off the CPU
		dbg_reg_sel			=>	dbg_mem_addr(3 downto 0),	-- Debug selected register for rd/wr access
		dma_en				=>	dma_en,					-- Direct Memory Access enable (high active)
		gie					=>	wires.gie,					-- General interrupt enable
		-- Interrupts
		irq					=>	irq,						-- Maskable interrupts
		irq_acc				=>	irq_acc,					-- Interrupt request accepted
		-- Non maskable interrupt
		nmi_acc				=>	nmi_acc,					-- Non-Maskable interrupt request accepted
		nmi_pnd				=>	nmi_pnd,					-- Non-maskable interrupt pending
		nmi_wkup				=>	nmi_wkup,				-- NMI Wakeup
		-- Watchdog interrupt
		wdt_irq				=>	wdt_irq,					-- Watchdog-timer interrupt
		wdt_wkup				=>	wdt_wkup,				-- Watchdog Wakeup
		-- OUTPUTs
		cpu_halt_st			=>	wires.cpu_halt_st,	-- Halt/Run status from CPU
		decode_noirq		=>	decode_noirq,			-- Frontend decode instruction
		-- Decoded signals to execution unit
		e_state				=>	wires.exec_state,		-- Execution state
		exec_done			=>	wires.exec_done,		-- Execution completed
		inst_ad				=>	wires.inst_ad,			-- Decoded Inst: destination addressing mode
		inst_as				=>	wires.inst_as,			-- Decoded Inst: source addressing mode
		inst_alu				=>	wires.inst_alu,		-- ALU control signals
		inst_bw				=>	wires.inst_bw,			-- Decoded Inst: byte width
		inst_dest			=>	wires.inst_dest,		-- Decoded Inst: destination (one hot)
		inst_dext			=>	wires.inst_dext,		-- Decoded Inst: destination extended instruction word
		inst_irq_rst		=>	wires.inst_irq_rst,	-- Decoded Inst: Reset interrupt
		inst_jmp				=>	wires.inst_jmp,		-- Decoded Inst: Conditional jump
		inst_mov				=>	wires.inst_mov,		-- Decoded Inst: mov instruction
		inst_sext			=>	wires.inst_sext,		-- Decoded Inst: source extended instruction word
		inst_so				=>	wires.inst_so,			-- Decoded Inst: Single-operand arithmetic
		inst_src				=>	wires.inst_src,		-- Decoded Inst: source (one hot)
		inst_type			=>	wires.inst_type,		-- Decoded Instruction type
		-- Program counter
		pc_sw					=>	wires.eu_pc_sw,		-- Program counter software value
		pc_sw_wr				=>	wires.eu_pc_sw_wr,	-- Program counter software write
		pc						=>	wires.fe_pc,			-- Program counter
		pc_nxt				=>	wires.fe_pc_nxt		-- Next PC value (for CALL & IRQ)
	);


	--=============================================================================
	-- 4)  EXECUTION UNIT
	--=============================================================================

	execution_unit_0 : fmsp_execution_unit
	port map(
		mclk				=>	mclk,						-- Main system clock
		mrst				=>	mrst,						-- Main system reset
		-- INPUTs
		gie				=>	wires.gie,				-- General interrupt enable
		dbg_halt_st		=>	wires.cpu_halt_st,	-- Halt/Run status from CPU
		dbg_mem_dout	=>	dbg_mem_dout,			-- Debug unit data output
		dbg_reg_wr		=>	dbg_reg_wr,				-- Debug unit CPU register write
		dbg_reg_din		=>	dbg_reg_din,			-- Debug unit CPU register data input
		-- Memory bus
		mdb_in			=>	wires.eu_mem_din,				-- Memory data bus input
		mab				=>	wires.eu_mem_addr,			-- Memory address bus
		mb_en				=>	wires.eu_mem_en,				-- Memory bus enable
		mb_wr				=>	wires.eu_mem_wr,				-- Memory bus write transfer
		mdb_out			=>	wires.eu_mem_dout,			-- Memory data bus output
		-- Decoded signals from frontend
		e_state			=>	wires.exec_state,		-- Execution state
		exec_done		=>	wires.exec_done,		-- Execution completed
		inst_ad			=>	wires.inst_ad,			-- Decoded Inst: destination addressing mode
		inst_as			=>	wires.inst_as,			-- Decoded Inst: source addressing mode
		inst_alu			=>	wires.inst_alu,		-- ALU control signals
		inst_bw			=>	wires.inst_bw,			-- Decoded Inst: byte width
		inst_dest		=>	wires.inst_dest,		-- Decoded Inst: destination (one hot)
		inst_dext		=>	wires.inst_dext,		-- Decoded Inst: destination extended instruction word
		inst_irq_rst	=>	wires.inst_irq_rst,	-- Decoded Inst: reset interrupt
		inst_jmp			=>	wires.inst_jmp,		-- Decoded Inst: Conditional jump
		inst_mov			=>	wires.inst_mov,		-- Decoded Inst: mov instruction
		inst_sext		=>	wires.inst_sext,		-- Decoded Inst: source extended instruction word
		inst_so			=>	wires.inst_so,			-- Decoded Inst: Single-operand arithmetic
		inst_src			=>	wires.inst_src,		-- Decoded Inst: source (one hot)
		inst_type		=>	wires.inst_type,		-- Decoded Instruction type
		-- Program counter
		pc					=>	wires.fe_pc,			-- Program counter
		pc_nxt			=>	wires.fe_pc_nxt,		-- Next PC value (for CALL & IRQ)
		pc_sw				=>	wires.eu_pc_sw,		-- Program counter software value
		pc_sw_wr			=>	wires.eu_pc_sw_wr,	-- Program counter software write
		-- OUTPUTs
		cpuoff			=>	wires.cpuoff,			-- Turns off the CPU
		oscoff			=>	wires.oscoff,			-- Turns off LFXT1 clock input
		scg1				=>	wires.scg1				-- System clock generator 1. Turns off the SMCLK
	);


	--=============================================================================
	-- 5)  MEMORY BACKBONE
	--=============================================================================

	mem_backbone_0 : fmsp_mem_backbone
	generic map(
		PMEM_SIZE		=>	PMEM_SIZE,		-- Program Memory Size
		DMEM_SIZE		=>	DMEM_SIZE,		-- Data Memory Size
		PER_SIZE			=>	PER_SIZE,		-- Peripheral Memory Size
		DMA_IF_EN		=>	DMA_IF_EN		-- Wakeup condition from DMA interface
	)
	port map(
		mclk				=>	mclk,								-- Main system clock
		mrst				=>	mrst,								-- Main system reset
		-- INPUTs
		-- OUTPUTs
		cpu_halt_cmd	=>	wires.cpu_halt_cmd,			-- Halt CPU command
		cpu_halt_st		=>	wires.cpu_halt_st,			-- Halt/Run status from CPU
		-- Debug interface
		dbg_halt_cmd	=>	dbg_halt_cmd,					-- Debug interface Halt CPU command
		dbg_mem_addr	=>	dbg_mem_addr,					-- Debug address for rd/wr access
		dbg_mem_dout	=>	dbg_mem_dout,					-- Debug unit data output
		dbg_mem_din		=>	dbg_mem_din,					-- Debug unit Memory data input
		dbg_mem_en		=>	dbg_mem_en,						-- Debug unit memory enable
		dbg_mem_wr		=>	dbg_mem_wr,						-- Debug unit memory write
		-- Frontend memory bus
		fe_mab			=>	wires.fe_mem_addr(15 downto 1),	-- Frontend Memory address bus
		fe_mdb_in		=>	wires.fe_mem_din,						-- Frontend Memory data bus input
		fe_mb_en			=>	wires.fe_mem_en,						-- Frontend Memory bus enable
		fe_pmem_wait	=>	wires.fe_mem_wait,					-- Frontend wait for Instruction fetch
		-- Execution Unit  memory bus
		eu_mab			=>	wires.eu_mem_addr(15 downto 1),	-- Execution Unit Memory address bus
		eu_mdb_out		=>	wires.eu_mem_dout,					-- Execution Unit Memory data bus output
		eu_mdb_in		=>	wires.eu_mem_din,						-- Execution Unit Memory data bus input
		eu_mb_en			=>	wires.eu_mem_en,						-- Execution Unit Memory bus enable
		eu_mb_wr			=>	wires.eu_mem_wr,						-- Execution Unit Memory bus write transfer
		-- DMA bus
		dma_addr			=>	dma_addr&'0',					-- Direct Memory Access address
		dma_dout			=>	dma_dout,						-- Direct Memory Access data output
		dma_din			=>	dma_din,							-- Direct Memory Access data input
		dma_en			=>	dma_en,							-- Direct Memory Access enable (high active)
		dma_we			=>	dma_we,							-- Direct Memory Access write byte enable (high active)
		dma_priority	=>	dma_priority,					-- Direct Memory Access priority (0:low / 1:high)
		dma_ready		=>	dma_ready,						-- Direct Memory Access is complete
		dma_resp			=>	dma_resp,						-- Direct Memory Access response (0:Okay / 1:Error)
		-- Peripheral memory
		per_addr			=>	per_addr,						-- Peripheral address
		per_dout			=>	per_dout,						-- Peripheral data output
		per_din			=>	per_din,							-- Peripheral data input
		per_en			=>	per_en,							-- Peripheral enable (high active)
		per_we			=>	per_we,							-- Peripheral write enable (high active)
		-- Data memory
		dmem_addr		=>	dmem_addr,						-- Data Memory address
		dmem_dout		=>	dmem_dout,						-- Data Memory data output
		dmem_din			=>	dmem_din,						-- Data Memory data input
		dmem_cen			=>	dmem_cen,						-- Data Memory chip enable (low active)
		dmem_wen			=>	dmem_wen,						-- Data Memory write enable (low active)
		-- Program memory
		pmem_addr		=>	pmem_addr,						-- Program Memory address
		pmem_dout		=>	pmem_dout,						-- Program Memory data output
		pmem_din			=>	pmem_din,						-- Program Memory data input (optional)
		pmem_cen			=>	pmem_cen,						-- Program Memory chip enable (low active)
		pmem_wen			=>	pmem_wen							-- Program Memory write enable (low active) (optional)
	);

	-- Execution unit memory bus
	eu_mem_addr	<=	wires.eu_mem_addr;
	eu_mem_en	<=	wires.eu_mem_en;
	eu_mem_wr	<=	wires.eu_mem_wr;
	-- Frontend memory bus
	fe_mem_din	<=	wires.fe_mem_din;
	dbg_halt_st	<=	wires.cpu_halt_st;
	cpuoff		<=	wires.cpuoff;
	oscoff		<=	wires.oscoff;
	pc				<=	wires.fe_pc;
	scg1			<=	wires.scg1;


end RTL;	-- fmsp430

