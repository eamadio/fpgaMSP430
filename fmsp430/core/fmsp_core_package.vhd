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
--! @file fmsp_core_package.vhd
--! 
--! @brief fpgaMSP430 core package
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

package fmsp_core_package is


	component fmsp_frontend is
	generic (
		CPUOFF_EN			: in	boolean := false;						--! Wakeup condition from DMA interface
		DMA_IF_EN			: in	boolean := false;						--! Wakeup condition from DMA interface
		IRQ_nr				: in	integer := 16							--! Number of IRQs
	);
	port (
		mclk					: in	std_logic;								--! Main system clock
		mrst				: in	std_logic;								--! Main system reset
		--! INPUTs
		cpu_en_s				: in	std_logic;								--! Enable CPU code execution (synchronous)
		cpu_halt_cmd		: in	std_logic;								--! Halt CPU command
		cpuoff				: in	std_logic;								--! Turns off the CPU
		dbg_reg_sel			: in	std_logic_vector(3 downto 0);		--! Debug selected register for rd/wr access
		dma_en				: in	std_logic;								--! Direct Memory Access enable (high active)
		fe_pmem_wait		: in	std_logic;								--! Frontend wait for Instruction fetch
		gie					: in	std_logic;								--! General interrupt enable
		irq					: in	std_logic_vector((IRQ_nr-3) downto 0);	--! Maskable interrupts
		mdb_in				: in	std_logic_vector(15 downto 0);	--! Frontend Memory data bus input
		nmi_pnd				: in	std_logic;								--! Non-maskable interrupt pending
		nmi_wkup				: in	std_logic;								--! NMI Wakeup
		pc_sw					: in	std_logic_vector(15 downto 0);	--! Program counter software value
		pc_sw_wr				: in	std_logic;								--! Program counter software write
		wdt_irq				: in	std_logic;								--! Watchdog-timer interrupt
		wdt_wkup				: in	std_logic;								--! Watchdog Wakeup
		--! OUTPUTs
		cpu_halt_st			: out	std_logic;								--! Halt/Run status from CPU
		decode_noirq		: out	std_logic;								--! Frontend v_decode instruction
		e_state				: out	std_logic_vector(3 downto 0);		--! Execution state
		exec_done			: out	std_logic;								--! Execution completed
		inst_ad				: out	std_logic_vector(7 downto 0);		--! Decoded Inst: destination addressing mode
		inst_as				: out	std_logic_vector(7 downto 0);		--! Decoded Inst: source addressing mode
		inst_alu				: out	std_logic_vector(11 downto 0);	--! ALU control signals
		inst_bw				: out	std_logic;								--! Decoded Inst: byte width
		inst_dest			: out	std_logic_vector(15 downto 0);	--! Decoded Inst: destination (one hot)
		inst_dext			: out	std_logic_vector(15 downto 0);	--! Decoded Inst: destination extended instruction word
		inst_irq_rst		: out	std_logic;								--! Decoded Inst: Reset interrupt
		inst_jmp				: out	std_logic_vector(7 downto 0);		--! Decoded Inst: Conditional jump
		inst_mov				: out	std_logic;								--! Decoded Inst: mov instruction
		inst_sext			: out	std_logic_vector(15 downto 0);	--! Decoded Inst: source extended instruction word
		inst_so				: out	std_logic_vector(7 downto 0);		--! Decoded Inst: Single-operand arithmetic
		inst_src				: out	std_logic_vector(15 downto 0);	--! Decoded Inst: source (one hot)
		inst_type			: out	std_logic_vector(2 downto 0);		--! Decoded Instruction type
		irq_acc				: out	std_logic_vector(13 downto 0);	--! Interrupt request accepted (one-hot signal)
		mab					: out	std_logic_vector(15 downto 0);	--! Frontend Memory address bus
		mb_en					: out	std_logic;								--! Frontend Memory bus enable
		nmi_acc				: out	std_logic;								--! Non-Maskable interrupt request accepted
		pc						: out	std_logic_vector(15 downto 0);	--! Program counter
		pc_nxt				: out	std_logic_vector(15 downto 0)		--! Next PC value (for CALL & IRQ)
	);
	end component fmsp_frontend;

	component fmsp_register_file is 
	port (
		mclk				: in	std_logic;								--! Main system clock
		mrst			: in	std_logic;								--! Main system reset
		--! INPUTs
		alu_stat			: in	std_logic_vector(3 downto 0);		--! ALU Status {V,N,Z,C}
		alu_stat_wr		: in	std_logic_vector(3 downto 0);		--! ALU Status write {V,N,Z,C}
		inst_bw			: in	std_logic;								--! Decoded Inst: byte width
		inst_dest		: in	std_logic_vector(15 downto 0);	--! Register destination selection
		inst_src			: in	std_logic_vector(15 downto 0);	--! Register source selection
		pc					: in	std_logic_vector(15 downto 0);	--! Program counter
		reg_dest_val	: in	std_logic_vector(15 downto 0);	--! Selected register destination value
		reg_dest_wr		: in	std_logic;								--! Write selected register destination
		reg_pc_call		: in	std_logic;								--! Trigger PC update for a CALL instruction
		reg_sp_val		: in	std_logic_vector(15 downto 0);	--! Stack Pointer next value
		reg_sp_wr		: in	std_logic;								--! Stack Pointer write
		reg_sr_wr		: in	std_logic;								--! Status register update for RETI instruction
		reg_sr_clr		: in	std_logic;								--! Status register clear for interrupts
		reg_incr			: in	std_logic;								--! Increment source register
		--! OUTPUTs
		cpuoff			: out	std_logic;								--! Turns off the CPU
		gie				: out	std_logic;								--! General interrupt enable
		oscoff			: out	std_logic;								--! Turns off LFXT1 clock input
		pc_sw				: out	std_logic_vector(15 downto 0);	--! Program counter software value
		pc_sw_wr			: out	std_logic;								--! Program counter software write
		reg_dest			: out	std_logic_vector(15 downto 0); 	--! Selected register destination content
		reg_src			: out	std_logic_vector(15 downto 0);	--! Selected register source content
		scg0				: out	std_logic;								--! System clock generator 1. Turns off te DCO
		scg1				: out	std_logic;								--! System clock generator 1. Turns off the SMmclk
		status			: out	std_logic_vector(3 downto 0)		--! R2 Status {V,N,Z,C}
	);
	end component fmsp_register_file;

	component fmsp_alu is
	port (
		--! INPUTs
		dbg_halt_st		: in	std_logic;								--! Halt/Run status from CPU
		exec_cycle		: in	std_logic;								--! Instruction execution cycle
		inst_alu			: in	std_logic_vector(11 downto 0);	--! ALU control signals
		inst_bw			: in	std_logic;								--! Decoded Inst: byte width
		inst_jmp			: in	std_logic_vector(7 downto 0);		--! Decoded Inst: Conditional jump
		inst_so			: in	std_logic_vector(7 downto 0);		--! Single-operand arithmetic
		op_dst			: in	std_logic_vector(15 downto 0);	--! Destination operand
		op_src			: in	std_logic_vector(15 downto 0);	--! Source operand
		status			: in	std_logic_vector(3 downto 0);		--! R2 Status {V,N,Z,C}
		--! OUTPUTs
		alu_out			: out	std_logic_vector(15 downto 0);	--! ALU output value
		alu_out_add		: out	std_logic_vector(15 downto 0);	--! ALU adder output value
		alu_stat			: out	std_logic_vector(3 downto 0);		--! ALU Status {V,N,Z,C}
		alu_stat_wr		: out	std_logic_vector(3 downto 0)		--! ALU Status write {V,N,Z,C}
	);
	end component fmsp_alu;

	component fmsp_execution_unit is 
	port (
		mclk				: in	std_logic;								--! Main system clock
		mrst			: in	std_logic;								--! Main system reset
		--! INPUTs
		dbg_halt_st		: in	std_logic; 								--! Halt/Run status from CPU
		dbg_mem_dout	: in	std_logic_vector(15 downto 0);	--! Debug unit data output
		dbg_reg_wr		: in	std_logic;  							--! Debug unit CPU register write
		e_state			: in	std_logic_vector(3 downto 0);		--! Execution state
		exec_done		: in	std_logic;								--! Execution completed
		inst_ad			: in	std_logic_vector(7 downto 0);		--! Decoded Inst: destination addressing mode
		inst_as			: in	std_logic_vector(7 downto 0);		--! Decoded Inst: source addressing mode
		inst_alu			: in	std_logic_vector(11 downto 0);	--! ALU control signals
		inst_bw			: in	std_logic;								--! Decoded Inst: byte width
		inst_dest		: in	std_logic_vector(15 downto 0);	--! Decoded Inst: destination (one hot)
		inst_dext		: in	std_logic_vector(15 downto 0);	--! Decoded Inst: destination extended instruction word
		inst_irq_rst	: in	std_logic;								--! Decoded Inst: reset interrupt
		inst_jmp			: in	std_logic_vector(7 downto 0);		--! Decoded Inst: Conditional jump
		inst_mov			: in	std_logic;								--! Decoded Inst: mov instruction
		inst_sext		: in	std_logic_vector(15 downto 0);	--! Decoded Inst: source extended instruction word
		inst_so			: in	std_logic_vector(7 downto 0);		--! Decoded Inst: Single-operand arithmetic
		inst_src			: in	std_logic_vector(15 downto 0);	--! Decoded Inst: source (one hot)
		inst_type		: in	std_logic_vector(2 downto 0);		--! Decoded Instruction type
		mdb_in			: in	std_logic_vector(15 downto 0);	--! Memory data bus input
		pc					: in	std_logic_vector(15 downto 0);	--! Program counter
		pc_nxt			: in	std_logic_vector(15 downto 0);	--! Next PC value (for CALL & IRQ)
		--! OUTPUTs
		cpuoff			: out	std_logic;								--! Turns off the CPU
		dbg_reg_din		: out	std_logic_vector(15 downto 0);	--! Debug unit CPU register data input
		gie				: out	std_logic;								--! General interrupt enable
		mab				: out	std_logic_vector(15 downto 0);	--! Memory address bus
		mb_en				: out	std_logic;								--! Memory bus enable
		mb_wr				: out	std_logic_vector(1 downto 0);		--! Memory bus write transfer
		mdb_out			: out	std_logic_vector(15 downto 0);	--! Memory data bus output
		oscoff			: out	std_logic;								--! Turns off LFXT1 clock input
		pc_sw				: out	std_logic_vector(15 downto 0);	--! Program counter software value
		pc_sw_wr			: out	std_logic;								--! Program counter software write
		scg1				: out	std_logic								--! System clock generator 1. Turns off the SMCLK
	);
	end component fmsp_execution_unit;

	component fmsp_mem_backbone is
	generic (
		PMEM_SIZE		: integer := 32768;		--! Program Memory Size
		DMEM_SIZE		: integer := 16384;		--! Data Memory Size
		PER_SIZE			: integer := 16384;		--! Peripheral Memory Size
		DMA_IF_EN		: boolean := false					--! Wakeup condition from DMA interface
	);
	port (
		mclk				: in	std_logic;								--! Main system clock
		mrst			: in	std_logic;								--! Main system reset
		--! INPUTs
		cpu_halt_st		: in	std_logic;								--! Halt/Run status from CPU
		dbg_halt_cmd	: in	std_logic;								--! Debug interface Halt CPU command
		dbg_mem_addr	: in	std_logic_vector(15 downto 0);	--! Debug address for rd/wr access
		dbg_mem_dout	: in	std_logic_vector(15 downto 0);	--! Debug unit data output
		dbg_mem_en		: in	std_logic;								--! Debug unit memory enable
		dbg_mem_wr		: in	std_logic_vector(1 downto 0);		--! Debug unit memory write
		dmem_dout		: in	std_logic_vector(15 downto 0);	--! Data Memory data output
		eu_mab			: in	std_logic_vector(14 downto 0);	--! Execution Unit Memory address bus
		eu_mb_en			: in	std_logic;								--! Execution Unit Memory bus enable
		eu_mb_wr			: in	std_logic_vector(1 downto 0);		--! Execution Unit Memory bus write transfer
		eu_mdb_out		: in	std_logic_vector(15 downto 0);	--! Execution Unit Memory data bus output
		fe_mab			: in	std_logic_vector(14 downto 0);	--! Frontend Memory address bus
		fe_mb_en			: in	std_logic;								--! Frontend Memory bus enable
		dma_addr			: in	std_logic_vector(15 downto 0);	--! Direct Memory Access address
		dma_din			: in	std_logic_vector(15 downto 0);	--! Direct Memory Access data input
		dma_en			: in	std_logic;								--! Direct Memory Access enable (high active)
		dma_priority	: in	std_logic;								--! Direct Memory Access priority (0:low / 1:high)
		dma_we			: in	std_logic_vector(1 downto 0);		--! Direct Memory Access write byte enable (high active)
		per_dout			: in	std_logic_vector(15 downto 0);	--! Peripheral data output
		pmem_dout		: in	std_logic_vector(15 downto 0);	--! Program Memory data output
		--! OUTPUTs
		cpu_halt_cmd	: out	std_logic;													--! Halt CPU command
		dbg_mem_din		: out	std_logic_vector(15 downto 0);						--! Debug unit Memory data input
		dmem_addr		: out	std_logic_vector(f_log2(DMEM_SIZE)-2 downto 0);	--! Data Memory address
		dmem_cen			: out	std_logic;													--! Data Memory chip enable (low active)
		dmem_din			: out	std_logic_vector(15 downto 0);						--! Data Memory data input
		dmem_wen			: out	std_logic_vector(1 downto 0);							--! Data Memory write enable (low active)
		eu_mdb_in		: out	std_logic_vector(15 downto 0);						--! Execution Unit Memory data bus input
		fe_mdb_in		: out	std_logic_vector(15 downto 0);						--! Frontend Memory data bus input
		fe_pmem_wait	: out	std_logic;													--! Frontend wait for Instruction fetch
		dma_dout			: out	std_logic_vector(15 downto 0);						--! Direct Memory Access data output
		dma_ready		: out	std_logic;													--! Direct Memory Access is complete
		dma_resp			: out	std_logic;													--! Direct Memory Access response (0:Okay / 1:Error)
		per_addr			: out	std_logic_vector(13 downto 0);						--! Peripheral address
		per_din			: out	std_logic_vector(15 downto 0);						--! Peripheral data input
		per_we			: out	std_logic_vector(1 downto 0);							--! Peripheral write enable (high active)
		per_en			: out	std_logic;													--! Peripheral enable (high active)
		pmem_addr		: out	std_logic_vector(f_log2(PMEM_SIZE)-2 downto 0);	--! Program Memory address
		pmem_cen			: out	std_logic;													--! Program Memory chip enable (low active)
		pmem_din			: out	std_logic_vector(15 downto 0);						--! Program Memory data input (optional)
		pmem_wen			: out	std_logic_vector(1 downto 0)							--! Program Memory write enable (low active) (optional)
	);
	end component fmsp_mem_backbone;

	component  fmsp_core is
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
		--============
		nmi_acc				: out	std_logic;
		nmi_pnd				: in	std_logic;
		nmi_wkup				: in	std_logic;
		wdt_irq				: in	std_logic;
		wdt_wkup				: in	std_logic;
		irq					: in	std_logic_vector(IRQ_NR-3 downto 0);	-- Maskable interrupts (14, 30 or 62)
		irq_acc				: out	std_logic_vector(IRQ_NR-3 downto 0);	-- Interrupt request accepted (one-hot signal)
		cpu_en_s				: in	std_logic;
		decode_noirq		: out	std_logic;
		pc						: out	std_logic_vector(15 downto 0);
		cpuoff				: out	std_logic;
		oscoff				: out	std_logic;
		scg1					: out	std_logic
	);
	end component fmsp_core;
	
	
	
	
--
--! STATES, REGISTER FIELDS, ...
--======================================

--! Instructions type
	constant	C_INST_SO	: integer := 0;
	constant	C_INST_JMP	: integer := 1;
	constant	C_INST_TO	: integer := 2;

--! Single-operand arithmetic
	constant	C_RRC		: integer := 0;
	constant	C_SWPB	: integer := 1;
	constant	C_RRA		: integer := 2;
	constant	C_SXT		: integer := 3;
	constant	C_PUSH	: integer := 4;
	constant	C_CALL	: integer := 5;
	constant	C_RETI	: integer := 6;
	constant	C_IRQ		: integer := 7;

--! Conditional jump
	constant	C_JNE		: integer := 0;
	constant	C_JEQ		: integer := 1;
	constant	C_JNC		: integer := 2;
	constant	C_JC		: integer := 3;
	constant	C_JN		: integer := 4;
	constant	C_JGE		: integer := 5;
	constant	C_JL		: integer := 6;
	constant	C_JMP		: integer := 7;

--! Two-operand arithmetic
	constant	C_MOV		: integer := 0;
	constant	C_ADD		: integer := 1;
	constant	C_ADDC	: integer := 2;
	constant	C_SUBC	: integer := 3;
	constant	C_SUB		: integer := 4;
	constant	C_CMP		: integer := 5;
	constant	C_DADD	: integer := 6;
	constant	C_BIT		: integer := 7;
	constant	C_BIC		: integer := 8;
	constant	C_BIS		: integer := 9;
	constant	C_XOR		: integer := 10;
	constant	C_AND		: integer := 11;

--! Addressing modes
	constant	C_DIR			: integer := 0;
	constant	C_IDX			: integer := 1;
	constant	C_INDIR		: integer := 2;
	constant	C_INDIR_I	: integer := 3;
	constant	C_SYMB		: integer := 4;
	constant	C_IMM			: integer := 5;
	constant	C_ABS			: integer := 6;
	constant	C_CONST		: integer := 7;

--! Instruction state machine
	constant	C_I_IRQ_FETCH	: integer := 0;
	constant	C_I_IRQ_DONE	: integer := 1;
	constant	C_I_DEC			: integer := 2;
	constant	C_I_EXT1			: integer := 3;
	constant	C_I_EXT2			: integer := 4;
	constant	C_I_IDLE			: integer := 5;
--! Instruction state machine
	constant	I_IRQ_FETCH		: std_logic_vector(2 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_I_IRQ_FETCH,3));
	constant	I_IRQ_DONE		: std_logic_vector(2 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_I_IRQ_DONE,3));
	constant	I_DEC				: std_logic_vector(2 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_I_DEC,3));
	constant	I_EXT1			: std_logic_vector(2 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_I_EXT1,3));
	constant	I_EXT2			: std_logic_vector(2 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_I_EXT2,3));
	constant	I_IDLE			: std_logic_vector(2 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_I_IDLE,3));

--! Execution state machine
	constant	C_E_IRQ_0		: integer := 0;
	constant	C_E_IRQ_1		: integer := 1;
	constant	C_E_IRQ_2		: integer := 2;
	constant	C_E_IRQ_3		: integer := 3;
	constant	C_E_IRQ_4		: integer := 4;
	constant	C_E_SRC_AD		: integer := 5;
	constant	C_E_SRC_RD		: integer := 6;
	constant	C_E_SRC_WR		: integer := 7;
	constant	C_E_DST_AD		: integer := 8;
	constant	C_E_DST_RD		: integer := 9;
	constant	C_E_DST_WR		: integer := 10;
	constant	C_E_EXEC			: integer := 11;
	constant	C_E_JUMP			: integer := 12;
	constant	C_E_IDLE			: integer := 13;

--! Execution state machine
	constant	E_IRQ_0		: std_logic_vector(3 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_E_IRQ_0,4));
	constant	E_IRQ_1		: std_logic_vector(3 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_E_IRQ_1,4));
	constant	E_IRQ_2		: std_logic_vector(3 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_E_IRQ_2,4));
	constant	E_IRQ_3		: std_logic_vector(3 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_E_IRQ_3,4));
	constant	E_IRQ_4		: std_logic_vector(3 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_E_IRQ_4,4));
	constant	E_SRC_AD		: std_logic_vector(3 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_E_SRC_AD,4));
	constant	E_SRC_RD		: std_logic_vector(3 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_E_SRC_RD,4));
	constant	E_SRC_WR		: std_logic_vector(3 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_E_SRC_WR,4));
	constant	E_DST_AD		: std_logic_vector(3 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_E_DST_AD,4));
	constant	E_DST_RD		: std_logic_vector(3 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_E_DST_RD,4));
	constant	E_DST_WR		: std_logic_vector(3 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_E_DST_WR,4));
	constant	E_EXEC			: std_logic_vector(3 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_E_EXEC,4));
	constant	E_JUMP			: std_logic_vector(3 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_E_JUMP,4));
	constant	E_IDLE			: std_logic_vector(3 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_E_IDLE,4));


--! ALU control signals
	constant	C_ALU_SRC_INV	: integer := 0;
	constant	C_ALU_INC		: integer := 1;
	constant	C_ALU_INC_C		: integer := 2;
	constant	C_ALU_ADD		: integer := 3;
	constant	C_ALU_AND		: integer := 4;
	constant	C_ALU_OR			: integer := 5;
	constant	C_ALU_XOR		: integer := 6;
	constant	C_ALU_DADD		: integer := 7;
	constant	C_ALU_STAT_7	: integer := 8;
	constant	C_ALU_STAT_F	: integer := 9;
	constant	C_ALU_SHIFT		: integer := 10;
	constant	C_EXEC_NO_WR	: integer := 11;

--! Debug interface
	constant	C_DBG_UART_WR	: integer := 18;
	constant	C_DBG_UART_BW	: integer := 17;
--	constant	C_DBG_UART_ADDR	: integer := 16:11

--! Debug interface CPU_CTL register
	constant	C_HALT			: integer := 0;
	constant	C_RUN				: integer := 1;
	constant	C_ISTEP			: integer := 2;
	constant	C_SW_BRK_EN		: integer := 3;
	constant	C_FRZ_BRK_EN	: integer := 4;
	constant	C_RST_BRK_EN	: integer := 5;
	constant	C_CPU_RST		: integer := 6;

--! Debug interface CPU_STAT register
	constant	C_HALT_RUN		: integer := 0;
	constant	C_PUC_PND		: integer := 1;
	constant	C_SWBRK_PND		: integer := 3;
	constant	C_HWBRK0_PND	: integer := 4;
	constant	C_HWBRK1_PND	: integer := 5;

--! Debug interface BRKx_CTL register
	constant	C_BRK_MODE_RD	: integer := 0;
	constant	C_BRK_MODE_WR	: integer := 1;
--	constant	C_BRK_MODE		: integer := 1:0
	constant	C_BRK_EN			: integer := 2;
	constant	C_BRK_I_EN		: integer := 3;
	constant	C_BRK_RANGE		: integer := 4;

	
--! Basic clock module: BCSCTL1 Control Register
--	constant	C_DIVAx       5:4
	constant	C_DMA_CPUOFF	: integer := 0;
	constant	C_DMA_OSCOFF	: integer := 1;
	constant	C_DMA_SCG0		: integer := 2;
	constant	C_DMA_SCG1		: integer := 3;

--! Basic clock module: BCSCTL2 Control Register
	constant	C_SELMx			: integer := 7;
	constant	C_SELS			: integer := 3;
--	constant	C_DIVSx       2:1


--
--! DEBUG INTERFACE EXTRA CONFIGURATION
--======================================

--! Debug interface: CPU version
--!   1 - FPGA support only (Pre-BSD licence era)
--!   2 - Add ASIC support
--!   3 - Add DMA interface support
	constant	C_CPU_VERSION	: integer	range 0 to 7 := 1;

--! Debug interface: Software breakpoint opcode
	constant	C_DBG_SWBRK_OP : std_logic_vector(15 downto 0) := x"4343";

--! Debug UART interface auto data synchronization
--! If the following define is commented out, then
--! the DBG_UART_BAUD and DBG_DCO_FREQ need to be properly
--! defined.
--	constant	C_DBG_UART_AUTO_SYNC

--! Debug UART interface data rate
--!      In order to properly setup the UART debug interface, you
--!      need to specify the DCO_CLK frequency (DBG_DCO_FREQ) and
--!      the chosen BAUD rate from the UART interface.
--
--	constant	C_DBG_UART_BAUD    9600
--	constant	C_DBG_UART_BAUD   19200
--	constant	C_DBG_UART_BAUD   38400
--	constant	C_DBG_UART_BAUD   57600
--	constant	C_DBG_UART_BAUD  115200
--	constant	C_DBG_UART_BAUD  230400
--	constant	C_DBG_UART_BAUD  460800
--	constant	C_DBG_UART_BAUD  576000
--	constant	C_DBG_UART_BAUD  921600
--	constant	C_DBG_UART_BAUD 2000000
--	constant	C_DBG_DCO_FREQ  20000000
--	constant	C_DBG_UART_CNT ((`DBG_DCO_FREQ/`DBG_UART_BAUD)-1)

--! Debug interface selection
--!             	constant	C_DBG_UART -> Enable UART (8N1) debug interface
--!             	constant	C_DBG_JTAG -> DON'T UNCOMMENT, NOT SUPPORTED
--
--	constant	C_DBG_UART
--	constant	C_DBG_JTAG


end fmsp_core_package; --! fmsp_core_package

