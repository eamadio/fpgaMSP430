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
--! @file fmsp_per_package.vhd
--! 
--! @brief fpgaMSP430 Peripherals package
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
	use ieee.math_real.all;
	use work.fmsp_functions.all;

package fmsp_per_package is


	component  fmsp_clock_module is
	generic (
		INST_NR				: integer := 0;			--! Current fmsp instance number     (for multicore systems)
		TOTAL_NR				: integer := 0;			--! Total number of fmsp instances-1 (for multicore systems)
		PMEM_SIZE			: integer := 32768;		--! Program Memory Size
		DMEM_SIZE			: integer := 16384;		--! Data Memory Size
		PER_SIZE				: integer := 16384;		--! Peripheral Memory Size
		DEBUG_EN				: boolean := false;		--! Include/Exclude Serial Debug interface
		WATCHDOG				: boolean := false;		--! Include/Exclude Watchdog timer
		DMA_IF_EN			: boolean := false;		--! Include/Exclude DMA interface support
		NMI_EN				: boolean := false		--! Include/Exclude Non-Maskable-Interrupt support
	);
	port (
		mclk					: in	std_logic;										--! Main system clock
		--! INPUTs
		cpu_en				: in	std_logic;										--! Enable CPU code execution (asynchronous)
		cpuoff				: in	std_logic;										--! Turns off the CPU
		dbg_cpu_reset		: in	std_logic;										--! Reset CPU from debug interface
		dbg_en				: in	std_logic;										--! Debug interface enable (asynchronous)
		lfxt_clk				: in	std_logic;										--! Low frequency oscillator (typ 32kHz)
		oscoff				: in	std_logic;										--! Turns off LFXT1 clock input
		per_addr				: in	std_logic_vector(13 downto 0);			--! Peripheral address
		per_din				: in	std_logic_vector(15 downto 0);			--! Peripheral data input
		per_en				: in	std_logic;										--! Peripheral enable (high active)
		per_we				: in	std_logic_vector(1 downto 0);				--! Peripheral write enable (high active)
		reset_n				: in	std_logic;										--! Reset Pin (low active, asynchronous)
		scg1					: in	std_logic;										--! System clock generator 1. Turns off the SMCLK
		wdt_reset			: in	std_logic;										--! Watchdog-timer reset
		--! OUTPUTs
		dbg_rst				: out	std_logic;										--! Debug unit reset
		per_dout				: out	std_logic_vector(15 downto 0);			--! Peripheral data output
		por					: out	std_logic;										--! Power-on reset
		puc_pnd_set			: out	std_logic;										--! PUC pending set for the serial debug interface
		puc_rst				: out	std_logic;										--! Main system reset
		aclk_en				: out	std_logic;										--! ACLK enable
		smclk_en				: out	std_logic										--! SMCLK enable
	);
	end component fmsp_clock_module;

	component fmsp_sfr is 
	generic (
		INST_NR				: integer := 0;			--! Current fmsp instance number     (for multicore systems)
		TOTAL_NR				: integer := 0;			--! Total number of fmsp instances-1 (for multicore systems)
		PMEM_SIZE			: integer := 32768;		--! Program Memory Size
		DMEM_SIZE			: integer := 16384;		--! Data Memory Size
		PER_SIZE				: integer := 16384;		--! Peripheral Memory Size
		MULTIPLIER			: boolean := false;		--! Include/Exclude Hardware Multiplier
		USER_VERSION		: integer := 0;			--! Custom user version number
		WATCHDOG				: boolean := false;		--! Include/Exclude Watchdog timer
		NMI_EN				: boolean := false		--! Include/Exclude Non-Maskable-Interrupt support
	);
	port (
		mclk				: in	std_logic;       						--! Main system clock
		mrst				: in	std_logic;       						--! Main system reset
		--! INPUTs
		nmi				: in	std_logic;       						--! Non-maskable interrupt (asynchronous)
		nmi_acc			: in	std_logic;       						--! Non-Maskable interrupt request accepted
		per_addr			: in	std_logic_vector(13 downto 0);		--! Peripheral address
		per_din			: in	std_logic_vector(15 downto 0);	--! Peripheral data input
		per_en			: in	std_logic;       						--! Peripheral enable (high active)
		per_we			: in	std_logic_vector(1 downto 0);		--! Peripheral write enable (high active)
		wdtifg			: in	std_logic;       						--! Watchdog-timer interrupt flag
		wdtnmies			: in	std_logic;       						--! Watchdog-timer NMI edge selection
		--! OUTPUTs
		cpu_id			: out	std_logic_vector(31 downto 0);	--! CPU ID
		nmi_pnd			: out	std_logic;								--! NMI Pending
		nmi_wkup			: out	std_logic;								--! NMI Wakeup
		per_dout			: out	std_logic_vector(15 downto 0);	--! Peripheral data output
		wdtie				: out	std_logic;								--! Watchdog-timer interrupt enable
		wdtifg_sw_clr	: out	std_logic;								--! Watchdog-timer interrupt flag software clear
		wdtifg_sw_set	: out	std_logic								--! Watchdog-timer interrupt flag software set
	);
	end component fmsp_sfr;

	component fmsp_watchdog is
	port (
		mclk				: in	std_logic;								--! Main system clock
		mrst				: in	std_logic;								--! Main system reset
		por				: in	std_logic;								--! Power-on reset
		--! INPUTs
		dbg_freeze		: in	std_logic;								--! Freeze Watchdog counter
		aclk_en			: in	std_logic;								--! ACLK enable
		smclk_en			: in	std_logic;								--! SMCLK enable
		per_addr			: in	std_logic_vector(13 downto 0);	--! Peripheral address
		per_din			: in	std_logic_vector(15 downto 0);	--! Peripheral data input
		per_en			: in	std_logic;								--! Peripheral enable (high active)
		per_we			: in	std_logic_vector(1 downto 0);		--! Peripheral write enable (high active)
		wdtie				: in	std_logic;								--! Watchdog timer interrupt enable
		wdtifg_irq_clr	: in	std_logic;								--! Watchdog-timer interrupt flag irq accepted clear
		wdtifg_sw_clr	: in	std_logic;								--! Watchdog-timer interrupt flag software clear
		wdtifg_sw_set	: in	std_logic;								--! Watchdog-timer interrupt flag software set
		--! OUTPUTs
		per_dout			: out	std_logic_vector(15 downto 0);	--! Peripheral data output
		wdt_irq			: out	std_logic;								--! Watchdog-timer interrupt
		wdt_reset		: out	std_logic;								--! Watchdog-timer reset
		wdt_wkup			: out	std_logic;								--! Watchdog Wakeup
		wdtifg			: out	std_logic;								--! Watchdog-timer interrupt flag
		wdtnmies			: out	std_logic								--! Watchdog-timer NMI edge selection
	);	
	end component fmsp_watchdog;

	component	fmsp_multiplier is
	port (
		mclk			: in	std_logic;								--! Main system clock
		mrst			: in	std_logic;								--! Main system reset
		--! INPUTs
		per_addr		: in	std_logic_vector(13 downto 0);	--! Peripheral address
		per_din		: in	std_logic_vector(15 downto 0);	--! Peripheral data input
		per_en		: in	std_logic;								--! Peripheral enable (high active)
		per_we		: in	std_logic_vector(1 downto 0);		--! Peripheral write enable (high active)
		--! OUTPUTs
		per_dout		: out	std_logic_vector(15 downto 0)		--! Peripheral data output
	);
	end component fmsp_multiplier;


component fmsp_timerA is 
--	generic (
--		INST_NR				: integer := 0;			--! Current fmsp instance number     (for multicore systems)
--		TOTAL_NR				: integer := 0;			--! Total number of fmsp instances-1 (for multicore systems)
--		PMEM_SIZE			: integer := 32768;		--! Program Memory Size
--		DMEM_SIZE			: integer := 16384;		--! Data Memory Size
--		PER_SIZE				: integer := 16384;		--! Peripheral Memory Size
--		MULTIPLIER			: boolean := false;		--! Include/Exclude Hardware Multiplier
--		USER_VERSION		: integer := 0;			--! Custom user version number
--		WATCHDOG				: boolean := false;		--! Include/Exclude Watchdog timer
--		NMI_EN				: boolean := false;		--! Include/Exclude Non-Maskable-Interrupt support
--		SYNC_NMI_EN			: boolean := true			--! 
--	);
	port (
		mclk			: in	std_logic;       						--! Main system clock
		mrst			: in	std_logic;       						--! Main system reset
		--! INPUTs
		aclk_en		: in	std_logic;       						--! ACLK enable (from CPU)
		smclk_en		: in	std_logic;       						--! SMCLK enable (from CPU)
		dbg_freeze	: in	std_logic;       						--! Freeze Timer A counter
		inclk			: in	std_logic;       						--! INCLK external timer clock (SLOW)
		irq_ta0_acc	: in	std_logic;       						--! Interrupt request TACCR0 accepted
		per_addr		: in	std_logic_vector(13 downto 0);   --! Peripheral address
		per_din		: in	std_logic_vector(15 downto 0);   --! Peripheral data input
		per_en		: in	std_logic;       						--! Peripheral enable (high active)
		per_we		: in	std_logic_vector(1 downto 0);    --! Peripheral write enable (high active)
		ta_cci0a		: in	std_logic;       						--! Timer A capture 0 input A
		ta_cci0b		: in	std_logic;       						--! Timer A capture 0 input B
		ta_cci1a		: in	std_logic;       						--! Timer A capture 1 input A
		ta_cci1b		: in	std_logic;       						--! Timer A capture 1 input B
		ta_cci2a		: in	std_logic;       						--! Timer A capture 2 input A
		ta_cci2b		: in	std_logic;       						--! Timer A capture 2 input B
		taclk			: in	std_logic;       						--! TACLK external timer clock (SLOW)
		--! OUTPUTs
		irq_ta0		: out	std_logic;								--! Timer A interrupt: TACCR0
		irq_ta1		: out	std_logic;								--! Timer A interrupt: TAIV, TACCR1, TACCR2
		per_dout		: out	std_logic_vector(15 downto 0);	--! Peripheral data output
		ta_out0		: out	std_logic;								--! Timer A output 0
		ta_out0_en	: out	std_logic;								--! Timer A output 0 enable
		ta_out1		: out	std_logic;								--! Timer A output 1
		ta_out1_en	: out	std_logic;								--! Timer A output 1 enable
		ta_out2		: out	std_logic;								--! Timer A output 2
		ta_out2_en	: out	std_logic								--! Timer A output 2 enable
	);
end component fmsp_timerA;


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

----! Instruction state machine
--	constant	C_I_IRQ_FETCH	: integer := 0;
--	constant	C_I_IRQ_DONE	: integer := 1;
--	constant	C_I_DEC			: integer := 2;
--	constant	C_I_EXT1			: integer := 3;
--	constant	C_I_EXT2			: integer := 4;
--	constant	C_I_IDLE			: integer := 5;
----! Instruction state machine
--	constant	I_IRQ_FETCH		: std_logic_vector(2 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_I_IRQ_FETCH,3));
--	constant	I_IRQ_DONE		: std_logic_vector(2 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_I_IRQ_DONE,3));
--	constant	I_DEC				: std_logic_vector(2 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_I_DEC,3));
--	constant	I_EXT1			: std_logic_vector(2 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_I_EXT1,3));
--	constant	I_EXT2			: std_logic_vector(2 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_I_EXT2,3));
--	constant	I_IDLE			: std_logic_vector(2 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_I_IDLE,3));

----! Execution state machine
--	constant	C_E_IRQ_0		: integer := 0;
--	constant	C_E_IRQ_1		: integer := 1;
--	constant	C_E_IRQ_2		: integer := 2;
--	constant	C_E_IRQ_3		: integer := 3;
--	constant	C_E_IRQ_4		: integer := 4;
--	constant	C_E_SRC_AD		: integer := 5;
--	constant	C_E_SRC_RD		: integer := 6;
--	constant	C_E_SRC_WR		: integer := 7;
--	constant	C_E_DST_AD		: integer := 8;
--	constant	C_E_DST_RD		: integer := 9;
--	constant	C_E_DST_WR		: integer := 10;
--	constant	C_E_EXEC			: integer := 11;
--	constant	C_E_JUMP			: integer := 12;
--	constant	C_E_IDLE			: integer := 13;
--
----! Execution state machine
--	constant	E_IRQ_0		: std_logic_vector(3 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_E_IRQ_0,4));
--	constant	E_IRQ_1		: std_logic_vector(3 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_E_IRQ_1,4));
--	constant	E_IRQ_2		: std_logic_vector(3 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_E_IRQ_2,4));
--	constant	E_IRQ_3		: std_logic_vector(3 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_E_IRQ_3,4));
--	constant	E_IRQ_4		: std_logic_vector(3 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_E_IRQ_4,4));
--	constant	E_SRC_AD		: std_logic_vector(3 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_E_SRC_AD,4));
--	constant	E_SRC_RD		: std_logic_vector(3 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_E_SRC_RD,4));
--	constant	E_SRC_WR		: std_logic_vector(3 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_E_SRC_WR,4));
--	constant	E_DST_AD		: std_logic_vector(3 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_E_DST_AD,4));
--	constant	E_DST_RD		: std_logic_vector(3 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_E_DST_RD,4));
--	constant	E_DST_WR		: std_logic_vector(3 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_E_DST_WR,4));
--	constant	E_EXEC			: std_logic_vector(3 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_E_EXEC,4));
--	constant	E_JUMP			: std_logic_vector(3 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_E_JUMP,4));
--	constant	E_IDLE			: std_logic_vector(3 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(C_E_IDLE,4));


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


end fmsp_per_package; --! fmsp_per_package

