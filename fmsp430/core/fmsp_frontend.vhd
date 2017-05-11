------------------------------------------------------------------------------
--! Copyright (C) 2009 , Olivier Girard
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
--! @file fmsp_frontend.vhd
--! 
--! @brief fpgaMSP430 Instruction fetch and decode unit
--
--! @author Olivier Girard,    olgirard@gmail.com
--! @author Emmanuel Amadio,   emmanuel.amadio@gmail.com (VHDL Rewrite)
--
------------------------------------------------------------------------------
--! @version 1
--! @date: 2017-04-21
------------------------------------------------------------------------------
library ieee;
	use ieee.std_logic_1164.all;
	use ieee.numeric_std.all;
	use work.fmsp_core_package.all;
	use work.fmsp_functions.all;

entity	fmsp_frontend is
generic (
	CPUOFF_EN			: boolean := false;						--! Wakeup condition from DMA interface
	DMA_IF_EN			: boolean := false;						--! Wakeup condition from DMA interface
	IRQ_nr				: integer := 16							--! Number of IRQs
);
port (
	mclk					: in	std_logic;								--! Main system clock
	mrst					: in	std_logic;								--! Main system reset
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
	decode_noirq		: out	std_logic;								--! Frontend decode instruction
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
--	mclk_dma_enable	: out	std_logic;								--! DMA Sub-System Clock enable
--	mclk_dma_wkup		: out	std_logic;								--! DMA Sub-System Clock wake-up (asynchronous)
--	mclk_enable			: out	std_logic;								--! Main System Clock enable
--	mclk_wkup			: out	std_logic;								--! Main System Clock wake-up (asynchronous)
	nmi_acc				: out	std_logic;								--! Non-Maskable interrupt request accepted
	pc						: out	std_logic_vector(15 downto 0);	--! Program counter
	pc_nxt				: out	std_logic_vector(15 downto 0)		--! Next PC value (for CALL & IRQ)
);
end entity fmsp_frontend;

architecture RTL of fmsp_frontend is 


	
	
	--	constant	SUMEXT_D		: std_logic_vector(3 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(SUMEXT,3));
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

	type fmsp_frontend_in_type is record
		cpu_en_s				: std_logic;								--! Enable CPU code execution (synchronous)
		cpu_halt_cmd		: std_logic;								--! Halt CPU command
		cpuoff				: std_logic;								--! Turns off the CPU
		dbg_reg_sel			: std_logic_vector(3 downto 0);		--! Debug selected register for rd/wr access
		dma_en				: std_logic;								--! Direct Memory Access enable (high active)
		fe_pmem_wait		: std_logic;								--! Frontend wait for Instruction fetch
		gie					: std_logic;								--! General interrupt enable
		irq					: std_logic_vector((IRQ_nr-3) downto 0);	--! Maskable interrupts
		mdb_in				: std_logic_vector(15 downto 0);	--! Frontend Memory data bus input
		nmi_pnd				: std_logic;								--! Non-maskable interrupt pending
		nmi_wkup				: std_logic;								--! NMI Wakeup
		pc_sw					: std_logic_vector(15 downto 0);	--! Program counter software value
		pc_sw_wr				: std_logic;								--! Program counter software write
		wdt_irq				: std_logic;								--! Watchdog-timer interrupt
		wdt_wkup				: std_logic;								--! Watchdog Wakeup
	end record;

	type reg_type is record
		i_state			: std_logic_vector(2 downto 0);
		i_state_nxt		: std_logic_vector(2 downto 0);
		inst_sz			: std_logic_vector(1 downto 0);
		sconst_nxt		: std_logic_vector(15 downto 0);
		e_state_nxt		: std_logic_vector(3 downto 0);
		cpu_halt_st		: std_logic;	--! Debug interface cpu status	
		inst_nmi			: std_logic;	--! Detect nmi interrupt
		inst_irq_rst	: std_logic;	--! Detect reset interrupt
		irq_num			: std_logic_vector(5 downto 0);	--! Select interrupt vector
		pc					: std_logic_vector(15 downto 0); --! Program counter
		pmem_busy		: std_logic; --! Check if ROM has been busy in order to retry ROM access
		inst_sext		: std_logic_vector(15 downto 0); --! Store source extension word
		inst_dext		: std_logic_vector(15 downto 0); --! Store destination extension word
		inst_type		: std_logic_vector(2 downto 0);
		inst_so			: std_logic_vector(7 downto 0);
		inst_jmp_bin	: std_logic_vector(2 downto 0);
		inst_mov			: std_logic;
		inst_dest_bin	: std_logic_vector(3 downto 0);	--! Destination register
		inst_src_bin	: std_logic_vector(3 downto 0);	--! Source register
		inst_as_nxt		: std_logic_vector(12 downto 0);
		inst_as			: std_logic_vector(7 downto 0);
		inst_ad_nxt		: std_logic_vector(7 downto 0);
		inst_ad			: std_logic_vector(7 downto 0);
		inst_bw			: std_logic;	--! Operation size
		e_state			: std_logic_vector(3 downto 0);	--! State machine registers
		exec_jmp			: std_logic;
		exec_dst_wr		: std_logic;
		exec_src_wr		: std_logic;
		exec_dext_rdy	: std_logic;
		inst_alu			: std_logic_vector(11 downto 0);
	end record;

	signal	d		: fmsp_frontend_in_type;
	signal	r		: reg_type :=	(	i_state			=> "000",
												i_state_nxt		=> "000",
												inst_sz			=> "00",
												sconst_nxt		=> x"0000",
												e_state_nxt		=> "0000",
												cpu_halt_st		=> '0',	--! Debug interface cpu status	
												inst_nmi			=> '0',	--! Detect nmi interrupt
												inst_irq_rst	=> '1',	--! Detect reset interrupt
												irq_num			=> "111111",	--! Select interrupt vector
												pc					=> x"0000", --! Program counter
												pmem_busy		=> '0', --! Check if ROM has been busy in order to retry ROM access
												inst_sext		=> x"0000", --! Store source extension word
												inst_dext		=> x"0000", --! Store destination extension word
												inst_type		=> "000",
												inst_so			=> x"00",
												inst_jmp_bin	=> "000",
												inst_mov			=> '0',
												inst_dest_bin	=> "0000",	--! Destination register
												inst_src_bin	=> "0000",	--! Source register
												inst_as_nxt		=> "0000000000000",
												inst_as			=> x"00",
												inst_ad_nxt		=> x"00",
												inst_ad			=> x"00",
												inst_bw			=> '0',	--! Operation size
												e_state			=> "0000",	--! State machine registers
												exec_jmp			=> '0',
												exec_dst_wr		=> '0',
												exec_src_wr		=> '0',
												exec_dext_rdy	=> '0',
												inst_alu			=> x"000"
											);
	signal	rin	: reg_type;

begin

		d.cpu_en_s		<=	cpu_en_s;
		d.cpu_halt_cmd	<=	cpu_halt_cmd;
		d.cpuoff			<=	cpuoff;
		d.dbg_reg_sel	<=	dbg_reg_sel;
		d.dma_en			<=	dma_en;
		d.fe_pmem_wait	<=	fe_pmem_wait;
		d.gie				<=	gie;
		d.irq				<=	irq;
		d.mdb_in			<=	mdb_in;
		d.nmi_pnd		<=	nmi_pnd;
		d.nmi_wkup		<=	nmi_wkup;
		d.pc_sw			<=	pc_sw;
		d.pc_sw_wr		<=	pc_sw_wr;
		d.wdt_irq		<=	wdt_irq;
		d.wdt_wkup		<=	wdt_wkup;

	COMB : process (d, r)
		variable	v							: reg_type;
		variable	v_ir    					: std_logic_vector(15 downto 0);
		variable	v_inst_sz_nxt			: std_logic_vector(1 downto 0);
		variable	v_irq_detect			: std_logic;
		variable	v_inst_type_nxt		: std_logic_vector(2 downto 0);
		variable	v_is_const				: std_logic;
		variable	v_cpu_halt_req			: std_logic;
		--! Utility signals
		variable	v_decode_noirq			: std_logic;
		variable	v_decode					: std_logic;
		variable	v_fetch					: std_logic;
		variable	v_irq_addr				: std_logic_vector(15 downto 0);
		--! Interrupt request accepted
		variable	v_irq_all				: std_logic_vector(62 downto 0);
		variable	v_irq_acc_all			: std_logic_vector(63 downto 0);
		variable	v_irq_acc				: std_logic_vector((IRQ_nr-3) downto 0);--! is v_irq_acc_all(13 downto 0);
		variable	v_nmi_acc    			: std_logic;--! is v_irq_acc_all(14);
		--! Compute next PC value
		variable	v_pc_incr				: std_logic_vector(15 downto 0);
		variable	v_pc_nxt					: std_logic_vector(15 downto 0);
		--! Memory interface
		variable	v_mab						: std_logic_vector(15 downto 0);
		variable	v_mb_en					: std_logic;
		--! Instruction register
		--variable	v_ir						: std_logic_vector(15 downto 0);
		--! Detect if source extension word is required
		variable	v_is_sext				: std_logic;
		--! Detect if destination extension word is required
--		variable	v_is_dext				: std_logic;
		--! For the Symbolic addressing mode, add -2 to the extension word in order to make up for the PC address
		variable	v_ext_incr				: std_logic_vector(15 downto 0);
		--! Source extension word is ready
		variable	v_inst_sext_rdy		: std_logic;
		--! Destination extension word is ready
		variable	v_inst_dext_rdy		: std_logic;
		variable	v_inst_so_nxt			: std_logic_vector(7 downto 0);
		variable	v_inst_jmp				: std_logic_vector(7 downto 0);
		variable	v_inst_to_1hot			: std_logic_vector(15 downto 0);
		variable	v_inst_to_nxt			: std_logic_vector(11 downto 0);
		variable	v_inst_dest				: std_logic_vector(15 downto 0);
		variable	v_inst_src				: std_logic_vector(15 downto 0);
		variable	v_src_reg				: std_logic_vector(3 downto 0);
		variable	v_dest_reg				: std_logic_vector(3 downto 0);
		variable	v_src_acalc_pre		: std_logic;
		variable	v_src_rd_pre			: std_logic;
		variable	v_dst_acalc_pre		: std_logic;
		variable	v_dst_acalc				: std_logic;
		variable	v_dst_rd_pre			: std_logic;
		variable	v_dst_rd					: std_logic;
		variable	v_inst_branch			: std_logic;
		--! Execution first state
		variable	v_e_first_state		: std_logic_vector(3 downto 0);
		variable	v_exec_done				: std_logic;
		variable	v_alu_src_inv			: std_logic; 
		variable	v_alu_inc				: std_logic;
		variable	v_alu_inc_c				: std_logic;
		variable	v_alu_add				: std_logic;
		variable	v_alu_and				: std_logic;
		variable	v_alu_or					: std_logic;
		variable	v_alu_xor				: std_logic;
		variable	v_alu_dadd				: std_logic;
		variable	v_alu_stat_7			: std_logic;
		variable	v_alu_stat_f			: std_logic;
		variable	v_alu_shift				: std_logic;
		variable	v_exec_no_wr			: std_logic;
--		--! 3)  FRONTEND STATE MACHINE
--		--! The 		variable	v_"conv" is used as state bits to calculate the next response
--		variable	v_inst_sz_nxt		: std_logic_vector(1 downto 0);
--		variable	v_irq_detect		: std_logic;
--		variable	v_inst_type_nxt	: std_logic_vector(2 downto 0);
--		variable	v_is_const			: std_logic;
--		--! CPU on/off through the debug interface or cpu_en port
--		variable	v_cpu_halt_req		: std_logic;
--		--! Utility signals
--		variable	v_decode_noirq		: std_logic;
--		variable	v_decode				: std_logic;
--		variable	v_fetch				: std_logic;
--		--! 4)  INTERRUPT HANDLING
--		variable	v_irq_addr			: std_logic_vector(15 downto 0);
--		--! Interrupt request accepted
--		variable	v_irq_acc_all		: std_logic_vector(15 downto 0);
--		alias		v_irq_acc			: std_logic_vector(13 downto 0) is irq_acc_all(13 downto 0);
--		alias		v_nmi_acc			: std_logic is irq_acc_all(14);
--		--! Compute next PC value
--		variable	v_pc_incr			: std_logic_vector(15 downto 0);
--		variable	v_pc_nxt				: std_logic_vector(15 downto 0);
--		--! Memory interface
--		alias		v_mab    			: std_logic_vector(15 downto 0) is v_pc_nxtr(15 downto 0);
--		variable	v_mb_en				: std_logic;
--		--! 5.2) INSTRUCTION REGISTER
--		--! Instruction register
--		alias		v_ir    					: std_logic_vector(15 downto 0) is d.mdb_in(15 downto 0);
--		--! Detect if source extension word is required
--		variable	v_is_sext				: std_logic;
--		--! Detect if destination extension word is required
--		variable	v_is_dext				: std_logic;
--		--! For the Symbolic addressing mode, add -2 to the extension word in order to make up for the PC address
--		variable	v_ext_incr				: std_logic_vector(15 downto 0);
		variable	v_ext_nxt				: std_logic_vector(15 downto 0);
--		--! Source extension word is ready
--		variable	v_inst_sext_rdy		: std_logic;
--		--! Destination extension word is ready
--		variable	v_inst_dext_rdy		: std_logic;
--		variable	v_inst_so_nxt			: std_logic_vector(7 downto 0);
--		variable	v_inst_jmp				: std_logic_vector(7 downto 0);
--		variable	v_inst_to_1hot			: std_logic_vector(15 downto 0);
--		alias		v_inst_to_nxt			: std_logic_vector(11 downto 0) is v_inst_to_1hot(15 downto 0);
--		variable	v_inst_dest				: std_logic_vector(15 downto 0);
--		variable	v_inst_src				: std_logic_vector(15 downto 0);
--		variable	v_src_reg				: std_logic_vector(3 downto 0);
--		variable	v_dest_reg				: std_logic_vector(3 downto 0);
--		variable	v_src_acalc_pre		: std_logic;
--		variable	v_src_rd_pre			: std_logic;
--		variable	v_dst_acalc_pre		: std_logic;
--		variable	v_dst_acalc				: std_logic;
--		variable	v_dst_rd_pre			: std_logic;
--		variable	v_dst_rd					: std_logic;
--		variable	v_inst_branch			: std_logic;

	begin
		--! default assignment
		v := r;
		--! overriding assignments

		--! The wire "conv" is used as state bits to calculate the next response
			
		--! CPU on/off through the debug interface or cpu_en port
		v_cpu_halt_req := d.cpu_halt_cmd or not(d.cpu_en_s);
		
		--! States Transitions
		case(r.i_state) is
			when I_IDLE =>
				if ( (v_irq_detect = '1') and (v_cpu_halt_req = '0') ) then
					v.i_state_nxt :=  I_IRQ_FETCH;
				elsif ( (d.cpuoff = '0') and (v_cpu_halt_req = '0') ) then
					v.i_state_nxt :=  I_DEC;
				else
					v.i_state_nxt :=  I_IDLE;
				end if;
			when I_IRQ_FETCH =>
				v.i_state_nxt :=  I_IRQ_DONE;
			when I_IRQ_DONE =>
				v.i_state_nxt :=  I_DEC;
			when I_DEC =>
				if (v_irq_detect = '1') then
					v.i_state_nxt :=  I_IRQ_FETCH;
				elsif ( ((d.cpuoff or v_cpu_halt_req) and exec_done) = '1' ) then
					v.i_state_nxt :=  I_IDLE;
				elsif ( (v_cpu_halt_req = '1') and (e_state=E_IDLE) ) then
					v.i_state_nxt :=  I_IDLE;
				elsif (d.pc_sw_wr = '1') then
					v.i_state_nxt :=  I_DEC;
				elsif ( (d.pc_sw_wr = '0') and (e_state/=E_IDLE) ) then
					v.i_state_nxt :=  I_DEC;
				elsif (r.inst_sz /= "00") then
					v.i_state_nxt :=  I_EXT1;
				else
					v.i_state_nxt :=  I_DEC;
				end if;	
			when I_EXT1 =>
				if (d.pc_sw_wr = '1') then
					v.i_state_nxt :=  I_DEC;
				elsif (r.inst_sz /= "01") then
					v.i_state_nxt :=  I_EXT2;
				else
					v.i_state_nxt :=  I_DEC;
				end if;
			when I_EXT2 =>
				v.i_state_nxt :=  I_DEC;
			when others =>
				v.i_state_nxt :=  I_IRQ_FETCH;
		end case;

		--! State machine
		v.i_state  := r.i_state_nxt;

		--! Utility signals
		if ( (r.i_state=I_DEC) and ((v_exec_done = '1') or (r.e_state=E_IDLE)) ) then
			v_decode_noirq := '1';
		else
			v_decode_noirq := '0';
		end if;

		v_decode	:=  v_decode_noirq or v_irq_detect;
		--! Debug interface cpu status
		if ( not( (r.i_state=I_DEC) and not((v_exec_done = '1') or (r.e_state=E_IDLE)) ) and (r.e_state_nxt/=E_IDLE) ) then
			v_fetch := '1';
		else
			v_fetch := '0';
		end if;

		--! Debug interface cpu status
		if ( (r.i_state_nxt=I_IDLE) and (v_cpu_halt_req = '1') ) then
			v.cpu_halt_st := '1';
		else
			v.cpu_halt_st := '0';
		end if;


--=============================================================================
--! 4)  INTERRUPT HANDLING & SYSTEM WAKEUP
--=============================================================================

	--
	--! 4.1) INTERRUPT HANDLING
	-------------------------------------------
--		--! Detect nmi interrupt
--		if (d.nmi_evt = '1') then
--			v.inst_nmi :='1';
--		elsif (r.i_state=I_IRQ_DONE) then
--			v.inst_nmi :='0';
--		end if;

		--! Detect reset interrupt
		if (v_exec_done = '1') then
			v.inst_irq_rst :='0';
		end if;

		--!  Detect other interrupts
		if ( 		 (		(d.nmi_pnd = '1') 
						or ( ( (d.irq /= "00000000000000") or (d.wdt_irq = '1') ) and (d.gie = '1') ) )
				and (v_cpu_halt_req = '0')
				and (r.cpu_halt_st = '0')
				and ( (v_exec_done = '1') or (r.i_state=I_IDLE) ) ) then
			v_irq_detect := '1';
		else
			v_irq_detect := '0';
		end if;

		--! Combine all IRQs
		v_irq_all		:= STD_LOGIC_VECTOR(TO_UNSIGNED(0,63));
		v_irq_all(62)	:= d.nmi_pnd;
		v_irq_all(61 downto (64-IRQ_nr))	:= d.irq;
		v_irq_all(59)	:= d.wdt_irq or v_irq_all(59);

		--! Select highest priority IRQ
		if (v_irq_detect ='1') then
			v.irq_num := get_irq_num(v_irq_all);
		end if;
		
		--! Generate selected IRQ vector address
		v_irq_addr := "111111111" & r.irq_num & '0';

		--! Interrupt request accepted
		if (r.i_state=I_IRQ_FETCH) then
			v_irq_acc_all := one_hot64(r.irq_num);
		else
			v_irq_acc_all := x"0000000000000000";
		end if;
		v_irq_acc	:= v_irq_acc_all(61 downto (64-IRQ_nr));
		v_nmi_acc   := v_irq_acc_all(62);

		--=============================================================================
		--! 5)  FETCH INSTRUCTION
		--=============================================================================

		--
		--! 5.1) PROGRAM COUNTER & MEMORY INTERFACE
		-------------------------------------------

		--! Compute next PC value
--------v_pc_incr := r.pc + ("00000000000000" & v_fetch & '0');
		if (v_fetch = '1') then
			v_pc_incr := STD_LOGIC_VECTOR(UNSIGNED(r.pc) + TO_UNSIGNED(2,16));
		else
			v_pc_incr := r.pc;
		end if;

		if (d.pc_sw_wr = '1') then
			v_pc_nxt  := d.pc_sw;
		elsif (r.i_state=I_IRQ_FETCH) then
			v_pc_nxt  := v_irq_addr;
		elsif (r.i_state=I_IRQ_DONE) then
			v_pc_nxt  := d.mdb_in;
		else
			v_pc_nxt  := v_pc_incr;
		end if;

		v.pc := v_pc_nxt;

		--! Check if ROM has been busy in order to retry ROM access
		v.pmem_busy := d.fe_pmem_wait;
   
		--! Memory interface
		v_mab := v_pc_nxt;
		if (		(v_fetch = '1')
				or	(r.i_state=I_IRQ_FETCH)
				or	(d.pc_sw_wr = '1')
				or	(r.pmem_busy = '1')
				or	((r.cpu_halt_st and not(v_cpu_halt_req)) = '1')	) then
			v_mb_en := '1';
		else
			v_mb_en := '0';
		end if;


--
--! 5.2) INSTRUCTION REGISTER
----------------------------------

		--! Instruction register
		v_ir := d.mdb_in;

		--! Detect if source extension word is required
		v_is_sext := r.inst_as(C_IDX) or r.inst_as(C_SYMB) or r.inst_as(C_ABS) or r.inst_as(C_IMM);

		--! Detect if destination extension word is required
--		v_is_dext := r.inst_ad(C_IDX) or r.inst_ad(C_SYMB) or r.inst_ad(C_ABS);

		--! For the Symbolic addressing mode, add -2 to the extension word in order
		--! to make up for the PC address
		if (		( (r.i_state=I_EXT1) and (r.inst_as(C_SYMB) = '1') )
				or	( (r.i_state=I_EXT2) and (r.inst_ad(C_SYMB) = '1') )
				or	(			(r.i_state=I_EXT1) and (r.inst_as(C_SYMB) = '0') 
						and	(r.i_state/=I_EXT2) and (r.inst_ad(C_SYMB) = '1') )	) then
			v_ext_incr := x"FFFE";
		else
			v_ext_incr := x"0000";
		end if;

		v_ext_nxt  := STD_LOGIC_VECTOR(UNSIGNED(v_ir) + UNSIGNED(v_ext_incr));

		--! Store source extension word
		if ( (v_decode = '1') and (v_is_const = '1') ) then
			v.inst_sext := r.sconst_nxt;
		elsif ( (v_decode = '1') and (v_inst_type_nxt(C_INST_JMP) = '1') ) then
			v.inst_sext := v_ir(9) & v_ir(9) & v_ir(9) & v_ir(9) & v_ir(9) & v_ir(9 downto 0) & '0';
		elsif ( (r.i_state=I_EXT1) and (v_is_sext = '1') ) then
			v.inst_sext := v_ext_nxt;
		end if;

		--! Source extension word is ready
		if ( (r.i_state=I_EXT1) and (v_is_sext = '1') ) then
			v_inst_sext_rdy := '1';
		else
			v_inst_sext_rdy := '0';
		end if;


		--! Store destination extension word
		if ( (r.i_state=I_EXT1) and (v_is_sext = '0') ) then
			v.inst_dext := v_ext_nxt;
		elsif (r.i_state=I_EXT2) then
			v.inst_dext := v_ext_nxt;
		end if;

		--! Destination extension word is ready
		if ( ((r.i_state=I_EXT1) and (v_is_sext = '0')) or (r.i_state=I_EXT2) ) then
			v_inst_dext_rdy := '1';
		else
			v_inst_dext_rdy := '0';
		end if;


		--=============================================================================
		--! 6)  DECODE INSTRUCTION
		--=============================================================================

		--
		--! 6.1) OPCODE: INSTRUCTION TYPE
		------------------------------------------
		--! Instructions type is encoded in a one hot fashion as following:
		--
		--! 3'b001: Single-operand arithmetic
		--! 3'b010: Conditional jump
		--! 3'b100: Two-operand arithmetic

		v_inst_type_nxt	:=	  ( (    v_ir(15)  or      v_ir(14)                   ) and not(v_irq_detect) )
									& ( (not(v_ir(15)) and not(v_ir(14)) and     v_ir(13) ) and not(v_irq_detect) )
									& ( (not(v_ir(15)) and not(v_ir(14)) and not(v_ir(13))) and not(v_irq_detect) );
   
		if (v_decode = '1') then
			v.inst_type := v_inst_type_nxt;
		end if;

		--
		--! 6.2) OPCODE: SINGLE-OPERAND ARITHMETIC
		------------------------------------------
		--! Instructions are encoded in a one hot fashion as following:
		--
		--! 8'b00000001: RRC
		--! 8'b00000010: SWPB
		--! 8'b00000100: RRA
		--! 8'b00001000: SXT
		--! 8'b00010000: PUSH
		--! 8'b00100000: CALL
		--! 8'b01000000: RETI
		--! 8'b10000000: IRQ

		if (v_irq_detect = '1') then
			v_inst_so_nxt := x"80";
		elsif (v_inst_type_nxt(C_INST_SO) = '1') then
			v_inst_so_nxt := one_hot8(v_ir(9 downto 7));
		else
			v_inst_so_nxt := x"00";
		end if;

		if (v_decode = '1') then
			v.inst_so := v_inst_so_nxt;
		end if;

		--
		--! 6.3) OPCODE: CONDITIONAL JUMP
		----------------------------------
		--! Instructions are encoded in a one hot fashion as following:
		--
		--! 8'b00000001: JNE/JNZ
		--! 8'b00000010: JEQ/JZ
		--! 8'b00000100: JNC/JLO
		--! 8'b00001000: JC/JHS
		--! 8'b00010000: JN
		--! 8'b00100000: JGE
		--! 8'b01000000: JL
		--! 8'b10000000: JMP
		if (v_decode = '1') then
			v.inst_jmp_bin := v_ir(12 downto 10);
		end if;

		if (r.inst_type(C_INST_JMP) = '1') then
			v_inst_jmp := one_hot8(r.inst_jmp_bin);
		else
			v_inst_jmp := x"00";
		end if;


		--
		--! 6.4) OPCODE: TWO-OPERAND ARITHMETIC
		---------------------------------------
		--! Instructions are encoded in a one hot fashion as following:
		--
		--! 12'b000000000001: MOV
		--! 12'b000000000010: ADD
		--! 12'b000000000100: ADDC
		--! 12'b000000001000: SUBC
		--! 12'b000000010000: SUB
		--! 12'b000000100000: CMP
		--! 12'b000001000000: DADD
		--! 12'b000010000000: BIT
		--! 12'b000100000000: BIC
		--! 12'b001000000000: BIS
		--! 12'b010000000000: XOR
		--! 12'b100000000000: AND

		if (v_inst_type_nxt(C_INST_TO) = '1') then
			v_inst_to_1hot := one_hot16(v_ir(15 downto 12));
		else
			v_inst_to_1hot := x"0000";
		end if;

		v_inst_to_nxt := v_inst_to_1hot(15 downto 4);

		if (v_decode = '1') then
			v.inst_mov := v_inst_to_nxt(C_MOV);
		end if;


		--
		--! 6.5) SOURCE AND DESTINATION REGISTERS
		-----------------------------------------

		--! Destination register
		v.inst_dest_bin := v_ir(3 downto 0);

		if (cpu_halt_st = '1') then
			v_inst_dest := one_hot16(d.dbg_reg_sel);
		elsif (r.inst_type(C_INST_JMP) = '1') then
			v_inst_dest := x"0001";
		elsif ( ( r.inst_so(C_IRQ) or r.inst_so(C_PUSH) or r.inst_so(C_CALL) ) = '1' ) then
			v_inst_dest := x"0002";
		else
			v_inst_dest := one_hot16(r.inst_dest_bin);
		end if;

		--! Source register
		v.inst_src_bin := v_ir(11 downto 8);

		if (r.inst_type(C_INST_TO) = '1') then
			v_inst_src := one_hot16(r.inst_src_bin);
		elsif (r.inst_so(C_RETI) = '1') then
			v_inst_src := x"0002";
		elsif (r.inst_so(C_IRQ) = '1') then
			v_inst_src := x"0001";
		elsif (r.inst_so(C_IRQ) = '1') then
			v_inst_src := one_hot16(r.inst_dest_bin);
		else
			v_inst_src := x"0000";
		end if;


		--
		--! 6.6) SOURCE ADDRESSING MODES
		----------------------------------
		--! Source addressing modes are encoded in a one hot fashion as following:
		--
		--! 13'b0000000000001: Register direct.
		--! 13'b0000000000010: Register indexed.
		--! 13'b0000000000100: Register indirect.
		--! 13'b0000000001000: Register indirect autoincrement.
		--! 13'b0000000010000: Symbolic (operand is in memory at address PC+x).
		--! 13'b0000000100000: Immediate (operand is next word in the instruction stream).
		--! 13'b0000001000000: Absolute (operand is in memory at address x).
		--! 13'b0000010000000: Constant 4.
		--! 13'b0000100000000: Constant 8.
		--! 13'b0001000000000: Constant 0.
		--! 13'b0010000000000: Constant 1.
		--! 13'b0100000000000: Constant 2.
		--! 13'b1000000000000: Constant -1.

		if (v_inst_type_nxt(C_INST_TO) = '1') then
			v_src_reg := v_ir(3 downto 0);
		else
			v_src_reg := v_ir(11 downto 8);
		end if;

		if (v_inst_type_nxt(C_INST_JMP) = '1') then
			v.inst_as_nxt :=  "0000000000001";
		elsif (v_src_reg = x"3") then--! Addressing mode using R3
			case (v_ir(5 downto 4)) is
				when "11" =>
					v.inst_as_nxt :=  "1000000000000";
				when "10" =>
					v.inst_as_nxt :=  "0100000000000";
				when "01" =>
					v.inst_as_nxt :=  "0010000000000";
				when others =>
					v.inst_as_nxt :=  "0001000000000";
			end case;
		elsif (v_src_reg = x"2") then--! Addressing mode using R2
			case (v_ir(5 downto 4)) is
				when "11" =>
					v.inst_as_nxt :=  "0000100000000";
				when "10" =>
					v.inst_as_nxt :=  "0000010000000";
				when "01" =>
					v.inst_as_nxt :=  "0000001000000";
				when others =>
					v.inst_as_nxt :=  "0000000000001";
			end case;
		elsif (v_src_reg = x"0") then--! Addressing mode using R0
			case (v_ir(5 downto 4)) is
				when "11" =>
					v.inst_as_nxt :=  "0000000100000";
				when "10" =>
					v.inst_as_nxt :=  "0000000000100";
				when "01" =>
					v.inst_as_nxt :=  "0000000010000";
				when others =>
					v.inst_as_nxt :=  "0000000000001";
			end case;
		else                    --! General Addressing mode
			case (v_ir(5 downto 4)) is
				when "11" =>
					v.inst_as_nxt :=  "0000000001000";
				when "10" =>
					v.inst_as_nxt :=  "0000000000100";
				when "01" =>
					v.inst_as_nxt :=  "0000000000010";
				when others =>
					v.inst_as_nxt :=  "0000000000001";
			end case;
		end if;
	
		v_is_const := r.inst_as_nxt(12) or r.inst_as_nxt(11) or r.inst_as_nxt(10) or r.inst_as_nxt(9) or r.inst_as_nxt(8) or r.inst_as_nxt(7);

		v.inst_as := v_is_const & r.inst_as_nxt(6 downto 0);


		--! 13'b0000010000000: Constant 4.
		--! 13'b0000100000000: Constant 8.
		--! 13'b0001000000000: Constant 0.
		--! 13'b0010000000000: Constant 1.
		--! 13'b0100000000000: Constant 2.
		--! 13'b1000000000000: Constant -1.
		if (r.inst_as_nxt(7) = '1') then
			v.sconst_nxt := x"0004";
		elsif (r.inst_as_nxt(8) = '1') then
			v.sconst_nxt := x"0008";
		elsif (r.inst_as_nxt(9) = '1') then
			v.sconst_nxt := x"0000";
		elsif (r.inst_as_nxt(10) = '1') then
			v.sconst_nxt := x"0001";
		elsif (r.inst_as_nxt(11) = '1') then
			v.sconst_nxt := x"0002";
		elsif (r.inst_as_nxt(12) = '1') then
			v.sconst_nxt := x"FFFF";
		else
			v.sconst_nxt := x"0000";
		end if;

		--
		--! 6.7) DESTINATION ADDRESSING MODES
		-------------------------------------
		--! Destination addressing modes are encoded in a one hot fashion as following:
		--
		--! 8'b00000001: Register direct.
		--! 8'b00000010: Register indexed.
		--! 8'b00010000: Symbolic (operand is in memory at address PC+x).
		--! 8'b01000000: Absolute (operand is in memory at address x).

		v_dest_reg := v_ir(3 downto 0);

		if (v_inst_type_nxt(C_INST_TO) = '0') then
			v.inst_ad_nxt := "00000000";
		elsif (v_dest_reg = x"2") then	--! Addressing mode using R2
			if (v_ir(7) = '1') then
				v.inst_ad_nxt := "01000000";
			else
				v.inst_ad_nxt := "00000001";
			end if;
		elsif (v_dest_reg=x"0") then		--! Addressing mode using R0
			if (v_ir(7) = '1') then
				v.inst_ad_nxt := "00010000";
			else
				v.inst_ad_nxt := "00000001";
			end if;
		else                      		--! General Addressing mode
			if (v_ir(7) = '1') then
				v.inst_ad_nxt := "00000010";
			else
				v.inst_ad_nxt := "00000001";
			end if;
		end if;

		v.inst_ad := r.inst_ad_nxt;


		--
		--! 6.8) REMAINING INSTRUCTION DECODING
		---------------------------------------

		--! Operation size
		if (v_decode = '1') then
			v.inst_bw := v_ir(6) and not(v_inst_type_nxt(C_INST_JMP)) and not(v_irq_detect) and not(v_cpu_halt_req);
		end if;

		--! Extended instruction size
--		v_inst_sz_nxt	:=		STD_LOGIC_VECTOR(		UNSIGNED( '0' &  (r.inst_as_nxt(C_IDX) or r.inst_as_nxt(C_SYMB) or r.inst_as_nxt(C_ABS) or r.inst_as_nxt(C_IMM)) )
--															+	UNSIGNED( '0' & ((r.inst_ad_nxt(C_IDX) or r.inst_ad_nxt(C_SYMB) or r.inst_ad_nxt(C_ABS)) and not(v_inst_type_nxt(C_INST_SO))) )  );
		if (			( (r.inst_as_nxt(C_IDX) or r.inst_as_nxt(C_SYMB) or r.inst_as_nxt(C_ABS) or r.inst_as_nxt(C_IMM)) = '0' )
				and	( ((r.inst_ad_nxt(C_IDX) or r.inst_ad_nxt(C_SYMB) or r.inst_ad_nxt(C_ABS)) and not(v_inst_type_nxt(C_INST_SO))) = '0' )  ) then
			v_inst_sz_nxt := "00";
		elsif (			( (r.inst_as_nxt(C_IDX) or r.inst_as_nxt(C_SYMB) or r.inst_as_nxt(C_ABS) or r.inst_as_nxt(C_IMM)) = '1' )
				and	( ((r.inst_ad_nxt(C_IDX) or r.inst_ad_nxt(C_SYMB) or r.inst_ad_nxt(C_ABS)) and not(v_inst_type_nxt(C_INST_SO))) = '1' )  ) then
			v_inst_sz_nxt := "10";
		else
			v_inst_sz_nxt := "01";
		end if;
		

		if (v_decode = '1') then
			v.inst_sz := v_inst_sz_nxt;
		end if;


		--=============================================================================
		--! 7)  EXECUTION-UNIT STATE MACHINE
		--=============================================================================




		--! State machine control signals
		----------------------------------

		v_src_acalc_pre :=  r.inst_as_nxt(C_IDX)   or r.inst_as_nxt(C_SYMB)    or r.inst_as_nxt(C_ABS);
		v_src_rd_pre    :=  r.inst_as_nxt(C_INDIR) or r.inst_as_nxt(C_INDIR_I) or r.inst_as_nxt(C_IMM)  or v_inst_so_nxt(C_RETI);
		v_dst_acalc_pre :=  r.inst_ad_nxt(C_IDX)   or r.inst_ad_nxt(C_SYMB)    or r.inst_ad_nxt(C_ABS);
		v_dst_acalc     :=  inst_ad(C_IDX)       or inst_ad(C_SYMB)        or inst_ad(C_ABS);
		v_dst_rd_pre    :=  r.inst_ad_nxt(C_IDX)   or v_inst_so_nxt(C_PUSH)    or v_inst_so_nxt(C_CALL) or v_inst_so_nxt(C_RETI);
		v_dst_rd        :=  inst_ad(C_IDX)       or inst_so(C_PUSH)        or inst_so(C_CALL)     or inst_so(C_RETI);

		if ( ((r.inst_ad_nxt(C_DIR) = '1') and (v_ir(3 downto 0) = "0000")) or (v_inst_type_nxt(C_INST_JMP) = '1') or (v_inst_so_nxt(C_RETI) = '1') ) then
			v_inst_branch := '1';
		else
			v_inst_branch := '0';
		end if;

		if ( (v_inst_branch = '1') and (v_decode = '1') ) then
			v.exec_jmp := '1';
		elsif (e_state = E_JUMP) then
			v.exec_jmp := '0';
		end if;

		if (r.e_state = E_DST_RD) then
			v.exec_dst_wr := '1';
		elsif (e_state = E_DST_WR) then
			v.exec_dst_wr := '0';
		end if;

		if ( (inst_type(C_INST_SO) = '1') and (r.e_state = E_SRC_RD) ) then
			v.exec_src_wr := '1';
		elsif ( (r.e_state = E_SRC_WR) or (r.e_state = E_DST_WR) ) then
			v.exec_src_wr := '0';
		end if;
	
		if (r.e_state=E_DST_RD) then
			v.exec_dext_rdy := '0';
		elsif (v_inst_dext_rdy = '1') then
			v.exec_dext_rdy := '1';
		end if;

		--! Execution first state
		if ( (r.cpu_halt_st = '0') and (v_inst_so_nxt(C_IRQ) = '1') ) then
			v_e_first_state := E_IRQ_0;
		elsif ( (v_cpu_halt_req = '1') or (r.i_state = I_IDLE) ) then
			v_e_first_state := E_IDLE;
		elsif (d.cpuoff = '1') then
			v_e_first_state := E_IDLE;
		elsif (v_src_acalc_pre = '1') then
			v_e_first_state := E_SRC_AD;
		elsif (v_src_rd_pre = '1') then
			v_e_first_state := E_SRC_RD;
		elsif (v_dst_acalc_pre = '1') then
			v_e_first_state := E_DST_AD;
		elsif (v_dst_rd_pre = '1') then
			v_e_first_state := E_DST_RD;
		else
			v_e_first_state := E_EXEC;
		end if;


		--! State machine
		----------------------------------

		--! States Transitions
		case(r.e_state) is
			when E_IDLE =>
				v.e_state_nxt	:=  v_e_first_state;
			when E_IRQ_0 =>
				v.e_state_nxt	:=  E_IRQ_1;
			when E_IRQ_1 =>
				v.e_state_nxt	:=  E_IRQ_2;
			when E_IRQ_2 =>
				v.e_state_nxt	:=  E_IRQ_3;
			when E_IRQ_3 =>
				v.e_state_nxt	:=  E_IRQ_4;
			when E_IRQ_4 =>
				v.e_state_nxt	:=  E_EXEC;
			when E_SRC_AD =>
				if (v_inst_sext_rdy = '1') then
					v.e_state_nxt := E_SRC_RD;
				else
					v.e_state_nxt := E_SRC_AD;
				end if;
			when E_SRC_RD =>
				if (v_dst_acalc = '1') then
					v.e_state_nxt := E_DST_AD;
				elsif (v_dst_rd = '1') then
					v.e_state_nxt := E_DST_RD;
				else
					v.e_state_nxt := E_EXEC;
				end if;
			when E_DST_AD =>
				if ( (v_inst_dext_rdy = '1') or (r.exec_dext_rdy = '1') )then
					v.e_state_nxt := E_DST_RD;
				else
					v.e_state_nxt := E_DST_AD;
				end if;
			when E_DST_RD =>
				v.e_state_nxt	:=	E_EXEC;
			when E_EXEC =>
				if (r.exec_dst_wr = '1') then
					v.e_state_nxt := E_DST_WR;
				elsif (r.exec_jmp = '1') then
					v.e_state_nxt := E_JUMP;
				elsif (r.exec_src_wr = '1') then
					v.e_state_nxt := E_SRC_WR;
				else
					v.e_state_nxt := v_e_first_state;
				end if;
			when E_JUMP =>
				v.e_state_nxt	:=	v_e_first_state;
			when E_DST_WR =>
				if (r.exec_jmp = '1') then
					v.e_state_nxt := E_JUMP;
				else
					v.e_state_nxt := v_e_first_state;
				end if;
			when E_SRC_WR =>
				v.e_state_nxt	:=	v_e_first_state;
			when others =>
				v.e_state_nxt	:=	E_IRQ_0;
		end case;

		--! State machine
		v.e_state  := r.e_state_nxt;


		--! Frontend State machine control signals
		------------------------------------------

		if (r.exec_jmp = '1') then
			if (r.e_state = E_JUMP) then
				v_exec_done := '1';
			else
				v_exec_done := '0';
			end if;
		elsif (r.exec_dst_wr = '1') then
			if (r.e_state = E_DST_WR) then
				v_exec_done := '1';
			else
				v_exec_done := '0';
			end if;
		elsif (r.exec_src_wr = '1') then
			if (r.e_state = E_SRC_WR) then
				v_exec_done := '1';
			else
				v_exec_done := '0';
			end if;
		else
			if (r.e_state = E_EXEC) then
				v_exec_done := '1';
			else
				v_exec_done := '0';
			end if;
		end if;
	


		--=============================================================================
		--! 8)  EXECUTION-UNIT STATE CONTROL
		--=============================================================================

		--
		--! 8.1) ALU CONTROL SIGNALS
		---------------------------------------
		--
		--! 12'b000000000001: Enable ALU source inverter
		--! 12'b000000000010: Enable Incrementer
		--! 12'b000000000100: Enable Incrementer on carry bit
		--! 12'b000000001000: Select Adder
		--! 12'b000000010000: Select AND
		--! 12'b000000100000: Select OR
		--! 12'b000001000000: Select XOR
		--! 12'b000010000000: Select DADD
		--! 12'b000100000000: Update N, Z & C (C=~Z)
		--! 12'b001000000000: Update all status bits
		--! 12'b010000000000: Update status bit for XOR instruction
		--! 12'b100000000000: Don't write to destination

		v_alu_src_inv	:=		v_inst_to_nxt(C_SUB)  or v_inst_to_nxt(C_SUBC)
								or	v_inst_to_nxt(C_CMP)  or v_inst_to_nxt(C_BIC) ;

		v_alu_inc		:=		v_inst_to_nxt(C_SUB)  or v_inst_to_nxt(C_CMP);

		v_alu_inc_c		:=		v_inst_to_nxt(C_ADDC) or v_inst_to_nxt(C_DADD)
								or	v_inst_to_nxt(C_SUBC);

		v_alu_add		:=		v_inst_to_nxt(C_ADD)  or v_inst_to_nxt(C_ADDC)
								or	v_inst_to_nxt(C_SUB)  or v_inst_to_nxt(C_SUBC)
								or	v_inst_to_nxt(C_CMP)  or v_inst_type_nxt(C_INST_JMP)
								or	v_inst_so_nxt(C_RETI);

 
		v_alu_and		:=		v_inst_to_nxt(C_AND)  or v_inst_to_nxt(C_BIC)
								or	v_inst_to_nxt(C_BIT);

		v_alu_or			:= v_inst_to_nxt(C_BIS);

		v_alu_xor		:= v_inst_to_nxt(C_XOR);

		v_alu_dadd		:= v_inst_to_nxt(C_DADD);

		v_alu_stat_7	:=		v_inst_to_nxt(C_BIT)  or v_inst_to_nxt(C_AND)
								or	v_inst_so_nxt(C_SXT);

		v_alu_stat_f	:=		v_inst_to_nxt(C_ADD)  or v_inst_to_nxt(C_ADDC)
								or	v_inst_to_nxt(C_SUB)  or v_inst_to_nxt(C_SUBC)
								or	v_inst_to_nxt(C_CMP)  or v_inst_to_nxt(C_DADD)
								or	v_inst_to_nxt(C_BIT)  or v_inst_to_nxt(C_XOR)
								or	v_inst_to_nxt(C_AND)
								or	v_inst_so_nxt(C_RRC)  or v_inst_so_nxt(C_RRA)
								or	v_inst_so_nxt(C_SXT);

		v_alu_shift     := v_inst_so_nxt(C_RRC)  or v_inst_so_nxt(C_RRA);

		v_exec_no_wr    := v_inst_to_nxt(C_CMP) or v_inst_to_nxt(C_BIT);

		if (v_decode = '1') then
			v.inst_alu	:=		v_exec_no_wr
								&	v_alu_shift
								&	v_alu_stat_f
								&	v_alu_stat_7
								&	v_alu_dadd
								&	v_alu_xor
								&	v_alu_or
								&	v_alu_and
								&	v_alu_add
								&	v_alu_inc_c
								&	v_alu_inc
								&	v_alu_src_inv;
		end if;

		--! drive register inputs
		rin <= v;
		--! drive module outputs
		cpu_halt_st		<= r.cpu_halt_st;   --! Halt/Run status from CPU
		decode_noirq	<= v_decode_noirq;  --! Frontend v_decode instruction
		e_state			<= r.e_state;       --! Execution state
		exec_done		<= v_exec_done;     --! Execution completed
		inst_ad			<= r.inst_ad;       --! Decoded Inst: destination addressing mode
		inst_as			<= r.inst_as;       --! Decoded Inst: source addressing mode
		inst_alu			<= r.inst_alu;      --! ALU control signals
		inst_bw			<= r.inst_bw;       --! Decoded Inst: byte width
		inst_dest		<= v_inst_dest;     --! Decoded Inst: destination (one hot)
		inst_dext		<= r.inst_dext;     --! Decoded Inst: destination extended instruction word
		inst_irq_rst	<= r.inst_irq_rst;  --! Decoded Inst: Reset interrupt
		inst_jmp			<= v_inst_jmp;      --! Decoded Inst: Conditional jump
		inst_mov			<= r.inst_mov;      --! Decoded Inst: mov instruction
		inst_sext		<= r.inst_sext;     --! Decoded Inst: source extended instruction word
		inst_so			<= r.inst_so;       --! Decoded Inst: Single-operand arithmetic
		inst_src			<= v_inst_src;      --! Decoded Inst: source (one hot)
		inst_type		<= r.inst_type;     --! Decoded Instruction type
		irq_acc			<= v_irq_acc;       --! Interrupt request accepted (one-hot signal)
		mab				<= v_mab;           --! Frontend Memory address bus
		mb_en				<= v_mb_en;         --! Frontend Memory bus enable
		nmi_acc			<= v_nmi_acc;       --! Non-Maskable interrupt request accepted
		pc					<= r.pc;            --! Program counter
		pc_nxt			<= v_pc_nxt;        --! Next PC value (for CALL & IRQ)

	end process COMB;

	REGS : process (mclk, mrst)
	begin
		if (mrst = '1') then
			r	<=	(	i_state			=> "000",
						i_state_nxt		=> "000",
						inst_sz			=> "00",
						sconst_nxt		=> x"0000",
						e_state_nxt		=> "0000",
						cpu_halt_st		=> '0',	--! Debug interface cpu status	
						inst_nmi			=> '0',	--! Detect nmi interrupt
						inst_irq_rst	=> '1',	--! Detect reset interrupt
						irq_num			=> "111111",	--! Select interrupt vector
						pc					=> x"0000", --! Program counter
						pmem_busy		=> '0', --! Check if ROM has been busy in order to retry ROM access
						inst_sext		=> x"0000", --! Store source extension word
						inst_dext		=> x"0000", --! Store destination extension word
						inst_type		=> "000",
						inst_so			=> x"00",
						inst_jmp_bin	=> "000",
						inst_mov			=> '0',
						inst_dest_bin	=> "0000",	--! Destination register
						inst_src_bin	=> "0000",	--! Source register
						inst_as_nxt		=> "0000000000000",
						inst_as			=> x"00",
						inst_ad_nxt		=> x"00",
						inst_ad			=> x"00",
						inst_bw			=> '0',	--! Operation size
						e_state			=> "0001",	--! State machine registers
						exec_jmp			=> '0',
						exec_dst_wr		=> '0',
						exec_src_wr		=> '0',
						exec_dext_rdy	=> '0',
						inst_alu			=> x"000"
					);
		elsif rising_edge(mclk) then
			r	<= rin;
		end if;
	end process REGS;

end RTL;