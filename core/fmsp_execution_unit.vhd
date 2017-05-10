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
--! @file fmsp_execution_unit.vhd
--! 
--! @brief fpgaMSP430 Execution unit
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

entity	fmsp_execution_unit is
port (
	mclk				: in	std_logic;	--! Main system clock
	mrst			: in	std_logic;	--! Main system reset
	--! INPUTs
	dbg_halt_st		: in	std_logic;								--! Halt/Run status from CPU
	dbg_mem_dout	: in	std_logic_vector(15 downto 0);	--! Debug unit data output
	dbg_reg_wr		: in	std_logic;								--! Debug unit CPU register write
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
	pc_nxt			: in	std_logic_vector(15 downto 0);	--! Next d.pc value (for CALL & IRQ)
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
	scg0				: out	std_logic;								--! System clock generator 1. Turns off te DCO
	scg1				: out	std_logic								--! System clock generator 1. Turns off the SMCLK
);
end entity fmsp_execution_unit;

architecture RTL of fmsp_execution_unit is 


	type fmsp_execution_unit_in_type is record
		dbg_halt_st		: std_logic;							--! Halt/Run status from CPU
		dbg_mem_dout	: std_logic_vector(15 downto 0);	--! Debug unit data output
		dbg_reg_wr		: std_logic;							--! Debug unit CPU register write
		e_state			: std_logic_vector(3 downto 0);	--! Execution state
		exec_done		: std_logic;							--! Execution completed
		inst_ad			: std_logic_vector(7 downto 0);	--! Decoded Inst: destination addressing mode
		inst_as			: std_logic_vector(7 downto 0);	--! Decoded Inst: source addressing mode
		inst_alu			: std_logic_vector(11 downto 0);	--! ALU control signals
		inst_bw			: std_logic;							--! Decoded Inst: byte width
		inst_dest		: std_logic_vector(15 downto 0);	--! Decoded Inst: destination (one hot)
		inst_dext		: std_logic_vector(15 downto 0);	--! Decoded Inst: destination extended instruction word
		inst_irq_rst	: std_logic;							--! Decoded Inst: reset interrupt
		inst_jmp			: std_logic_vector(7 downto 0);	--! Decoded Inst: Conditional jump
		inst_mov			: std_logic;							--! Decoded Inst: mov instruction
		inst_sext		: std_logic_vector(15 downto 0);	--! Decoded Inst: source extended instruction word
		inst_so			: std_logic_vector(7 downto 0);	--! Decoded Inst: Single-operand arithmetic
		inst_src			: std_logic_vector(15 downto 0);	--! Decoded Inst: source (one hot)
		inst_type		: std_logic_vector(2 downto 0);	--! Decoded Instruction type
		mdb_in			: std_logic_vector(15 downto 0);	--! Memory data bus input
		pc					: std_logic_vector(15 downto 0);	--! Program counter
		pc_nxt			: std_logic_vector(15 downto 0);	--! Next pc value (for CALL & IRQ)
		
		reg_dest			: std_logic_vector(15 downto 0);
		reg_src			: std_logic_vector(15 downto 0);
		alu_stat			: std_logic_vector(3 downto 0);
		alu_stat_wr		: std_logic_vector(3 downto 0);
		alu_out			: std_logic_vector(15 downto 0);
		alu_out_add		: std_logic_vector(15 downto 0);
	end record;

	type reg_type is record
		mdb_out_nxt			: std_logic_vector(15 downto 0);	--! Memory data bus output
		mab_lsb				: std_logic;							--! Format memory data bus input depending on BW
		mdb_in_buf_en		: std_logic;							--! Memory data bus input buffer (buffer after a source read)
		mdb_in_buf_valid	: std_logic;							
		mdb_in_buf			: std_logic_vector(15 downto 0);	
	end record;

	signal	d		: fmsp_execution_unit_in_type;
	signal	r		: reg_type :=	(	mdb_out_nxt			=> x"0000", --! Memory data bus output
												mab_lsb				=> '0',--! Format memory data bus input depending on BW
												mdb_in_buf_en		=> '0',--! Memory data bus input buffer (buffer after a source read)
												mdb_in_buf_valid	=> '0',							
												mdb_in_buf			=> x"0000"	
											);
	signal	rin	: reg_type;

	signal	reg_dest_wr			: std_logic;
	signal	reg_sp_wr			: std_logic;
	signal	reg_sr_wr			: std_logic;
	signal	reg_sr_clr			: std_logic;
	signal	reg_pc_call			: std_logic;
	signal	reg_incr				: std_logic;
	signal	exec_cycle			: std_logic;
	signal	status				: std_logic_vector(3 downto 0);
	signal	op_dst				: std_logic_vector(15 downto 0);
	signal	op_src				: std_logic_vector(15 downto 0);

begin

		d.dbg_halt_st	<=	dbg_halt_st;
		d.dbg_mem_dout	<=	dbg_mem_dout;
		d.dbg_reg_wr	<=	dbg_reg_wr;
		d.e_state		<=	e_state;
		d.exec_done		<=	exec_done;
		d.inst_ad		<=	inst_ad;
		d.inst_as		<=	inst_as;
		d.inst_alu		<=	inst_alu;
		d.inst_bw		<=	inst_bw;
		d.inst_dest		<=	inst_dest;
		d.inst_dext		<=	inst_dext;
		d.inst_irq_rst	<=	inst_irq_rst;
		d.inst_jmp		<=	inst_jmp;
		d.inst_mov		<=	inst_mov;
		d.inst_sext		<=	inst_sext;
		d.inst_so		<=	inst_so;
		d.inst_src		<=	inst_src;
		d.inst_type		<=	inst_type;
		d.mdb_in			<=	mdb_in;
		d.pc				<=	pc;
		d.pc_nxt			<=	pc_nxt;

	COMB : process (all)
		variable	v							: reg_type;
		variable	v_alu_stat				: std_logic_vector(3 downto 0);
		variable	v_alu_stat_wr			: std_logic_vector(3 downto 0);
		variable	v_op_dst					: std_logic_vector(15 downto 0);
		variable	v_op_src					: std_logic_vector(15 downto 0);
		variable	v_status					: std_logic_vector(3 downto 0);
		variable	v_reg_dest_wr			: std_logic;
		variable	v_reg_sp_wr				: std_logic;
		variable	v_reg_sr_wr				: std_logic;
		variable	v_reg_sr_clr			: std_logic;
		variable	v_reg_pc_call			: std_logic;
		variable	v_reg_incr				: std_logic;
		variable	v_dbg_reg_din			: std_logic_vector(15 downto 0);
		variable	v_src_reg_src_sel		: std_logic;                 
		variable	v_src_reg_dest_sel	: std_logic;
		variable	v_src_mdb_in_val_sel	: std_logic;
		variable	v_src_inst_dext_sel	: std_logic;
		variable	v_src_inst_sext_sel	: std_logic;
		variable	v_dst_inst_sext_sel	: std_logic;
		variable	v_dst_mdb_in_bw_sel	: std_logic;
		variable	v_dst_fffe_sel			: std_logic;
		variable	v_dst_reg_dest_sel	: std_logic;
		variable	v_exec_cycle			: std_logic;
		--! Detect memory read/write access
		variable	v_mb_wr_det				: std_logic;
		variable	v_mb_rd_det				: std_logic;
		variable	v_mb_en					: std_logic;
		variable	v_mb_wr_msk				: std_logic_vector(1 downto 0);
		variable	v_mb_wr					: std_logic_vector(1 downto 0);
		--! Memory address bus
		variable	v_mab						: std_logic_vector(15 downto 0);
		variable	v_mdb_out				: std_logic_vector(15 downto 0);
		variable	v_mdb_in_bw				: std_logic_vector(15 downto 0);
		variable	v_mdb_in_val			: std_logic_vector(15 downto 0);

begin
		--! default assignment
		v := r;
		--! overriding assignments

		--=============================================================================
		--! 2)  REGISTER FILE
		--=============================================================================

		if ( ((d.e_state = E_EXEC) and (
                     ((d.inst_type(C_INST_TO) = '1') and (d.inst_ad(C_DIR) = '1') and not (d.inst_alu(C_EXEC_NO_WR) = '1'))  or
                     ((d.inst_type(C_INST_SO) = '1') and (d.inst_as(C_DIR) = '1') and not ((d.inst_so(C_PUSH) = '1') or (d.inst_so(C_CALL) = '1') or (d.inst_so(C_RETI) = '1') )) or
                      (d.inst_type(C_INST_JMP) = '1'))) or (d.dbg_reg_wr = '1')	) then
			v_reg_dest_wr := '1';
		else
			v_reg_dest_wr:= '0';
		end if;

		if ( (	( 		(d.e_state = E_IRQ_1)
						or (d.e_state = E_IRQ_3) )
					and (not(d.inst_irq_rst) = '1') )
				or	(			(d.e_state = E_DST_RD)
						and	( (d.inst_so(C_PUSH) = '1') or (d.inst_so(C_CALL) = '1') )
						and	(not(d.inst_as(C_IDX)) = '1')
						and	not(	(	(d.inst_as(C_INDIR) = '1') or (d.inst_as(C_INDIR_I) = '1') ) and (d.inst_src(1) = '1') ) )
				or	(			(d.e_state = E_SRC_AD)
						and	( (d.inst_so(C_PUSH) = '1') or (d.inst_so(C_CALL) = '1') )
						and	(d.inst_as(C_IDX) = '1') )

				or	(			(d.e_state = E_SRC_RD)
						and	( (d.inst_so(C_PUSH) = '1') or (d.inst_so(C_CALL) = '1') )
						and	(	((d.inst_as(C_INDIR) = '1') or (d.inst_as(C_INDIR_I) = '1')) and (d.inst_src(1) = '1') ) ) ) then
			v_reg_sp_wr := '1';
		else
			v_reg_sp_wr := '0';
		end if;

		if ( (d.e_state = E_DST_RD) and (d.inst_so(C_RETI) = '1')	) then
			v_reg_sr_wr := '1';
		else
			v_reg_sr_wr := '0';
		end if;

		if ( (d.e_state = E_IRQ_2)	) then
			v_reg_sr_clr := '1';
		else
			v_reg_sr_clr := '0';
		end if;

		if ( ((d.e_state = E_EXEC)   and (d.inst_so(C_CALL) = '1')) or 
                    ((d.e_state = E_DST_WR) and (d.inst_so(C_RETI) = '1'))	) then
			v_reg_pc_call := '1';
		else
			v_reg_pc_call := '0';
		end if;

		if ( ((d.exec_done = '1')  and (d.inst_as(C_INDIR_I) = '1')) or
                    ((d.e_state = E_SRC_RD) and (d.inst_so(C_RETI) = '1'))    or
                    ((d.e_state = E_EXEC)   and (d.inst_so(C_RETI) = '1'))	) then
			v_reg_incr := '1';
		else
			v_reg_incr := '0';
		end if;

		v_dbg_reg_din := d.reg_dest;




		--=============================================================================
		--! 3)  SOURCE OPERAND MUXING
		--=============================================================================
		--! d.inst_as(C_DIR) = '1')    : Register direct.   -> Source is in register
		--! d.inst_as(C_IDX) = '1')    : Register indexed.  -> Source is in memory, address is register+offset
		--! d.inst_as(C_INDIR) = '1')  : Register indirect.
		--! d.inst_as(C_INDIR_I) = '1'): Register indirect autoincrement.
		--! d.inst_as(C_SYMB) = '1')   : Symbolic (operand is in memory at address d.pc+x).
		--! d.inst_as(C_IMM) = '1')    : Immediate (operand is next word in the instruction stream).
		--! d.inst_as(C_ABS) = '1')    : Absolute (operand is in memory at address x).
		--! d.inst_as(C_CONST) = '1')  : Constant.

		if (		( (d.e_state = E_IRQ_0) or (d.e_state = E_IRQ_2) )
				or	( (d.e_state = E_SRC_RD) and not(d.inst_as(C_ABS) = '1') )
				or	( (d.e_state = E_SRC_WR) and not(d.inst_as(C_ABS) = '1') )
				or	( (d.e_state = E_EXEC) and (d.inst_as(C_DIR) = '1') and not(d.inst_type(C_INST_JMP) = '1') )	) then
			v_src_reg_src_sel := '1';
		else
			v_src_reg_src_sel := '0';
		end if;

		if (		( (d.e_state = E_IRQ_1) or (d.e_state = E_IRQ_3) )
				or	( (d.e_state = E_DST_RD) and ( (d.inst_so(C_PUSH) = '1') or (d.inst_so(C_CALL) = '1') ) )
				or	( (d.e_state = E_SRC_AD) and ( (d.inst_so(C_PUSH) = '1') or (d.inst_so(C_CALL) = '1') ) and (d.inst_as(C_IDX) = '1') )	) then
			v_src_reg_dest_sel := '1';
		else
			v_src_reg_dest_sel := '0';
		end if;

		if (		( (d.e_state = E_DST_RD) and (d.inst_so(C_RETI) = '1') )
				or	( (d.e_state = E_EXEC) and ( (d.inst_as(C_INDIR) = '1') or (d.inst_as(C_INDIR_I) = '1') 
															or	(d.inst_as(C_IDX) = '1') or (d.inst_as(C_SYMB) = '1') 
															or	(d.inst_as(C_ABS) = '1') )	) ) then
			v_src_mdb_in_val_sel := '1';
		else
			v_src_mdb_in_val_sel := '0';
		end if;

		if (		( (d.e_state = E_DST_RD) and not( (d.inst_so(C_PUSH) = '1') or (d.inst_so(C_CALL) = '1') ) )
				or	( (d.e_state = E_DST_WR) and not( (d.inst_so(C_PUSH) = '1') or (d.inst_so(C_CALL) = '1') 
															or	(d.inst_as(C_IDX) = '1') or (d.inst_as(C_SYMB) = '1') 
															or	(d.inst_so(C_RETI) = '1') ) )	) then
			v_src_inst_dext_sel := '1';
		else
			v_src_inst_dext_sel := '0';
		end if;

		if (	(d.e_state = E_EXEC) and ( (d.inst_type(C_INST_JMP) = '1') or	(d.inst_as(C_IMM) = '1') or (d.inst_as(C_CONST) = '1') 
															or	(d.inst_so(C_RETI) = '1') ) ) then
			v_src_inst_sext_sel := '1';
		else
			v_src_inst_sext_sel := '0';
		end if;

		if (v_src_reg_src_sel = '1') then
			v_op_src := d.reg_src;
		elsif (v_src_reg_dest_sel = '1') then
			v_op_src := d.reg_dest;
		elsif (v_src_mdb_in_val_sel = '1') then
			v_op_src := v_mdb_in_val;
		elsif (v_src_inst_dext_sel = '1') then
			v_op_src := d.inst_dext;
		elsif (v_src_inst_sext_sel = '1') then
			v_op_src := d.inst_sext;
		else
			v_op_src := x"0000";
		end if;


		--=============================================================================
		--! 4)  DESTINATION OPERAND MUXING
		--=============================================================================
		--! d.inst_ad(C_DIR) = '1')    : Register direct.
		--! d.inst_ad(C_IDX) = '1')    : Register indexed.
		--! d.inst_ad(C_SYMB) = '1')   : Symbolic (operand is in memory at address d.pc+x).
		--! d.inst_ad(C_ABS) = '1')    : Absolute (operand is in memory at address x).

		if (		( (d.e_state = E_SRC_RD) and ( (d.inst_as(C_IDX) = '1') or (d.inst_as(C_SYMB) = '1')
																							  or (d.inst_as(C_ABS) = '1') ) )
				or	( (d.e_state = E_SRC_RD) and ( (d.inst_as(C_IDX) = '1') or (d.inst_as(C_SYMB) = '1')
																							  or (d.inst_as(C_ABS) = '1') ) )	) then
			v_dst_inst_sext_sel := '1';
		else
			v_dst_inst_sext_sel := '0';
		end if;

		if (		( (d.e_state = E_DST_WR) and (d.inst_so(C_RETI) = '1') )
				or	( (d.e_state = E_EXEC) and not( (d.inst_ad(C_IDX) = '1') or (d.inst_type(C_INST_JMP) = '1')
																or (d.inst_type(C_INST_SO) = '1') ) and not(d.inst_so(C_RETI) = '1') )	) then
			v_dst_mdb_in_bw_sel := '1';
		else
			v_dst_mdb_in_bw_sel := '0';
		end if;

		if (		( (d.e_state = E_IRQ_0) or (d.e_state = E_IRQ_0) or (d.e_state = E_IRQ_3) )
				or	( (d.e_state = E_DST_RD) and ( (d.inst_so(C_PUSH) = '1') or (d.inst_so(C_CALL) = '1') ) and not(d.inst_so(C_RETI) = '1') )
				or	( (d.e_state = E_SRC_AD) and ( (d.inst_so(C_PUSH) = '1') or (d.inst_so(C_CALL) = '1') ) and (d.inst_as(C_IDX) = '1') )
				or	( (d.e_state = E_SRC_RD) and ( (d.inst_so(C_PUSH) = '1') or (d.inst_so(C_CALL) = '1') ) and ( (d.inst_as(C_INDIR) = '1') or (d.inst_as(C_INDIR_I) = '1') ) and (d.inst_src(1) = '1') )	) then
			v_dst_fffe_sel := '1';
		else
			v_dst_fffe_sel := '0';
		end if;

		if (		( (d.e_state = E_DST_RD) and not( (d.inst_so(C_PUSH) = '1') or (d.inst_so(C_CALL) = '1') or (d.inst_ad(C_ABS) = '1') or (d.inst_so(C_RETI) = '1') )  )
				or	( (d.e_state = E_DST_WR) and not(d.inst_ad(C_ABS) = '1') )
				or	( (d.e_state = E_EXEC) and ( (d.inst_ad(C_DIR) = '1') or (d.inst_type(C_INST_JMP) = '1') or (d.inst_type(C_INST_SO) = '1') ) and not(d.inst_so(C_RETI) = '1') )	) then
			v_dst_reg_dest_sel := '1';
		else
			v_dst_reg_dest_sel := '0';
		end if;


		if (d.dbg_halt_st = '1') then
			v_op_dst := d.dbg_mem_dout;
		elsif (v_dst_inst_sext_sel = '1') then
			v_op_dst := d.inst_sext;
		elsif (v_dst_mdb_in_bw_sel = '1') then
			v_op_dst := v_mdb_in_bw;
		elsif (v_dst_reg_dest_sel = '1') then
			v_op_dst := d.reg_dest;
		elsif (v_dst_fffe_sel = '1') then
			v_op_dst := x"FFFE";
		else
			v_op_dst := x"0000";
		end if;


		--=============================================================================
		--! 5)  ALU
		--=============================================================================
		if (d.e_state = E_EXEC) then
			v_exec_cycle := '1';
		else
			v_exec_cycle := '0';
		end if;

		--=============================================================================
		--! 6)  MEMORY INTERFACE
		--=============================================================================

		--! Detect memory read/write access
		if (		( (d.e_state = E_SRC_RD)	and (d.inst_as(C_IMM) = '0') )
				or	( (d.e_state = E_EXEC)		and (d.inst_so(C_RETI) = '0') )
				or	( (d.e_state = E_SRC_RD)	and (d.inst_type(C_INST_SO) = '0') and (d.inst_mov = '0') )
			) then
			v_mb_rd_det := '1';
		else
			v_mb_rd_det := '0';
		end if;

		--! Detect memory read/write access
		if (		( (d.e_state = E_IRQ_1) and (not(d.inst_irq_rst) = '0') )
				or	( (d.e_state = E_IRQ_3) and (not(d.inst_irq_rst) = '0') )
				or	( (d.e_state = E_DST_WR) and (not(d.inst_so(C_RETI)) = '0') )
				or	  (d.e_state = E_SRC_WR)	) then
			v_mb_wr_det := '1';
		else
			v_mb_wr_det := '0';
		end if;

		if (d.inst_alu(C_EXEC_NO_WR) = '1') then
			v_mb_wr_msk := "00";
		elsif (d.inst_bw = '0') then
			v_mb_wr_msk := "11";
		elsif (d.alu_out_add(0) = '1') then
			v_mb_wr_msk := "10";
		else
			v_mb_wr_msk := "01";
		end if;

		if (		( (not(d.inst_alu(C_EXEC_NO_WR)) = '1')	and (v_mb_wr_det= '1') )
				or	(v_mb_rd_det= '1')
			) then
			v_mb_en := '1';
		else
			v_mb_en := '0';
		end if;

		if (v_mb_wr_det = '1') then
			v_mb_wr := v_mb_wr_msk;
		else
			v_mb_wr := "00";
		end if;

		--! Memory address bus
		v_mab       := d.alu_out_add;

--! Memory data bus output
	if (d.e_state = E_DST_RD) then
		v.mdb_out_nxt := d.pc_nxt;
	elsif ( ( (d.e_state = E_EXEC) and (not(d.inst_so(C_CALL)) = '1') )
				or (d.e_state = E_IRQ_0) or (d.e_state = E_IRQ_2) ) then
		v.mdb_out_nxt := d.alu_out;
	end if;

	if (d.inst_bw = '1') then
		v_mdb_out := r.mdb_out_nxt(7 downto 0) & r.mdb_out_nxt(7 downto 0);
	else
		v_mdb_out := r.mdb_out_nxt;
	end if;

	--! Format memory data bus input depending on BW
	if (v_mb_en = '1') then
		v.mab_lsb := d.alu_out_add(0);
	end if;

	if (not(d.inst_bw) = '1') then
		v_mdb_in_bw := d.mdb_in;
	elsif (r.mab_lsb = '1') then
		v_mdb_in_bw := d.mdb_in(15 downto 8) & d.mdb_in(15 downto 8);
	else
		v_mdb_in_bw := d.mdb_in;
	end if;

	--! Memory data bus input buffer (buffer after a source read)
	if (d.e_state = E_SRC_RD) then
		v.mdb_in_buf_en := '1';
	else
		v.mdb_in_buf_en := '0';
	end if;

	if (d.e_state = E_EXEC) then
		v.mdb_in_buf_valid := '0';
	elsif (r.mdb_in_buf_en) then
		v.mdb_in_buf_valid := '1';
	end if;

	if (r.mdb_in_buf_en = '1') then
		v.mdb_in_buf := v_mdb_in_bw;
	end if;

	if (r.mdb_in_buf_valid = '1') then
		v_mdb_in_val := r.mdb_in_buf;
	else
		v_mdb_in_val := v_mdb_in_bw;
	end if;


		--! drive register inputs
		rin <= v;
		--! drive module outputs
		reg_dest_wr	<= v_reg_dest_wr;
		reg_sp_wr	<= v_reg_sp_wr;
		reg_sr_wr	<= v_reg_sr_wr;
		reg_sr_clr	<= v_reg_sr_clr;
		reg_pc_call	<= v_reg_pc_call;
		reg_incr		<= v_reg_incr;
		exec_cycle	<= v_exec_cycle;
--	signal	status				: std_logic_vector(3 downto 0);
		op_dst		<= v_op_dst;
		op_src		<= v_op_src;
	--! OUTPUTs
--	cpuoff			: out	std_logic;								--! Turns off the CPU
		dbg_reg_din	<= v_dbg_reg_din;	--! Debug unit CPU register data input
--	gie				: out	std_logic;								--! General interrupt enable
	mab				<= v_mab;	--! Memory address bus
	mb_en				<= v_mb_en;								--! Memory bus enable
	mb_wr				<= v_mb_wr;		--! Memory bus write transfer
	mdb_out			<= v_mdb_out;	--! Memory data bus output
--	oscoff			: out	std_logic;								--! Turns off LFXT1 clock input
--	pc_sw				<= v_dbg_reg_di;	--! Program counter software value
--	pc_sw_wr			<= v_dbg_reg_di;								--! Program counter software write
--	scg0				: out	std_logic;								--! System clock generator 1. Turns off hte DCO
--	scg1				: out	std_logic;								--! System clock generator 1. Turns off the SMCLK

	end process COMB;

	REGS : process (mclk,mrst)
	begin
		if (mrst = '1') then
			r	<=	(	mdb_out_nxt			=> x"0000", --! Memory data bus output
						mab_lsb				=> '0',--! Format memory data bus input depending on BW
						mdb_in_buf_en		=> '0',--! Memory data bus input buffer (buffer after a source read)
						mdb_in_buf_valid	=> '0',							
						mdb_in_buf			=> x"0000"	
					);
	elsif rising_edge(mclk) then
			r	<= rin;
		end if;
	end process REGS;


	register_file : fmsp_register_file
	port map(
		mclk				=>	mclk,				--! Main system clock
		mrst			=>	mrst,			--! Main system reset
		--! INPUTs
		alu_stat			=>	d.alu_stat,		--! ALU Status {V,N,Z,C}
		alu_stat_wr		=>	d.alu_stat_wr,	--! ALU Status write {V,N,Z,C}
		inst_bw			=>	d.inst_bw,		--! Decoded Inst: byte width
		inst_dest		=>	d.inst_dest,	--! Register destination selection
		inst_src			=>	d.inst_src,		--! Register source selection
		pc					=>	d.pc,				--! Program counter
		reg_dest_val	=>	d.alu_out,		--! Selected register destination value
		reg_dest_wr		=>	reg_dest_wr,	--! Write selected register destination
		reg_pc_call		=>	reg_pc_call,	--! Trigger pc update for a CALL instruction
		reg_sp_val		=>	d.alu_out_add,	--! Stack Pointer next value
		reg_sp_wr		=>	reg_sp_wr,		--! Stack Pointer write
		reg_sr_clr		=>	reg_sr_clr,		--! Status register clear for interrupts
		reg_sr_wr		=>	reg_sr_wr,		--! Status Register update for RETI instruction
		reg_incr			=>	reg_incr,		--! Increment source register
		--! OUTPUTs
		cpuoff			=>	cpuoff,			--! Turns off the CPU
		gie				=>	gie,				--! General interrupt enable
		oscoff			=>	oscoff,			--! Turns off LFXT1 clock input
		pc_sw				=>	pc_sw,			--! Program counter software value
		pc_sw_wr			=>	pc_sw_wr,		--! Program counter software write
		reg_dest			=>	d.reg_dest,		--! Selected register destination content
		reg_src			=>	d.reg_src,		--! Selected register source content
		scg0				=>	scg0,				--! System clock generator 1. Turns off the DCOK
		scg1				=>	scg1,				--! System clock generator 1. Turns off the SMCLK
		status			=>	status			--! R2 Status {V,N,Z,C}
	);

	alu : fmsp_alu
	port map(
		--! INPUTs
		dbg_halt_st	=>	d.dbg_halt_st,	--! Halt/Run status from CPU
		exec_cycle	=>	exec_cycle,		--! Instruction execution cycle
		inst_alu		=>	d.inst_alu,		--! ALU control signals
		inst_bw		=>	d.inst_bw,		--! Decoded Inst: byte width
		inst_jmp		=>	d.inst_jmp,		--! Decoded Inst: Conditional jump
		inst_so		=>	d.inst_so,		--! Single-operand arithmetic
		op_dst		=>	op_dst,			--! Destination operand
		op_src		=>	op_src,			--! Source operand
		status		=>	status,			--! R2 Status {V,N,Z,C}
		--! OUTPUTs
		alu_out		=>	d.alu_out,		--! ALU output value
		alu_out_add	=>	d.alu_out_add,	--! ALU adder output value
		alu_stat		=>	d.alu_stat,		--! ALU Status {V,N,Z,C}
		alu_stat_wr	=>	d.alu_stat_wr	--! ALU Status write {V,N,Z,C}
	);


end RTL;
