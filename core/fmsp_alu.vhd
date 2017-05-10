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
--! @file fmsp_alu.vhd
--! 
--! @brief fpgaMSP430 ALU
--
--! @author Olivier Girard,    olgirard@gmail.com
--! @author Emmanuel Amadio,   emmanuel.amadio@gmail.com (VHDL Rewrite)
--
------------------------------------------------------------------------------
--! @version 1
--! @date: 2017-04-21
------------------------------------------------------------------------------

library ieee;
	use ieee.std_logic_1164.all;	--! standard unresolved logic UX01ZWLH-
	use ieee.numeric_std.all;		--! for the signed, unsigned types and arithmetic ops
	use work.fmsp_core_package.all;
	use work.fmsp_functions.all;

entity fmsp_alu is
port (
	--! INPUTs
	dbg_halt_st		: in	std_logic;								--! Halt/Run status from CPU
	exec_cycle		: in	std_logic;								--! Instruction execution cycle
	inst_alu			: in	std_logic_vector(11 downto 0);	--! ALU control variables
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
end entity fmsp_alu;

architecture RTL of fmsp_alu is 

--=============================================================================
--! 1)  FUNCTIONS
--=============================================================================

	type fmsp_alu_in_type is record
		dbg_halt_st	: std_logic;							--! Halt/Run status from CPU
		exec_cycle	: std_logic;							--! Instruction execution cycle
		inst_alu		: std_logic_vector(11 downto 0);	--! ALU control variables
		inst_bw		: std_logic;							--! Decoded Inst: byte width
		inst_jmp		: std_logic_vector(7 downto 0);	--! Decoded Inst: Conditional jump
		inst_so		: std_logic_vector(7 downto 0);	--! Single-operand arithmetic
		op_dst		: std_logic_vector(15 downto 0);	--! Destination operand
		op_src		: std_logic_vector(15 downto 0);	--! Source operand
		status		: std_logic_vector(3 downto 0);	--! R2 Status {V,N,Z,C}
	end record;

	signal	d		: fmsp_alu_in_type;

begin

		d.dbg_halt_st	<=	dbg_halt_st;							--! Halt/Run status from CPU
		d.exec_cycle	<=	exec_cycle;							--! Instruction execution cycle
		d.inst_alu		<=	inst_alu;	--! ALU control variables
		d.inst_bw		<=	inst_bw;							--! Decoded Inst: byte width
		d.inst_jmp		<=	inst_jmp;	--! Decoded Inst: Conditional jump
		d.inst_so		<=	inst_so;	--! Single-operand arithmetic
		d.op_dst			<=	op_dst;	--! Destination operand
		d.op_src			<=	op_src;	--! Source operand
		d.status			<=	status;	--! R2 Status {V,N,Z,C}

	COMB : process (d)

		--! Invert source for substract and compare instructions.
		variable	v_op_src_inv_cmd	: std_logic;
		variable	v_op_src_inv		: std_logic_vector(15 downto 0);--! Mask the bit 8 for the Byte instructions for correct flags generation
		variable	v_op_bit8_msk		: std_logic;
		variable	v_op_src_in			: std_logic_vector(16 downto 0);
		variable	v_op_dst_in			: std_logic_vector(16 downto 0);
		--! Clear the source operand (= jump offset) for conditional jumps
		variable	v_jmp_not_taken	: std_logic;
		variable	v_op_src_in_jmp	: std_logic_vector(16 downto 0);
		--! Adder / AND / OR / XOR
		variable	v_alu_add			: unsigned(16 downto 0);
		variable	v_alu_and			: std_logic_vector(16 downto 0);
		variable	v_alu_or				: std_logic_vector(16 downto 0);
		variable	v_alu_xor			: std_logic_vector(16 downto 0);
		--! Incrementer
		variable	v_alu_inc_vector	: std_logic_vector(16 downto 0);
		variable	v_alu_inc			: unsigned(16 downto 0);
		variable	v_alu_add_inc		: unsigned(16 downto 0);
		--! Decimal adder (DADD)
		variable	v_alu_dadd0			: std_logic_vector(4 downto 0);
		variable	v_alu_dadd1			: std_logic_vector(4 downto 0);
		variable	v_alu_dadd2			: std_logic_vector(4 downto 0);
		variable	v_alu_dadd3			: std_logic_vector(4 downto 0);
		variable v_alu_dadd			: std_logic_vector(16 downto 0);
		--! Shifter for rotate instructions (RRC & RRA)
		variable	v_alu_shift_msb	: std_logic;
		variable	v_alu_shift_7		: std_logic;
		variable	v_alu_shift			: std_logic_vector(16 downto 0);
		--! Swap bytes / Extend Sign
		variable	v_alu_swpb			: std_logic_vector(16 downto 0);
		variable	v_alu_sxt			: std_logic_vector(16 downto 0);
		--! Combine short paths toghether to simplify final ALU mux
		variable	v_alu_short			: std_logic_vector(16 downto 0);
		--! ALU output mux
		variable	v_alu_out_nxt		: std_logic_vector(16 downto 0);
		--! STATUS FLAG GENERATION
		variable	v_V_xor				: std_logic;
		variable	v_V					: std_logic;
		variable	v_N					: std_logic;
		variable	v_Z					: std_logic;
		variable	v_C					: std_logic;
		variable	v_alu_out			: std_logic_vector(15 downto 0);	--! ALU output value
		variable	v_alu_out_add		: std_logic_vector(15 downto 0);	--! ALU adder output value
		variable	v_alu_stat			: std_logic_vector(3 downto 0);		--! ALU Status {V,N,Z,C}
		variable	v_alu_stat_wr		: std_logic_vector(3 downto 0);		--! ALU Status write {V,N,Z,C}

--=============================================================================
--! 2)  INSTRUCTION FETCH/DECODE CONTROL STATE MACHINE
--=============================================================================
--! SINGLE-OPERAND ARITHMETIC:
-------------------------------------------------------------------------------
--!   Mnemonic   S-Reg,   Operation                               Status bits
--!              D-Reg,                                            V  N  Z  C
--
--!   RRC         dst     C->MSB->...LSB->C                        *  *  *  *
--!   RRA         dst     MSB->MSB->...LSB->C                      0  *  *  *
--!   SWPB        dst     Swap bytes                               -  -  -  -
--!   SXT         dst     Bit7->Bit8...Bit15                       0  *  *  *
--!   PUSH        src     SP-2->SP, src->@SP                       -  -  -  -
--!   CALL        dst     SP-2->SP, PC+2->@SP, dst->PC             -  -  -  -
--!   RETI                TOS->SR, SP+2->SP, TOS->PC, SP+2->SP     *  *  *  *
--
-------------------------------------------------------------------------------
--! TWO-OPERAND ARITHMETIC:
-------------------------------------------------------------------------------
--!   Mnemonic   S-Reg,   Operation                               Status bits
--!              D-Reg,                                            V  N  Z  C
--
--!   MOV       src,dst    src            -> dst                   -  -  -  -
--!   ADD       src,dst    src +  dst     -> dst                   *  *  *  *
--!   ADDC      src,dst    src +  dst + C -> dst                   *  *  *  *
--!   SUB       src,dst    dst + ~src + 1 -> dst                   *  *  *  *
--!   SUBC      src,dst    dst + ~src + C -> dst                   *  *  *  *
--!   CMP       src,dst    dst + ~src + 1                          *  *  *  *
--!   DADD      src,dst    src +  dst + C -> dst (decimaly)        *  *  *  *
--!   BIT       src,dst    src &  dst                              0  *  *  *
--!   BIC       src,dst   ~src &  dst     -> dst                   -  -  -  -
--!   BIS       src,dst    src |  dst     -> dst                   -  -  -  -
--!   XOR       src,dst    src ^  dst     -> dst                   *  *  *  *
--!   AND       src,dst    src &  dst     -> dst                   0  *  *  *
--
-------------------------------------------------------------------------------
--! * the status bit is affected
--! - the status bit is not affected
--! 0 the status bit is cleared
--! 1 the status bit is set
-------------------------------------------------------------------------------
	begin
		--! overriding assignments


		--! Invert source for substract and compare instructions.
		v_op_src_inv_cmd	:= d.exec_cycle and (d.inst_alu(C_ALU_SRC_INV));
		if ( v_op_src_inv_cmd = '1' ) then
			v_op_src_inv	:= not(d.op_src);
		else
			v_op_src_inv	:= d.op_src;
		end if;

		--! Mask the bit 8 for the Byte instructions for correct flags generation
		v_op_bit8_msk	:= not(d.exec_cycle) or not(d.inst_bw);
		if ( v_op_bit8_msk = '1' ) then
			v_op_src_in	:= '0' & v_op_src_inv(15 downto 8)	& v_op_src_inv(7 downto 0);
			v_op_dst_in	:= '0' & d.op_dst(15 downto 8)		& d.op_dst(7 downto 0);
		else
			v_op_src_in	:= '0' & x"00"	& v_op_src_inv(7 downto 0);
			v_op_dst_in	:= '0' & x"00"	& d.op_dst(7 downto 0);
		end if;

		--! Clear the source operand (= jump offset) for conditional jumps
		v_jmp_not_taken	:= ( d.inst_jmp(C_JL)  and not(d.status(3) xor d.status(2)) ) or
									( d.inst_jmp(C_JGE) and    (d.status(3) xor d.status(2)) ) or
									( d.inst_jmp(C_JN)  and not(d.status(2))               ) or
									( d.inst_jmp(C_JC)  and not(d.status(0))               ) or
									( d.inst_jmp(C_JNC) and     d.status(0)                ) or
									( d.inst_jmp(C_JEQ) and not(d.status(1))               ) or
									( d.inst_jmp(C_JNE) and     d.status(1)                );
		for i in 0 to 16 loop
			v_op_src_in_jmp(i)	:= v_op_src_in(i) and not(v_jmp_not_taken);
		end loop;
		--	v_op_src_in_jmp  = v_op_src_in and not(jmp_not_taken);

		--! Adder / AND / OR / XOR
		v_alu_add	:= UNSIGNED(v_op_src_in_jmp) + UNSIGNED(v_op_dst_in);
		v_alu_and	:= v_op_src_in			and	v_op_dst_in;
		v_alu_or		:= v_op_src_in			or		v_op_dst_in;
		v_alu_xor	:= v_op_src_in			xor	v_op_dst_in;


		--! Incrementer
		v_alu_inc_vector	:= x"0000" & (d.exec_cycle and ((d.inst_alu(C_ALU_INC_C) and d.status(0)) or d.inst_alu(C_ALU_INC)));
		v_alu_inc			:= UNSIGNED(v_alu_inc_vector);
		v_alu_add_inc		:= v_alu_add + v_alu_inc;



		--! Decimal adder (DADD)
		v_alu_dadd0	:= bcd_add(v_op_src_in(3 downto 0),		v_op_dst_in(3 downto 0),	d.status(0));
		v_alu_dadd1	:= bcd_add(v_op_src_in(7 downto 4),		v_op_dst_in(7 downto 4),	v_alu_dadd0(4));
		v_alu_dadd2	:= bcd_add(v_op_src_in(11 downto 8),	v_op_dst_in(11 downto 8),	v_alu_dadd1(4));
		v_alu_dadd3	:= bcd_add(v_op_src_in(15 downto 12),	v_op_dst_in(15 downto 12),	v_alu_dadd2(4));
		v_alu_dadd	:= v_alu_dadd3 & v_alu_dadd2(3 downto 0) & v_alu_dadd1(3 downto 0) & v_alu_dadd0(3 downto 0);


		--! Shifter for rotate instructions (RRC & RRA)
		if ( d.inst_so(C_RRC) = '1' ) then
				v_alu_shift_msb	:= d.status(0);
		else
			if ( d.inst_bw = '1' ) then
					v_alu_shift_msb	:= d.op_src(7);
			else
					v_alu_shift_msb	:= d.op_src(15);
			end if;
		end if;
		if ( d.inst_bw = '1' ) then
			v_alu_shift_7	:= v_alu_shift_msb;
		else
			v_alu_shift_7	:= d.op_src(8);
		end if;
		v_alu_shift	:= '0' & v_alu_shift_msb & d.op_src(15 downto 9) & v_alu_shift_7 & d.op_src(7 downto 1);


		--! Swap bytes / Extend Sign
		v_alu_swpb	:= '0' & d.op_src(7 downto 0) & d.op_src(15 downto 8);
		v_alu_sxt	:= '0' & d.op_src(7) & d.op_src(7) & d.op_src(7) & d.op_src(7) & d.op_src(7) & d.op_src(7) & d.op_src(7) & d.op_src(7) & d.op_src(7 downto 0);


		--! Combine short paths toghether to simplify final ALU mux
		--	alu_short_thro	:= not(	d.inst_alu(C_ALU_AND)   or
		--									d.inst_alu(C_ALU_OR)    or
		--									d.inst_alu(C_ALU_XOR)   or
		--									d.inst_alu(C_ALU_SHIFT) or
		--									d.inst_so(C_SWPB)       or
		--									d.inst_so(C_SXT) );

		for i in 0 to 16 loop
			v_alu_short(i)	:= (	d.inst_alu(C_ALU_AND)	and v_alu_and(i)		) or
									(	d.inst_alu(C_ALU_OR)		and v_alu_or(i)		) or
									(	d.inst_alu(C_ALU_XOR)	and v_alu_xor(i)		) or
									(	d.inst_alu(C_ALU_SHIFT)	and v_alu_shift(i)	) or
									(	d.inst_so(C_SWPB)			and v_alu_swpb(i)		) or
									(	d.inst_so(C_SXT)			and v_alu_sxt(i)		) or
									(	not(	d.inst_alu(C_ALU_AND)   or
												d.inst_alu(C_ALU_OR)    or
												d.inst_alu(C_ALU_XOR)   or
												d.inst_alu(C_ALU_SHIFT) or
												d.inst_so(C_SWPB)       or
												d.inst_so(C_SXT) )	and v_op_src_in(i) );
		end loop;

		--! ALU output mux
		if ( (d.inst_so(C_IRQ) or d.dbg_halt_st or d.inst_alu(C_ALU_ADD)) = '1' ) then
			v_alu_out_nxt	:= STD_LOGIC_VECTOR(v_alu_add_inc);
		else
			if ( d.inst_alu(C_ALU_DADD) = '1' ) then
				v_alu_out_nxt	:= v_alu_dadd;
			else
				v_alu_out_nxt	:= v_alu_short;
			end if;
		end if;

		v_alu_out		:=  v_alu_out_nxt(15 downto 0);
		v_alu_out_add	:=  STD_LOGIC_VECTOR(v_alu_add(15 downto 0));


		-------------------------------------------------------------------------------
		--! STATUS FLAG GENERATION
		-------------------------------------------------------------------------------
		if ( d.inst_bw = '1' ) then		
			v_V_xor	:= (v_op_src_in(7) and v_op_dst_in(7));
			v_V	:=	( (not(v_op_src_in(7))  and not(v_op_dst_in(7))  and  v_alu_out(7)) or (v_op_src_in(7)  and  v_op_dst_in(7)  and not(	v_alu_out(7))) );
			v_N	:= 	v_alu_out(7);
			if ( 	v_alu_out(7 downto 0) = x"00" ) then		
				v_Z	:= '1';
			else
				v_Z	:= '0';
			end if;
			v_C	:= v_alu_out(8);
		else
			v_V_xor	:= (v_op_src_in(15) and v_op_dst_in(15));
			v_V	:=	( (not(v_op_src_in(15)) and not(v_op_dst_in(15)) and v_alu_out(15)) or (v_op_src_in(15) and  v_op_dst_in(15) and not(	v_alu_out(15))) );
			v_N	:= v_alu_out(15);
			if ( v_alu_out = x"0000" ) then		
				v_Z	:= '1';
			else
				v_Z	:= '0';
			end if;
			v_C	:= v_alu_out_nxt(16);
		end if;
				
		if ( d.inst_alu(C_ALU_SHIFT) = '1' ) then		
				v_alu_stat	:= '0' & v_N & v_Z & v_op_src_in(0);
		else
			if ( d.inst_alu(C_ALU_STAT_7) = '1' ) then		
					v_alu_stat	:= '0' & v_N & v_Z & not(v_Z);
			else
				if ( d.inst_alu(C_ALU_XOR) = '1' ) then		
						v_alu_stat	:= v_V_xor & v_N & v_Z & not(v_Z);
				else
						v_alu_stat	:= v_V & v_N & v_Z & v_C;
				end if;
			end if;
		end if;

		if ( (d.inst_alu(C_ALU_STAT_F) and d.exec_cycle) = '1' ) then		
				v_alu_stat_wr	:= "1111";
		else
				v_alu_stat_wr	:= "0000";
		end if;

		--! OUTPUTs
		alu_out			<= v_alu_out;		--! ALU output value
		alu_out_add		<= v_alu_out_add;	--! ALU adder output value
		alu_stat			<= v_alu_stat;		--! ALU Status {V,N,Z,C}
		alu_stat_wr		<= v_alu_stat_wr;	--! ALU Status write {V,N,Z,C}
	end process COMB;

end RTL; --! fmsp_alu
