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
--! @file fmsp_register_file.vhd
--! 
--! @brief fpgaMSP430 Register files
--
--! @author Olivier Girard,    olgirard@gmail.com
--! @author Emmanuel Amadio,   emmanuel.amadio@gmail.com (VHDL Rewrite)
--
------------------------------------------------------------------------------
--! @version 1
--! @date: 2017-04-21
------------------------------------------------------------------------------
library ieee;
	use ieee.std_logic_1164.all;    --! standard unresolved logic UX01ZWLH-
	use ieee.numeric_std.all;       --! for the signed, unsigned types and arithmetic ops
	use work.fmsp_functions.all;

entity fmsp_register_file is 
port (
	mclk				: in	std_logic;								--! Main system clock
	mrst				: in	std_logic;								--! Main system reset
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
	reg_dest			: out	std_logic_vector(15 downto 0);	--! Selected register destination content
	reg_src			: out	std_logic_vector(15 downto 0);	--! Selected register source content
	scg0				: out	std_logic;								--! System clock generator 1. Turns off te DCO
	scg1				: out	std_logic;								--! System clock generator 1. Turns off the SMmclk
	status			: out	std_logic_vector(3 downto 0)		--! R2 Status {V,N,Z,C}
);
end entity fmsp_register_file;

architecture RTL of fmsp_register_file is 

--type single_register is std_logic_vector(15 downto 0);--
type array_16registers is array(0 to 15) of std_logic_vector(15 downto 0);

	type fmsp_register_file_in_type is record
		alu_stat			: std_logic_vector(3 downto 0);	--! ALU Status {V,N,Z,C}
		alu_stat_wr		: std_logic_vector(3 downto 0);	--! ALU Status write {V,N,Z,C}
		inst_bw			: std_logic;							--! Decoded Inst: byte width
		inst_dest		: std_logic_vector(15 downto 0);	--! Register destination selection
		inst_src			: std_logic_vector(15 downto 0);	--! Register source selection
		pc					: std_logic_vector(15 downto 0);	--! Program counter
		reg_dest_val	: std_logic_vector(15 downto 0);	--! Selected register destination value
		reg_dest_wr		: std_logic;							--! Write selected register destination
		reg_pc_call		: std_logic;							--! Trigger PC update for a CALL instruction
		reg_sp_val		: std_logic_vector(15 downto 0);	--! Stack Pointer next value
		reg_sp_wr		: std_logic;							--! Stack Pointer write
		reg_sr_wr		: std_logic;							--! Status register update for RETI instruction
		reg_sr_clr		: std_logic;							--! Status register clear for interrupts
		reg_incr			: std_logic;							--! Increment source register
	end record;

	type reg_type is record
		--! From outside of module
		--! Internal mess of module
--		incr_op				: std_logic_vector(15 downto 0);
--		reg_incr_val		: std_logic_vector(15 downto 0);
--		reg_dest_val_in	: std_logic_vector(15 downto 0);
--		inst_src_in			: std_logic_vector(15 downto 0);
--		r1_wr				: std_logic;
--		r1_inc			: std_logic;
--		r2_wr				: std_logic;
--		r2_c				: std_logic; --! C
--		r2_z				: std_logic; --! Z
--		r2_n				: std_logic; --! N
--		r2_nxt			: std_logic_vector(7 downto 3);
--		r2_v				: std_logic; --! V
--		reg_wr			: std_logic_vector(15 downto 0);
--		reg_inc			: std_logic_vector(15 downto 0);
		regs_array			: array_16registers;

		--! To outside of module
--		cpuoff	: std_logic;							--! Turns off the CPU
--		gie		: std_logic;							--! General interrupt enable
--		oscoff	: std_logic;							--! Turns off LFXT1 clock input
--		pc_sw		: std_logic_vector(15 downto 0);	--! Program counter software value
--		pc_sw_wr	: std_logic;							--! Program counter software write
--		reg_dest	: std_logic_vector(15 downto 0);	--! Selected register destination content
--		reg_src	: std_logic_vector(15 downto 0);	--! Selected register source content
--		scg1		: std_logic;							--! System clock generator 1. Turns off the SMmclk
--		status	: std_logic_vector(3 downto 0)	--! R2 Status {V,N,Z,C}
	end record;

	signal	d		: fmsp_register_file_in_type;
	signal	r		: reg_type :=	(	regs_array			=> (Others => (Others => '0'))
										);
	signal	rin	: reg_type;

begin

		d.alu_stat		<=	alu_stat;
		d.alu_stat_wr	<=	alu_stat_wr;
		d.inst_bw		<=	inst_bw;
		d.inst_dest		<=	inst_dest;
		d.inst_src		<=	inst_src;
		d.pc				<=	pc;
		d.reg_dest_val	<=	reg_dest_val;
		d.reg_dest_wr	<=	reg_dest_wr;
		d.reg_pc_call	<=	reg_pc_call;
		d.reg_sp_val	<=	reg_sp_val;
		d.reg_sp_wr		<=	reg_sp_wr;
		d.reg_sr_wr		<=	reg_sr_wr;
		d.reg_sr_clr	<=	reg_sr_clr;
		d.reg_incr		<=	reg_incr;

	COMB : process (d, r)
		variable	v						: reg_type;
		variable	v_incr_op			: std_logic_vector(15 downto 0);
		variable	v_reg_incr_val		: std_logic_vector(15 downto 0);
		variable	v_reg_dest_val_in	: std_logic_vector(15 downto 0);
		variable	v_inst_src_in		: std_logic_vector(15 downto 0);
--		variable	v_r0					: std_logic_vector(15 downto 0);
		variable	v_r1_wr				: std_logic;
		variable	v_r1_inc				: std_logic;
		variable	v_r2_wr				: std_logic;
		variable	v_r2_c				: std_logic; --! C
		variable	v_r2_z				: std_logic; --! Z
		variable	v_r2_n				: std_logic; --! N
		variable	v_r2_nxt				: std_logic_vector(7 downto 3);
		variable	v_r2_v				: std_logic; --! V
		variable	v_reg_wr				: std_logic_vector(15 downto 0);
		variable	v_reg_inc			: std_logic_vector(15 downto 0);
		variable	v_pc_sw				: std_logic_vector(15 downto 0);	--! Program counter software value
		variable	v_pc_sw_wr			: std_logic;							--! Program counter software write
		variable	v_reg_dest			: std_logic_vector(15 downto 0);	--! Selected register destination content
		variable	v_reg_src			: std_logic_vector(15 downto 0);	--! Selected register source content
		variable	UNUSED_reg_sp_val_0			: std_logic;
	begin
		--! default assignment
		v := r;
		--! overriding assignments

		--! Source input selection mask (for interrupt support)
	-------------------------------------------------------
		if (d.reg_sr_clr = '1') then
			v_inst_src_in	:= x"0004";
		else
			v_inst_src_in	:= d.inst_src;
		end if; 

		REG_MUXING_SRC : for i in 0 to 15 loop
			v_reg_src(i)	:=		(d.pc(i)				and v_inst_src_in(0)) 
									or (r.regs_array(1)(i)	and v_inst_src_in(1)) 
									or (r.regs_array(2)(i)	and v_inst_src_in(2)) 
									or (r.regs_array(3)(i)	and v_inst_src_in(3)) 
									or (r.regs_array(4)(i)	and v_inst_src_in(4)) 
									or (r.regs_array(5)(i)	and v_inst_src_in(5)) 
									or (r.regs_array(6)(i)	and v_inst_src_in(6)) 
									or (r.regs_array(7)(i)	and v_inst_src_in(7)) 
									or (r.regs_array(8)(i)	and v_inst_src_in(8)) 
									or (r.regs_array(9)(i)	and v_inst_src_in(9)) 
									or (r.regs_array(10)(i)	and v_inst_src_in(10)) 
									or (r.regs_array(11)(i)	and v_inst_src_in(11)) 
									or (r.regs_array(12)(i)	and v_inst_src_in(12)) 
									or (r.regs_array(13)(i)	and v_inst_src_in(13)) 
									or (r.regs_array(14)(i)	and v_inst_src_in(14)) 
									or (r.regs_array(15)(i)	and v_inst_src_in(15));
		end loop;
		
		--=============================================================================
		--! 1)  AUTOINCREMENT UNIT
		--=============================================================================

		if (d.inst_bw = '1') then
			v_incr_op   :=	x"0001";
		else
			v_incr_op   :=	x"0002";
		end if;

		v_reg_incr_val	:= STD_LOGIC_VECTOR(UNSIGNED(v_reg_src) + UNSIGNED(v_incr_op));

		if (d.inst_bw = '1') then
			v_reg_dest_val_in	:= x"00" & d.reg_dest_val(7 downto 0);
		else
			v_reg_dest_val_in	:= d.reg_dest_val;
		end if;


		--=============================================================================
		--! 2)  SPECIAL REGISTERS (R1/R2/R3)
		--=============================================================================


		--! R0: Program counter
		-----------------------

		--v.regs_array(0)	:=	d.pc;
		--v_r0			:=	d.pc;

		v_pc_sw		:= v_reg_dest_val_in;
		v_pc_sw_wr	:= (d.inst_dest(0) and d.reg_dest_wr) or d.reg_pc_call;


		--! R1: Stack pointer
		---------------------

		v_r1_wr	:= d.inst_dest(1) and d.reg_dest_wr;
		v_r1_inc	:=	v_inst_src_in(1) and d.reg_incr;

		if (v_r1_wr = '1') then
			v.regs_array(1)  := v_reg_dest_val_in and x"FFFE";
		elsif (d.reg_sp_wr = '1') then
			v.regs_array(1)  := d.reg_sp_val and x"FFFE";
		elsif (v_r1_inc = '1') then
			v.regs_array(1)  := v_reg_incr_val and x"FFFE";
		end if;
		UNUSED_reg_sp_val_0	:= d.reg_sp_val(0);


		--! R2: Status register
		-----------------------
		v_r2_wr		:= (d.inst_dest(2) and d.reg_dest_wr) or d.reg_sr_wr;

		--! C
		if (d.alu_stat_wr(0) = '1') then
			v_r2_c	:= d.alu_stat(0);
		else
			if (v_r2_wr = '1') then
				v_r2_c	:= v_reg_dest_val_in(0);
			else
				v_r2_c	:= r.regs_array(2)(0);
			end if;
		end if;

		--! Z
		if (d.alu_stat_wr(1) = '1') then
			v_r2_z	:= d.alu_stat(1);
		else
			if (v_r2_wr = '1') then
				v_r2_z	:= v_reg_dest_val_in(1);
			else
				v_r2_z	:= r.regs_array(2)(1);
			end if;
		end if;

		--! N
		if (d.alu_stat_wr(2) = '1') then
			v_r2_n	:= d.alu_stat(2);
		else
			if (v_r2_wr = '1') then
				v_r2_n	:= v_reg_dest_val_in(2);
			else
				v_r2_n	:= r.regs_array(2)(2);
			end if;
		end if;

		--! NXT
		if (v_r2_wr = '1') then
			v_r2_nxt	:= v_reg_dest_val_in(7 downto 3);
		else
			v_r2_nxt	:= r.regs_array(2)(7 downto 3);
		end if;

		--! V
		if (d.alu_stat_wr(3) = '1') then
			v_r2_v	:= d.alu_stat(3);
		else
			if (v_r2_wr = '1') then
				v_r2_v	:= v_reg_dest_val_in(8);
			else
				v_r2_v	:= r.regs_array(2)(8);
			end if;
		end if;

		--R2_REG : process(mclk,mrst)
		if (d.reg_sr_clr = '1') then
			v.regs_array(2)  := v_reg_dest_val_in;
		else
			v.regs_array(2)  := "0000000" & v_r2_v & v_r2_nxt & v_r2_n & v_r2_z & v_r2_c;
		end if;

		--! R3: Constant generator
		--------------------------
		v_reg_wr(3)		:= d.inst_dest(3) and d.reg_dest_wr;
		v_reg_inc(3)	:= v_inst_src_in(3)  and d.reg_incr;

		--R3_REG : process(mclk,mrst)
		if (v_reg_wr(3) = '1') then
			v.regs_array(3)  := v_reg_dest_val_in;
		elsif (v_reg_inc(3) = '1') then
			v.regs_array(3)  := v_reg_incr_val;
		end if;


		--=============================================================================
		--! 4)  GENERAL PURPOSE REGISTERS (R4...R15)
		--=============================================================================

		--	GENERAL_PURPOSE_REGISTERS : process(mclk,mrst)
		for i in 4 to 15 loop
			if ( (d.inst_dest(i) and d.reg_dest_wr) = '1') then
				v.regs_array(i)  := v_reg_dest_val_in;
			elsif ( (v_inst_src_in(i)  and d.reg_incr) = '1') then
				v.regs_array(i)  := v_reg_incr_val;
			end if;
		end loop;



		--=============================================================================
		--! 5)  READ MUX
		--=============================================================================
		REG_MUXING_DEST : for i in 0 to 15 loop
			v_reg_dest(i)	:=		(d.pc(i)					and d.inst_dest(0)) 
									or (r.regs_array(1)(i)	and d.inst_dest(1)) 
									or (r.regs_array(2)(i)	and d.inst_dest(2)) 
									or (r.regs_array(3)(i)	and d.inst_dest(3)) 
									or (r.regs_array(4)(i)	and d.inst_dest(4)) 
									or (r.regs_array(5)(i)	and d.inst_dest(5)) 
									or (r.regs_array(6)(i)	and d.inst_dest(6)) 
									or (r.regs_array(7)(i)	and d.inst_dest(7)) 
									or (r.regs_array(8)(i)	and d.inst_dest(8)) 
									or (r.regs_array(9)(i)	and d.inst_dest(9)) 
									or (r.regs_array(10)(i)	and d.inst_dest(10)) 
									or (r.regs_array(11)(i)	and d.inst_dest(11)) 
									or (r.regs_array(12)(i)	and d.inst_dest(12)) 
									or (r.regs_array(13)(i)	and d.inst_dest(13)) 
									or (r.regs_array(14)(i)	and d.inst_dest(14)) 
									or (r.regs_array(15)(i)	and d.inst_dest(15));
		end loop;


		--! drive register inputs
		rin <= v;
		--! drive module outputs
		
		status	<=	r.regs_array(2)(8) & r.regs_array(2)(2 downto 0);
		scg1		<= r.regs_array(2)(7);
		scg0		<=	r.regs_array(2)(6);
		oscoff	<=	r.regs_array(2)(5);
		cpuoff	<= r.regs_array(2)(4) or (v_r2_nxt(4) and v_r2_wr); 
		gie		<= r.regs_array(2)(3);
		pc_sw		<= v_pc_sw;
		pc_sw_wr	<= v_pc_sw_wr;
		reg_dest	<= v_reg_dest;
		reg_src	<= v_reg_src;

	end process COMB;

	REGS : process (mclk,mrst)
	begin
		if (mrst = '1') then
			r.regs_array	<= (Others => (Others => '0'));
		elsif rising_edge(mclk) then
			r	<= rin;
		end if;
	end process REGS;


end RTL;