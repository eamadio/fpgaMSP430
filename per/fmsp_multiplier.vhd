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
--! @file fmsp_multiplier.vhd
--! 
--! @brief fpgaMSP430 16x16 Hardware multiplier.
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
	use work.fmsp_misc_package.all;
	use work.fmsp_per_package.all;
	use work.fmsp_functions.all;

entity	fmsp_multiplier is
port (
	mclk			: in	std_logic;								--! Main system clock
	puc_rst		: in	std_logic;								--! Main system reset
	--! INPUTs
	per_addr		: in	std_logic_vector(13 downto 0);	--! Peripheral address
	per_din		: in	std_logic_vector(15 downto 0);	--! Peripheral data input
	per_en		: in	std_logic;								--! Peripheral enable (high active)
	per_we		: in	std_logic_vector(1 downto 0);		--! Peripheral write enable (high active)
	--! OUTPUTs
	per_dout		: out	std_logic_vector(15 downto 0)		--! Peripheral data output
);
end entity fmsp_multiplier;

architecture RTL of fmsp_multiplier is 
--=============================================================================
--! 1)  PARAMETER/REGISTERS & WIRE DECLARATION
--=============================================================================

	--! Register base address (must be aligned to decoder bit width)
	constant	BASE_ADDR	: std_logic_vector(14 downto 0) := "000000100110000";
	--! Decoder bit width (defines how many bits are considered for address decoding)
	constant	DEC_WD      :	integer	:= 4;
	--! Register addresses offset
	constant	OP1_MPY	: integer := 0;
	constant	OP1_MPYS	: integer := 2;
	constant	OP1_MAC	: integer := 4;
	constant	OP1_MACS	: integer := 6;
	constant	OP2		: integer := 8;
	constant	RESLO		: integer := 10;
	constant	RESHI		: integer := 12;
	constant	SUMEXT	: integer := 14;
	--! Register one-hot decoder utilities
	constant	DEC_SZ      : integer :=  2**DEC_WD;
	constant	BASE_REG		: std_logic_vector(DEC_SZ-1 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(1, 			DEC_SZ));
	constant	OP1_MPY_D	: std_logic_vector(DEC_SZ-1 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(OP1_MPY,	DEC_SZ));
	constant	OP1_MPYS_D	: std_logic_vector(DEC_SZ-1 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(OP1_MPYS,	DEC_SZ));
	constant	OP1_MAC_D	: std_logic_vector(DEC_SZ-1 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(OP1_MAC,	DEC_SZ));
	constant	OP1_MACS_D	: std_logic_vector(DEC_SZ-1 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(OP1_MACS,	DEC_SZ));
	constant	OP2_D			: std_logic_vector(DEC_SZ-1 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(OP2,			DEC_SZ));
	constant	RESLO_D		: std_logic_vector(DEC_SZ-1 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(RESLO,		DEC_SZ));
	constant	RESHI_D		: std_logic_vector(DEC_SZ-1 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(RESHI,		DEC_SZ));
	constant	SUMEXT_D		: std_logic_vector(DEC_SZ-1 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(SUMEXT,		DEC_SZ));

	type fmsp_multiplier_in_type is record
		puc_rst	: std_logic;								--! Main system reset
		per_addr	: std_logic_vector(13 downto 0);	--! Peripheral address
		per_din	: std_logic_vector(15 downto 0);	--! Peripheral data input
		per_en	: std_logic;								--! Peripheral enable (high active)
		per_we	: std_logic_vector(1 downto 0);		--! Peripheral write enable (high active)
	end record;

	type reg_type is record
		op1			: std_logic_vector(15 downto 0);
		op2			: std_logic_vector(15 downto 0);
		reslo			: std_logic_vector(15 downto 0);
		reshi			: std_logic_vector(15 downto 0);
		sumext_s		: std_logic_vector(1 downto 0);
		sign_sel		: std_logic;	--! Detect signed mode
		acc_sel		: std_logic;	--! Detect accumulate mode
		cycle			: std_logic;--! Detect start of a multiplication
		--! To outside of module
		nmie			: std_logic;								--! Non-maskable interrupt enable
		wdtie			: std_logic;								--! Watchdog-timer interrupt enable
		nmiifg		: std_logic;
		wdtifg		: std_logic;
		wdt_reset	: std_logic;								--! Watchdog-timer reset
	end record;

	signal	d		: fmsp_multiplier_in_type;
	signal	r		: reg_type :=	(	op1			=> x"0000",
												op2			=> x"0000",
												reslo			=> x"0000",
												reshi			=> x"0000",
												sumext_s		=> "00",
												sign_sel		=> '0',	--! Detect signed mode
												acc_sel		=> '0',	--! Detect accumulate mode
												cycle			=> '0',	--! Detect start of a multiplication
												nmie			=> '0',
												wdtie			=> '0',
												nmiifg		=> '0',
												wdtifg		=> '0',
												wdt_reset	=> '0'
										);
	signal	rin	: reg_type;

begin

		d.per_addr		<=	per_addr;
		d.per_din		<=	per_din;
		d.per_en			<=	per_en;
		d.per_we			<=	per_we;

	COMB : process (d, r)
		variable	v		: reg_type;

		--! Wire pre-declarations
		variable	v_result_wr		: std_logic;
		variable	v_result_clr	: std_logic;
--		variable	v_early_read	: std_logic;
		--! Local register selection
		variable	v_reg_sel		: std_logic;
		--! Register local address
		variable	v_reg_addr		: std_logic_vector(DEC_WD-2 downto 0);
		--! Register address decode
		variable	v_reg_dec		: std_logic_vector((DEC_SZ/2)-1 downto 0);
		--! Read/Write probes
		variable	v_reg_write		: std_logic;
		variable	v_reg_read		: std_logic;
		--! Read/Write vectors
		variable	v_reg_wr			: std_logic_vector(DEC_SZ-1 downto 0);
		variable	v_reg_rd			: std_logic_vector(DEC_SZ-1 downto 0);
		--! OP1 Register
		-------------------!   
		variable	v_op1_wr			: std_logic;
		variable	v_op1_rd			: std_logic_vector(15 downto 0);
		--! OP2 Register
		-------------------!   
		variable	v_op2_wr			: std_logic;
		variable	v_op2_rd			: std_logic_vector(15 downto 0);
		--! RESLO Register
		-------------------!   
		variable	v_reslo_nxt		: std_logic_vector(15 downto 0);
		variable	v_reslo_wr		: std_logic;
		variable	v_reslo_rd		: std_logic_vector(15 downto 0);
		--! RESHI Register
		-------------------!   
		variable	v_reshi_nxt		: std_logic_vector(15 downto 0);
		variable	v_reshi_wr		: std_logic;
		variable	v_reshi_rd		: std_logic_vector(15 downto 0);
		--! SUMEXT Register
		-------------------!   
		variable	v_sumext_s_nxt	: std_logic_vector(1 downto 0);
		variable	v_sumext_nxt	: std_logic_vector(15 downto 0);
		variable	v_sumext			: std_logic_vector(15 downto 0);
		variable	v_sumext_rd		: std_logic_vector(15 downto 0);

		variable	v_op1_mux		: std_logic_vector(15 downto 0);
		variable	v_op2_mux		: std_logic_vector(15 downto 0);
		variable	v_reslo_mux		: std_logic_vector(15 downto 0);
		variable	v_reshi_mux		: std_logic_vector(15 downto 0);
		variable	v_sumext_mux	: std_logic_vector(15 downto 0);
		--! Combine RESHI & RESLO 
		variable	v_result			: std_logic_vector(31 downto 0);
--		variable	v_op1_xp			: std_logic_vector(16 downto 0);
--		variable	v_op2_xp			: std_logic_vector(16 downto 0);
		--! 17x17 signed multiplication
		variable	v_product		: signed(33 downto 0);
		--! Accumulate
		variable	v_result_nxt	: signed(32 downto 0);
		--! Next register values
--		variable	v_reslo_nxt		: std_logic_vector(15 downto 0);
--		variable	v_reshi_nxt		: std_logic_vector(15 downto 0);
--		variable	v_sumext_s_nxt	: std_logic_vector(1 downto 0);
		--! Expand the operands to support signed & unsigned operations
		variable	v_op1_xp			: std_logic_vector(16 downto 0);
		variable	v_op2_xp			: std_logic_vector(16 downto 0);
		--! 17x9 signed multiplication
--		variable	v_product		: std_logic_vector(25 downto 0);
		variable	v_product_xp	: std_logic_vector(31 downto 0);
		--! Accumulate
--		variable	v_result_nxt	: std_logic_vector(32 downto 0);
		--! Next register values
		variable	v_per_dout		: std_logic_vector(15 downto 0);
--		variable	v_reshi_nxt		: std_logic_vector(15 downto 0);
--		variable	v_sumext_s_nxt	: std_logic_vector(1 downto 0);

	begin
		--! default assignment
		v := r;
		--! overriding assignments

		--============================================================================
		--! 2)  REGISTER DECODER
		--============================================================================

		--! Local register selection
		if (d.per_addr(13 downto DEC_WD-1) = BASE_ADDR(14 downto DEC_WD)) then
			v_reg_sel   :=	d.per_en;
		else
			v_reg_sel   :=	'0';
		end if;

		--! Register local address
		v_reg_addr  :=  d.per_addr(DEC_WD-2 downto 0);

		--! Register address decode
		v_reg_dec := onehot(v_reg_addr);
		--! Read/Write probes
		v_reg_write :=	(d.per_we(1) or d.per_we(0)) and v_reg_sel;
		v_reg_read  :=	not(d.per_we(1) or d.per_we(0)) and v_reg_sel;
		--! Read/Write vectors
		for i in 0 to (DEC_SZ/2)-1 loop
			v_reg_wr((i*2)+0)	:= v_reg_dec(i) and v_reg_write;
			v_reg_wr((i*2)+1)	:= v_reg_dec(i) and v_reg_write;
			v_reg_rd((i*2)+0)	:= v_reg_dec(i) and v_reg_read;
			v_reg_rd((i*2)+1)	:= v_reg_dec(i) and v_reg_read;
		end loop;

	--============================================================================
	--! 3) REGISTERS
	--============================================================================

		--! OP1 Register
		-------------------!   
		v_op1_wr	:=	v_reg_wr(OP1_MPY)		or
						v_reg_wr(OP1_MPYS)	or
						v_reg_wr(OP1_MAC)		or
						v_reg_wr(OP1_MACS);

		if (v_op1_wr = '1') then
			v.op1	:=	d.per_din;
		end if;

		v_op1_rd	:=	r.op1;

		
		--! OP2 Register
		-------------------!   
		v_op2_wr	:=	v_reg_wr(OP2);

		if (v_op2_wr = '1') then
			v.op2	:=	d.per_din;
		end if;

		v_op2_rd	:=	r.op2;

		
		--! RESLO Register
		-------------------!   
		v_reslo_wr	:=	v_reg_wr(RESLO);

		if (v_reslo_wr = '1') then
			v.reslo	:=	d.per_din;
		elsif (v_result_clr = '1') then
			v.reslo	:=	x"0000";
		elsif (v_result_wr = '1') then
			v.reslo	:=	v_reslo_nxt;
		end if;

--		if (v_early_read = '1') then
--			v_reslo_rd	:=	v_reslo_nxt;
--		else
			v_reslo_rd	:=	r.reslo;
--		end if;


		--! RESHI Register
		-------------------!   
		v_reshi_wr	:=	v_reg_wr(RESLO);

		if (v_reshi_wr = '1') then
			v.reshi	:=	d.per_din;
		elsif (v_result_clr = '1') then
			v.reshi	:=	x"0000";
		elsif (v_result_wr = '1') then
			v.reshi	:=	v_reshi_nxt;
		end if;

--		if (v_early_read = '1') then
--			v_reshi_rd	:=	v_reshi_nxt;
--		else
			v_reshi_rd	:=	r.reshi;
--		end if;

	 
		--! SUMEXT Register
		-------------------!   

--		v_sumext_nxt	:= (0 => v_sumext_s_nxt(0), others => v_sumext_s_nxt(1));
		v_sumext			:= (0 => r.sumext_s(0), others => r.sumext_s(1));
		v_sumext_rd		:=	v_sumext;

		if (v_op2_wr = '1') then
			v.sumext_s	:=	(Others => '0');
		elsif (v_result_wr = '1') then
			v.sumext_s	:=	v_sumext_s_nxt;
		end if;

		--============================================================================
		--! 4) DATA OUTPUT GENERATION
		--============================================================================

		--! Data output mux
		v_op1_mux		:= word_per_select_dout( OP1_MPY,	v_reg_rd, v_op2_rd	);
--		if ( (v_reg_rd(OP1_MPY) = '1') or (v_reg_rd(OP1_MPYS) = '1') or (v_reg_rd(OP1_MAC) = '1') or (v_reg_rd(OP1_MACS) = '1') ) then
		if ( (v_reg_rd(OP1_MPYS) = '1') or (v_reg_rd(OP1_MAC) = '1') or (v_reg_rd(OP1_MACS) = '1') ) then
			v_op1_mux		:=	v_op1_rd;
		end if;
		v_op2_mux		:= word_per_select_dout( OP2,			v_reg_rd, v_op2_rd		);
		v_reslo_mux		:= word_per_select_dout( RESLO,		v_reg_rd, v_reslo_rd		);
		v_reshi_mux		:= word_per_select_dout( RESHI,		v_reg_rd, v_reshi_rd		);
		v_sumext_mux	:= word_per_select_dout( SUMEXT,		v_reg_rd, v_sumext_rd	);

		v_per_dout	:=	v_op1_mux		or
							v_op2_mux		or
							v_reslo_mux		or
							v_reshi_mux		or
							v_sumext_mux;


		--============================================================================
		--! 5) HARDWARE MULTIPLIER FUNCTIONAL LOGIC
		--============================================================================

		--! Multiplier configuration
		----------------------------

		--! Detect signed mode
		if (v_op1_wr = '1') then
			v.sign_sel	:=	v_reg_wr(OP1_MPYS) or v_reg_wr(OP1_MACS);
		end if;

		--! Detect accumulate mode
		if (v_op1_wr = '1') then
			v.acc_sel	:=	v_reg_wr(OP1_MAC) or v_reg_wr(OP1_MACS);
		end if;


		--! Detect whenever the RESHI and RESLO registers should be cleared
		v_result_clr := v_op2_wr and not(r.acc_sel);

		--! Combine RESHI & RESLO 
		v_result	:=	r.reshi & r.reslo;

		
		--! 16x16 Multiplier (result computed in 1 clock cycle)
		-------------------------------------------------------

		--! Detect start of a multiplication
		v.cycle	:=	v_op2_wr;
			
		v_result_wr	:= r.cycle;

		--! Expand the operands to support signed & unsigned operations
		v_op1_xp	:=	(r.sign_sel and r.op1(15)) & r.op1;
		v_op2_xp	:=	(r.sign_sel and r.op2(15)) & r.op2;


		--! 17x17 signed multiplication
		v_product	:=	SIGNED(v_op1_xp) * SIGNED(v_op2_xp);

		--! Accumulate
		v_result_nxt := SIGNED('0' & v_result) + SIGNED('0' & v_product(31 downto 0));


		--! Next register values
		v_reslo_nxt    := STD_LOGIC_VECTOR(v_result_nxt(15 downto 0));
		v_reshi_nxt    := STD_LOGIC_VECTOR(v_result_nxt(31 downto 16));
		if (r.sign_sel = '1') then
			v_sumext_s_nxt	:=	v_result_nxt(31) & v_result_nxt(31);
		else
			v_sumext_s_nxt	:=	'0' & v_result_nxt(31);
		end if;

		--! Since the MAC is completed within 1 clock cycle,
		--! an early read can't happen.
--		v_early_read	:=	'0';




		--! drive register inputs
		rin <= v;
		--! drive module outputs
		per_dout	<= v_per_dout;		--! Peripheral data output

	end process COMB;

	REGS : process (mclk,puc_rst)
	begin
		if (puc_rst = '1') then
			r	<=	(	op1			=> x"0000",
						op2			=> x"0000",
						reslo			=> x"0000",
						reshi			=> x"0000",
						sumext_s		=> "00",
						sign_sel		=> '0',	--! Detect signed mode
						acc_sel		=> '0',	--! Detect accumulate mode
						cycle			=> '0',	--! Detect start of a multiplication
						nmie			=> '0',
						wdtie			=> '0',
						nmiifg		=> '0',
						wdtifg		=> '0',
						wdt_reset	=> '0'
					);
	elsif rising_edge(mclk) then
			r	<= rin;
		end if;
	end process REGS;


end RTL;