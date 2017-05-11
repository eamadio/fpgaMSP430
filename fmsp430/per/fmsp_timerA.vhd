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
--! @file fmsp_timerA.vhd
--! 
--! @brief fpgaMSP430 Timer A top-level
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
	use ieee.math_real.all;
	use work.fmsp_misc_package.all;
	use work.fmsp_per_package.all;
	use work.fmsp_functions.all;

entity fmsp_timerA is 
	port (
		mclk			: in	std_logic;       						--! Main system clock
		mrst		: in	std_logic;       						--! Main system reset
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
end entity fmsp_timerA;

architecture RTL of fmsp_timerA is 
	--=============================================================================
	--! 1)  PARAMETER DECLARATION
	--=============================================================================

	--! Register base address (must be aligned to decoder bit width)
	constant	BASE_ADDR	: std_logic_vector(14 downto 0) := "000000100000000";
	--! Decoder bit width (defines how many bits are considered for address decoding)
	constant	DEC_WD		: integer := 7;
	--! Register addresses offset
	constant	TACTL			: integer := 096;	--! 'h60,
	constant	TAR			: integer := 112;	--! 'h70,
	constant	TACCTL0		: integer := 098;	--! 'h62,
	constant	TACCR0		: integer := 114;	--! 'h72,
	constant	TACCTL1		: integer := 100;	--! 'h64,
	constant	TACCR1		: integer := 116;	--! 'h74,
	constant	TACCTL2		: integer := 102;	--! 'h66,
	constant	TACCR2		: integer := 118;	--! 'h76,
	constant	TAIV			: integer := 046;	--! 'h2E;
	--! Register one-hot decoder utilities
	constant	DEC_SZ      : integer :=  (2**DEC_WD);

	--! Timer A: TACCTLx Capture/Compare Control Register
	constant	TACLR			: integer := 2;
	constant	TAIE			: integer := 1;
	constant	TAIFG			: integer := 0;
	--! constant	TACMx			15 downto 14
	--! constant	TACCISx		13 downto 12
	constant	TASCS			: integer := 11;
	constant	TASCCI		: integer := 10;
	constant	TACAP			: integer := 8;
	--! constant	TAOUTMODx	7 downto 5
	constant	TACCIE		: integer := 4;
	constant	TACCI			: integer := 3;
	constant	TAOUT			: integer := 2;
	constant	TACOV			: integer := 1;
	constant	TACCIFG		: integer := 0;

	constant	capture_unit_nb	: integer := 3;
	type array_ofregisters is array(0 to capture_unit_nb-1) of std_logic_vector(15 downto 0);
	type array_ofunsigneds is array(0 to capture_unit_nb-1) of unsigned(15 downto 0);

	type fmsp_timerA_in_type is record
		aclk_en		: std_logic;       						--! ACLK enable (from CPU)
		smclk_en		: std_logic;       						--! SMCLK enable (from CPU)
		dbg_freeze	: std_logic;       						--! Freeze Timer A counter
		irq_ta0_acc	: std_logic;       						--! Interrupt request TACCR0 accepted
		per_addr		: std_logic_vector(13 downto 0);		--! Peripheral address
		per_din		: std_logic_vector(15 downto 0);		--! Peripheral data input
		per_en		: std_logic;       						--! Peripheral enable (high active)
		per_we		: std_logic_vector(1 downto 0);		--! Peripheral write enable (high active)
		taclk_s		: std_logic;
		inclk_s		: std_logic;
		--! Shared
		ta_ccixa		: std_logic_vector(2 downto 0);       						--! Timer capture input A
		ta_ccixb		: std_logic_vector(2 downto 0);       						--! Timer capture input B
		ccix_s		: std_logic_vector(2 downto 0);
	end record;

	type reg_type is record
		tactl			: std_logic_vector(9 downto 0);
		tar			: unsigned(15 downto 0);
		taclk_dly	: std_logic;
		inclk_dly	: std_logic;
		clk_div		: unsigned(2 downto 0);
		tar_dir		: std_logic;
		--! Shared
		tacctlx		: array_ofregisters;
		taccrx		: array_ofunsigneds;
		sccix			: std_logic_vector(2 downto 0);
		ccix_dly		: std_logic_vector(2 downto 0);
		ccix_evt_s	: std_logic_vector(2 downto 0);
		ccix_sync	: std_logic_vector(2 downto 0);
		capx_taken	: std_logic_vector(2 downto 0);
		ta_outx		: std_logic_vector(2 downto 0);
	end record;

	signal	d		: fmsp_timerA_in_type;
	signal	r		: reg_type :=	(	tactl			=> "0000000000",
												tar			=> x"0000",
												taclk_dly	=> '0',
												inclk_dly	=> '0',
												clk_div		=> "000",
												tar_dir		=> '0',
												tacctlx		=> (Others =>  x"0000"),
												taccrx		=> (Others =>  x"0000"),
												sccix			=> "000",
												ccix_dly		=> "000",
												ccix_evt_s	=> "000",
												ccix_sync	=> "000",
												capx_taken	=> "000",
												ta_outx		=> "000"
											);
	signal	rin	: reg_type;
	signal	cci	: std_logic_vector(2 downto 0) := "000";

begin

	d.aclk_en		<=	aclk_en;
	d.smclk_en		<=	smclk_en;
	d.dbg_freeze	<=	dbg_freeze;
	d.irq_ta0_acc	<=	irq_ta0_acc;
--	d.taclk			<=	taclk;
--	d.inclk			<=	inclk;
	d.per_addr		<=	per_addr;
	d.per_din		<=	per_din;
	d.per_en			<=	per_en;
	d.per_we			<=	per_we;
	d.ta_ccixa(0)	<=	ta_cci0a;
	d.ta_ccixb(0)	<=	ta_cci0b;
	d.ta_ccixa(1)	<=	ta_cci1a;
	d.ta_ccixb(1)	<=	ta_cci1b;
	d.ta_ccixa(2)	<=	ta_cci2a;
	d.ta_ccixb(2)	<=	ta_cci2b;

	COMB : process (d, r)
		variable	v						: reg_type;
		--! Local register selection
		variable	v_reg_sel			: std_logic;
		--! Register local address
		variable	v_reg_addr			: std_logic_vector(DEC_WD-2 downto 0);
		--! Register address decode
		variable	v_reg_dec			: std_logic_vector((DEC_SZ/2)-1 downto 0);
		--! Read/Write probes
		variable	v_reg_write			: std_logic;
		variable	v_reg_read			: std_logic;
		--! Read/Write vectors
		variable	v_reg_wr				: std_logic_vector(DEC_SZ-1 downto 0);
		variable	v_reg_rd				: std_logic_vector(DEC_SZ-1 downto 0);
		variable	v_tactl_wr			: std_logic;
		variable	v_ta_clr				: std_logic;
		variable	v_taifg_set			: std_logic;
		variable	v_taifg_clr			: std_logic;
		--! TAR Register
		variable	v_tar_wr				: std_logic;
		variable	v_tar_clk			: std_logic;
		variable	v_tar_clr			: std_logic;
		variable	v_tar_inc			: std_logic;
		variable	v_tar_dec			: std_logic;
		variable	v_tar_add			: unsigned(15 downto 0);
		variable	v_tar_nxt			: unsigned(15 downto 0);
		--! TACCTL0 Register
		variable	v_tacctlx_wr		: std_logic_vector(2 downto 0);
		variable	v_ccifgx_set		: std_logic_vector(2 downto 0);
		variable	v_covx_set			: std_logic_vector(2 downto 0);
		variable	v_ccix				: std_logic_vector(2 downto 0);
		variable	v_tacctlx_full		: array_ofregisters;
		--! TACCR0 Register
		variable	v_taccrx_wr			: std_logic_vector(2 downto 0);
		variable	v_ccix_cap			: std_logic_vector(2 downto 0);
		variable	v_equx				: std_logic_vector(2 downto 0);
		--! Input selection
--		variable	v_ccix				: std_logic_vector(2 downto 0);
		--! Capture mode
		variable	v_ccix_evt			: std_logic_vector(2 downto 0);
		--! Generate final capture command
--		variable	v_ccix_cap			: std_logic_vector(2 downto 0);
		--! Generate capture overflow flag
		variable	v_capx_taken_clr	: std_logic_vector(2 downto 0);
--		variable	v_covx_set			: std_logic_vector(2 downto 0);
		--! Output unit 0
		variable	v_ta_outx_mode0	: std_logic_vector(2 downto 0);
		variable	v_ta_outx_mode1	: std_logic_vector(2 downto 0);
		variable	v_ta_outx_mode2	: std_logic_vector(2 downto 0);
		variable	v_ta_outx_mode3	: std_logic_vector(2 downto 0);
		variable	v_ta_outx_mode4	: std_logic_vector(2 downto 0);
		variable	v_ta_outx_mode5	: std_logic_vector(2 downto 0);
		variable	v_ta_outx_mode6	: std_logic_vector(2 downto 0);
		variable	v_ta_outx_mode7	: std_logic_vector(2 downto 0);
		variable	v_ta_outx_nxt		: std_logic_vector(2 downto 0);
		variable	v_ta_outx_en		: std_logic_vector(2 downto 0);
		--! 9) Timer A interrupt generation
--		variable	v_ccifgx_set		: std_logic_vector(2 downto 0);
		--! TAIV Register
		variable	v_taiv				: std_logic_vector(3 downto 0);
		variable	v_ccifg1_clr		: std_logic;
		variable	v_ccifg2_clr		: std_logic;
		variable	v_ccifgx_clr		: std_logic_vector(2 downto 0);
--		variable	v_taifg_clr			: std_logic;
		--! Data output mux
		variable	v_tactl_rd			: std_logic_vector(15 downto 0);
		variable	v_tar_rd				: std_logic_vector(15 downto 0);
		variable	v_tacctl0_rd		: std_logic_vector(15 downto 0);
		variable	v_taccr0_rd			: std_logic_vector(15 downto 0);
		variable	v_tacctl1_rd		: std_logic_vector(15 downto 0);
		variable	v_taccr1_rd			: std_logic_vector(15 downto 0);
		variable	v_tacctl2_rd		: std_logic_vector(15 downto 0);
		variable	v_taccr2_rd			: std_logic_vector(15 downto 0);
		variable	v_taiv_rd			: std_logic_vector(15 downto 0);
		variable	v_per_dout			: std_logic_vector(15 downto 0);
		--! Clock edge detection (TACLK & INCLK)
		variable	v_taclk_en			: std_logic;
		variable	v_inclk_en			: std_logic;
		--! Timer clock input mux
		variable	v_sel_clk			: std_logic;
		--! Generate update pluse for the counter (<=> divided clock)
--		variable	v_tar_clk			: std_logic;
--		--! Time counter control signals
--		variable	v_tar_clr			: std_logic;
--		variable	v_tar_inc			: std_logic;
--		variable	v_tar_dec			: std_logic;
		--! 9) Timer A interrupt generation
--		variable	v_taifg_set			: std_logic;
		variable	v_irq_ta0			: std_logic;
		variable	v_irq_ta1			: std_logic;
	--! Timer A: TACTL Control Register
		alias		a_TACTL_TASSELx		: std_logic_vector(1 downto 0) is r.tactl(9 downto 8);
		alias		a_TACTL_TAIDx			: std_logic_vector(1 downto 0) is r.tactl(7 downto 6);
		alias		a_TACTL_TAMCx			: std_logic_vector(1 downto 0) is r.tactl(5 downto 4);
		alias		a_TACTL_TACLR			: std_logic is r.tactl(2);
		alias		a_TACTL_TAIE			: std_logic is r.tactl(1);
		alias		a_TACTL_TAIFG			: std_logic is r.tactl(0);
	begin
		--! default assignment
		v := r;
		--! overriding assignments

		--============================================================================
		--! 1)  REGISTER DECODER
		--============================================================================

		--! Local register selection
		if ( d.per_addr(13 downto DEC_WD-1) = BASE_ADDR(14 downto DEC_WD) ) then
			v_reg_sel	:=	d.per_en;
		else
			v_reg_sel	:=	'0';
		end if;
		--! Register local address
		v_reg_addr	:=  d.per_addr(DEC_WD-2 downto 0);
		--! Register address decode
		v_reg_dec	:= onehot(v_reg_addr);
		
		--! Read/Write probes
		v_reg_write		:= v_reg_sel and    (d.per_we(0) or d.per_we(1));
		v_reg_read		:= v_reg_sel and not(d.per_we(0) or d.per_we(1));
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
		v_tactl_wr	:= v_reg_wr(TACTL);
		v_ta_clr		:= v_tactl_wr and d.per_din(TACLR);
		v_tar_wr		:= v_reg_wr(TAR);

		--! TAIV Register
		--------------------
		if		( (r.tacctlx(1)(TACCIFG) = '1') and (r.tacctlx(1)(TACCIE) = '1') ) then
			v_taiv	:= x"2";
		elsif	( (r.tacctlx(2)(TACCIFG) = '1') and (r.tacctlx(2)(TACCIE) = '1') ) then
			v_taiv	:= x"4";
		elsif	( (r.tactl(TAIFG) = '1') and (r.tactl(TAIE) = '1') ) then
			v_taiv	:= x"A";
		else
			v_taiv	:= x"0";
		end if;

		v_ccifg1_clr	:= '0';
		v_ccifg2_clr	:= '0';
		v_taifg_clr		:= '0';
		if (v_taiv = x"2") then
			v_ccifg1_clr	:=	v_reg_rd(TAIV) or v_reg_wr(TAIV);
		end if;
		if (v_taiv = x"4") then
			v_ccifg2_clr	:=	v_reg_rd(TAIV) or v_reg_wr(TAIV);
		end if;
		if (v_taiv = x"A") then
			v_taifg_clr		:=	v_reg_rd(TAIV) or v_reg_wr(TAIV);
		end if;

		--============================================================================
		--! 5) Timer A counter control
		--============================================================================
		--! Clock edge detection (TACLK & INCLK)
		-------------------------------------------------------------
		v.taclk_dly	:=	d.taclk_s;
		v_taclk_en	:= d.taclk_s and not(r.taclk_dly);

		v.inclk_dly	:=	d.inclk_s;
		v_inclk_en	:= d.inclk_s and not(r.inclk_dly);

		--! Timer clock input mux
		-------------------------------------------------------------
		if (r.tactl(9 downto 8) = "00") then
			v_sel_clk	:=	v_taclk_en;
		elsif (r.tactl(9 downto 8) = "01") then
			v_sel_clk	:=	d.aclk_en;
		elsif (r.tactl(9 downto 8) = "10") then
			v_sel_clk	:=	d.smclk_en;
		else
			v_sel_clk	:=	v_inclk_en;
		end if;
		--! Generate update pulse for the counter (<=> divided clock)
		-------------------------------------------------------------
		if (r.tactl(7 downto 6) = "00") then
			v_tar_clk	:=	v_sel_clk;
		elsif (r.tactl(7 downto 6) = "01") then
			v_tar_clk	:=	v_sel_clk and r.clk_div(0);
		elsif (r.tactl(7 downto 6) = "10") then
			v_tar_clk	:=	v_sel_clk and r.clk_div(0) and r.clk_div(1);
		else
			v_tar_clk	:=	v_sel_clk and r.clk_div(0) and r.clk_div(1) and r.clk_div(2);
		end if;

		--! Time counter control signals
		-------------------------------------------------------------
		v_tar_clr	:= '0';
		if ( 		( (r.tactl(5 downto 4) = "01") and (r.tar >= r.taccrx(0)) ) 
				or	( (r.tactl(5 downto 4) = "11") and (r.taccrx(0) = x"0000") ) ) then
			v_tar_clr	:= '1';
		end if;

		v_tar_dec	:= '0';
		if ( 		(r.tar_dir = '1')
				or	( (r.tactl(5 downto 4) = "11") and (r.tar >= r.taccrx(0)) ) ) then
			v_tar_dec	:= '1';
		end if;

		v_tar_inc	:= '0';
		if ( 		(r.tactl(5 downto 4) = "01")
				or (r.tactl(5 downto 4) = "01")
				or	( (r.tactl(5 downto 4) = "11") and (not(v_tar_dec) = '1') ) ) then
			v_tar_inc	:= '1';
		end if;

		if ( (v_tar_clk or v_ta_clr) = '1' ) then
			v.clk_div	:=	"000";
		elsif ( (r.tactl(5 downto 4) /= "00") and (v_sel_clk = '1') ) then
			v.clk_div	:= r.clk_div + TO_UNSIGNED(1,3);
		end if;


		if ( v_ta_clr = '1' ) then
			v.tar_dir	:=	 '0';
		elsif (r.tactl(5 downto 4) = "11") then
			if ( (r.tar = x"0001") and (not(v_tar_clk) = '1') ) then
				v.tar_dir	:=	 '0';
			elsif (r.tar >= r.taccrx(0)) then
				v.tar_dir	:=  '1';
			end if;
		else
			v.tar_dir	:=  '0';
		end if;

		--! TAR Register
		-------------------
		if (v_tar_inc = '1') then
			v_tar_add	:=	x"0001";
		elsif (v_tar_dec = '1') then
			v_tar_add	:=	x"FFFF";
		else
			v_tar_add	:=	x"0000";
		end if;

		if (v_tar_clr = '1') then
			v_tar_nxt :=  x"0000";
		else
			v_tar_nxt :=  r.tar + v_tar_add;
		end if;

		if (v_tar_wr = '1') then
			v.tar	:=	UNSIGNED(d.per_din);
		elsif ((v_tar_clk and not(d.dbg_freeze)) = '1') then
			v.tar	:=	v_tar_nxt;
		end if;

		--============================================================================
		--! 9) Timer A interrupt generation
		--============================================================================
		if (	( (r.tactl(5 downto 4) = "01") and (r.tar = r.taccrx(0)) )	or
				( (r.tactl(5 downto 4) = "10") and (r.tar = x"ffff") )	or
				( (r.tactl(5 downto 4) = "11") and (v_tar_nxt = x"0000") and (v_tar_dec = '1') ) ) then
			v_taifg_set := v_tar_clk;
		else
			v_taifg_set := '0';
		end if;

		--! TACTL Register
		-------------------

		if (v_tactl_wr = '1') then
			v.tactl	:=	( (d.per_din(9 downto 0) and "1111110011") or ("000000000" & v_taifg_set) )
							and ("111111111" & not(v_taifg_clr));
		else
			v.tactl	:=	( r.tactl or ("000000000" & v_taifg_set) )
							and ("111111111" & not(v_taifg_clr));
		end if;




		--============================================================================
		--! 7) Timer A capture logic
		--============================================================================

		for i in 0 to capture_unit_nb-1 loop

			--! Input selection
			--------------------
			if (r.tacctlx(i)(13 downto 12) = "00") then
				v_ccix(i)	:=	d.ta_ccixa(i);
			elsif (r.tacctlx(i)(13 downto 12) = "01") then
				v_ccix(i)	:=	d.ta_ccixb(i);
			elsif (r.tacctlx(i)(13 downto 12) = "10") then
				v_ccix(i)	:=	'0';
			else
				v_ccix(i)	:=	'1';
			end if;


			v.ccix_dly(i)	:=	d.ccix_s(i);
			
			--! Timer A comparator
			v_equx(i) :=  '0';
			if ( (v_tar_nxt = r.taccrx(i)) and (r.tar /= r.taccrx(i)) ) then
				v_equx(i) :=  '1';
			end if;

			--! Generate SCCIx
			--------------------
			if ((v_tar_clk and v_equx(i)) = '1') then
				v.sccix(i)	:=	 d.ccix_s(i);
			end if;


			--! Capture mode
			--------------------
			if (r.tacctlx(i)(15 downto 14) = "00") then
				v_ccix_evt(i)	:=	'0';
			elsif (r.tacctlx(i)(15 downto 14) = "01") then
				v_ccix_evt(i)	:=			d.ccix_s(i)		and not(	r.ccix_dly(i));	--! Rising edge
			elsif (r.tacctlx(i)(15 downto 14) = "10") then
				v_ccix_evt(i)	:=	not(	d.ccix_s(i)	)	and		r.ccix_dly(i);		--! Falling edge
			else
				v_ccix_evt(i)	:=			d.ccix_s(i)		xor		r.ccix_dly(i);		--! Both edges
			end if;

			--! Event Synchronization
			-------------------------
			if (v_tar_clk = '1') then
				v.ccix_evt_s(i)	:=	'0';
			elsif (v_ccix_evt(i) = '1') then
				v.ccix_evt_s(i)	:=	'1';
			end if;

			if (v_tar_clk = '1') then
				v.ccix_sync(i)	:=	(v_tar_clk and r.ccix_evt_s(i)) or (v_tar_clk and v_ccix_evt(i) and not(r.ccix_evt_s(i)));
			end if;

			--! Generate final capture command
			-------------------------------------
			if (r.tacctlx(i)(TASCS) = '1') then
				v_ccix_cap(i)	:=	r.ccix_sync(i);
			else
				v_ccix_cap(i)	:=	r.ccix_evt_s(i);
			end if;

			--! Generate capture overflow flag
			-------------------------------------
			v_capx_taken_clr(i) := v_reg_rd(TACCR0+(i*2)) or (v_tacctlx_wr(i) and v.tacctlx(i)(TACOV) and not(d.per_din(TACOV)));
			if (v_ccix_cap(i) = '1') then
				v.capx_taken(i)	:=	'1';
			elsif (v_capx_taken_clr(i) = '1') then
				v.capx_taken(i)	:=	'0';
			end if;

			v_covx_set(i)	:= r.capx_taken(i) and v_ccix_cap(i) and not(v_reg_rd(TACCR0+(i*2)));

			if (r.tacctlx(i)(TACAP) = '1') then
				v_ccifgx_set(i)	:=	v_ccix_cap(i);
			elsif (r.tactl(5 downto 4) /= "00") then	
				v_ccifgx_set(i)	:=	v_tar_clk and v_equx(i);
			else
				v_ccifgx_set(i)	:=	'0';
			end if;

		end loop;

		v_ccifgx_clr(0)	:=	d.irq_ta0_acc;
		v_ccifgx_clr(1)	:=	v_ccifg1_clr;
		v_ccifgx_clr(2)	:=	v_ccifg2_clr;

		for i in 0 to capture_unit_nb-1 loop

			v_tacctlx_wr(i)	:= v_reg_wr((TACCTL0+(i*2)));
			--! TACCTLx Registers
			if (v_tacctlx_wr(i) = '1') then
				v.tacctlx(i)	:=	( (d.per_din and x"F9F7") or ("00000000000000" & v_covx_set(i) & v_ccifgx_set(i)) )
													and ("111111111111111" & not(v_ccifgx_clr(i)));
			else
				v.tacctlx(i)	:=	( r.tacctlx(i) or ("00000000000000" & v_covx_set(i) & v_ccifgx_set(i)) )
													and ("111111111111111" & not(v_ccifgx_clr(i)));
			end if;

			v_tacctlx_full(i)	:= r.tacctlx(i) or ("00000" & r.sccix(i) & "000000" & d.ccix_s(i) & "000");

			--! TACCRx Registers
			--------------------
			v_taccrx_wr(i)		:= v_reg_wr((TACCR0+(i*2)));
			if (v_taccrx_wr(i) = '1') then
				v.taccrx(i)	:=	UNSIGNED(d.per_din);
			elsif (v_ccix_cap(i) = '1') then
				v.taccrx(i)	:=	r.tar;
			end if;

		end loop;

		--============================================================================
		--! 9) Timer A interrupt generation
		--============================================================================
		v_irq_ta0    := r.tacctlx(0)(TACCIFG) and r.tacctlx(0)(TACCIE);

		v_irq_ta1    :=		(r.tactl(TAIFG)			and r.tactl(TAIE))
								or	(r.tacctlx(1)(TACCIFG)	and r.tacctlx(1)(TACCIE))
								or	(r.tacctlx(2)(TACCIFG)	and r.tacctlx(2)(TACCIE));
		--============================================================================
		--! 8) Timer A output unit
		--============================================================================
		for i in 0 to capture_unit_nb-1 loop

			v_ta_outx_mode0(i) := r.tacctlx(i)(TAOUT);	--! Output
			if (v_equx(i) = '1') then							--! Set
				v_ta_outx_mode1(i)	:=	'1';
			else
				v_ta_outx_mode1(i)	:=	r.ta_outx(i);
			end if;
			if (v_equx(i) = '1') then							--! Toggle/Reset
				v_ta_outx_mode2(i)	:=	not(r.ta_outx(i));
			elsif (v_equx(0) = '1') then	
				v_ta_outx_mode2(i)	:=	'0';
			else
				v_ta_outx_mode2(i)	:=	r.ta_outx(i);
			end if;
			if (v_equx(i) = '1') then							--! Set/Reset
				v_ta_outx_mode3(i)	:=	'1';
			elsif (v_equx(0) = '1') then	
				v_ta_outx_mode3(i)	:=	'0';
			else
				v_ta_outx_mode3(i)	:=	r.ta_outx(i);
			end if;
			if (v_equx(i) = '1') then							--! Toggle
				v_ta_outx_mode4(i)	:=	not(r.ta_outx(i));
			else
				v_ta_outx_mode4(i)	:=	r.ta_outx(i);
			end if;
			if (v_equx(i) = '1') then							--! Reset
				v_ta_outx_mode5(i)	:=	'0';
			else
				v_ta_outx_mode5(i)	:=	r.ta_outx(i);
			end if;
			if (v_equx(i) = '1') then							--! Toggle/Set
				v_ta_outx_mode6(i)	:=	not(r.ta_outx(i));
			elsif (v_equx(0) = '1') then	
				v_ta_outx_mode6(i)	:=	'1';
			else
				v_ta_outx_mode6(i)	:=	r.ta_outx(i);
			end if;
			if (v_equx(i) = '1') then							--! Reset/Set
				v_ta_outx_mode7(i)	:=	'0';
			elsif (v_equx(0) = '1') then	
				v_ta_outx_mode7(i)	:=	'1';
			else
				v_ta_outx_mode7(i)	:=	r.ta_outx(i);
			end if;

			if (r.tacctlx(i)(7 downto 5) = "000") then
				v_ta_outx_nxt(i)	:=	v_ta_outx_mode0(i);
			elsif (r.tacctlx(i)(7 downto 5) = "001") then
				v_ta_outx_nxt(i)	:=	v_ta_outx_mode1(i);
			elsif (r.tacctlx(i)(7 downto 5) = "010") then
				v_ta_outx_nxt(i)	:=	v_ta_outx_mode2(i);
			elsif (r.tacctlx(i)(7 downto 5) = "011") then
				v_ta_outx_nxt(i)	:=	v_ta_outx_mode3(i);
			elsif (r.tacctlx(i)(7 downto 5) = "100") then
				v_ta_outx_nxt(i)	:=	v_ta_outx_mode4(i);
			elsif (r.tacctlx(i)(7 downto 5) = "101") then
				v_ta_outx_nxt(i)	:=	v_ta_outx_mode5(i);
			elsif (r.tacctlx(i)(7 downto 5) = "110") then
				v_ta_outx_nxt(i)	:=	v_ta_outx_mode6(i);
			else
				v_ta_outx_nxt(i)	:=	v_ta_outx_mode7(i);
			end if;

			if ( (r.tacctlx(i)(7 downto 5) = "001") and (v_ta_clr = '1') ) then
				v.ta_outx(i)	:=	'0';
			elsif (v_tar_clk = '1') then
				v.ta_outx(i)	:=	v_ta_outx_nxt(i);
			end if;

			v_ta_outx_en(i) := not(r.tacctlx(i)(TACAP));

		end loop;
		--============================================================================
		--! 4) DATA OUTPUT GENERATION
		--============================================================================
		--! Data output mux
		v_tactl_rd		:= word_per_select_dout( TACTL,		v_reg_rd, ("000000" & r.tactl)				);
		v_tar_rd			:= word_per_select_dout( TAR,			v_reg_rd, STD_LOGIC_VECTOR(r.tar)			);
		v_tacctl0_rd	:= word_per_select_dout( TACCTL0,	v_reg_rd, v_tacctlx_full(0)					);
		v_taccr0_rd		:= word_per_select_dout( TACCR0,		v_reg_rd,  STD_LOGIC_VECTOR(r.taccrx(0))	);
		v_tacctl1_rd	:= word_per_select_dout( TACCTL1,	v_reg_rd, v_tacctlx_full(1)					);
		v_taccr1_rd		:= word_per_select_dout( TACCR1,		v_reg_rd,  STD_LOGIC_VECTOR(r.taccrx(1))	);
		v_tacctl2_rd	:= word_per_select_dout( TACCTL2,	v_reg_rd, v_tacctlx_full(2)					);
		v_taccr2_rd		:= word_per_select_dout( TACCR2,		v_reg_rd,  STD_LOGIC_VECTOR(r.taccrx(2))	);
		v_taiv_rd		:= word_per_select_dout( TAIV,		v_reg_rd, (x"000" & v_taiv)					);

		v_per_dout	:=	v_tactl_rd   or
							v_tar_rd     or
							v_tacctl0_rd or
							v_taccr0_rd  or
							v_tacctl1_rd or
							v_taccr1_rd  or
							v_tacctl2_rd or
							v_taccr2_rd  or
							v_taiv_rd;


		--! drive register inputs
		rin <= v;

		--! drive module outputs
		cci			<= v_ccix;
		irq_ta0		<= v_irq_ta0;			--! Timer A interrupt: TACCR0
		irq_ta1		<= v_irq_ta1;			--! Timer A interrupt: TAIV, TACCR1, TACCR2
		per_dout		<= v_per_dout;			--! Peripheral data output
		ta_out0		<= r.ta_outx(0);		--! Timer A output 0
		ta_out0_en	<= v_ta_outx_en(0);	--! Timer A output 0 enable
		ta_out1		<= r.ta_outx(1);		--! Timer A output 1
		ta_out1_en	<= v_ta_outx_en(1);	--! Timer A output 1 enable
		ta_out2		<= r.ta_outx(2);		--! Timer A output 2
		ta_out2_en	<= v_ta_outx_en(2);	--! Timer A output 2 enable

	end process COMB;

	REGS : process (mclk,mrst)
	begin
		if (mrst = '1') then
			r	<=	(	tactl			=> "0000000000",
						tar			=> x"0000",
						taclk_dly	=> '0',
						inclk_dly	=> '0',
						clk_div		=> "000",
						tar_dir		=> '0',
						tacctlx		=> (Others =>  x"0000"),
						taccrx		=> (Others =>  x"0000"),
						sccix			=> "000",
						ccix_dly		=> "000",
						ccix_evt_s	=> "000",
						ccix_sync	=> "000",
						capx_taken	=> "000",
						ta_outx		=> "000"
					);
		elsif rising_edge(mclk) then
			r	<= rin;
		end if;
	end process REGS;

		
	
	--! CCIx synchronization
	sync_cell_cci0 : fmsp_sync_cell
	port map(
		clk		=> mclk,
		rst		=> mrst,
		data_in	=> cci(0),
		data_out	=> d.ccix_s(0)
	);
	sync_cell_cci1 : fmsp_sync_cell
	port map(
		clk		=> mclk,
		rst		=> mrst,
		data_in	=> cci(1),
		data_out	=> d.ccix_s(1)
	);
	sync_cell_cci2 : fmsp_sync_cell
	port map(
		clk		=> mclk,
		rst		=> mrst,
		data_in	=> cci(2),
		data_out	=> d.ccix_s(2)
	);
	--! Synchronization
	sync_cell_taclk : fmsp_sync_cell
	port map(
		clk		=> mclk,
		rst		=> mrst,
		data_in	=> taclk,
		data_out	=> d.taclk_s
	);
	sync_cell_inclk : fmsp_sync_cell
	port map(
		clk		=> mclk,
		rst		=> mrst,
		data_in	=> inclk,
		data_out	=> d.inclk_s
	);


end RTL; --! fmsp_timerA
