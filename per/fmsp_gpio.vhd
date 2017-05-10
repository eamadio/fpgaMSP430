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
--! @file fmsp_gpio.vhd
--! 
--! @brief fpgaMSP430 constant	Digital I/O interface
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
	use work.fmsp_misc_package.all;
	use work.fmsp_per_package.all;
	use work.fmsp_functions.all;

entity fmsp_gpio is 
	generic (
		P1_EN		: boolean := true;	--! Enable Port 1
		P2_EN		: boolean := true;	--! Enable Port 2
		P3_EN		: boolean := false;	--! Enable Port 3
		P4_EN		: boolean := false;	--! Enable Port 4
		P5_EN		: boolean := false;	--! Enable Port 5
		P6_EN		: boolean := false;	--! Enable Port 6
		SYNC_P1	: boolean := true;	--! Synchronize Port 1 inputs
		SYNC_P2	: boolean := true;	--! Synchronize Port 2 inputs
		SYNC_P3	: boolean := true;	--! Synchronize Port 3 inputs
		SYNC_P4	: boolean := true;	--! Synchronize Port 4 inputs
		SYNC_P5	: boolean := true;	--! Synchronize Port 5 inputs
		SYNC_P6	: boolean := true		--! Synchronize Port 6 inputs
	);
	port (
		mclk			: in	std_logic;       						--! Main system clock
		mrst			: in	std_logic;       						--! Main system reset
		--! INPUTs
		p1_din		: in	std_logic_vector(7 downto 0);		--! Port 1 data input
		p2_din		: in	std_logic_vector(7 downto 0);		--! Port 2 data input
		p3_din		: in	std_logic_vector(7 downto 0);		--! Port 3 data input
		p4_din		: in	std_logic_vector(7 downto 0);		--! Port 4 data input
		p5_din		: in	std_logic_vector(7 downto 0);		--! Port 5 data input
		p6_din		: in	std_logic_vector(7 downto 0);		--! Port 6 data input
		per_addr		: in	std_logic_vector(13 downto 0);	--! Peripheral address
		per_din		: in	std_logic_vector(15 downto 0);	--! Peripheral data input
		per_en		: in	std_logic;       						--! Peripheral enable (high active)
		per_we		: in	std_logic_vector(1 downto 0);		--! Peripheral write enable (high active)
		--! OUTPUTs
		irq_port1	: out	std_logic;       						--! Port 1 interrupt
		irq_port2	: out	std_logic;       						--! Port 2 interrupt
		p1_dout		: out	std_logic_vector(7 downto 0);		--! Port 1 data output
		p1_dout_en	: out	std_logic_vector(7 downto 0);		--! Port 1 data output enable
		p1_sel		: out	std_logic_vector(7 downto 0);		--! Port 1 function select
		p2_dout		: out	std_logic_vector(7 downto 0);		--! Port 2 data output
		p2_dout_en	: out	std_logic_vector(7 downto 0);		--! Port 2 data output enable
		p2_sel		: out	std_logic_vector(7 downto 0);		--! Port 2 function select
		p3_dout		: out	std_logic_vector(7 downto 0);		--! Port 3 data output
		p3_dout_en	: out	std_logic_vector(7 downto 0);		--! Port 3 data output enable
		p3_sel		: out	std_logic_vector(7 downto 0);		--! Port 3 function select
		p4_dout		: out	std_logic_vector(7 downto 0);		--! Port 4 data output
		p4_dout_en	: out	std_logic_vector(7 downto 0);		--! Port 4 data output enable
		p4_sel		: out	std_logic_vector(7 downto 0);		--! Port 4 function select
		p5_dout		: out	std_logic_vector(7 downto 0);		--! Port 5 data output
		p5_dout_en	: out	std_logic_vector(7 downto 0);		--! Port 5 data output enable
		p5_sel		: out	std_logic_vector(7 downto 0);		--! Port 5 function select
		p6_dout		: out	std_logic_vector(7 downto 0);		--! Port 6 data output
		p6_dout_en	: out	std_logic_vector(7 downto 0);		--! Port 6 data output enable
		p6_sel		: out	std_logic_vector(7 downto 0);		--! Port 6 function select
		per_dout		: out	std_logic_vector(15 downto 0)		--! Peripheral data output
	);
end entity fmsp_gpio;

architecture RTL of fmsp_gpio is 

	--=============================================================================
	--! 1)  PARAMETER DECLARATION
	--=============================================================================

	--! Register base address (must be aligned to decoder bit width)
	constant	BASE_ADDR		: std_logic_vector(14 downto 0) := "000000000000000";
	--! Decoder bit width (defines how many bits are considered for address decoding)
	constant	DEC_WD			: integer := 7;
	--! Register addresses offset
	constant	LED_CTRL			: integer := 0;	--! ''h0,
	constant	P1IN				: integer := 32;	--! ''h20,                    --! Port 1
	constant	P1OUT				: integer := 33;	--! ''h21,
	constant	P1DIR				: integer := 34;	--! ''h22,
	constant	P1IFG				: integer := 35;	--! ''h23,
	constant	P1IES				: integer := 36;	--! ''h24,
	constant	P1IE				: integer := 37;	--! ''h25,
	constant	P1SEL				: integer := 38;	--! ''h26,
	constant	P2IN				: integer := 40;	--! ''h28,                    --! Port 2
	constant	P2OUT				: integer := 41;	--! ''h29,
	constant	P2DIR				: integer := 42;	--! ''h2A,
	constant	P2IFG				: integer := 43;	--! ''h2B,
	constant	P2IES				: integer := 44;	--! ''h2C,
	constant	P2IE				: integer := 45;	--! ''h2D,
	constant	P2SEL				: integer := 46;	--! ''h2E,
	constant	P3IN				: integer := 24;	--! ''h18,                    --! Port 3
	constant	P3OUT				: integer := 25;	--! ''h19,
	constant	P3DIR				: integer := 26;	--! ''h1A,
	constant	P3SEL				: integer := 27;	--! ''h1B,
	constant	P4IN				: integer := 28;	--! ''h1C,                    --! Port 4
	constant	P4OUT				: integer := 29;	--! ''h1D,
	constant	P4DIR				: integer := 30;	--! ''h1E,
	constant	P4SEL				: integer := 31;	--! ''h1F,
	constant	P5IN				: integer := 48;	--! ''h30,                    --! Port 5
	constant	P5OUT				: integer := 49;	--! ''h31,
	constant	P5DIR				: integer := 50;	--! ''h32,
	constant	P5SEL				: integer := 51;	--! ''h33,
	constant	P6IN				: integer := 52;	--! ''h34,                    --! Port 6
	constant	P6OUT				: integer := 53;	--! ''h35,
	constant	P6DIR				: integer := 54;	--! ''h36,
	constant	P6SEL				: integer := 55;	--! ''h37;
	--! Register one-hot decoder utilities
	constant	DEC_SZ  			: integer :=  (2**DEC_WD);

	type fmsp_gpio_in_type is record
		p1_din	: std_logic_vector(7 downto 0);	--! Port 1 data input
		p2_din	: std_logic_vector(7 downto 0);	--! Port 2 data input
		p3_din	: std_logic_vector(7 downto 0);	--! Port 3 data input
		p4_din	: std_logic_vector(7 downto 0);	--! Port 4 data input
		p5_din	: std_logic_vector(7 downto 0);	--! Port 5 data input
		p6_din	: std_logic_vector(7 downto 0);	--! Port 6 data input
		per_addr	: std_logic_vector(13 downto 0);	--! Peripheral address
		per_din	: std_logic_vector(15 downto 0);	--! Peripheral data input
		per_en	: std_logic;       					--! Peripheral enable (high active)
		per_we	: std_logic_vector(1 downto 0);	--! Peripheral write enable (high active)
		--! From sub modules
		p1in		: std_logic_vector(7 downto 0);	--! Port 1 data input
		p2in		: std_logic_vector(7 downto 0);	--! Port 2 data input
		p3in		: std_logic_vector(7 downto 0);	--! Port 3 data input
		p4in		: std_logic_vector(7 downto 0);	--! Port 4 data input
		p5in		: std_logic_vector(7 downto 0);	--! Port 5 data input
		p6in		: std_logic_vector(7 downto 0);	--! Port 6 data input
	end record;

	type reg_type is record
		p1out		: std_logic_vector(7 downto 0);
		p1dir		: std_logic_vector(7 downto 0);
		p1ifg		: std_logic_vector(7 downto 0);
		p1ies		: std_logic_vector(7 downto 0);
		p1ie		: std_logic_vector(7 downto 0);
		p1sel		: std_logic_vector(7 downto 0);
		p2out		: std_logic_vector(7 downto 0);
		p2dir		: std_logic_vector(7 downto 0);
		p2ifg		: std_logic_vector(7 downto 0);
		p2ies		: std_logic_vector(7 downto 0);
		p2ie		: std_logic_vector(7 downto 0);
		p2sel		: std_logic_vector(7 downto 0);
		p3out		: std_logic_vector(7 downto 0);
		p3dir		: std_logic_vector(7 downto 0);
		p3sel		: std_logic_vector(7 downto 0);
		p4out		: std_logic_vector(7 downto 0);
		p4dir		: std_logic_vector(7 downto 0);
		p4sel		: std_logic_vector(7 downto 0);
		p5out		: std_logic_vector(7 downto 0);
		p5dir		: std_logic_vector(7 downto 0);
		p5sel		: std_logic_vector(7 downto 0);
		p6out		: std_logic_vector(7 downto 0);
		p6dir		: std_logic_vector(7 downto 0);
		p6sel		: std_logic_vector(7 downto 0);
		p1in_dly	: std_logic_vector(7 downto 0);
		p2in_dly	: std_logic_vector(7 downto 0);
	end record;

	signal	d		: fmsp_gpio_in_type;
	signal	r		: reg_type :=	(	p1out		=> "00000000",
												p1dir		=> "00000000",
												p1ifg		=> "00000000",
												p1ies		=> "00000000",
												p1ie		=> "00000000",
												p1sel		=> "00000000",
												p2out		=> "00000000",
												p2dir		=> "00000000",
												p2ifg		=> "00000000",
												p2ies		=> "00000000",
												p2ie		=> "00000000",
												p2sel		=> "00000000",
												p3out		=> "00000000",
												p3dir		=> "00000000",
												p3sel		=> "00000000",
												p4out		=> "00000000",
												p4dir		=> "00000000",
												p4sel		=> "00000000",
												p5out		=> "00000000",
												p5dir		=> "00000000",
												p5sel		=> "00000000",
												p6out		=> "00000000",
												p6dir		=> "00000000",
												p6sel		=> "00000000",
												p1in_dly	=> "00000000",
												p2in_dly	=> "00000000"
											);
	signal	rin	: reg_type;

begin

		d.p1_din		<=	p1_din;
		d.p2_din		<=	p2_din;
		d.p3_din		<=	p3_din;
		d.p4_din		<=	p4_din;
		d.p5_din		<=	p5_din;
		d.p6_din		<=	p6_din;
		d.per_addr	<=	per_addr;
		d.per_din	<=	per_din;
		d.per_en		<=	per_en;
		d.per_we		<=	per_we;

	COMB : process (d, r)
		variable	v					: reg_type;
		--! Local register selection
		variable	v_reg_sel		: std_logic;
		--! Register local address
		variable	v_reg_addr		: std_logic_vector(DEC_WD-2 downto 0);
		--! Register address decode
		variable	v_reg_dec		: std_logic_vector((DEC_SZ/2)-1 downto 0);
		--! Read/Write probes
		variable	v_reg_lo_write	: std_logic;
		variable	v_reg_hi_write	: std_logic;
		variable	v_reg_read		: std_logic;
		--! Read/Write vectors
		variable	v_reg_wr			: std_logic_vector(DEC_SZ-1 downto 0);
		variable	v_reg_rd			: std_logic_vector(DEC_SZ-1 downto 0);
		--! P1OUT Register
		variable	v_p1in			: std_logic_vector(7 downto 0);
		variable	v_p1out_wr		: std_logic;
		variable	v_p1out_nxt		: std_logic_vector(7 downto 0);
		--! P1DIR Register
		variable	v_p1dir_wr		: std_logic;
		variable	v_p1dir_nxt		: std_logic_vector(7 downto 0);
		variable	v_p1_dout_en	: std_logic_vector(7 downto 0);
		--! P1IFG Register
		variable	v_p1ifg_wr		: std_logic;
		variable	v_p1ifg_nxt		: std_logic_vector(7 downto 0);
		variable	v_p1ifg_set		: std_logic_vector(7 downto 0);
		--! P1IES Register
		variable	v_p1ies_wr		: std_logic;
		variable	v_p1ies_nxt		: std_logic_vector(7 downto 0);
		--! P1IE Register
		variable	v_p1ie_wr		: std_logic;
		variable	v_p1ie_nxt		: std_logic_vector(7 downto 0);
		--! P1SEL Register
		variable	v_p1sel_wr		: std_logic;
		variable	v_p1sel_nxt		: std_logic_vector(7 downto 0);
		variable	v_p1_sel			: std_logic_vector(7 downto 0);
		--! P2IN Register
		variable	v_p2in			: std_logic_vector(7 downto 0);
		--! P2OUT Register
		variable	v_p2out_wr		: std_logic;
		variable	v_p2out_nxt		: std_logic_vector(7 downto 0);
		--! P2DIR Register
		variable	v_p2dir_wr		: std_logic;
		variable	v_p2dir_nxt		: std_logic_vector(7 downto 0);
		variable	v_p2_dout_en	: std_logic_vector(7 downto 0);
		--! P2IFG Register
		variable	v_p2ifg_wr		: std_logic;
		variable	v_p2ifg_nxt		: std_logic_vector(7 downto 0);
		variable	v_p2ifg_set		: std_logic_vector(7 downto 0);
		--! P2IES Register
		variable	v_p2ies_wr		: std_logic;
		variable	v_p2ies_nxt		: std_logic_vector(7 downto 0);
		--! P2IE Register
		variable	v_p2ie_wr		: std_logic;
		variable	v_p2ie_nxt		: std_logic_vector(7 downto 0);
		--! P2SEL Register
		variable	v_p2sel_wr		: std_logic;
		variable	v_p2sel_nxt		: std_logic_vector(7 downto 0);
		variable	v_p2_sel			: std_logic_vector(7 downto 0);
		--! P3OUT Register
		variable	v_p3in			: std_logic_vector(7 downto 0);
		variable	v_p3out_wr		: std_logic;
		variable	v_p3out_nxt		: std_logic_vector(7 downto 0);
		--! P3DIR Register
		variable	v_p3dir_wr		: std_logic;
		variable	v_p3dir_nxt		: std_logic_vector(7 downto 0);
		variable	v_p3_dout_en	: std_logic;
		--! P3SEL Register
		variable	v_p3sel_wr		: std_logic;
		variable	v_p3sel_nxt		: std_logic_vector(7 downto 0);
		variable	v_p3_sel			: std_logic_vector(7 downto 0);
		--! P4OUT Register
		variable	v_p4in			: std_logic_vector(7 downto 0);
		variable	v_p4out_wr		: std_logic;
		variable	v_p4out_nxt		: std_logic_vector(7 downto 0);
		--! P4DIR Register
		variable	v_p4dir_wr		: std_logic;
		variable	v_p4dir_nxt		: std_logic_vector(7 downto 0);
		variable	v_p4_dout_en	: std_logic_vector(7 downto 0);
		--! P4SEL Register
		variable	v_p4sel_wr		: std_logic;
		variable	v_p4sel_nxt		: std_logic_vector(7 downto 0);
		variable	v_p4_sel			: std_logic_vector(7 downto 0);
		--! P5OUT Register
		variable	v_p5in			: std_logic_vector(7 downto 0);
		variable	v_p5out_wr		: std_logic;
		variable	v_p5out_nxt		: std_logic_vector(7 downto 0);
		--! P5DIR Register
		variable	v_p5dir_wr		: std_logic;
		variable	v_p5dir_nxt		: std_logic_vector(7 downto 0);
		variable	v_p5_dout_en	: std_logic;
		--! P5SEL Register
		variable	v_p5sel_wr		: std_logic;
		variable	v_p5sel_nxt		: std_logic_vector(7 downto 0);
		variable	v_p5_sel			: std_logic_vector(7 downto 0);
		--! P6OUT Register
		variable	v_p6in			: std_logic_vector(7 downto 0);
		variable	v_p6out_wr		: std_logic;
		variable	v_p6out_nxt		: std_logic_vector(7 downto 0);
		--! P6DIR Register
		variable	v_p6dir_wr		: std_logic;
		variable	v_p6dir_nxt		: std_logic_vector(7 downto 0);
		variable	v_p6_dout_en	: std_logic_vector(7 downto 0);
		--! P6SEL Register
		variable	v_p6sel_wr		: std_logic;
		variable	v_p6sel_nxt		: std_logic_vector(7 downto 0);
		variable	v_p6_sel			: std_logic_vector(7 downto 0);
		--! 4) INTERRUPT GENERATION
		--! Port 1 interrupt
		--! Edge detection
		variable	v_p1in_re		: std_logic_vector(7 downto 0);
		variable	v_p1in_fe		: std_logic_vector(7 downto 0);
		--! Set interrupt flag
		--variable	v_p1ifg_set		: std_logic;
		--! Generate CPU interrupt
		variable	v_irq_port1		: std_logic;
		--! Port 1 interrupt
		--------------------
		--! Delay input
		--! Edge detection
		variable	v_p2in_re		: std_logic_vector(7 downto 0);
		variable	v_p2in_fe		: std_logic_vector(7 downto 0);
		--! Set interrupt flag
		--variable	v_p2ifg_set		: std_logic;
		--! Generate CPU interrupt
		variable	v_irq_port2		: std_logic;
		--! Data output mux
		variable	v_p1in_rd		: std_logic_vector(15 downto 0);
		variable	v_p1out_rd		: std_logic_vector(15 downto 0);
		variable	v_p1dir_rd		: std_logic_vector(15 downto 0);
		variable	v_p1ifg_rd		: std_logic_vector(15 downto 0);
		variable	v_p1ies_rd		: std_logic_vector(15 downto 0);
		variable	v_p1ie_rd		: std_logic_vector(15 downto 0);
		variable	v_p1sel_rd		: std_logic_vector(15 downto 0);
		variable	v_p2in_rd		: std_logic_vector(15 downto 0);
		variable	v_p2out_rd		: std_logic_vector(15 downto 0);
		variable	v_p2dir_rd		: std_logic_vector(15 downto 0);
		variable	v_p2ifg_rd		: std_logic_vector(15 downto 0);
		variable	v_p2ies_rd		: std_logic_vector(15 downto 0);
		variable	v_p2ie_rd		: std_logic_vector(15 downto 0);
		variable	v_p2sel_rd		: std_logic_vector(15 downto 0);
		variable	v_p3in_rd		: std_logic_vector(15 downto 0);
		variable	v_p3out_rd		: std_logic_vector(15 downto 0);
		variable	v_p3dir_rd		: std_logic_vector(15 downto 0);
		variable	v_p3sel_rd		: std_logic_vector(15 downto 0);
		variable	v_p4in_rd		: std_logic_vector(15 downto 0);
		variable	v_p4out_rd		: std_logic_vector(15 downto 0);
		variable	v_p4dir_rd		: std_logic_vector(15 downto 0);
		variable	v_p4sel_rd		: std_logic_vector(15 downto 0);
		variable	v_p5in_rd		: std_logic_vector(15 downto 0);
		variable	v_p5out_rd		: std_logic_vector(15 downto 0);
		variable	v_p5dir_rd		: std_logic_vector(15 downto 0);
		variable	v_p5sel_rd		: std_logic_vector(15 downto 0);
		variable	v_p6in_rd		: std_logic_vector(15 downto 0);
		variable	v_p6out_rd		: std_logic_vector(15 downto 0);
		variable	v_p6dir_rd		: std_logic_vector(15 downto 0);
		variable	v_p6sel_rd		: std_logic_vector(15 downto 0);
		variable	v_per_dout		: std_logic_vector(15 downto 0);
	begin
		--! default assignment
		v := r;
		--! overriding assignments

		--============================================================================
		--! 2)  REGISTER DECODER
		--============================================================================

		--! Local register selection
		if ( d.per_addr(13 downto DEC_WD-1) = BASE_ADDR(14 downto DEC_WD) ) then
			v_reg_sel	:=	d.per_en;
		else
			v_reg_sel	:=	'0';
		end if;
		--! Register local address
		v_reg_addr	:=	d.per_addr(DEC_WD-2 downto 0);
		--! Register address decode
		v_reg_dec := onehot(v_reg_addr);
		--! Read/Write probes
		v_reg_lo_write	:=	v_reg_sel and d.per_we(0);
		v_reg_hi_write	:= v_reg_sel and d.per_we(1);
		v_reg_read		:= v_reg_sel and not(d.per_we(0) or d.per_we(1));
		--! Read/Write vectors
		for i in 0 to (DEC_SZ/2)-1 loop
			v_reg_wr((i*2)+0)	:= v_reg_dec(i) and v_reg_lo_write;
			v_reg_wr((i*2)+1)	:= v_reg_dec(i) and v_reg_hi_write;
			v_reg_rd((i*2)+0)	:= v_reg_dec(i) and v_reg_read;
			v_reg_rd((i*2)+1)	:= v_reg_dec(i) and v_reg_read;
		end loop;

		--============================================================================
		--! 4) INTERRUPT GENERATION
		--============================================================================

		--! Port 1 interrupt

		if (P1_EN	= false) then
			v_p1in	:= "00000000";
		elsif (SYNC_P1	= true) then
			v_p1in	:= d.p1in;
		else
			v_p1in	:= d.p1_din;
		end if;

		--! Delay input
		v.p1in_dly	:= v_p1in;

		--! Edge detection
		v_p1in_re	:=		 v_p1in	and not(	r.p1in_dly);
		v_p1in_fe	:= not(v_p1in)	and		r.p1in_dly;

		--! Set interrupt flag
		for i in 0 to 7 loop
			if (P1_EN	= false) then
				v_p1ifg_set(i)	:=	'0';
			elsif (r.p1ies(i) = '1') then
				v_p1ifg_set(i)	:=	v_p1in_fe(i);
			else
				v_p1ifg_set(i)	:=	v_p1in_re(i);
			end if;
		end loop;
		
		--! Generate CPU interrupt
		if (P1_EN	= false) then
			v_irq_port1	:=	'0';
		elsif ( (r.p1ie and r.p1ifg) /= "00000000") then
			v_irq_port1	:=	'1';
		else
			v_irq_port1	:=	'0';
		end if;


		--! Port 2 interrupt

		if (P2_EN	= false) then
			v_p2in	:= "00000000";
		elsif (SYNC_P2	= true) then
			v_p2in	:= d.p2in;
		else
			v_p2in	:= d.p2_din;
		end if;

		--! Delay input
		v.p2in_dly	:= v_p2in;

		--! Edge detection
		v_p2in_re	:=		 d.p2in	and not(	r.p2in_dly);
		v_p2in_fe	:= not(d.p2in)	and		r.p2in_dly;

		--! Set interrupt flag
		for i in 0 to 7 loop
			if (P2_EN	= false) then
				v_p2ifg_set(i)	:=	'0';
			elsif (r.p2ies(i) = '1') then
				v_p2ifg_set(i)	:=	v_p2in_fe(i);
			else
				v_p2ifg_set(i)	:=	v_p2in_re(i);
			end if;
		end loop;
		
		--! Generate CPU interrupt
		if (P2_EN	= false) then
			v_irq_port2	:=	'0';
		elsif ( (r.p2ie and r.p2ifg) /= "00000000") then
			v_irq_port2	:=	'1';
		else
			v_irq_port2	:=	'0';
		end if;


		--============================================================================
		--! 3) REGISTERS
		--============================================================================

		--! P1IFG Register
		v_p1ifg_wr	:= v_reg_wr(P1IFG);
		v_p1ifg_nxt	:= byte_per_select_din( P1IFG, d.per_din );
		if (P1_EN	= false) then
			v.p1ifg	:= "00000000";
		elsif (v_p1ies_wr = '1') then
			v.p1ifg	:= v_p1ifg_nxt or v_p1ifg_set;
		else
			v.p1ifg	:= r.p1ifg or v_p1ifg_set;
		end if;

		--! P1IES Register
		v_p1ies_wr	:= v_reg_wr(P1IES);
		v_p1ies_nxt	:= byte_per_select_din( P1IES, d.per_din );

		if (P1_EN	= false) then
			v.p1ies	:= "00000000";
		elsif (v_p1ies_wr = '1') then
			v.p1ies	:= v_p1ies_nxt;
		end if;

		--! P1IE Register
		v_p1ie_wr	:= v_reg_wr(P1IE);
		v_p1ie_nxt	:= byte_per_select_din( P1IE, d.per_din );

		if (P1_EN	= false) then
			v.p1ie	:= "00000000";
		elsif (v_p1ie_wr = '1') then
			v.p1ie	:= v_p1ie_nxt;
		end if;

	
		--! P2IFG Register
		v_p2ifg_wr	:= v_reg_wr(P2IFG);
		v_p2ifg_nxt	:= byte_per_select_din( P2IFG, d.per_din );
		if (P2_EN	= false) then
			v.p2ifg	:= "00000000";
		elsif (v_p2ies_wr = '1') then
			v.p2ifg	:= v_p2ifg_nxt or v_p2ifg_set;
		else
			v.p2ifg	:= r.p2ifg or v_p2ifg_set;
		end if;

		--! P2IES Register
		v_p2ies_wr	:= v_reg_wr(P2IES);
		v_p2ies_nxt	:= byte_per_select_din( P2IES, d.per_din );

		if (P2_EN	= false) then
			v.p2ies	:= "00000000";
		elsif (v_p2ies_wr = '1') then
			v.p2ies	:= v_p2ies_nxt;
		end if;

		--! P2IE Register
		v_p2ie_wr	:= v_reg_wr(P2IE);
		v_p2ie_nxt	:= byte_per_select_din( P2IE, d.per_din );

		if (P2_EN	= false) then
			v.p2ie	:= "00000000";
		elsif (v_p2ie_wr = '1') then
			v.p2ie	:= v_p2ie_nxt;
		end if;

  
		--! P1OUT Register
		v_p1out_wr	:= v_reg_wr(P1OUT);
		v_p1out_nxt	:= byte_per_select_din( P1OUT, d.per_din );

		if (P1_EN	= false) then
			v.p1out	:= "00000000";
		elsif (v_p1out_wr = '1') then
			v.p1out	:= v_p1out_nxt;
		end if;

		--! P1DIR Register
		v_p1dir_wr	:= v_reg_wr(P1DIR);
		v_p1dir_nxt	:= byte_per_select_din( P1DIR, d.per_din );

		if (P1_EN	= false) then
			v.p1dir	:= "00000000";
		elsif (v_p1dir_wr = '1') then
			v.p1dir	:= v_p1dir_nxt;
		end if;

		--! P1SEL Register
		v_p1sel_wr	:= v_reg_wr(P1SEL);
		v_p1sel_nxt	:= byte_per_select_din( P1SEL, d.per_din );

		if (P1_EN	= false) then
			v.p1sel	:= "00000000";
		elsif (v_p1sel_wr = '1') then
			v.p1sel	:= v_p1sel_nxt;
		end if;

		--! P2OUT Register
		v_p2out_wr	:= v_reg_wr(P2OUT);
		v_p2out_nxt	:= byte_per_select_din( P2OUT, d.per_din );

		if (P2_EN	= false) then
			v.p2out	:= "00000000";
		elsif (v_p2out_wr = '1') then
			v.p2out	:= v_p2out_nxt;
		end if;

		--! P2DIR Register
		v_p2dir_wr	:= v_reg_wr(P2DIR);
		v_p2dir_nxt	:= byte_per_select_din( P2DIR, d.per_din );

		if (P2_EN	= false) then
			v.p2dir	:= "00000000";
		elsif (v_p2dir_wr = '1') then
			v.p2dir	:= v_p2dir_nxt;
		end if;

		--! P2SEL Register
		v_p2sel_wr	:= v_reg_wr(P2SEL);
		v_p2sel_nxt	:= byte_per_select_din( P2SEL, d.per_din );

		if (P2_EN	= false) then
			v.p2sel	:= "00000000";
		elsif (v_p2sel_wr = '1') then
			v.p2sel	:= v_p2sel_nxt;
		end if;

		--! P3IN Register
		if (P3_EN	= false) then
			v_p3in	:= "00000000";
		elsif (SYNC_P3	= true) then
			v_p3in	:= d.p3in;
		else
			v_p3in	:= d.p3_din;
		end if;

		--! P3OUT Register
		v_p3out_wr	:= v_reg_wr(P3OUT);
		v_p3out_nxt	:= byte_per_select_din( P3OUT, d.per_din );

		if (P3_EN	= false) then
			v.p3out	:= "00000000";
		elsif (v_p3out_wr = '1') then
			v.p3out	:= v_p3out_nxt;
		end if;

		--! P3DIR Register
		v_p3dir_wr	:= v_reg_wr(P3DIR);
		v_p3dir_nxt	:= byte_per_select_din( P3DIR, d.per_din );

		if (P3_EN	= false) then
			v.p3dir	:= "00000000";
		elsif (v_p3dir_wr = '1') then
			v.p3dir	:= v_p3dir_nxt;
		end if;

		--! P3SEL Register
		v_p3sel_wr	:= v_reg_wr(P3SEL);
		v_p3sel_nxt	:= byte_per_select_din( P3SEL, d.per_din );

		if (P3_EN	= false) then
			v.p3sel	:= "00000000";
		elsif (v_p3sel_wr = '1') then
			v.p3sel	:= v_p3sel_nxt;
		end if;

		--! P4IN Register
		if (P4_EN	= false) then
			v_p4in	:= "00000000";
		elsif (SYNC_P4	= true) then
			v_p4in	:= d.p4in;
		else
			v_p4in	:= d.p4_din;
		end if;

		--! P4OUT Register
		v_p4out_wr	:= v_reg_wr(P4OUT);
		v_p4out_nxt	:= byte_per_select_din( P4OUT, d.per_din );

		if (P4_EN	= false) then
			v.p4out	:= "00000000";
		elsif (v_p4out_wr = '1') then
			v.p4out	:= v_p4out_nxt;
		end if;

		--! P4DIR Register
		v_p4dir_wr	:= v_reg_wr(P4DIR);
		v_p4dir_nxt	:= byte_per_select_din( P4DIR, d.per_din );

		if (P4_EN	= false) then
			v.p4dir	:= "00000000";
		elsif (v_p4dir_wr = '1') then
			v.p4dir	:= v_p4dir_nxt;
		end if;

		--! P4SEL Register
		v_p4sel_wr	:= v_reg_wr(P4SEL);
		v_p4sel_nxt	:= byte_per_select_din( P4SEL, d.per_din );

		if (P4_EN	= false) then
			v.p4sel	:= "00000000";
		elsif (v_p4sel_wr = '1') then
			v.p4sel	:= v_p4sel_nxt;
		end if;

		--! P5IN Register
		if (P5_EN	= false) then
			v_p5in	:= "00000000";
		elsif (SYNC_P5	= true) then
			v_p5in	:= d.p5in;
		else
			v_p5in	:= d.p5_din;
		end if;

		--! P5OUT Register
		v_p5out_wr	:= v_reg_wr(P5OUT);
		v_p5out_nxt	:= byte_per_select_din( P5OUT, d.per_din );

		if (P5_EN	= false) then
			v.p5out	:= "00000000";
		elsif (v_p5out_wr = '1') then
			v.p5out	:= v_p5out_nxt;
		end if;

		--! P5DIR Register
		v_p5dir_wr	:= v_reg_wr(P5DIR);
		v_p5dir_nxt	:= byte_per_select_din( P5DIR, d.per_din );

		if (P5_EN	= false) then
			v.p5dir	:= "00000000";
		elsif (v_p5dir_wr = '1') then
			v.p5dir	:= v_p5dir_nxt;
		end if;

		--! P5SEL Register
		v_p5sel_wr	:= v_reg_wr(P5SEL);
		v_p5sel_nxt	:= byte_per_select_din( P5SEL, d.per_din );

		if (P5_EN	= false) then
			v.p5sel	:= "00000000";
		elsif (v_p5sel_wr = '1') then
			v.p5sel	:= v_p5sel_nxt;
		end if;

		--! P6IN Register
		if (P6_EN	= false) then
			v_p6in	:= "00000000";
		elsif (SYNC_P6	= true) then
			v_p6in	:= d.p6in;
		else
			v_p6in	:= d.p6_din;
		end if;

		--! P6OUT Register
		v_p6out_wr	:= v_reg_wr(P6OUT);
		v_p6out_nxt	:= byte_per_select_din( P6OUT, d.per_din );

		if (P6_EN	= false) then
			v.p6out	:= "00000000";
		elsif (v_p6out_wr = '1') then
			v.p6out	:= v_p6out_nxt;
		end if;

		--! P6DIR Register
		v_p6dir_wr	:= v_reg_wr(P6DIR);
		v_p6dir_nxt	:= byte_per_select_din( P6DIR, d.per_din );

		if (P6_EN	= false) then
			v.p6dir	:= "00000000";
		elsif (v_p6dir_wr = '1') then
			v.p6dir	:= v_p6dir_nxt;
		end if;

		--! P6SEL Register
		v_p6sel_wr	:= v_reg_wr(P6SEL);
		v_p6sel_nxt	:= byte_per_select_din( P6SEL, d.per_din );

		if (P6_EN	= false) then
			v.p6sel	:= "00000000";
		elsif (v_p6sel_wr = '1') then
			v.p6sel	:= v_p6sel_nxt;
		end if;

		--============================================================================
		--! 5) DATA OUTPUT GENERATION
		--============================================================================

		--! Data output mux
		v_p1in_rd	:= byte_per_select_dout( P1IN,	v_reg_rd, v_p1in		);
		v_p1out_rd	:= byte_per_select_dout( P1OUT,	v_reg_rd, r.p1out		);
		v_p1dir_rd	:= byte_per_select_dout( P1DIR,	v_reg_rd, r.p1dir		);
		v_p1ifg_rd	:= byte_per_select_dout( P1IFG,	v_reg_rd, r.p1ifg		);
		v_p1ies_rd	:= byte_per_select_dout( P1IES,	v_reg_rd, r.p1ies		);
		v_p1ie_rd	:= byte_per_select_dout( P1IE,	v_reg_rd, r.p1ie		);
		v_p1sel_rd	:= byte_per_select_dout( P1SEL,	v_reg_rd, r.p1sel		);
		v_p2in_rd	:= byte_per_select_dout( P2IN,	v_reg_rd, v_p2in		);
		v_p2out_rd	:= byte_per_select_dout( P2OUT,	v_reg_rd, r.p2out		);
		v_p2dir_rd	:= byte_per_select_dout( P2DIR,	v_reg_rd, r.p2dir		);
		v_p2ifg_rd	:= byte_per_select_dout( P2IFG,	v_reg_rd, r.p2ifg		);
		v_p2ies_rd	:= byte_per_select_dout( P2IES,	v_reg_rd, r.p2ies		);
		v_p2ie_rd	:= byte_per_select_dout( P2IE,	v_reg_rd, r.p2ie		);
		v_p2sel_rd	:= byte_per_select_dout( P2SEL,	v_reg_rd, r.p2sel		);
		v_p3in_rd	:= byte_per_select_dout( P3IN,	v_reg_rd, v_p3in		);
		v_p3out_rd	:= byte_per_select_dout( P3OUT,	v_reg_rd, r.p3out		);
		v_p3dir_rd	:= byte_per_select_dout( P3DIR,	v_reg_rd, r.p3dir		);
		v_p3sel_rd	:= byte_per_select_dout( P3SEL,	v_reg_rd, r.p3sel		);
		v_p4in_rd	:= byte_per_select_dout( P4IN,	v_reg_rd, v_p4in		);
		v_p4out_rd	:= byte_per_select_dout( P4OUT,	v_reg_rd, r.p4out		);
		v_p4dir_rd	:= byte_per_select_dout( P4DIR,	v_reg_rd, r.p4dir		);
		v_p4sel_rd	:= byte_per_select_dout( P4SEL,	v_reg_rd, r.p4sel		);
		v_p5in_rd	:= byte_per_select_dout( P5IN,	v_reg_rd, v_p5in		);
		v_p5out_rd	:= byte_per_select_dout( P5OUT,	v_reg_rd, r.p5out		);
		v_p5dir_rd	:= byte_per_select_dout( P5DIR,	v_reg_rd, r.p5dir		);
		v_p5sel_rd	:= byte_per_select_dout( P5SEL,	v_reg_rd, r.p5sel		);
		v_p6in_rd	:= byte_per_select_dout( P6IN,	v_reg_rd, v_p6in		);
		v_p6out_rd	:= byte_per_select_dout( P6OUT,	v_reg_rd, r.p6out		);
		v_p6dir_rd	:= byte_per_select_dout( P6DIR,	v_reg_rd, r.p6dir		);
		v_p6sel_rd	:= byte_per_select_dout( P6SEL,	v_reg_rd, r.p6sel		);

		v_per_dout	:=		v_p1in_rd
							or v_p1out_rd
							or v_p1dir_rd
							or v_p1ifg_rd
							or v_p1ies_rd
							or v_p1ie_rd 
							or v_p1sel_rd
							or v_p2in_rd 
							or v_p2out_rd
							or v_p2dir_rd
							or v_p2ifg_rd
							or v_p2ies_rd
							or v_p2ie_rd 
							or v_p2sel_rd
							or v_p3in_rd 
							or v_p3out_rd
							or v_p3dir_rd
							or v_p3sel_rd
							or v_p4in_rd 
							or v_p4out_rd
							or v_p4dir_rd
							or v_p4sel_rd
							or v_p5in_rd 
							or v_p5out_rd
							or v_p5dir_rd
							or v_p5sel_rd
							or v_p6in_rd 
							or v_p6out_rd
							or v_p6dir_rd
							or v_p6sel_rd;

		--! drive register inputs
		rin <= v;
		--! drive module outputs
		irq_port1	<=	v_irq_port1;	--! Port 1 interrupt
		irq_port2	<=	v_irq_port2;	--! Port 2 interrupt
		p1_dout		<=	r.p1out;			--! Port 1 data output
		p1_dout_en	<=	r.p1dir;			--! Port 1 data output enable
		p1_sel		<=	r.p1sel;			--! Port 1 function select
		p2_dout		<=	r.p2out;			--! Port 2 data output
		p2_dout_en	<=	r.p2dir;			--! Port 2 data output enable
		p2_sel		<=	r.p2sel;			--! Port 2 function select
		p3_dout		<=	r.p3out;			--! Port 3 data output
		p3_dout_en	<=	r.p3dir;			--! Port 3 data output enable
		p3_sel		<=	r.p3sel;			--! Port 3 function select
		p4_dout		<=	r.p4out;			--! Port 4 data output
		p4_dout_en	<=	r.p4dir;			--! Port 4 data output enable
		p4_sel		<=	r.p4sel;			--! Port 4 function select
		p5_dout		<=	r.p5out;			--! Port 5 data output
		p5_dout_en	<=	r.p5dir;			--! Port 5 data output enable
		p5_sel		<=	r.p5sel;			--! Port 5 function select
		p6_dout		<=	r.p6out;			--! Port 6 data output
		p6_dout_en	<=	r.p6dir;			--! Port 6 data output enable
		p6_sel		<=	r.p6sel;			--! Port 6 function select
		per_dout		<=	v_per_dout;		--! Peripheral data output

		end process COMB;

	REGS : process (mclk,mrst)
	begin
		if (mrst = '1') then
			r.p1out		<=	"00000000";
			r.p1dir		<=	"00000000";
			r.p1ifg		<=	"00000000";
			r.p1ies		<=	"00000000";
			r.p1ie		<=	"00000000";
			r.p1sel		<=	"00000000";
			r.p2out		<=	"00000000";
			r.p2dir		<=	"00000000";
			r.p2ifg		<=	"00000000";
			r.p2ies		<=	"00000000";
			r.p2ie		<=	"00000000";
			r.p2sel		<=	"00000000";
			r.p3out		<=	"00000000";
			r.p3dir		<=	"00000000";
			r.p3sel		<=	"00000000";
			r.p4out		<=	"00000000";
			r.p4dir		<=	"00000000";
			r.p4sel		<=	"00000000";
			r.p5out		<=	"00000000";
			r.p5dir		<=	"00000000";
			r.p5sel		<=	"00000000";
			r.p6out		<=	"00000000";
			r.p6dir		<=	"00000000";
			r.p6sel		<=	"00000000";
			r.p1in_dly	<=	"00000000";
			r.p2in_dly	<=	"00000000";
		elsif rising_edge(mclk) then
			r	<= rin;
		end if;
	end process REGS;

	p1_in_sync : for i in 0 to 7 generate
		sync_cell_p1in : fmsp_sync_cell
		port map(
			clk		=> mclk,
			rst		=> mrst,
			data_in	=> p1_din(i),
			data_out	=> d.p1in(i)
		);
	end generate;
	p2_in_sync : for i in 0 to 7 generate
		sync_cell_p2in : fmsp_sync_cell
		port map(
			clk		=> mclk,
			rst		=> mrst,
			data_in	=> p2_din(i),
			data_out	=> d.p2in(i)
		);
	end generate;
	p3_in_sync : for i in 0 to 7 generate
		sync_cell_p3in : fmsp_sync_cell
		port map(
			clk		=> mclk,
			rst		=> mrst,
			data_in	=> p3_din(i),
			data_out	=> d.p3in(i)
		);
	end generate;
	p4_in_sync : for i in 0 to 7 generate
		sync_cell_p4in : fmsp_sync_cell
		port map(
			clk		=> mclk,
			rst		=> mrst,
			data_in	=> p4_din(i),
			data_out	=> d.p4in(i)
		);
	end generate;
	p5_in_sync : for i in 0 to 7 generate
		sync_cell_p5in : fmsp_sync_cell
		port map(
			clk		=> mclk,
			rst		=> mrst,
			data_in	=> p5_din(i),
			data_out	=> d.p5in(i)
		);
	end generate;
	p6_in_sync : for i in 0 to 7 generate
		sync_cell_p6in : fmsp_sync_cell
		port map(
			clk		=> mclk,
			rst		=> mrst,
			data_in	=> p6_din(i),
			data_out	=> d.p6in(i)
		);
	end generate;


end RTL; --! fmsp_gpio

