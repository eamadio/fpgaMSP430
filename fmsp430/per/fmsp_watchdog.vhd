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
--! @file fmsp_watchdog.vhd
--! 
--! @brief fpgaMSP430 Watchdog Timer
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
	
entity fmsp_watchdog is 
generic (
	WATCHDOG_MUX			: boolean := true;		--! 
	WATCHDOG_NOMUX_ACLK	: boolean := true;		--! 
	NMI_EN					: boolean := false		--! Include/Exclude Non-Maskable-Interrupt support
);
port (
	mclk				: in	std_logic;								--! Main system clock
	mrst			: in	std_logic;								--! Main system reset
	por				: in	std_logic;								--! Power-on reset
	--! INPUTs
	per_addr			: in	std_logic_vector(13 downto 0);	--! Peripheral address
	per_dout			: out	std_logic_vector(15 downto 0);	--! Peripheral data output
	per_din			: in	std_logic_vector(15 downto 0);	--! Peripheral data input
	per_en			: in	std_logic;								--! Peripheral enable (high active)
	per_we			: in	std_logic_vector(1 downto 0);		--! Peripheral write enable (high active)
	aclk_en			: in	std_logic;								--! ACLK enable
	smclk_en			: in	std_logic;								--! SMCLK enable
	dbg_freeze		: in	std_logic;								--! Freeze Watchdog counter
	wdtie				: in	std_logic;								--! Watchdog timer interrupt enable
	wdtifg_irq_clr	: in	std_logic;								--! Watchdog-timer interrupt flag irq accepted clear
	wdtifg_sw_clr	: in	std_logic;								--! Watchdog-timer interrupt flag software clear
	wdtifg_sw_set	: in	std_logic;								--! Watchdog-timer interrupt flag software set
--! OUTPUTs
	wdt_irq			: out	std_logic;								--! Watchdog-timer interrupt
	wdt_reset		: out	std_logic;								--! Watchdog-timer reset
	wdt_wkup			: out	std_logic;								--! Watchdog Wakeup
	wdtifg			: out	std_logic;								--! Watchdog-timer interrupt flag
	wdtnmies			: out	std_logic								--! Watchdog-timer NMI edge selection
	);
end entity fmsp_watchdog;

architecture RTL of fmsp_watchdog is 

--! Register base address (must be aligned to decoder bit width)
constant	BASE_ADDR	: std_logic_vector(14 downto 0) := "000000100100000";
--! Decoder bit width (defines how many bits are considered for address decoding)
constant	DEC_WD		: integer := 2;
--! Register addresses offset
constant	WDTCTL		: integer := 0;
--! Register one-hot decoder utilities
constant	DEC_SZ		: integer := 2**DEC_WD;


	type fmsp_watchdog_in_type is record
		por				: std_logic;								--! Power-on reset
		aclk_en			: std_logic;								--! ACLK enable
		dbg_freeze		: std_logic;								--! Freeze Watchdog counter
		per_addr			: std_logic_vector(13 downto 0);	--! Peripheral address
		per_din			: std_logic_vector(15 downto 0);	--! Peripheral data input
		per_en			: std_logic;								--! Peripheral enable (high active)
		per_we			: std_logic_vector(1 downto 0);		--! Peripheral write enable (high active)
		smclk_en			: std_logic;								--! SMCLK enable
		wdtie				: std_logic;								--! Watchdog timer interrupt enable
		wdtifg_irq_clr	: std_logic;								--! Watchdog-timer interrupt flag irq accepted clear
		wdtifg_sw_clr	: std_logic;								--! Watchdog-timer interrupt flag software clear
		wdtifg_sw_set	: std_logic;								--! Watchdog-timer interrupt flag software set
	end record;

	type reg_type is record
		nmi_dly		: std_logic;			
		wdtctl		: std_logic_vector(7 downto 0);
		wdtcnt		: unsigned(15 downto 0);			--! Watchdog 16 bit counter
		wdtqn			: std_logic;							--! Interval selection mux
		wdtqn_dly	: std_logic;							--! Watchdog event detection
		wdtifg		: std_logic;							--! Watchdog interrupt flag
		--! To outside of module
	end record;

	signal	d		: fmsp_watchdog_in_type;
	signal	r		: reg_type :=	(	nmi_dly		=> '0',
												wdtctl		=> x"00",
												wdtcnt		=> TO_UNSIGNED(0,16),	--! Watchdog 16 bit counter
												wdtqn			=> '0',						--! Interval selection mux
												wdtqn_dly	=> '0',						--! Watchdog event detection
												wdtifg		=> '0'						--! Watchdog interrupt flag
											);
	signal	rin	: reg_type;

begin

		d.aclk_en			<=	aclk_en;
		d.por					<=	por;
		d.dbg_freeze		<=	dbg_freeze;
		d.per_addr			<=	per_addr;
		d.per_din			<=	per_din;
		d.per_en				<=	per_en;
		d.per_we				<=	per_we;
		d.smclk_en			<=	smclk_en;
		d.wdtie				<=	wdtie;
		d.wdtifg_irq_clr	<=	wdtifg_irq_clr;
		d.wdtifg_sw_clr	<=	wdtifg_sw_clr;
		d.wdtifg_sw_set	<=	wdtifg_sw_set;

	COMB : process (d, r)
		variable	v				: reg_type;
		--============================================================================
		--! 2)  REGISTER DECODER
		--============================================================================
		--! Local register selection
		variable	v_reg_sel		: std_logic;
		--! Register local address
		variable	v_reg_addr			: std_logic_vector(DEC_WD-2 downto 0);
		--! Register address decode
		variable	v_reg_dec			: std_logic_vector((DEC_SZ/2)-1 downto 0);
		--! Read/Write probes
		variable	v_reg_write		: std_logic;
		variable	v_reg_read		: std_logic;
		--! Read/Write vectors
		variable	v_reg_wr			: std_logic_vector(DEC_SZ-1 downto 0);
		variable	v_reg_rd			: std_logic_vector(DEC_SZ-1 downto 0);
		--============================================================================
		--! 3) REGISTERS
		--============================================================================
		--! WDTCTL Register
		-------------------
		--! WDTNMI & WDTSSEL are not implemented and therefore masked
		variable	v_wdtctl_wr		: std_logic;
		variable	v_wdtpw_error	: std_logic;
		variable	v_wdttmsel		: std_logic;
		variable	v_wdtnmies			: std_logic;
		--! Data output mux
		variable	v_per_dout		: std_logic_vector(15 downto 0);
		variable	v_wdtnmies_mask	: std_logic_vector(7 downto 0);
		variable	v_wdtssel_mask		: std_logic_vector(7 downto 0);
		variable	v_wdtctl_mask		: std_logic_vector(7 downto 0);


		--============================================================================
		--! 4) DATA OUTPUT GENERATION
		--============================================================================
		variable	v_wdtnmi_rd_mask	: std_logic_vector(7 downto 0);
		variable	v_wdtssel_rd_mask	: std_logic_vector(7 downto 0);
		variable	v_wdtctl_rd_mask	: std_logic_vector(7 downto 0);
		--! Data output mux
		variable	v_wdtctl_rd			: std_logic_vector(15 downto 0);
		--=============================================================================
		--! 5)  WATCHDOG TIMER
		--=============================================================================
		--! Watchdog clock source selection
		variable	v_clk_src_en	: std_logic;
		--! Watchdog 16 bit counter
		variable	v_wdtcnt_clr	: std_logic;
		--! Watchdog event detection
		variable	v_wdtqn_dly		: std_logic;

		variable	v_wdtcnt_incr	: std_logic;
		variable	v_wdtcnt_nxt	: unsigned(15 downto 0);
		variable	v_wdtifg_evt	: std_logic;
		variable	v_wdtifg_set	: std_logic;
		variable	v_wdtifg_clr	: std_logic;
		variable	v_wdt_irq		: std_logic;
		variable	v_wdt_wkup		: std_logic;

		variable	v_wdt_reset		: std_logic;

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

		--! WDTCTL Register
		-------------------
		--! WDTNMI is not implemented and therefore masked

		v_wdtctl_wr := v_reg_wr(WDTCTL);

		if (NMI_EN = true) then
			v_wdtnmies_mask	:= x"40";
		else
			v_wdtnmies_mask	:= x"00";
		end if;

		v_wdtssel_mask			:= x"04";
		v_wdtctl_mask			:= x"93" or v_wdtssel_mask or v_wdtnmies_mask;

		if (v_wdtctl_wr = '1') then
			v.wdtctl  := d.per_din(7 downto 0) and v_wdtctl_mask;
		end if;

		if ( d.per_din(15 downto 8) = x"5A" ) then
			v_wdtpw_error	:=	'0';
		else
			v_wdtpw_error	:=	v_wdtctl_wr;
		end if;
		v_wdttmsel			:= r.wdtctl(4);
		v_wdtnmies			:= r.wdtctl(6);

		--============================================================================
		--! 4) DATA OUTPUT GENERATION
		--============================================================================
		if (NMI_EN = true) then
			v_wdtnmi_rd_mask	:= x"20";
		else
			v_wdtnmi_rd_mask	:= x"00";
		end if;

		if (WATCHDOG_MUX = true) then
			v_wdtssel_rd_mask	:= x"00";
		else
			if (WATCHDOG_NOMUX_ACLK = true) then
				v_wdtssel_rd_mask	:= x"04";
			else
				v_wdtssel_rd_mask	:= x"00";
			end if;
		end if;

		v_wdtctl_rd_mask			:= v_wdtnmi_rd_mask or v_wdtssel_rd_mask;

--		v_wdtctl_rd := x"0000";
--		--! Data output mux
--		if ( v_reg_rd(WDTCTL) = '1' ) then
--			v_wdtctl_rd := x"69" & ( r.wdtctl(7 downto 0) or v_wdtctl_rd_mask);
--		end if;
		v_wdtctl_rd	:= word_per_select_dout( WDTCTL,	v_reg_rd, x"69" & ( r.wdtctl(7 downto 0) or v_wdtctl_rd_mask)				);

		v_per_dout	:= v_wdtctl_rd;

		--=============================================================================
		--! 5)  WATCHDOG TIMER
		--=============================================================================

		--! Watchdog clock source selection
		-----------------------------------
		if (v.wdtctl(2) = '1') then
			v_clk_src_en	:=	d.aclk_en;
		else
			v_clk_src_en	:= d.smclk_en;
		end if;

		--! Watchdog 16 bit counter
		----------------------------
		v_wdtcnt_clr	:= (v_wdtctl_wr and d.per_din(3)) or v_wdtifg_evt;
		v_wdtcnt_incr	:= not(r.wdtctl(7)) and v_clk_src_en and not(d.dbg_freeze);

		v_wdtcnt_nxt	:= r.wdtcnt + TO_UNSIGNED(1,16);

		if (v_wdtcnt_clr = '1') then
			v.wdtcnt	:=	TO_UNSIGNED(0,16);
		elsif (v_wdtcnt_incr = '1') then
			v.wdtcnt	:= v_wdtcnt_nxt;
		end if;
			
		--! Interval selection mux
		----------------------------
		if (r.wdtcnt = TO_UNSIGNED(1,16)) then
			case r.wdtctl(1 downto 0) is
				when "00" =>	v.wdtqn :=  v_wdtcnt_nxt(15);
				when "01" =>	v.wdtqn :=  v_wdtcnt_nxt(13);
				when "10" =>	v.wdtqn :=  v_wdtcnt_nxt(9);
				when others =>	v.wdtqn :=  v_wdtcnt_nxt(6);
			end case;
		end if;

		--! Watchdog event detection
		-------------------------------

		v_wdtifg_evt :=  (r.wdtqn and v_wdtcnt_incr) or v_wdtpw_error;

		--! Watchdog event detection
		-------------------------------
		v.wdtqn_dly	:=	r.wdtqn;
		--! Watchdog interrupt flag
		--------------------------------

		v_wdtifg_set :=  v_wdtifg_evt or d.wdtifg_sw_set;
		v_wdtifg_clr :=  (d.wdtifg_irq_clr and v_wdttmsel) or  d.wdtifg_sw_clr;

		if (v_wdtifg_set = '1') then
			v.wdtifg := '1';
		elsif (v_wdtifg_clr = '1') then
			v.wdtifg := '0';
		end if;


		--! Watchdog interrupt generation
		-----------------------------------
		v_wdt_irq	:= v_wdttmsel and r.wdtifg and d.wdtie;
		v_wdt_wkup	:= '0';


		--! Watchdog reset generation
		-------------------------------
		v_wdt_reset := v_wdtpw_error or (v_wdtifg_set and not(v_wdttmsel));


		--! drive register inputs
		rin <= v;
		--! drive module outputs
		
		per_dout		<=	v_per_dout;		--! Peripheral data output
		wdt_irq		<=	v_wdt_irq;		--! Watchdog-timer interrupt
		wdt_reset	<=	v_wdt_reset;	--! Watchdog-timer reset
		wdt_wkup		<=	v_wdt_wkup;		--! Watchdog Wakeup
		wdtifg		<=	r.wdtifg;		--! Watchdog-timer interrupt flag
		wdtnmies		<=	v_wdtnmies;		--! Watchdog-timer NMI edge selection

	end process COMB;

	REGS : process (mclk,mrst)
	begin
		if (mrst = '1') then
			r	<=	(	nmi_dly		=> '0',
						wdtctl		=> x"00",
						wdtcnt		=> TO_UNSIGNED(0,16),	--! Watchdog 16 bit counter
						wdtqn			=> '0',						--! Interval selection mux
						wdtqn_dly	=> '0',						--! Watchdog event detection
						wdtifg		=> '0'						--! Watchdog interrupt flag
					);
		elsif rising_edge(mclk) then
			r	<= rin;
		end if;
	end process REGS;


end RTL;


