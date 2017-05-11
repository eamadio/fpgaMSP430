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
--! @file fmsp_clock_module.vhd
--! 
--! @brief fpgaMSP430 Basic clock module implementation.
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

entity  fmsp_clock_module is
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
end entity fmsp_clock_module;

architecture RTL of fmsp_clock_module is 

	--=============================================================================
	--! 1)  WIRES & PARAMETER DECLARATION
	--=============================================================================

	--! Register base address (must be aligned to decoder bit width)
	constant	BASE_ADDR	: std_logic_vector(14 downto 0) := "000000000000000";
	--! Decoder bit width (defines how many bits are considered for address decoding)
	constant	DEC_WD		: integer := 4;
	--! Register addresses offset
	constant	BCSCTL1		: integer := 7;
	constant	BCSCTL2		: integer := 8;
	--! Register one-hot decoder utilities
	constant	DEC_SZ      : integer :=  (2**DEC_WD);

	type fmsp_clock_module_in_type is record
		cpu_en				: std_logic;										--! Enable CPU code execution (asynchronous)
		cpuoff				: std_logic;										--! Turns off the CPU
		dbg_cpu_reset		: std_logic;										--! Reset CPU from debug interface
		dbg_en				: std_logic;										--! Debug interface enable (asynchronous)
		dco_clk				: std_logic;										--! Fast oscillator (fast clock)
		lfxt_clk				: std_logic;										--! Low frequency oscillator (typ 32kHz)
		oscoff				: std_logic;										--! Turns off LFXT1 clock input
		per_addr				: std_logic_vector(13 downto 0);				--! Peripheral address
		per_din				: std_logic_vector(15 downto 0);				--! Peripheral data input
		per_en				: std_logic;										--! Peripheral enable (high active)
		per_we				: std_logic_vector(1 downto 0);				--! Peripheral write enable (high active)
		reset_n				: std_logic;										--! Reset Pin (low active, asynchronous)
		scg1					: std_logic;										--! System clock generator 1. Turns off the SMCLK
		wdt_reset			: std_logic;										--! Watchdog-timer reset
		puc_rst				: std_logic;										--! Watchdog-timer reset
		por_noscan			: std_logic;
		puc_noscan_n		: std_logic;
		puc_s			: std_logic;
	end record;

	type reg_type is record
		--! To outside of module
		bcsctl1			: std_logic_vector(7 downto 0);	--! BCSCTL1 Register
		bcsctl2			: std_logic_vector(7 downto 0);	--! BCSCTL2 Register
		lfxt_clk_dly	: std_logic;
		aclk_en			: std_logic;							--! ACLK GENERATION
		aclk_div			: unsigned(2 downto 0);
		smclk_en			: std_logic;
		smclk_div		: unsigned(2 downto 0);
		dbg_rst	: std_logic;							--! Reset Generation
	end record;
							  

	signal	d		: fmsp_clock_module_in_type;
	signal	r		: reg_type :=	(
												bcsctl1			=> (Others => '0'),
												bcsctl2			=> (Others => '0'),
												lfxt_clk_dly	=> '0',
												aclk_en			=> '0',
												aclk_div			=> (Others => '0'),
												smclk_en			=> '0',
												smclk_div		=> (Others => '0'),
												dbg_rst			=> '1'
											);
	signal	rin	: reg_type;

begin

		d.cpu_en				<=	cpu_en;
		d.cpuoff				<=	cpuoff;
		d.dbg_cpu_reset	<=	dbg_cpu_reset;
		d.dbg_en				<=	dbg_en;
		d.lfxt_clk			<=	lfxt_clk;
		d.oscoff				<=	oscoff;
		d.per_addr			<=	per_addr;
		d.per_din			<=	per_din;
		d.per_en				<=	per_en;
		d.per_we				<=	per_we;
		d.reset_n			<=	reset_n;
		d.scg1				<=	scg1;
		d.wdt_reset			<=	wdt_reset;

	COMB : process (d, r)
		variable	v						: reg_type;
		--! Local register selection
		variable	v_reg_sel			: std_logic;
		--! Register local address
		variable	v_reg_addr			: std_logic_vector(DEC_WD-2 downto 0);
		--! Register address decode
		variable	v_reg_dec			: std_logic_vector((DEC_SZ/2)-1 downto 0);
		--! Read/Write probes
		variable	v_reg_lo_write		: std_logic;
		variable	v_reg_hi_write		: std_logic;
		variable	v_reg_read			: std_logic;
		--! Read/Write vectors
		variable	v_reg_wr				: std_logic_vector(DEC_SZ-1 downto 0);
		variable	v_reg_rd				: std_logic_vector(DEC_SZ-1 downto 0);
		--! BCSCTL1 Register
		variable	v_bcsctl1_wr		: std_logic;
		variable	v_bcsctl1_nxt		: std_logic_vector(7 downto 0);
		variable	v_bcsctl1_mask		: std_logic_vector(7 downto 0);
		--! BCSCTL2 Register
		variable	v_bcsctl2_wr		: std_logic;
		variable	v_bcsctl2_nxt		: std_logic_vector(7 downto 0);
		variable	v_bcsctl2_mask		: std_logic_vector(7 downto 0);
		--! Data output mux
		variable	v_bcsctl1_rd		: std_logic_vector(15 downto 0);
		variable	v_bcsctl2_rd		: std_logic_vector(15 downto 0);
		variable	v_per_dout			: std_logic_vector(15 downto 0);
		--! Synchronize LFXT_CLK & edge detection
		variable	v_lfxt_clk_en		: std_logic;
		--! Synchronize CPU_EN signal to the ACLK domain
		------------------------------------------------
		variable	v_mclk_div_sel				: std_logic;
		--! ACLK GENERATION
		variable	v_aclk_en_nxt				: std_logic;
		--! Clock MUX
		variable	v_smclk_in					: std_logic;
		variable	v_smclk_en_nxt				: std_logic;
		--! Synchronize DBG_EN signal to MCLK domain
		variable	v_dbg_rst_nxt				: std_logic;
		--! Asynchronous reset source
		--! Scan Reset Mux
--		variable	v_dbg_rst					: std_logic;
		--! Generate main system reset (PUC_RST)
		variable	v_puc_noscan_n				: std_logic;
		--! Reset Synchronizer
		--! (required because of the asynchronous watchdog reset)
		variable	v_puc_rst					: std_logic;
		--! PUC pending set the serial debug interface
		variable	v_puc_pnd_set				: std_logic;

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
		--! 3) REGISTERS
		--============================================================================

		--! BCSCTL1 Register
		----------------
		v_bcsctl1_wr	:= v_reg_wr(BCSCTL1);
		v_bcsctl1_nxt	:= byte_per_select_din( BCSCTL1, d.per_din );

		if (DMA_IF_EN = true) then
			v_bcsctl1_mask := x"3A";
		else
			v_bcsctl1_mask := x"30";
		end if;

		if (v_bcsctl1_wr = '1') then
			v.bcsctl1  :=  v_bcsctl1_nxt and v_bcsctl1_mask; --! Mask unused bits
		end if;

		--! BCSCTL2 Register
		----------------
		v_bcsctl2_wr	:= v_reg_wr(BCSCTL2);
		v_bcsctl2_nxt	:= byte_per_select_din( BCSCTL2, d.per_din );

		v_bcsctl2_mask	:= x"0E";

		if (v_bcsctl2_wr = '1') then
			v.bcsctl2	:= v_bcsctl2_nxt and v_bcsctl2_mask; --! Mask unused bits
		end if;


		--============================================================================
		--! 4) DATA OUTPUT GENERATION
		--============================================================================

		v_bcsctl1_rd	:= byte_per_select_dout( BCSCTL1,	v_reg_rd, r.bcsctl1	);
		v_bcsctl2_rd	:= byte_per_select_dout( BCSCTL2,	v_reg_rd, r.bcsctl2	);
		--! Data output mux
		v_per_dout		:= v_bcsctl1_rd or v_bcsctl2_rd;


		--=============================================================================
		--! 5)  DCO_CLK / LFXT_CLK INTERFACES (WAKEUP, ENABLE, ...)
		--=============================================================================

		-------------------------------------------------------------
		--! 5.2) LOW FREQUENCY CRYSTAL CLOCK GENERATOR (LFXT_CLK)
		-------------------------------------------------------------

		--! Synchronize LFXT_CLK & edge detection

		v.lfxt_clk_dly	:= d.lfxt_clk;
		v_lfxt_clk_en	:= d.lfxt_clk and not(r.lfxt_clk_dly) and not(d.oscoff);


		--=============================================================================
		--! 6)  CLOCK GENERATION
		--=============================================================================

		-------------------------------------------------------------
		--! 6.1) GLOBAL CPU ENABLE
		------------------------------------------------------------
		--! ACLK and SMCLK are directly switched-off
		--! with the cpu_en pin (after synchronization).
		--! MCLK will be switched off once the CPU reaches
		--! its IDLE state (through the mclk_enable signal)

		-------------------------------------------------------------
		--! 6.3) ACLK GENERATION
		-------------------------------------------------------------

		v_aclk_en_nxt := '0';
		if (v_lfxt_clk_en = '1') then
			if (r.bcsctl1(5 downto 4) = "00") then		-- Divide mclk by 1
				v_aclk_en_nxt := '1';
			elsif (r.bcsctl1(5 downto 4) = "01") then	-- Divide mclk by 2
				v_aclk_en_nxt := r.aclk_div(0);
			elsif (r.bcsctl1(5 downto 4) = "10") then	-- Divide mclk by 4
				v_aclk_en_nxt := r.aclk_div(1) and r.aclk_div(0);
			else													-- Divide mclk by 8
				v_aclk_en_nxt := r.aclk_div(1) and r.aclk_div(1) and r.aclk_div(0);
			end if;
		end if;

		if ( (r.bcsctl1(5 downto 4) /= "00") and (v_lfxt_clk_en = '1') ) then
			v.aclk_div :=  r.aclk_div + 1;
		end if;

		v.aclk_en	:=  v_aclk_en_nxt and d.cpu_en;

		-------------------------------------------------------------
		--! 6.4) SMCLK GENERATION
		-------------------------------------------------------------

		if ( d.scg1 = '1' ) then
			v_smclk_in := '0';
		elsif (r.bcsctl2(C_SELS) = '1') then
			v_smclk_in := v_lfxt_clk_en;
		else
			v_smclk_in := '1';
		end if;

		v_smclk_en_nxt := '0';
		if (v_smclk_in = '1') then
			if (r.bcsctl2(2 downto 1) = "00") then		-- Divide mclk by 1
				v_smclk_en_nxt := '1';
			elsif (r.bcsctl2(2 downto 1) = "01") then	-- Divide mclk by 2
				v_smclk_en_nxt := r.smclk_div(0);
			elsif (r.bcsctl2(2 downto 1) = "10") then	-- Divide mclk by 4
				v_smclk_en_nxt := r.smclk_div(1) and r.smclk_div(0);
			else													-- Divide mclk by 8
				v_smclk_en_nxt := r.smclk_div(2) and r.smclk_div(1) and r.smclk_div(0);
			end if;
		end if;

		if ( (r.bcsctl2(2 downto 1) /= "00") and (v_smclk_in = '1') ) then
			v.smclk_div :=  r.smclk_div + 1;
		end if;

		v.smclk_en := v_smclk_en_nxt and d.cpu_en;

		--=============================================================================
		--! 7)  RESET GENERATION
		--=============================================================================
		--
		--! Whenever the reset pin (reset_n) is deasserted, the internal resets of the
		--! openMSP430 will be released in the following order:
		--!                1- POR
		--!                2- DBG_RST (if the sdi interface is enabled, i.e. dbg_en=1)
		--!                3- PUC
		--
		--! Note: releasing the DBG_RST before PUC is particularly important in order
		--!       to allow the sdi interface to halt the cpu immediately after a PUC.
		--

		--! Generate synchronized POR to MCLK domain
		--------------------------------------------


		--! Generate synchronized reset for the SDI
		--------------------------------------------
		if (DEBUG_EN = true) then
			--! Reset Generation
			v.dbg_rst	:= not(d.dbg_en);
		else
			v.dbg_rst	:= '1';
		end if;


		--! Generate main system reset (PUC_RST)
		----------------------------------------

		--! Asynchronous PUC reset
--		v_puc_a	:=	d.por_noscan or d.wdt_reset;

		--! Synchronous PUC reset
--		v_puc_s := d.dbg_cpu_reset or											--! With the debug interface command
--!            (d.dbg_en and r.dbg_rst and not(d.puc_noscan_n));  --! Sequencing making sure PUC is released
--																						--! after DBG_RST if the debug interface is
--																						--! enabled at power-on-reset time
		--! Scan Reset Mux
--		v_puc_a_scan := v_puc_a;

		--! PUC pending set the serial debug interface
		v_puc_pnd_set := not(d.puc_noscan_n);

		--! drive register inputs
		rin <= v;

		--! drive module outputs
		dbg_rst		<= r.dbg_rst;										--! Debug unit reset
		per_dout		<= v_per_dout;										--! Peripheral data output
		por			<= d.por_noscan;									--! Power-on reset
		puc_pnd_set	<= v_puc_pnd_set;									--! PUC pending set for the serial debug interface
		puc_rst		<= d.puc_rst;										--! Main system reset
		aclk_en		<= r.aclk_en;										--! ACLK enable
		smclk_en		<= r.smclk_en;										--! SMCLK enable

	end process COMB;

--	d.puc_rst <= '0';
	d.puc_rst <= not(d.puc_noscan_n);
	d.puc_s <= d.dbg_cpu_reset or											--! With the debug interface command
            (d.dbg_en and r.dbg_rst and not(d.puc_noscan_n));  --! Sequencing making sure PUC is released
																						--! after DBG_RST if the debug interface is
																						--! enabled at power-on-reset time
--	d.puc_s <= '0';	--! Sequencing making sure PUC is released
--							--! after DBG_RST if the debug interface is
--							--! enabled at power-on-reset time

	REGS_POR : process (mclk,d.por_noscan)
	begin
		if (d.por_noscan = '1') then
			r.dbg_rst		<= '1';
		elsif rising_edge(mclk) then
			r.dbg_rst		<= rin.dbg_rst;
		end if;
	end process REGS_POR;

	REGS_PUC : process (mclk,d.puc_rst)
	begin
		if (d.puc_rst = '1') then
			r.bcsctl1		<= (Others => '0');
			r.bcsctl2		<= (Others => '0');
			r.lfxt_clk_dly	<= '1';
			r.aclk_en		<= '0';
			r.aclk_div		<= (Others => '0');
			r.smclk_en		<= '0';
			r.smclk_div		<= (Others => '0');
		elsif rising_edge(mclk) then
			r.bcsctl1		<= rin.bcsctl1;
			r.bcsctl2		<= rin.bcsctl2;
			r.lfxt_clk_dly	<= rin.lfxt_clk_dly;
			r.aclk_en		<= rin.aclk_en;
			r.aclk_div		<= rin.aclk_div;
			r.smclk_en		<= rin.smclk_en;
			r.smclk_div		<= rin.smclk_div;
		end if;
	end process REGS_PUC;

	--! Reset Synchronizer
	--! (required because of the asynchronous watchdog reset)
	sync_cell_puc : fmsp_sync_cell
	port map(
		rst		=> d.por_noscan or wdt_reset,
		clk		=> mclk,
		data_in	=> not(d.puc_s),
		data_out	=> d.puc_noscan_n
	);
	--! Reset Synchronizer
	sync_reset_por : fmsp_sync_reset
	port map(
		clk		=> mclk,
		rst_a		=> not(d.reset_n),
		rst_s		=> d.por_noscan
	);

end RTL;	--	fmsp_clock_module
