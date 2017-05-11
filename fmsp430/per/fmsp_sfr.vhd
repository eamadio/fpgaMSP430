------------------------------------------------------------------------------
-- Copyright (C) 2009 , Olivier Girard
--
-- Redistribution and use in source and binary forms, with or without
-- modification, are permitted provided that the following conditions
-- are met:
--     * Redistributions of source code must retain the above copyright
--       notice, this list of conditions and the following disclaimer.
--     * Redistributions in binary form must reproduce the above copyright
--       notice, this list of conditions and the following disclaimer in the
--       documentation and/or other materials provided with the distribution.
--     * Neither the name of the authors nor the names of its contributors
--       may be used to endorse or promote products derived from this software
--       without specific prior written permission.
--
-- THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
-- AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
-- IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
-- ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
-- LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
-- OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
-- SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
-- INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
-- CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
-- ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
-- THE POSSIBILITY OF SUCH DAMAGE
--
------------------------------------------------------------------------------
--
-- *File Name: fmsp_sfr.v
-- 
-- *Module Description:
--                       Processor Special function register
--
-- *Author(s):
--              - Olivier Girard,    olgirard@gmail.com
--
------------------------------------------------------------------------------
-- $Rev: 117 $
-- $LastChangedBy: olivier.girard $
-- $LastChangedDate: 2011-06-23 21:30:51 +0200 (Thu, 23 Jun 2011) $
------------------------------------------------------------------------------
library ieee;
	use ieee.std_logic_1164.all;	--! standard unresolved logic UX01ZWLH-
	use ieee.numeric_std.all;		--! for the signed, unsigned types and arithmetic ops
	use ieee.math_real.all;
	use work.fmsp_misc_package.all;
	use work.fmsp_per_package.all;
	use work.fmsp_functions.all;

entity fmsp_sfr is 
generic (
	INST_NR				: integer := 0;			-- Current fmsp instance number     (for multicore systems)
	TOTAL_NR				: integer := 0;			-- Total number of fmsp instances-1 (for multicore systems)
	PMEM_SIZE			: integer := 32768;		-- Program Memory Size
	DMEM_SIZE			: integer := 16384;		-- Data Memory Size
	PER_SIZE				: integer := 16384;		-- Peripheral Memory Size
	MULTIPLIER			: boolean := false;		-- Include/Exclude Hardware Multiplier
	USER_VERSION		: integer := 0;			-- Custom user version number
	WATCHDOG				: boolean := false;		-- Include/Exclude Watchdog timer
	NMI_EN				: boolean := false		-- Include/Exclude Non-Maskable-Interrupt support
);
port (
		mclk				: in	std_logic;       						-- Main system clock
		mrst			: in	std_logic;       						-- Main system reset
		-- INPUTs
		nmi				: in	std_logic;       						-- Non-maskable interrupt (asynchronous)
		nmi_acc			: in	std_logic;       						-- Non-Maskable interrupt request accepted
		per_addr			: in	std_logic_vector(13 downto 0);	-- Peripheral address
		per_din			: in	std_logic_vector(15 downto 0);	-- Peripheral data input
		per_en			: in	std_logic;       						-- Peripheral enable (high active)
		per_we			: in	std_logic_vector(1 downto 0);		-- Peripheral write enable (high active)
		wdtifg			: in	std_logic;       						-- Watchdog-timer interrupt flag
		wdtnmies			: in	std_logic;       						-- Watchdog-timer NMI edge selection
		-- OUTPUTs
		cpu_id			: out	std_logic_vector(31 downto 0);	-- CPU ID
		nmi_pnd			: out	std_logic;								-- NMI Pending
		nmi_wkup			: out	std_logic;								-- NMI Wakeup
		per_dout			: out	std_logic_vector(15 downto 0);	-- Peripheral data output
		wdtie				: out	std_logic;								-- Watchdog-timer interrupt enable
		wdtifg_sw_clr	: out	std_logic;								-- Watchdog-timer interrupt flag software clear
		wdtifg_sw_set	: out	std_logic								-- Watchdog-timer interrupt flag software set
);
end entity fmsp_sfr;

architecture RTL of fmsp_sfr is 
	--=============================================================================
	-- 1)  PARAMETER DECLARATION
	--=============================================================================

	-- Register base address (must be aligned to decoder bit width)
	constant	BASE_ADDR	: std_logic_vector(14 downto 0) := "000000000000000";
	-- Decoder bit width (defines how many bits are considered for address decoding)
	constant	DEC_WD	: integer := 4;
	-- Register addresses offset
	constant	IE1			: integer := 0;
	constant	IFG1			: integer := 2;
	constant	CPU_ID_LO	: integer := 4;
	constant	CPU_ID_HI	: integer := 6;
	constant	CPU_NR		: integer := 8;
	-- Register one-hot decoder utilities
	constant	DEC_SZ      : integer :=  (2**DEC_WD);

	type fmsp_sfr_in_type is record
		nmi				: std_logic;       						-- Non-maskable interrupt (asynchronous)
		nmi_acc			: std_logic;       						-- Non-Maskable interrupt request accepted
		per_addr			: std_logic_vector(13 downto 0);		-- Peripheral address
		per_din			: std_logic_vector(15 downto 0);		-- Peripheral data input
		per_en			: std_logic;       						-- Peripheral enable (high active)
		per_we			: std_logic_vector(1 downto 0);		-- Peripheral write enable (high active)
		wdtifg			: std_logic;       						-- Watchdog-timer interrupt flag
		wdtnmies			: std_logic;       						-- Watchdog-timer NMI edge selection
	end record;

	type reg_type is record
		-- To outside of module
		nmi_dly		: std_logic;								-- Non-maskable interrupt enable
		nmie			: std_logic;								-- Non-maskable interrupt enable
		wdtie			: std_logic;								-- Watchdog-timer interrupt enable
		nmiifg		: std_logic;
		wdtifg		: std_logic;
		wdt_reset	: std_logic;								-- Watchdog-timer reset
	end record;

	signal	d		: fmsp_sfr_in_type;
	signal	r		: reg_type :=	(	nmi_dly		=> '0',
												nmie			=> '0',
												wdtie			=> '0',
												nmiifg		=> '0',
												wdtifg		=> '0',
												wdt_reset	=> '0'
										);
	signal	rin	: reg_type;

begin

		d.nmi			<=	nmi;
		d.nmi_acc	<=	nmi_acc;
		d.per_addr	<=	per_addr;
		d.per_din	<=	per_din;
		d.per_en		<=	per_en;
		d.per_we		<=	per_we;
		d.wdtifg		<=	wdtifg;
		d.wdtnmies	<=	wdtnmies;

	COMB : process (d, r)
		variable	v						: reg_type;
		-- Local register selection
		variable	v_reg_sel			: std_logic;
		-- Register local address
		variable	v_reg_addr			: std_logic_vector(DEC_WD-2 downto 0);
		-- Register address decode
		variable	v_reg_dec			: std_logic_vector((DEC_SZ/2)-1 downto 0);
		-- Read/Write probes
		variable	v_reg_lo_write		: std_logic;
		variable	v_reg_hi_write		: std_logic;
		variable	v_reg_read			: std_logic;
		-- Read/Write vectors
		variable	v_reg_wr			: std_logic_vector(DEC_SZ-1 downto 0);
		variable	v_reg_rd				: std_logic_vector(DEC_SZ-1 downto 0);
		-- IE1 Register
		variable	v_ie1					: std_logic_vector(7 downto 0);
		variable	v_ie1_wr				: std_logic;
		variable	v_ie1_nxt			: std_logic_vector(7 downto 0);
		-- IFG1 Register
		variable	v_ifg1				: std_logic_vector(7 downto 0);
		variable	v_ifg1_wr			: std_logic;
		variable	v_ifg1_nxt			: std_logic_vector(7 downto 0);
		variable	v_wdtifg_sw_clr	: std_logic; -- Watchdog-timer interrupt flag software clear
		variable	v_wdtifg_sw_set	: std_logic; -- Watchdog-timer interrupt flag software set
		variable	v_cpu_version		: std_logic_vector(2 downto 0);
		variable	v_cpu_asic			: std_logic;
		variable	v_user_version		: std_logic_vector(4 downto 0);
		variable	v_per_space			: std_logic_vector(6 downto 0);
		variable	v_cpu_id_lo			: std_logic_vector(15 downto 0);
		variable	v_mpy_info			: std_logic;
		variable	v_dmem_size			: std_logic_vector(8 downto 0);
		variable	v_pmem_size			: std_logic_vector(5 downto 0);
		variable	v_cpu_id_hi			: std_logic_vector(15 downto 0);
		variable	v_cpu_nr				: std_logic_vector(15 downto 0);
		-- Data output mux
		variable	v_reg_rdIE1			: std_logic_vector(15 downto 0);
		variable	v_reg_rdIFG1		: std_logic_vector(15 downto 0);
		variable	v_ie1_rd				: std_logic_vector(15 downto 0);
		variable	v_ifg1_rd			: std_logic_vector(15 downto 0);
		variable	v_cpu_id_lo_rd		: std_logic_vector(15 downto 0);
		variable	v_cpu_id_hi_rd		: std_logic_vector(15 downto 0);
		variable	v_cpu_nr_rd			: std_logic_vector(15 downto 0);
		variable	v_per_dout			: std_logic_vector(15 downto 0);
		-- Watchdog reset generation
		variable	v_wdt_irq			: std_logic;				-- Watchdog-timer interrupt
		variable	v_nmie				: std_logic;				-- Non-maskable interrupt enable
		variable	v_wdt_reset			: std_logic;				-- Watchdog-timer reset
		variable	v_wdtie				: std_logic;				-- Watchdog-timer interrupt enable
		variable	v_nmi_s				: std_logic;       		-- Non-maskable interrupt (synchronous)
		variable	v_nmi_edge			: std_logic;				-- Edge selection
		variable	v_nmi_pol			: std_logic;				-- Edge selection
		variable	v_nmi_pnd			: std_logic;				-- NMI Pending
		variable	v_nmi_wkup			: std_logic;				-- NMI Wakeup
	begin
		-- default assignment
		v := r;
		-- overriding assignments

		--============================================================================
		-- 2)  REGISTER DECODER
		--============================================================================

		-- Local register selection
		if ( d.per_addr(13 downto DEC_WD-1) = BASE_ADDR(14 downto DEC_WD) ) then
			v_reg_sel	:=	d.per_en;
		else
			v_reg_sel	:=	'0';
		end if;
		-- Register local address
		v_reg_addr	:=  d.per_addr(DEC_WD-2 downto 0);
		-- Register address decode
		v_reg_dec := onehot(v_reg_addr);
		
		-- Read/Write probes
		v_reg_lo_write	:=	v_reg_sel and d.per_we(0);
		v_reg_hi_write	:= v_reg_sel and d.per_we(1);
		v_reg_read		:= v_reg_sel and not(d.per_we(0) or d.per_we(1));
		-- Read/Write vectors
		for i in 0 to (DEC_SZ/2)-1 loop
			v_reg_wr((i*2)+0)	:= v_reg_dec(i) and v_reg_lo_write;
			v_reg_wr((i*2)+1)	:= v_reg_dec(i) and v_reg_hi_write;
			v_reg_rd((i*2)+0)	:= v_reg_dec(i) and v_reg_read;
			v_reg_rd((i*2)+1)	:= v_reg_dec(i) and v_reg_read;
		end loop;


		--============================================================================
		-- 3) REGISTERS
		--============================================================================

		-- IE1 Register
		----------------
		v_ie1_wr		:= v_reg_wr(IE1);
		v_ie1_nxt	:= byte_per_select_din( IE1, d.per_din );

		if (NMI_EN = true) then
				if (d.nmi_acc = '1') then
					v.nmie  := '0';
				elsif (v_ie1_wr = '1') then
					v.nmie  := v_ie1_nxt(4);
				end if;
		else
			v.nmie	:=	'0';
		end if;
		
		if (WATCHDOG = true) then
				if (v_ie1_wr = '1') then
					v.wdtie  := v_ie1_nxt(0);
				end if;
		else
			v.wdtie	:=	'0';
		end if;

		v_ie1	:= "000" & r.nmie & "000" & r.wdtie;


		-- IFG1 Register
		-----------------
		v_ifg1_wr	:= v_reg_wr(IFG1);
		v_ifg1_nxt	:= byte_per_select_din( IFG1, d.per_din );

--		if (NMI_EN = true) then
--			if (mrst = '1') then
--				v.nmiifg	:=	'0';
--			else
--				if (v_nmi_edge = '1') then
--					v.nmiifg  := '1';
--				elsif (v_ifg1_wr = '1') then
--					v.nmiifg  := v_ifg1_nxt(4);
--				end if;
--			end if;
--		else
--			v.nmiifg	:=	'0';
--		end if;

		if (WATCHDOG = true) then
			v_wdtifg_sw_clr	:= v_ifg1_wr and not(v_ifg1_nxt(0));
			v_wdtifg_sw_set	:= v_ifg1_wr and  v_ifg1_nxt(0);
		else
			v_wdtifg_sw_clr	:=	'0';
			v_wdtifg_sw_set	:=	'0';
		end if;

--		if (por = '1') then
--			v.wdtifg	:=	'0';
--		else
--			if (d.wdtifg_set = '1') then
--				v.wdtifg  := '1';
--			elsif ((d.wdttmsel and d.wdtifg_clr) = '1') then
--				v.wdtifg  := '0';
--			elsif (v_ifg1_wr = '1') then
--				v.wdtifg  := v_ifg1_nxt(0);
--			end if;
--		end if;

		v_ifg1 := "000" & r.nmiifg & "000" & d.wdtifg;

		-- CPU_ID Register (READ ONLY)
		-------------------------------
		--              -------------------------------------------------------------------
		-- CPU_ID_LO:  | 15  14  13  12  11  10  9  |  8  7  6  5  4  |  3   |   2  1  0   |
		--             |----------------------------+-----------------+------+-------------|
		--             |        PER_SPACE           |   USER_VERSION  | ASIC | CPU_VERSION |
		--              --------------------------------------------------------------------
		-- CPU_ID_HI:  |   15  14  13  12  11  10   |   9  8  7  6  5  4  3  2  1   |   0  |
		--             |----------------------------+-------------------------------+------|
		--             |         PMEM_SIZE          |            DMEM_SIZE          |  MPY |
		--              -------------------------------------------------------------------

		v_cpu_version	:= STD_LOGIC_VECTOR(TO_UNSIGNED(C_CPU_VERSION,3));
		v_cpu_asic		:= '0';
		v_user_version	:= STD_LOGIC_VECTOR(TO_UNSIGNED(USER_VERSION,5));
		v_per_space		:= STD_LOGIC_VECTOR(TO_UNSIGNED(PER_SIZE/512,7));  -- cpu_id_per  *  512 = peripheral space size
		if (MULTIPLIER = true) then
			v_mpy_info	:=  '1';
		else
			v_mpy_info	:=  '0';
		end if;
		v_dmem_size		:= STD_LOGIC_VECTOR(TO_UNSIGNED(DMEM_SIZE/128,9));  -- cpu_id_dmem *  128 = data memory size
		v_pmem_size		:= STD_LOGIC_VECTOR(TO_UNSIGNED(PMEM_SIZE/1024,6));  -- cpu_id_pmem * 1024 = program memory size

		v_cpu_id_hi		:= v_pmem_size & v_dmem_size & v_mpy_info;
		v_cpu_id_lo		:= v_per_space & v_user_version & v_cpu_asic & v_cpu_version;


		-- CPU_NR Register (READ ONLY)
		-------------------------------
		--    -------------------------------------------------------------------
		--   | 15  14  13  12  11  10   9   8  |  7   6   5   4   3   2   1   0  |
		--   |---------------------------------+---------------------------------|
		--   |            CPU_TOTAL_NR         |           CPU_INST_NR           |
		--    -------------------------------------------------------------------

		v_cpu_nr := STD_LOGIC_VECTOR(TO_UNSIGNED(TOTAL_NR,8)) & STD_LOGIC_VECTOR(TO_UNSIGNED(INST_NR,8));



		--============================================================================
		-- 4) DATA OUTPUT GENERATION
		--============================================================================
		v_ie1_rd			:= byte_per_select_dout( IE1,			v_reg_rd, v_ie1			);
		v_ifg1_rd		:= byte_per_select_dout( IFG1,		v_reg_rd, v_ifg1			);
		v_cpu_id_lo_rd	:= word_per_select_dout( CPU_ID_LO,	v_reg_rd, v_cpu_id_lo	);
		v_cpu_id_hi_rd	:= word_per_select_dout( CPU_ID_HI,	v_reg_rd, v_cpu_id_hi	);
		v_cpu_nr_rd		:= word_per_select_dout( CPU_NR,		v_reg_rd, v_cpu_nr		);

		v_per_dout	:=  v_ie1_rd or v_ifg1_rd or v_cpu_id_lo_rd or v_cpu_id_hi_rd or v_cpu_nr_rd;


		--=============================================================================
		-- 5)  NMI GENERATION
		--=============================================================================

		-- NOTE THAT THE NMI INPUT IS ASSUMED TO BE NON-GLITCHY
		if (NMI_EN = true) then

			-------------------------------------
			-- Edge selection
			-------------------------------------
			v_nmi_pol := d.nmi xor d.wdtnmies;
			-------------------------------------
			-- Pulse capture and synchronization
			v_nmi_s	:= v_nmi_pol;
			-------------------------------------
			-- NMI Pending flag
			-------------------------------------
			-- Delay
			v.nmi_dly	:= v_nmi_s;
			-- Edge detection
			v_nmi_edge	:= not(r.nmi_dly) and v_nmi_s;
			-- NMI pending
			v_nmi_pnd	:= r.nmiifg and r.nmie;
			-- NMI wakeup
			v_nmi_wkup	:= '0';
		else
			v_nmi_pnd	:= '0';
			v_nmi_wkup	:= '0';
		end if;

		if (NMI_EN = true) then
				if (v_nmi_edge = '1') then
					v.nmiifg  := '1';
				elsif (v_ifg1_wr = '1') then
					v.nmiifg  := v_ifg1_nxt(4);
				end if;
		else
			v.nmiifg	:=	'0';
		end if;

		-- drive register inputs
		rin <= v;

		-- drive module outputs
		cpu_id			<= v_cpu_id_hi & v_cpu_id_lo;
		nmi_pnd			<= v_nmi_pnd;			-- NMI pending
		nmi_wkup			<= v_nmi_wkup;			-- NMI wakeup
		per_dout			<= v_per_dout;			-- Peripheral data output
		wdtie				<= r.wdtie;				-- Watchdog-timer interrupt enable
		wdtifg_sw_clr	<= v_wdtifg_sw_clr;	-- Watchdog-timer interrupt flag software clear
		wdtifg_sw_set	<= v_wdtifg_sw_set;	-- Watchdog-timer interrupt flag software set

	end process COMB;

	REGS : process (mclk,mrst)
	begin
		if (mrst = '1') then
			r.nmie		<=	'0';
			r.nmi_dly	<=	'0';
			r.wdtie		<=	'0';
			r.nmiifg		<=	'0';
			r.wdtifg		<=	'0';
			r.wdt_reset	<=	'0';
		elsif rising_edge(mclk) then
			r	<= rin;
		end if;
	end process REGS;

end RTL;