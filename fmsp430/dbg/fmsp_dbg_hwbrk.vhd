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
--! @file fmsp_dbg_hwbrk.vhd
--! 
--! @brief fpgaMSP430 Hardware Breakpoint / Watchpoint module
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
	use work.fmsp_dbg_package.all;

entity fmsp_dbg_hwbrk is 
	generic (
		DBG_HWBRK_EN	: boolean := false		--	Include hardware breakpoints unit 
	);
	port (
		dbg_clk			: in	std_logic;       						--! Debug unit clock
		dbg_rst			: in	std_logic;       						--! Debug unit reset
		--! INPUTs
		brk_reg_rd		: in	std_logic_vector(3 downto 0);		--! Hardware break/watch-point register read select
		brk_reg_wr		: in	std_logic_vector(3 downto 0);		--! Hardware break/watch-point register write select
		dbg_din			: in	std_logic_vector(15 downto 0);	--! Debug register data input
		decode_noirq	: in	std_logic;								--! Frontend decode instruction
		eu_mab			: in	std_logic_vector(15 downto 0);	--! Execution-Unit Memory address bus
		eu_mb_en			: in	std_logic;								--! Execution-Unit Memory bus enable
		eu_mb_wr			: in	std_logic_vector(1 downto 0);		--! Execution-Unit Memory bus write transfer
		pc					: in	std_logic_vector(15 downto 0);	--! Program counter
		--! OUTPUTs
		brk_halt			: out	std_logic;								--! Hardware breakpoint command
		brk_pnd			: out	std_logic;								--! Hardware break/watch-point pending
		brk_dout			: out	std_logic_vector(15 downto 0)	--! Hardware break/watch-point register data input
	);
end entity fmsp_dbg_hwbrk;

architecture RTL of fmsp_dbg_hwbrk is 

	constant	C_HWBRK_RANGE	: std_logic := '0';
	
	constant	BRK_CTL		: integer := 0;
	constant	BRK_STAT		: integer := 1;
	constant	BRK_ADDR0	: integer := 2;
	constant	BRK_ADDR1	: integer := 3;

	type fmsp_dbg_hwbrk_in_type is record
		brk_reg_rd		: std_logic_vector(3 downto 0);	--! Hardware break/watch-point register read select
		brk_reg_wr		: std_logic_vector(3 downto 0);	--! Hardware break/watch-point register write select
		dbg_din			: std_logic_vector(15 downto 0);	--! Debug register data input
		decode_noirq	: std_logic;							--! Frontend decode instruction
		eu_mab			: std_logic_vector(15 downto 0);	--! Execution-Unit Memory address bus
		eu_mb_en			: std_logic;							--! Execution-Unit Memory bus enable
		eu_mb_wr			: std_logic_vector(1 downto 0);	--! Execution-Unit Memory bus write transfer
		pc					: std_logic_vector(15 downto 0);	--! Program counter
	end record;

	type reg_type is record
		brk_ctl			: std_logic_vector(4 downto 0);
		brk_stat			: std_logic_vector(5 downto 0);
		brk_addr0		: std_logic_vector(15 downto 0);
		brk_addr1		: std_logic_vector(15 downto 0);
	end record;

	signal	d		: fmsp_dbg_hwbrk_in_type;
	signal	r		: reg_type :=	(	brk_ctl		=> "00000",
												brk_stat		=> "000000",
												brk_addr0	=> x"0000",
												brk_addr1	=> x"0000"
											);
	signal	rin	: reg_type;

begin

	d.brk_reg_rd	<=	brk_reg_rd;
	d.brk_reg_wr	<=	brk_reg_wr;
	d.dbg_din		<=	dbg_din;
	d.decode_noirq	<=	decode_noirq;
	d.eu_mab			<=	eu_mab;
	d.eu_mb_en		<=	eu_mb_en;
	d.eu_mb_wr		<=	eu_mb_wr;
	d.pc				<=	pc;

	COMB : process (d, r)
		variable	v						: reg_type;
		variable	v_brk_ctl_wr		: std_logic;
		variable	v_brk_ctl_full		: std_logic_vector(7 downto 0);
		variable	v_brk_stat_wr		: std_logic;
		variable	v_brk_stat_set		: std_logic_vector(5 downto 0);
		variable	v_brk_stat_clr		: std_logic_vector(5 downto 0);
		variable	v_brk_stat_full	: std_logic_vector(7 downto 0);
		variable	v_brk_pnd			: std_logic;
		variable	v_brk_addr0_wr		: std_logic;
		variable	v_brk_addr1_wr		: std_logic;
		variable	v_brk_ctl_rd		: std_logic_vector(15 downto 0);
		variable	v_brk_stat_rd		: std_logic_vector(15 downto 0);
		variable	v_brk_addr0_rd		: std_logic_vector(15 downto 0);
		variable	v_brk_addr1_rd		: std_logic_vector(15 downto 0);
		variable	v_brk_dout			: std_logic_vector(15 downto 0);
		variable	v_equ_d_addr0		: std_logic;
		variable	v_equ_d_addr1		: std_logic;
		variable	v_equ_d_range		: std_logic;
		variable	v_equ_i_addr0		: std_logic;
		variable	v_equ_i_addr1		: std_logic;
		variable	v_equ_i_range		: std_logic;
		--! Detect Instruction read access
		variable	v_i_addr0_rd		: std_logic;
		variable	v_i_addr1_rd		: std_logic;
		variable	v_i_range_rd		: std_logic;
		--! Detect Execution-Unit write access
		variable	v_d_addr0_wr		: std_logic;
		variable	v_d_addr1_wr		: std_logic;
		variable	v_d_range_wr		: std_logic;
		--! Detect DATA read acces
		variable	v_d_addr0_rd		: std_logic;
		variable	v_d_addr1_rd		: std_logic;
		variable	v_d_range_rd		: std_logic;
		--! Set flags
		variable	v_addr0_rd_set		: std_logic;
		variable	v_addr0_wr_set		: std_logic;
		variable	v_addr1_rd_set		: std_logic;
		variable	v_addr1_wr_set		: std_logic;
		variable	v_range_rd_set		: std_logic;
		variable	v_range_wr_set		: std_logic;
		--! Break CPU
		variable	v_brk_halt			: std_logic;

	begin
	
		--! default assignment
		v := r;
		--! overriding assignments

		--============================================================================
		--! 4) BREAKPOINT / WATCHPOINT GENERATION
		--============================================================================

		--! Comparators
		-----------------------------
		--! Note: here the comparison logic is instanciated several times in order
		--!       to improve the timings, at the cost of a bit more area.
		v_equ_d_addr0	:=	'0';
		v_equ_d_addr1	:=	'0';
		v_equ_d_range	:=	'0';
		if ( UNSIGNED(d.eu_mab) = UNSIGNED(r.brk_addr0) ) then
			v_equ_d_addr0	:=	d.eu_mb_en and not(r.brk_ctl(C_BRK_RANGE));
		end if;
		if ( UNSIGNED(d.eu_mab) = UNSIGNED(r.brk_addr1) ) then
			v_equ_d_addr1	:=	d.eu_mb_en and not(r.brk_ctl(C_BRK_RANGE));
		end if;
		if (		( UNSIGNED(d.eu_mab) >= UNSIGNED(r.brk_addr0) )
				or	( UNSIGNED(d.eu_mab) <= UNSIGNED(r.brk_addr1) )	)	then
			v_equ_d_range	:=	d.eu_mb_en and not(r.brk_ctl(C_BRK_RANGE)) and C_HWBRK_RANGE;
		end if;

		v_equ_i_addr0	:=	'0';
		v_equ_i_addr1	:=	'0';
		v_equ_i_range	:=	'0';
		if ( UNSIGNED(d.pc) = UNSIGNED(r.brk_addr0) ) then
			v_equ_i_addr0	:=	d.decode_noirq and not(r.brk_ctl(C_BRK_RANGE));
		end if;
		if ( UNSIGNED(d.pc) = UNSIGNED(r.brk_addr1) ) then
			v_equ_i_addr1	:=	d.decode_noirq and not(r.brk_ctl(C_BRK_RANGE));
		end if;
		if (		( UNSIGNED(d.pc) >= UNSIGNED(r.brk_addr0) )
				or	( UNSIGNED(d.pc) <= UNSIGNED(r.brk_addr1) )	)	then
			v_equ_i_range	:=	d.decode_noirq and not(r.brk_ctl(C_BRK_RANGE)) and C_HWBRK_RANGE;
		end if;

		--! Detect accesses
		-----------------------------

		--! Detect Instruction read access
		v_i_addr0_rd	:=	v_equ_i_addr0 and r.brk_ctl(C_BRK_I_EN);
		v_i_addr1_rd	:=	v_equ_i_addr1 and r.brk_ctl(C_BRK_I_EN);
		v_i_range_rd	:=	v_equ_i_range and r.brk_ctl(C_BRK_I_EN);

		--! Detect Execution-Unit write access
		v_d_addr0_wr	:=	v_equ_d_addr0 and not(r.brk_ctl(C_BRK_I_EN)) and (d.eu_mb_wr(0) or d.eu_mb_wr(1));
		v_d_addr1_wr	:=	v_equ_d_addr1 and not(r.brk_ctl(C_BRK_I_EN)) and (d.eu_mb_wr(0) or d.eu_mb_wr(1));
		v_d_range_wr	:=	v_equ_d_range and not(r.brk_ctl(C_BRK_I_EN)) and (d.eu_mb_wr(0) or d.eu_mb_wr(1));

		--! Detect DATA read access
		v_d_addr0_rd	:=	v_equ_d_addr0 and not(r.brk_ctl(C_BRK_I_EN)) and not(d.eu_mb_wr(0) or d.eu_mb_wr(1));
		v_d_addr1_rd	:=	v_equ_d_addr1 and not(r.brk_ctl(C_BRK_I_EN)) and not(d.eu_mb_wr(0) or d.eu_mb_wr(1));
		v_d_range_rd	:=	v_equ_d_range and not(r.brk_ctl(C_BRK_I_EN)) and not(d.eu_mb_wr(0) or d.eu_mb_wr(1));

		--! Set flags
		v_addr0_rd_set	:=	r.brk_ctl(C_BRK_MODE_RD) and (v_d_addr0_rd  or v_i_addr0_rd);
		v_addr0_wr_set	:=	r.brk_ctl(C_BRK_MODE_WR) and  v_d_addr0_wr;
		v_addr1_rd_set	:=	r.brk_ctl(C_BRK_MODE_RD) and (v_d_addr1_rd  or v_i_addr1_rd);
		v_addr1_wr_set	:=	r.brk_ctl(C_BRK_MODE_WR) and  v_d_addr1_wr;
		v_range_rd_set	:=	r.brk_ctl(C_BRK_MODE_RD) and (v_d_range_rd  or v_i_range_rd);
		v_range_wr_set	:=	r.brk_ctl(C_BRK_MODE_WR) and  v_d_range_wr;

		--=============================================================================
		--! 2)  CONFIGURATION REGISTERS
		--=============================================================================

		--! BRK_CTL Register
		-------------------------------------------------------------------------------
		--!       7   6   5        4            3          2            1  0
		--!        Reserved    RANGE_MODE    INST_EN    BREAK_EN    ACCESS_MODE
		--
		--! ACCESS_MODE: - 00 : Disabled
		--!              - 01 : Detect read access
		--!              - 10 : Detect write access
		--!              - 11 : Detect read/write access
		--!              NOTE: '10' & '11' modes are not supported on the instruction flow
		--
		--! BREAK_EN:    -  0 : Watchmode enable
		--!              -  1 : Break enable
		--
		--! INST_EN:     -  0 : Checks are done on the execution unit (data flow)
		--!              -  1 : Checks are done on the frontend (instruction flow)
		--
		--! RANGE_MODE:  -  0 : Address match on BRK_ADDR0 or BRK_ADDR1
		--!              -  1 : Address match on BRK_ADDR0->BRK_ADDR1 range
		--
		-------------------------------------------------------------------------------
		v_brk_ctl_wr := d.brk_reg_wr(BRK_CTL);

		if (v_brk_ctl_wr) then
			v.brk_ctl :=  (C_HWBRK_RANGE and d.dbg_din(4)) & d.dbg_din(3 downto 0);
		end if;

		v_brk_ctl_full	:=	"000" & r.brk_ctl;


		--! BRK_STAT Register
		-------------------------------------------------------------------------------
		--!     7    6       5         4         3         2         1         0
		--!    Reserved  RANGE_WR  RANGE_RD  ADDR1_WR  ADDR1_RD  ADDR0_WR  ADDR0_RD
		-------------------------------------------------------------------------------
		v_brk_stat_wr	:=	d.brk_reg_wr(BRK_STAT);
		v_brk_stat_set	:=		(v_range_wr_set and C_HWBRK_RANGE)
								&	(v_range_rd_set and C_HWBRK_RANGE)
								&	v_addr1_wr_set & v_addr1_rd_set
								&	v_addr0_wr_set & v_addr0_rd_set;
		v_brk_stat_clr	:=	not(d.dbg_din(5 downto 0));

		if (v_brk_stat_wr) then
			v.brk_stat	:=	v_brk_stat_set or (r.brk_stat and v_brk_stat_clr);
		else
			v.brk_stat	:=	v_brk_stat_set or  r.brk_stat;
		end if;

		v_brk_stat_full	:=	"00" & r.brk_stat;
		v_brk_pnd			:=	r.brk_stat(0) or r.brk_stat(1) or r.brk_stat(2) or r.brk_stat(3) or r.brk_stat(4) or r.brk_stat(5);


		--! BRK_ADDR0 Register
		-------------------------------------------------------------------------------
		v_brk_addr0_wr	:=	d.brk_reg_wr(BRK_ADDR0);

		if (v_brk_addr0_wr) then
			v.brk_addr0	:=	d.dbg_din;
		end if;


		--! BRK_ADDR1/DATA0 Register
		-------------------------------------------------------------------------------
		v_brk_addr1_wr	:=	d.brk_reg_wr(BRK_ADDR1);
		if (v_brk_addr1_wr) then
			v.brk_addr1	:=	d.dbg_din;
		end if;


		--============================================================================
		--! 3) DATA OUTPUT GENERATION
		--============================================================================

		v_brk_ctl_rd	:=	x"0000";
		v_brk_stat_rd	:=	x"0000";
		v_brk_addr0_rd	:=	x"0000";
		v_brk_addr1_rd	:=	x"0000";
		if ( d.brk_reg_rd(BRK_CTL) = '1' ) then
			v_brk_ctl_rd	:=	x"00" & v_brk_ctl_full;
		end if;
		if ( d.brk_reg_rd(BRK_STAT) = '1' ) then
			v_brk_stat_rd	:=	x"00" & v_brk_stat_full;
		end if;
		if ( d.brk_reg_rd(BRK_CTL) = '1' ) then
			v_brk_addr0_rd	:=	r.brk_addr0;
		end if;
		if ( d.brk_reg_rd(BRK_STAT) = '1' ) then
			v_brk_addr1_rd	:=	r.brk_addr1;
		end if;

		v_brk_dout	:=		v_brk_ctl_rd
							or	v_brk_stat_rd
							or	v_brk_addr0_rd
							or	v_brk_addr1_rd;



		--! Break CPU
		v_brk_halt	:=	'0';
		if ( v_brk_stat_set /= "000000" ) then
			v_brk_halt	:=	r.brk_ctl(C_BRK_EN);
		end if;
		
		if (DBG_HWBRK_EN = false) then
			v_brk_halt	:=	'0';
			v_brk_pnd	:=	'0';
			v_brk_dout	:=	x"0000";
		end if;

		--! drive register inputs
		rin <= v;

		--! drive module outputs
		brk_halt		<= v_brk_halt;			--! Hardware breakpoint command
		brk_pnd		<= v_brk_pnd;			--! Hardware break/watch-point pending
		brk_dout		<= v_brk_dout;			--! Hardware break/watch-point register data input

	end process COMB;

	REGS : process (dbg_clk,dbg_rst)
	begin
		if (dbg_rst = '1') then
			r	<=	(	brk_ctl		=> "00000",
						brk_stat		=> "000000",
						brk_addr0	=> x"0000",
						brk_addr1	=> x"0000"
					);
		elsif rising_edge(dbg_clk) then
			r	<= rin;
		end if;
	end process REGS;


end RTL;