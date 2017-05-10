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
--! @file fmsp_dbg.vhd
--! 
--! @brief fpgaMSP430 Debug interface
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
	use work.fmsp_functions.all;
	use work.fmsp_dbg_package.all;

entity fmsp_dbg is 
	generic (
		DBG_UART					: boolean := false;		--! Enable UART (8N1) debug interface
		DBG_I2C					: boolean := true;		--! Enable I2C debug interface
		DBG_I2C_BROADCAST_EN	: boolean := false;		--! Enable the I2C broadcast address
		DBG_RST_BRK_EN			: boolean := false;		--! CPU break on PUC reset
		DBG_HWBRK_0_EN			: boolean := false;		--! Include hardware breakpoints unit 
		DBG_HWBRK_1_EN			: boolean := false;		--! Include hardware breakpoints unit 
		DBG_HWBRK_2_EN			: boolean := false;		--! Include hardware breakpoints unit 
		DBG_HWBRK_3_EN			: boolean := false;		--! Include hardware breakpoints unit 
		DBG_HWBRK_RANGE		: boolean := true;		--! Enable/Disable the hardware breakpoint RANGE mode
		DBG_UART_AUTO_SYNC	: boolean := true;		--! Debug UART interface auto data synchronization
		DBG_UART_BAUD			: integer := 9600;		--! Debug UART interface data rate
		DBG_DCO_FREQ			: integer := 20000000;	--! Debug DCO_CLK frequency
		SYNC_DBG_UART_RXD		: boolean := true			--! Synchronize RXD inputs
	);
	port (
		dbg_clk				: in	std_logic;       						--! Debug unit clock
		dbg_rst				: in	std_logic;       						--! Debug unit reset
		--! INPUTs
		cpu_en_s				: in	std_logic;       						--! Enable CPU code execution (synchronous)
		cpu_id				: in	std_logic_vector(31 downto 0);	--! CPU ID
		cpu_nr_inst			: in	std_logic_vector(7 downto 0);    --! Current fmsp instance number
		cpu_nr_total		: in	std_logic_vector(7 downto 0);		--! Total number of fmsp instances-1
		dbg_en_s				: in	std_logic;       						--! Debug interface enable (synchronous)
		dbg_halt_st			: in	std_logic;       						--! Halt/Run status from CPU
		dbg_i2c_addr		: in	std_logic_vector(6 downto 0);		--! Debug interface: I2C Address
		dbg_i2c_broadcast	: in	std_logic_vector(6 downto 0);		--! Debug interface: I2C Broadcast Address (for multicore systems)
		dbg_i2c_scl			: in	std_logic;       						--! Debug interface: I2C SCL
		dbg_i2c_sda_in		: in	std_logic;       						--! Debug interface: I2C SDA IN
		dbg_mem_din			: in	std_logic_vector(15 downto 0);	--! Debug unit Memory data input
		dbg_reg_din			: in	std_logic_vector(15 downto 0);	--! Debug unit CPU register data input
		dbg_uart_rxd		: in	std_logic;       						--! Debug interface: UART RXD (asynchronous)
		decode_noirq		: in	std_logic;       						--! Frontend decode instruction
		eu_mab				: in	std_logic_vector(15 downto 0);	--! Execution-Unit Memory address bus
		eu_mb_en				: in	std_logic;       						--! Execution-Unit Memory bus enable
		eu_mb_wr				: in	std_logic_vector(1 downto 0);		--! Execution-Unit Memory bus write transfer
		fe_mdb_in			: in	std_logic_vector(15 downto 0);  	--! Frontend Memory data bus input
		pc						: in	std_logic_vector(15 downto 0);	--! Program counter
		puc_pnd_set			: in	std_logic;       						--! PUC pending set for the serial debug interface
		--! OUTPUTs
		dbg_cpu_reset		: out	std_logic;       						--! Reset CPU from debug interface
		dbg_freeze			: out	std_logic;       						--! Freeze peripherals
		dbg_halt_cmd		: out	std_logic;       						--! Halt CPU command
		dbg_i2c_sda_out	: out	std_logic := '1';   					--! Debug interface: I2C SDA OUT
		dbg_mem_addr		: out	std_logic_vector(15 downto 0);	--! Debug address for rd/wr access
		dbg_mem_dout		: out	std_logic_vector(15 downto 0);	--! Debug unit data output
		dbg_mem_en			: out	std_logic;       						--! Debug unit memory enable
		dbg_mem_wr			: out	std_logic_vector(1 downto 0);		--! Debug unit memory write
		dbg_reg_wr			: out	std_logic;       						--! Debug unit CPU register write
		dbg_uart_txd		: out	std_logic := '1' 						--! Debug interface: UART TXD
	);
end entity fmsp_dbg;

architecture RTL of fmsp_dbg is 

	--! Debug interface: Software breakpoint opcode
	constant	DBG_SWBRK_OP			: std_logic_vector(15 downto 0) :=x"4343";
----! State machine definition
	constant	M_IDLE			: std_logic_vector(1 downto 0) := "00";
	constant	M_SET_BRK		: std_logic_vector(1 downto 0) := "01";
	constant	M_ACCESS_BRK	: std_logic_vector(1 downto 0) := "10";
	constant	M_ACCESS			: std_logic_vector(1 downto 0) := "11";
--! Debug interface
	constant	DBG_UART_WR	: integer := 18;
	constant	DBG_UART_BW	: integer := 17;
--	constant	DBG_UART_ADDR 16:11

--! Debug interface CPU_CTL register
	constant	HALT			: integer := 0;
	constant	RUN			: integer := 1;
	constant	ISTEP			: integer := 2;
	constant	SW_BRK_EN	: integer := 3;
	constant	FRZ_BRK_EN	: integer := 4;
	constant	RST_BRK_EN	: integer := 5;
	constant	CPU_RST		: integer := 6;

--! Debug interface CPU_STAT register
	constant	HALT_RUN		: integer := 0;
	constant	PUC_PND		: integer := 1;
	constant	SWBRK_PND	: integer := 3;
	constant	HWBRK0_PND	: integer := 4;
	constant	HWBRK1_PND	: integer := 5;

--! Debug interface BRKx_CTL register
	constant	BRK_MODE_RD	: integer := 0;
	constant	BRK_MODE_WR	: integer := 1;
--	constant	BRK_MODE    1:0
	constant	BRK_EN		: integer := 2;
	constant	BRK_I_EN		: integer := 3;
	constant	BRK_RANGE	: integer := 4;

--! Number of registers
	constant	NR_REG		: integer := 25;

--! Register addresses
	constant	CPU_ID_LO	: integer := 00;
	constant	CPU_ID_HI	: integer := 01;
	constant	CPU_CTL		: integer := 02;
	constant	CPU_STAT		: integer := 03;
	constant	MEM_CTL		: integer := 04;
	constant	MEM_ADDR		: integer := 05;
	constant	C_MEM_DATA		: integer := 06;
	constant	MEM_CNT		: integer := 07;
	constant	BRK0_CTL		: integer := 08;
	constant	BRK0_STAT	: integer := 09;
	constant	BRK0_ADDR0	: integer := 10;
	constant	BRK0_ADDR1	: integer := 11;
	constant	BRK1_CTL		: integer := 12;
	constant	BRK1_STAT	: integer := 13;
	constant	BRK1_ADDR0	: integer := 14;
	constant	BRK1_ADDR1	: integer := 15;
	constant	BRK2_CTL		: integer := 16;
	constant	BRK2_STAT	: integer := 17;
	constant	BRK2_ADDR0	: integer := 18;
	constant	BRK2_ADDR1	: integer := 19;
	constant	BRK3_CTL		: integer := 20;
	constant	BRK3_STAT	: integer := 21;
	constant	BRK3_ADDR0	: integer := 22;
	constant	BRK3_ADDR1	: integer := 23;
	constant	CPU_NR		: integer := 24;

	type fmsp_dbg_in_type is record
		cpu_en_s				: std_logic;       						--! Enable CPU code execution (synchronous)
		cpu_id				: std_logic_vector(31 downto 0);	--! CPU ID
		cpu_nr_inst			: std_logic_vector(7 downto 0);    --! Current fmsp instance number
		cpu_nr_total		: std_logic_vector(7 downto 0);		--! Total number of fmsp instances-1
		dbg_en_s				: std_logic;       						--! Debug interface enable (synchronous)
		dbg_halt_st			: std_logic;       						--! Halt/Run status from CPU
		dbg_i2c_addr		: std_logic_vector(6 downto 0);		--! Debug interface: I2C Address
		dbg_i2c_broadcast	: std_logic_vector(6 downto 0);		--! Debug interface: I2C Broadcast Address (for multicore systems)
		dbg_i2c_scl			: std_logic;       						--! Debug interface: I2C SCL
		dbg_i2c_sda_in		: std_logic;       						--! Debug interface: I2C SDA IN
		dbg_mem_din			: std_logic_vector(15 downto 0);	--! Debug unit Memory data input
		dbg_reg_din			: std_logic_vector(15 downto 0);	--! Debug unit CPU register data input
		dbg_uart_rxd		: std_logic;       						--! Debug interface: UART RXD (asynchronous)
		decode_noirq		: std_logic;       						--! Frontend decode instruction
		eu_mab				: std_logic_vector(15 downto 0);	--! Execution-Unit Memory address bus
		eu_mb_en				: std_logic;       						--! Execution-Unit Memory bus enable
		eu_mb_wr				: std_logic_vector(1 downto 0);		--! Execution-Unit Memory bus write transfer
		fe_mdb_in			: std_logic_vector(15 downto 0);  	--! Frontend Memory data bus input
		pc						: std_logic_vector(15 downto 0);	--! Program counter
		puc_pnd_set			: std_logic;       						--! PUC pending set for the serial debug interface
		--! frome Sub modules
		brk0_halt			: std_logic;
		brk0_pnd				: std_logic;
		brk0_dout			: std_logic_vector(15 downto 0);
		brk1_halt			: std_logic;
		brk1_pnd				: std_logic;
		brk1_dout			: std_logic_vector(15 downto 0);
		brk2_halt			: std_logic;
		brk2_pnd				: std_logic;
		brk2_dout			: std_logic_vector(15 downto 0);
		brk3_halt			: std_logic;
		brk3_pnd				: std_logic;
		brk3_dout			: std_logic_vector(15 downto 0);
		dbg_addr_rs232		: std_logic_vector(5 downto 0);
		dbg_din_rs232		: std_logic_vector(15 downto 0);
		dbg_wr_rs232		: std_logic;
		dbg_rd_rs232		: std_logic;
		dbg_addr_i2c		: std_logic_vector(5 downto 0);
		dbg_din_i2c			: std_logic_vector(15 downto 0);
		dbg_wr_i2c			: std_logic;
		dbg_rd_i2c			: std_logic;
	end record;

	type reg_type is record
		mem_burst		: std_logic;
		dbg_mem_rd_dly	: std_logic;
		dbg_rd_rdy		: std_logic;
		--! Register address decode
		reg_dec			: std_logic_vector(NR_REG-1 downto 0);
		cpu_ctl			: std_logic_vector(6 downto 3);
		cpu_stat			: std_logic_vector(3 downto 2);
		mem_ctl			: std_logic_vector(3 downto 1);
		mem_start		: std_logic;
		mem_data			: std_logic_vector(15 downto 0);
		mem_addr			: std_logic_vector(15 downto 0);
		mem_cnt			: std_logic_vector(15 downto 0);
		--! Single step
		inc_step			: std_logic_vector(1 downto 0);
		--! Run / Halt
		halt_flag		: std_logic;
		--! Memory access state machine
		mem_state		: std_logic_vector(1 downto 0);
		mem_state_nxt	: std_logic_vector(1 downto 0);
		--! Trigger CPU Register or memory access during a burst
		mem_startb		: std_logic;
	end record;

	signal	d		: fmsp_dbg_in_type;
	signal	r		: reg_type :=	(	mem_burst		=> '0',
												dbg_mem_rd_dly	=> '0',
												dbg_rd_rdy		=> '0',
												reg_dec			=> (Others => '0'),
												cpu_ctl			=> "0000",
												cpu_stat			=> "00",
												mem_ctl			=> "000",
												mem_start		=> '0',
												mem_data			=> x"0000",
												mem_addr			=> x"0000",
												mem_cnt			=> x"0000",
												inc_step			=> "00",
												halt_flag		=> '0',
												mem_state		=> "00",
												mem_state_nxt	=> "00",
												mem_startb		=> '0'
											);
	signal	rin	: reg_type;
	signal	mem_bw			: std_logic;
	--! Hardware Breakpoint/Watchpoint Register read select
	signal	brk0_reg_rd		: std_logic_vector(3 downto 0) := "0000";
	signal	brk1_reg_rd		: std_logic_vector(3 downto 0) := "0000";
	signal	brk2_reg_rd		: std_logic_vector(3 downto 0) := "0000";
	signal	brk3_reg_rd		: std_logic_vector(3 downto 0) := "0000";
	--! Hardware Breakpoint/Watchpoint Register write select
	signal	brk0_reg_wr		: std_logic_vector(3 downto 0) := "0000";
	signal	brk1_reg_wr		: std_logic_vector(3 downto 0) := "0000";
	signal	brk2_reg_wr		: std_logic_vector(3 downto 0) := "0000";
	signal	brk3_reg_wr		: std_logic_vector(3 downto 0) := "0000";
	signal	dbg_addr			: std_logic_vector(5 downto 0);
	signal	dbg_din			: std_logic_vector(15 downto 0);
	signal	dbg_wr			: std_logic;
	signal	dbg_rd			: std_logic;
	signal	dbg_dout			: std_logic_vector(15 downto 0);
	signal	mem_burst_end	: std_logic;
	signal	mem_burst_rd	: std_logic;
	signal	mem_burst_wr	: std_logic;

begin

		d.cpu_en_s				<=	cpu_en_s;				--! Enable CPU code execution (synchronous)
		d.cpu_id					<=	cpu_id;					--! CPU ID
		d.cpu_nr_inst			<=	cpu_nr_inst;    		--! Current fmsp instance number
		d.cpu_nr_total			<=	cpu_nr_total;			--! Total number of fmsp instances-1
		d.dbg_en_s				<=	dbg_en_s;       		--! Debug interface enable (synchronous)
		d.dbg_halt_st			<=	dbg_halt_st;      	--! Halt/Run status from CPU
		d.dbg_i2c_addr			<=	dbg_i2c_addr;			--! Debug interface: I2C Address
		d.dbg_i2c_broadcast	<=	dbg_i2c_broadcast;	--! Debug interface: I2C Broadcast Address (for multicore systems)
		d.dbg_i2c_scl			<=	dbg_i2c_scl;       	--! Debug interface: I2C SCL
		d.dbg_i2c_sda_in		<=	dbg_i2c_sda_in;   	--! Debug interface: I2C SDA IN
		d.dbg_mem_din			<=	dbg_mem_din;			--! Debug unit Memory data input
		d.dbg_reg_din			<=	dbg_reg_din;			--! Debug unit CPU register data input
		d.dbg_uart_rxd			<=	dbg_uart_rxd;     	--! Debug interface: UART RXD (asynchronous)
		d.decode_noirq			<=	decode_noirq;     	--! Frontend decode instruction
		d.eu_mab					<=	eu_mab;					--! Execution-Unit Memory address bus
		d.eu_mb_en				<=	eu_mb_en;       		--! Execution-Unit Memory bus enable
		d.eu_mb_wr				<=	eu_mb_wr;				--! Execution-Unit Memory bus write transfer
		d.fe_mdb_in				<=	fe_mdb_in;  			--! Frontend Memory data bus input
		d.pc						<=	pc;						--! Program counter
		d.puc_pnd_set			<=	puc_pnd_set;       	--! PUC pending set for the serial debug interface

	COMB : process (d, r)
		variable	v				: reg_type;
		--============================================================================
		--! 2)  REGISTER DECODER
		--============================================================================
		--=============================================================================
		--! 1)  		variable	v_& PARAMETER DECLARATION
		--=============================================================================

		--! Diverse wires and registers
		variable	v_dbg_addr			: std_logic_vector(5 downto 0);
		variable	v_dbg_din			: std_logic_vector(15 downto 0);
		variable	v_dbg_wr				: std_logic;
		variable	v_dbg_rd				: std_logic;
		--! Select Data register during a burst
		variable	v_dbg_addr_in		: std_logic_vector(5 downto 0);
		--! Register address decode
		variable	v_reg_dec			: std_logic_vector(63 downto 0);
		--! Read/Write probes
		variable	v_reg_write			: std_logic;
		variable	v_reg_read			: std_logic;
		--! Read/Write vectors
		variable	v_reg_wr				: std_logic_vector(NR_REG-1 downto 0);
		variable	v_reg_rd  			: std_logic_vector(NR_REG-1 downto 0);
--! 3)  REGISTER: CORE INTERFACE
--! CPU_NR Register
		variable	v_cpu_nr				: std_logic_vector(15 downto 0);
--! CPU_CTL Register
		variable	v_cpu_ctl_wr		: std_logic;
		variable	v_cpu_ctl_full		: std_logic_vector(7 downto 0);
		variable	v_halt_cpu			: std_logic;
		variable	v_run_cpu			: std_logic;
		variable	v_istep				: std_logic;
--! CPU_STAT Register
		variable	v_cpu_stat_wr		: std_logic;
		variable	v_cpu_stat_set		: std_logic_vector(3 downto 2);
		variable	v_cpu_stat_clr		: std_logic_vector(3 downto 2);
		variable	v_cpu_stat_full	: std_logic_vector(7 downto 0);
--! 4)  REGISTER: MEMORY INTERFACE
--! MEM_CTL Register
		variable	v_mem_ctl_wr		: std_logic;
		variable	v_mem_ctl_full		: std_logic_vector(7 downto 0);
		variable	v_mem_bw				: std_logic;
--! C_MEM_DATA Register
		variable	v_mem_access		: std_logic;
		variable	v_mem_data_wr		: std_logic;
		variable	v_dbg_mem_din_bw	: std_logic_vector(15 downto 0);
--! MEM_ADDR Register
		variable	v_mem_addr_wr		: std_logic;
		variable	v_dbg_mem_acc		: std_logic;
		variable	v_dbg_reg_acc		: std_logic;
		variable	v_mem_addr_inc		: std_logic_vector(15 downto 0);
--! MEM_CNT Register
		variable	v_mem_cnt_wr		: std_logic;
		variable	v_mem_cnt_dec		: std_logic_vector(15 downto 0);
--! 6) DATA OUTPUT GENERATION
		variable	v_cpu_id_lo_rd		: std_logic_vector(15 downto 0);
		variable	v_cpu_id_hi_rd		: std_logic_vector(15 downto 0);
		variable	v_cpu_ctl_rd		: std_logic_vector(15 downto 0);
		variable	v_cpu_stat_rd		: std_logic_vector(15 downto 0);
		variable	v_mem_ctl_rd		: std_logic_vector(15 downto 0);
		variable	v_mem_data_rd		: std_logic_vector(15 downto 0);
		variable	v_mem_addr_rd		: std_logic_vector(15 downto 0);
		variable	v_mem_cnt_rd		: std_logic_vector(15 downto 0);
		variable	v_cpu_nr_rd			: std_logic_vector(15 downto 0);
		variable	v_dbg_dout			: std_logic_vector(15 downto 0);
--! 7) CPU CONTROL
--! Reset CPU
		variable	v_dbg_cpu_reset	: std_logic;
--! Break after reset
		variable	v_halt_rst			: std_logic;
--! Freeze peripherals
		variable	v_dbg_freeze		: std_logic;
--! Software break
		variable	v_dbg_swbrk			: std_logic;
		variable	v_mem_halt_cpu		: std_logic;
		variable	v_mem_run_cpu		: std_logic;
		variable	v_halt_flag_clr	: std_logic;
		variable	v_halt_flag_set	: std_logic;
		variable	v_dbg_halt_cmd		: std_logic;
--! 8) MEMORY CONTROL
		variable	v_mem_state_nxt	: std_logic_vector(1 downto 0);
--! Control Memory bursts
		variable	v_mem_burst_start	: std_logic;
		variable	v_mem_burst_end	: std_logic;
--! Control signals for UART/I2C interface
		variable	v_mem_burst_rd		: std_logic;
		variable	v_mem_burst_wr		: std_logic;
--! Combine single and burst memory start of sequence
		variable	v_mem_seq_start	: std_logic;
--! Interface to CPU Registers and Memory bacbkone
		variable	v_dbg_mem_addr		: std_logic_vector(15 downto 0);
		variable	v_dbg_mem_dout		: std_logic_vector(15 downto 0);
		variable	v_dbg_reg_wr		: std_logic;
		variable	v_dbg_reg_rd		: std_logic;
		variable	v_dbg_mem_en		: std_logic;
		variable	v_dbg_mem_rd		: std_logic;
		variable	v_dbg_mem_wr_msk	: std_logic_vector(1 downto 0);
		variable	v_dbg_mem_wr		: std_logic_vector(1 downto 0);
		--! UNUSED UART COMMUNICATION
		variable	UNUSED_dbg_uart_rxd		: std_logic;
		--! UNUSED I2C COMMUNICATION
		variable	UNUSED_dbg_i2c_addr			: std_logic_vector(6 downto 0);
		variable	UNUSED_dbg_i2c_broadcast	: std_logic_vector(6 downto 0);
		variable	UNUSED_dbg_i2c_scl			: std_logic;
		variable	UNUSED_dbg_i2c_sda_in		: std_logic;
		variable	UNUSED_dbg_rd_rdy				: std_logic;

begin
		--! default assignment
		v := r;
		--! overriding assignments

		if (DBG_UART = true) then
			v_dbg_addr	:=	d.dbg_addr_rs232;	--! Debug register address
			v_dbg_din	:=	d.dbg_din_rs232;	--! Debug register data input
			v_dbg_rd		:=	d.dbg_rd_rs232;	--! Debug register data read
			v_dbg_wr		:=	d.dbg_wr_rs232;	--! Debug register data write
		elsif (DBG_I2C = true) then
			v_dbg_addr	:=	d.dbg_addr_i2c;	--! Debug register address
			v_dbg_din	:=	d.dbg_din_i2c;		--! Debug register data input
			v_dbg_rd		:=	d.dbg_rd_i2c;		--! Debug register data read
			v_dbg_wr		:=	d.dbg_wr_i2c;		--! Debug register data write
		else
			v_dbg_addr	:=	"000000";			--! Debug register address
			v_dbg_din	:=	x"0000";				--! Debug register data input
			v_dbg_rd		:=	'0';					--! Debug register data read
			v_dbg_wr		:=	'0';					--! Debug register data writ
		end if;

		--============================================================================
		--! 2)  REGISTER DECODER
		--============================================================================

		--! Select Data register during a burst
		if (r.mem_burst = '1') then
			v_dbg_addr_in	:=	STD_LOGIC_VECTOR(TO_UNSIGNED(C_MEM_DATA,6));
		else 
			v_dbg_addr_in	:=	v_dbg_addr;
		end if;

		--! Register address decode
		v_reg_dec	:=	onehot(v_dbg_addr_in);

		--! Read/Write probes
		v_reg_write		:=	v_dbg_wr;
		v_reg_read		:=	'1';

		--! Read/Write vectors
		if (v_reg_write = '1') then
			v_reg_wr	:=	v_reg_dec(NR_REG-1 downto 0);
		else 
			v_reg_wr	:=	(Others => '0');
		end if;
		if (v_reg_read = '1') then
			v_reg_rd	:=	v_reg_dec(NR_REG-1 downto 0);
		else 
			v_reg_rd	:=	(Others => '0');
		end if;


		--=============================================================================
		--! 3)  REGISTER: CORE INTERFACE
		--=============================================================================

		--! CPU_ID Register
		-------------------
		--!              -------------------------------------------------------------------
		--! CPU_ID_LO:  | 15  14  13  12  11  10  9  |  8  7  6  5  4  |  3   |   2  1  0   |
		--!             |----------------------------+-----------------+------+-------------|
		--!             |        PER_SPACE           |   USER_VERSION  | ASIC | CPU_VERSION |
		--!              --------------------------------------------------------------------
		--! CPU_ID_HI:  |   15  14  13  12  11  10   |   9  8  7  6  5  4  3  2  1   |   0  |
		--!             |----------------------------+-------------------------------+------|
		--!             |         PMEM_SIZE          |            DMEM_SIZE          |  MPY |
		--!              -------------------------------------------------------------------

		--! This register is assigned in the SFR module


		--! CPU_NR Register
		-------------------
		--!    -------------------------------------------------------------------
		--!   | 15  14  13  12  11  10   9   8  |  7   6   5   4   3   2   1   0  |
		--!   |---------------------------------+---------------------------------|
		--!   |            CPU_TOTAL_NR         |           CPU_INST_NR           |
		--!    -------------------------------------------------------------------

		v_cpu_nr	:=	d.cpu_nr_total & d.cpu_nr_inst;


		--! CPU_CTL Register
		-------------------------------------------------------------------------------
		--!       7         6          5          4           3        2     1    0
		--!   Reserved   CPU_RST  RST_BRK_EN  FRZ_BRK_EN  SW_BRK_EN  ISTEP  RUN  HALT
		-------------------------------------------------------------------------------
		v_cpu_ctl_wr	:=	v_reg_wr(CPU_CTL);

		if (v_cpu_ctl_wr = '1') then
			v.cpu_ctl	:=	v_dbg_din(6 downto 3);
		end if;
		
		v_cpu_ctl_full	:=	'0' & r.cpu_ctl & "000";

		v_halt_cpu	:=	v_cpu_ctl_wr and  v_dbg_din(HALT)	and not( d.dbg_halt_st);
		v_run_cpu	:=	v_cpu_ctl_wr and	v_dbg_din(RUN)		and		d.dbg_halt_st;
		v_istep		:=	v_cpu_ctl_wr and	v_dbg_din(ISTEP)	and		d.dbg_halt_st;


		--! Reset CPU
		v_dbg_cpu_reset	:=	r.cpu_ctl(CPU_RST);
		--! Break after reset
		v_halt_rst	:=	r.cpu_ctl(RST_BRK_EN) and d.dbg_en_s and d.puc_pnd_set;
		--! Freeze peripherals
		v_dbg_freeze	:=	d.dbg_halt_st and ( r.cpu_ctl(FRZ_BRK_EN) or not(d.cpu_en_s) );
		--! Software break
		v_dbg_swbrk	:=	'0';
		if (d.fe_mdb_in = DBG_SWBRK_OP) then
			v_dbg_swbrk	:=	d.decode_noirq and r.cpu_ctl(SW_BRK_EN);
		end if;
		--! Single step
		if (v_istep = '1') then
			v.inc_step	:=	"11";
		else
			v.inc_step	:=	r.inc_step(0) & '0';
		end if;
		--============================================================================
		--! 7) CPU CONTROL
		--============================================================================



		--! Run / Halt
		v_halt_flag_clr	:=	v_run_cpu or v_mem_run_cpu;
		v_halt_flag_set	:=	v_halt_cpu or v_halt_rst  or v_dbg_swbrk or v_mem_halt_cpu or d.brk0_halt or d.brk1_halt or d.brk2_halt or d.brk3_halt;

		if (v_halt_flag_clr = '1') then
			v.halt_flag	:=	'0';
		elsif (v_halt_flag_set = '1') then
			v.halt_flag	:=	'1';
		end if;

		v_dbg_halt_cmd	:=	(r.halt_flag or v_halt_flag_set) and not(r.inc_step(1));

		--! CPU_STAT Register
		--------------------------------------------------------------------------------------
		--!      7           6          5           4           3         2      1       0
		--! HWBRK3_PND  HWBRK2_PND  HWBRK1_PND  HWBRK0_PND  SWBRK_PND  PUC_PND  Res.  HALT_RUN
		--------------------------------------------------------------------------------------
		v_cpu_stat_wr	:=	v_reg_wr(CPU_STAT);
		v_cpu_stat_set	:=	v_dbg_swbrk & d.puc_pnd_set;
		v_cpu_stat_clr	:=	not(v_dbg_din(3 downto 2));

		if (v_cpu_stat_wr = '1') then
			v.cpu_stat	:=	v_cpu_stat_set or (r.cpu_stat and v_cpu_stat_clr);
		else 
			v.cpu_stat	:=	v_cpu_stat_set or r.cpu_stat;
		end if;
		v_cpu_stat_full	:=	d.brk3_pnd & d.brk2_pnd & d.brk1_pnd & d.brk0_pnd & r.cpu_stat & '0' & d.dbg_halt_st;


--		UNUSED_eu_mab		:=	eu_mab;
--		UNUSED_eu_mb_en	:=	eu_mb_en;
--		UNUSED_eu_mb_wr	:=	eu_mb_wr;
--		UNUSED_pc			:=	pc;



		--============================================================================
		--! 8) MEMORY CONTROL
		--============================================================================

		--! Control Memory bursts
		if ( (r.mem_cnt /= x"0000") and (r.mem_start = '1') ) then
			v_mem_burst_start	:=	'1';
		else
			v_mem_burst_start	:=	'0';
		end if;
		if ( (r.mem_cnt = x"0000") and ((v_dbg_wr or r.dbg_rd_rdy) = '1') ) then
			v_mem_burst_end	:=	'1';
		else
			v_mem_burst_end	:=	'0';
		end if;

		--! Detect when burst is on going
		if (v_mem_burst_start = '1') then
			v.mem_burst	:=	'1';
		elsif (v_mem_burst_end = '1') then
			v.mem_burst	:=	'0';
		end if;

		--! Control signals for UART/I2C interface
		v_mem_burst_rd	:=	v_mem_burst_start and not(r.mem_ctl(1));
		v_mem_burst_wr	:=	v_mem_burst_start and r.mem_ctl(1);

		--! Trigger CPU Register or memory access during a burst
		v.mem_startb	:=	(r.mem_burst and (v_dbg_wr or v_dbg_rd)) or v_mem_burst_rd;

		--! Combine single and burst memory start of sequence
		if (	(		(r.mem_cnt = x"0000")
				  and	(r.mem_start = '1') )
				or (r.mem_startb = '1') )then
			v_mem_seq_start	:=	'1';
		else
			v_mem_seq_start	:=	'0';
		end if;

		--! Memory access state machine
		--------------------------------
		--! State transition
		case (r.mem_state) is
			when	M_IDLE	=>
				if (not(v_mem_seq_start) = '1') then
					v_mem_state_nxt	:=	M_IDLE;
				elsif (d.dbg_halt_st = '1') then
					v_mem_state_nxt	:=	M_ACCESS;
				else
					v_mem_state_nxt	:=	M_SET_BRK;
				end if;
			when	M_SET_BRK	=>
				if (d.dbg_halt_st = '1') then
					v_mem_state_nxt	:=	M_ACCESS_BRK;
				else
					v_mem_state_nxt	:=	M_SET_BRK;
				end if;
			when	M_ACCESS_BRK	=>
				v_mem_state_nxt	:=	M_IDLE;
			when	M_ACCESS	=>
				v_mem_state_nxt	:=	M_IDLE;
			when	others	=>
				v_mem_state_nxt	:=	M_IDLE;
 		end case;

		--! State machine
		v.mem_state	:=	v_mem_state_nxt;

		--! Utility signals
		v_mem_halt_cpu	:=	'0';
		v_mem_run_cpu	:=	'0';
		v_mem_access	:=	'0';
		if (			(r.mem_state = M_IDLE)
				and	(v_mem_state_nxt = M_SET_BRK) ) then
			v_mem_halt_cpu	:=	'1';
		end if;
		if (			(r.mem_state = M_ACCESS_BRK)
				and	(v_mem_state_nxt = M_IDLE) ) then
			v_mem_run_cpu	:=	'1';
		end if;
		if (		(r.mem_state = M_ACCESS)
				or	(r.mem_state = M_ACCESS_BRK) ) then
			v_mem_access	:=	'1';
		end if;


--! Interface to CPU Registers and Memory bacbkone
--------------------------------------------------
		v_dbg_mem_addr	:=	r.mem_addr;
		if (not(v_mem_bw) = '1') then
			v_dbg_mem_dout	:=	r.mem_data;
		elsif (r.mem_addr(0) = '1') then
			v_dbg_mem_dout	:=	r.mem_data(7 downto 0) & x"00";
		else
			v_dbg_mem_dout	:=	x"00" & r.mem_data(7 downto 0);
		end if;

		v_dbg_reg_wr	:=	v_mem_access and r.mem_ctl(1) and  r.mem_ctl(2);
		v_dbg_reg_rd	:=	v_mem_access and not(r.mem_ctl(1)) and  r.mem_ctl(2);

		v_dbg_mem_en	:=	v_mem_access and not(r.mem_ctl(2));
		v_dbg_mem_rd	:=	dbg_mem_en	 and not(r.mem_ctl(1));

		if (not(v_mem_bw) = '1') then
			v_dbg_mem_wr_msk	:=	"11";
		elsif (r.mem_addr(0) = '1') then
			v_dbg_mem_wr_msk	:=	"10";
		else
			v_dbg_mem_wr_msk	:=	"01";
		end if;
		v_dbg_mem_wr	:=	"00";
		if (			(dbg_mem_en = '1')
			and	(r.mem_ctl(1) = '1') ) then
			v_dbg_mem_wr	:=	v_dbg_mem_wr_msk;
		end if;

		--! It takes one additional cycle to read from Memory as from registers
		v.dbg_mem_rd_dly	:=	v_dbg_mem_rd;


		--=============================================================================
		--! 4)  REGISTER: MEMORY INTERFACE
		--=============================================================================

		--! MEM_CTL Register
		-------------------------------------------------------------------------------
		--!       7     6     5     4          3        2         1       0
		--!            Reserved               B/W    MEM/REG    RD/WR   START
		--
		--! START  :  -  0 : Do nothing.
		--!           -  1 : Initiate memory transfer.
		--
		--! RD/WR  :  -  0 : Read access.
		--!           -  1 : Write access.
		--
		--! MEM/REG:  -  0 : Memory access.
		--!           -  1 : CPU Register access.
		--
		--! B/W    :  -  0 : 16 bit access.
		--!           -  1 :  8 bit access (not valid for CPU Registers).
		--
		-------------------------------------------------------------------------------
		v_mem_ctl_wr	:=	v_reg_wr(MEM_CTL);

		if (v_mem_ctl_wr = '1') then
			v.mem_ctl	:= v_dbg_din(3 downto 1);
		end if;

		v_mem_ctl_full	:= "0000" & r.mem_ctl & '0';

		v.mem_start	:= v_mem_ctl_wr and v_dbg_din(0);

		v_mem_bw	:=	r.mem_ctl(3);

		--! C_MEM_DATA Register
		--------------------
		v_mem_data_wr	:=	v_reg_wr(C_MEM_DATA);

		if (not(v_mem_bw) = '1') then
			v_dbg_mem_din_bw := d.dbg_mem_din;
		elsif (r.mem_addr(0) = '1') then
			v_dbg_mem_din_bw := x"00" & d.dbg_mem_din(15 downto 8);
		else
			v_dbg_mem_din_bw := x"00" & d.dbg_mem_din(7 downto 0);
		end if;

		if (v_mem_data_wr = '1') then
			v.mem_data	:= v_dbg_din;
		elsif (v_dbg_reg_rd = '1') then
			v.mem_data	:= d.dbg_reg_din;
		elsif (r.dbg_mem_rd_dly = '1') then
			v.mem_data	:= v_dbg_mem_din_bw;
		end if;


		--! MEM_ADDR Register
		--------------------
		v_mem_addr_wr	:=	v_reg_wr(MEM_ADDR);
		v_dbg_mem_acc	:=	( v_dbg_mem_wr(0) or v_dbg_mem_wr(1) or ( r.dbg_rd_rdy and not(r.mem_ctl(2)) ) );
		v_dbg_reg_acc	:=	( dbg_reg_wr or ( r.dbg_rd_rdy and r.mem_ctl(2) ) );

		if (r.mem_cnt = x"0000") then
			v_mem_addr_inc	:= x"0000";
		elsif ((r.mem_burst and v_dbg_mem_acc and not(v_mem_bw)) = '1') then
			v_mem_addr_inc	:= x"0000";
		elsif ((r.mem_burst and (v_dbg_mem_acc or v_dbg_reg_acc)) = '1') then
			v_mem_addr_inc	:= x"0001";
		else
			v_mem_addr_inc	:= x"0000";
		end if;

		if (v_mem_addr_wr = '1') then
			v.mem_addr	:= v_dbg_din;
		else
			v.mem_addr	:= STD_LOGIC_VECTOR( UNSIGNED(r.mem_addr) + UNSIGNED(v_mem_addr_inc) );
		end if;

		--! MEM_CNT Register
		--------------------

		v_mem_cnt_wr	:=	v_reg_wr(MEM_CNT);

		if (r.mem_cnt = x"0000") then
			v_mem_cnt_dec	:= x"0000";
		elsif ((r.mem_burst and (v_dbg_mem_acc or v_dbg_reg_acc)) = '1') then
			v_mem_cnt_dec	:= x"FFFF";
		else
			v_mem_cnt_dec	:= x"0000";
		end if;

		if (v_mem_cnt_wr = '1') then
			v.mem_cnt	:= v_dbg_din;
		else
			v.mem_cnt	:= STD_LOGIC_VECTOR( UNSIGNED(r.mem_cnt) + UNSIGNED(v_mem_cnt_dec) );
		end if;

		--=============================================================================
		--! 9)  UART COMMUNICATION
		--=============================================================================
		if (DBG_UART = false) then
			UNUSED_dbg_uart_rxd	:=	d.dbg_uart_rxd;
		end if;

		--=============================================================================
		--! 10)  I2C COMMUNICATION
		--=============================================================================
		if (DBG_I2C = false) then
			UNUSED_dbg_i2c_addr			:=	d.dbg_i2c_addr;
			UNUSED_dbg_i2c_broadcast	:=	d.dbg_i2c_broadcast;
			UNUSED_dbg_i2c_scl			:=	d.dbg_i2c_scl;
			UNUSED_dbg_i2c_sda_in		:=	d.dbg_i2c_sda_in;
			--v_UNUSED_dbg_rd_rdy			:=	d.dbg_rd_rdy;
		end if;

--============================================================================
--! 6) DATA OUTPUT GENERATION
--============================================================================

--		v_cpu_id_lo_rd	:= word_per_select_dout( CPU_ID_LO,		v_reg_rd, d.cpu_id(15 downto 0)		);
--		v_cpu_id_hi_rd	:= word_per_select_dout( CPU_ID_HI,		v_reg_rd, d.cpu_id(31 downto 16)		);
--		v_cpu_ctl_rd	:= word_per_select_dout( CPU_CTL,		v_reg_rd, (x"00" & v_cpu_ctl_full)	);
--		v_cpu_stat_rd	:= word_per_select_dout( CPU_STAT,		v_reg_rd, (x"00" & v_cpu_stat_full)	);
--		v_mem_ctl_rd	:= word_per_select_dout( MEM_CTL,		v_reg_rd, (x"00" & v_mem_ctl_full)	);
--		v_mem_data_rd	:= word_per_select_dout( C_MEM_DATA,	v_reg_rd, r.mem_data						);
--		v_mem_addr_rd	:= word_per_select_dout( MEM_ADDR,		v_reg_rd, r.mem_addr						);
--		v_mem_cnt_rd	:= word_per_select_dout( MEM_CNT,		v_reg_rd, r.mem_cnt						);
--		v_cpu_nr_rd		:= word_per_select_dout( CPU_NR,			v_reg_rd, v_cpu_nr						);
		if ( v_reg_rd(CPU_ID_LO) = '1' ) then
			v_cpu_id_lo_rd	:=	d.cpu_id(15 downto 0);
		else
			v_cpu_id_lo_rd	:=	x"0000";
		end if;
		if ( v_reg_rd(CPU_ID_HI) = '1' ) then
			v_cpu_id_hi_rd	:=	d.cpu_id(31 downto 16);
		else
			v_cpu_id_hi_rd	:=	x"0000";
		end if;
		if ( v_reg_rd(CPU_CTL) = '1' ) then
			v_cpu_ctl_rd	:=	x"00" & v_cpu_ctl_full;
		else
			v_cpu_ctl_rd	:=	x"0000";
		end if;
		if ( v_reg_rd(CPU_STAT) = '1' ) then
			v_cpu_stat_rd	:=	x"00" & v_cpu_stat_full;
		else
			v_cpu_stat_rd	:=	x"0000";
		end if;
		if ( v_reg_rd(MEM_CTL) = '1' ) then
			v_mem_ctl_rd	:=	x"00" & v_mem_ctl_full;
		else
			v_mem_ctl_rd	:=	x"0000";
		end if;
		if ( v_reg_rd(C_MEM_DATA) = '1' ) then
			v_mem_data_rd	:=	r.mem_data;
		else
			v_mem_data_rd	:=	x"0000";
		end if;
		if ( v_reg_rd(MEM_ADDR) = '1' ) then
			v_mem_addr_rd	:=	r.mem_addr;
		else
			v_mem_addr_rd	:=	x"0000";
		end if;
		if ( v_reg_rd(MEM_CNT) = '1' ) then
			v_mem_cnt_rd	:=	r.mem_cnt;
		else
			v_mem_cnt_rd	:=	x"0000";
		end if;
		if ( v_reg_rd(CPU_NR) = '1' ) then
			v_cpu_nr_rd		:=	v_cpu_nr;
		else
			v_cpu_nr_rd		:=	x"0000";
		end if;

		
		v_dbg_dout	:=	v_cpu_id_lo_rd	or
							v_cpu_id_hi_rd	or
							v_cpu_ctl_rd	or
							v_cpu_stat_rd	or
							v_mem_ctl_rd	or
							v_mem_data_rd	or
							v_mem_addr_rd	or
							v_mem_cnt_rd	or
							d.brk0_dout		or
							d.brk1_dout		or
							d.brk2_dout		or
							d.brk3_dout		or
							v_cpu_nr_rd;

		--! Tell UART/I2C interface that the data is ready to be read
		if ( (r.mem_burst = '1') or (v_mem_burst_rd = '1') ) then
			v.dbg_rd_rdy	:=	v_dbg_reg_rd or r.dbg_mem_rd_dly;
		else
			v.dbg_rd_rdy	:=	v_dbg_rd;
		end if;


--		if ( v_brk_stat_set /= "000000" ) then
--			v_brk_halt	:=	r.brk_ctl(C_BRK_EN);
--		end if;
		
		--! drive register inputs
		rin <= v;

		--! drive module outputs
--		brk_halt				<= v_brk_halt;				--! Hardware breakpoint command
--		brk_pnd				<= v_brk_pnd;				--! Hardware break/watch-point pending
--		brk_dout				<= v_brk_dout;				--! Hardware break/watch-point register data input
		dbg_cpu_reset		<= v_dbg_cpu_reset; 		--! Reset CPU from debug interface
		dbg_freeze			<= v_dbg_freeze;			--! Freeze peripherals
		dbg_halt_cmd		<= v_dbg_halt_cmd;		--! Halt CPU command
--		dbg_i2c_sda_out	<= v_dbg_i2c_sda_out;	--! Debug interface: I2C SDA OUT
		dbg_mem_addr		<= v_dbg_mem_addr;		--! Debug address for rd/wr access
		dbg_mem_dout		<= v_dbg_mem_dout;		--! Debug unit data output
		dbg_mem_en			<= v_dbg_mem_en;     	--! Debug unit memory enable
		dbg_mem_wr			<= v_dbg_mem_wr;			--! Debug unit memory write
		dbg_reg_wr			<= v_dbg_reg_wr;     	--! Debug unit CPU register write
--		dbg_uart_txd		<= v_dbg_uart_txd;  		--! Debug interface: UART TXD
		--! Hardware Breakpoint/Watchpoint Register read select
		brk0_reg_rd		<=	v_reg_rd(BRK0_ADDR1) & v_reg_rd(BRK0_ADDR0) & v_reg_rd(BRK0_STAT) & v_reg_rd(BRK0_CTL);
		brk1_reg_rd		<=	v_reg_rd(BRK1_ADDR1) & v_reg_rd(BRK1_ADDR0) & v_reg_rd(BRK1_STAT) & v_reg_rd(BRK1_CTL);
		brk2_reg_rd		<=	v_reg_rd(BRK2_ADDR1) & v_reg_rd(BRK2_ADDR0) & v_reg_rd(BRK2_STAT) & v_reg_rd(BRK2_CTL);
		brk3_reg_rd		<=	v_reg_rd(BRK3_ADDR1) & v_reg_rd(BRK3_ADDR0) & v_reg_rd(BRK3_STAT) & v_reg_rd(BRK3_CTL);
		--! Hardware Breakpoint/Watchpoint Register write select
		brk0_reg_wr		<=	v_reg_wr(BRK0_ADDR1) & v_reg_wr(BRK0_ADDR0) & v_reg_wr(BRK0_STAT) & v_reg_wr(BRK0_CTL);
		brk1_reg_wr		<=	v_reg_wr(BRK1_ADDR1) & v_reg_wr(BRK1_ADDR0) & v_reg_wr(BRK1_STAT) & v_reg_wr(BRK1_CTL);
		brk2_reg_wr		<=	v_reg_wr(BRK2_ADDR1) & v_reg_wr(BRK2_ADDR0) & v_reg_wr(BRK2_STAT) & v_reg_wr(BRK2_CTL);
		brk3_reg_wr		<=	v_reg_wr(BRK3_ADDR1) & v_reg_wr(BRK3_ADDR0) & v_reg_wr(BRK3_STAT) & v_reg_wr(BRK3_CTL);
		dbg_din			<=	v_dbg_din;	--! Debug register data input
		dbg_dout			<=	v_dbg_dout;	--! Debug register data output
		mem_burst_end	<=	v_mem_burst_end;
		mem_burst_rd	<=	v_mem_burst_rd;
		mem_burst_wr	<=	v_mem_burst_wr;
		mem_bw			<=	v_mem_bw;
	end process COMB;

	REGS : process (dbg_clk,dbg_rst)
	begin
		if (dbg_rst = '1') then
			if (DBG_RST_BRK_EN = true) then
				r.cpu_ctl	<=	x"6";
			else
				r.cpu_ctl	<=	x"2";
			end if;
			r.mem_burst			<=	'0';
			r.dbg_mem_rd_dly	<=	'0';
			r.dbg_rd_rdy		<=	'0';
			r.reg_dec			<=	(Others => '0');
			--r.cpu_ctl			: std_logic_vector(6 downto 3);
			r.cpu_stat			<=	"00";
			r.mem_ctl			<=	"000";
			r.mem_start			<=	'0';
			r.mem_data			<=	x"0000";
			r.mem_addr			<=	x"0000";
			r.mem_cnt			<=	x"0000";
			r.inc_step			<=	"00";
			r.halt_flag			<=	'0';
			r.mem_state			<=	"00";
			r.mem_startb		<=	'0';
		elsif rising_edge(dbg_clk) then
			r	<= rin;
		end if;
	end process REGS;

--	DBG_HWBRK_0 : if DBG_HWBRK_0_EN generate
		dbg_hwbr_0	: fmsp_dbg_hwbrk
		generic map (
			DBG_HWBRK_EN	=>	DBG_HWBRK_0_EN	--	Include hardware breakpoints unit 
		)
		port map (
			dbg_clk			=>	dbg_clk,			--! Debug unit clock
			dbg_rst			=>	dbg_rst,			--! Debug unit reset
			--! INPUTs
			brk_reg_rd		=>	brk0_reg_rd,	--! Hardware break/watch-point register read select
			brk_reg_wr		=>	brk0_reg_wr,	--! Hardware break/watch-point register write select
			dbg_din			=>	dbg_din,			--! Debug register data input
			decode_noirq	=>	decode_noirq,	--! Frontend decode instruction
			eu_mab			=>	eu_mab,			--! Execution-Unit Memory address bus
			eu_mb_en			=>	eu_mb_en,		--! Execution-Unit Memory bus enable
			eu_mb_wr			=>	eu_mb_wr,		--! Execution-Unit Memory bus write transfer
			pc					=>	pc,				--! Program counter
			--! OUTPUTs
			brk_halt			=>	d.brk0_halt,		--! Hardware breakpoint command
			brk_pnd			=>	d.brk0_pnd,		--! Hardware break/watch-point pending
			brk_dout			=>	d.brk0_dout		--! Hardware break/watch-point register data input
		);
--	end generate DBG_HWBRK_0;

--	DBG_HWBRK_1 : if DBG_HWBRK_1_EN generate
		dbg_hwbr_1	: fmsp_dbg_hwbrk
		generic map (
			DBG_HWBRK_EN	=>	DBG_HWBRK_1_EN	--	Include hardware breakpoints unit 
		)
		port map (
			dbg_clk			=>	dbg_clk,			--! Debug unit clock
			dbg_rst			=>	dbg_rst,			--! Debug unit reset
			--! INPUTs
			brk_reg_rd		=>	brk1_reg_rd,	--! Hardware break/watch-point register read select
			brk_reg_wr		=>	brk1_reg_wr,	--! Hardware break/watch-point register write select
			dbg_din			=>	dbg_din,			--! Debug register data input
			decode_noirq	=>	decode_noirq,	--! Frontend decode instruction
			eu_mab			=>	eu_mab,			--! Execution-Unit Memory address bus
			eu_mb_en			=>	eu_mb_en,		--! Execution-Unit Memory bus enable
			eu_mb_wr			=>	eu_mb_wr,		--! Execution-Unit Memory bus write transfer
			pc					=>	pc,				--! Program counter
			--! OUTPUTs
			brk_halt			=>	d.brk1_halt,		--! Hardware breakpoint command
			brk_pnd			=>	d.brk1_pnd,		--! Hardware break/watch-point pending
			brk_dout			=>	d.brk1_dout		--! Hardware break/watch-point register data input
		);
--	end generate DBG_HWBRK_1;

--	DBG_HWBRK_2 : if DBG_HWBRK_2_EN generate
		dbg_hwbr_2	: fmsp_dbg_hwbrk
		generic map (
			DBG_HWBRK_EN	=>	DBG_HWBRK_2_EN	--	Include hardware breakpoints unit 
		)
		port map (
			dbg_clk			=>	dbg_clk,			--! Debug unit clock
			dbg_rst			=>	dbg_rst,			--! Debug unit reset
			--! INPUTs
			brk_reg_rd		=>	brk2_reg_rd,	--! Hardware break/watch-point register read select
			brk_reg_wr		=>	brk2_reg_wr,	--! Hardware break/watch-point register write select
			dbg_din			=>	dbg_din,			--! Debug register data input
			decode_noirq	=>	decode_noirq,	--! Frontend decode instruction
			eu_mab			=>	eu_mab,			--! Execution-Unit Memory address bus
			eu_mb_en			=>	eu_mb_en,		--! Execution-Unit Memory bus enable
			eu_mb_wr			=>	eu_mb_wr,		--! Execution-Unit Memory bus write transfer
			pc					=>	pc,				--! Program counter
			--! OUTPUTs
			brk_halt			=>	d.brk2_halt,		--! Hardware breakpoint command
			brk_pnd			=>	d.brk2_pnd,		--! Hardware break/watch-point pending
			brk_dout			=>	d.brk2_dout		--! Hardware break/watch-point register data input
		);
--	end generate DBG_HWBRK_2;

--	DBG_HWBRK_3 : if DBG_HWBRK_3_EN generate
		dbg_hwbr_3	: fmsp_dbg_hwbrk
		generic map (
			DBG_HWBRK_EN	=>	DBG_HWBRK_3_EN	--	Include hardware breakpoints unit 
		)
		port map (
			dbg_clk			=>	dbg_clk,			--! Debug unit clock
			dbg_rst			=>	dbg_rst,			--! Debug unit reset
			--! INPUTs
			brk_reg_rd		=>	brk3_reg_rd,	--! Hardware break/watch-point register read select
			brk_reg_wr		=>	brk3_reg_wr,	--! Hardware break/watch-point register write select
			dbg_din			=>	dbg_din,			--! Debug register data input
			decode_noirq	=>	decode_noirq,	--! Frontend decode instruction
			eu_mab			=>	eu_mab,			--! Execution-Unit Memory address bus
			eu_mb_en			=>	eu_mb_en,		--! Execution-Unit Memory bus enable
			eu_mb_wr			=>	eu_mb_wr,		--! Execution-Unit Memory bus write transfer
			pc					=>	pc,				--! Program counter
			--! OUTPUTs
			brk_halt			=>	d.brk3_halt,	--! Hardware breakpoint command
			brk_pnd			=>	d.brk3_pnd,		--! Hardware break/watch-point pending
			brk_dout			=>	d.brk3_dout		--! Hardware break/watch-point register data input
		);
--	end generate DBG_HWBRK_3;

	ADD_DEBUG_I2C : if DBG_I2C generate
		dbg_i2c_0 : fmsp_dbg_i2c
		generic map(
			DBG_I2C_BROADCAST_EN	=>	DBG_I2C_BROADCAST_EN	--! Enable the I2C broadcast address
		)
		port map(
			dbg_clk				=>	dbg_clk,					--! Debug unit clock
			dbg_rst				=>	dbg_rst,					--! Debug unit reset
			--! INPUTs
			dbg_dout				=>	dbg_dout,				--! Debug register data output
			dbg_i2c_addr		=>	dbg_i2c_addr,			--! Debug interface: I2C Address
			dbg_i2c_broadcast	=>	dbg_i2c_broadcast,	--! Debug interface: I2C Broadcast Address (for multicore systems)
			mem_burst			=>	r.mem_burst,				--! Burst on going
			mem_burst_end		=>	mem_burst_end,			--! End TX/RX burst
			mem_burst_rd		=>	mem_burst_rd,			--! Start TX burst
			mem_burst_wr		=>	mem_burst_wr,			--! Start RX burst
			mem_bw				=>	mem_bw,					--! Burst byte width
			--! OUTPUTs
			dbg_addr				=>	d.dbg_addr_i2c,		--! Debug register address
			dbg_din				=>	d.dbg_din_i2c,			--! Debug register data input
			dbg_rd				=>	d.dbg_rd_i2c,			--! Debug register data read
			dbg_wr				=>	d.dbg_wr_i2c,			--! Debug register data write
			--! I2C Interface
			dbg_i2c_scl			=>	dbg_i2c_scl,			--! Debug interface: I2C SCL
			dbg_i2c_sda_in		=>	dbg_i2c_sda_in,		--! Debug interface: I2C SDA IN
			dbg_i2c_sda_out	=>	dbg_i2c_sda_out		--! Debug interface: I2C SDA OUT
		);
	end generate ADD_DEBUG_I2C;

	ADD_DEBUG_UART : if DBG_UART generate
		dbg_uart_0 : fmsp_dbg_uart
		generic map(
			DBG_UART_AUTO_SYNC	=>	DBG_UART_AUTO_SYNC,	--! Debug UART interface auto data synchronization
			DBG_UART_BAUD			=>	DBG_UART_BAUD,			--! Debug UART interface data rate
			DBG_DCO_FREQ			=>	DBG_DCO_FREQ,			--! Debug DCO_CLK frequency
			DBG_HWBRK_RANGE		=>	DBG_HWBRK_RANGE,		--! Enable/Disable the hardware breakpoint RANGE mode
			SYNC_DBG_UART_RXD		=>	SYNC_DBG_UART_RXD		--! Synchronize RXD inputs
		)
		port map(
			dbg_clk			=>	dbg_clk,				--! Debug unit clock
			dbg_rst			=>	dbg_rst,				--! Debug unit reset
			--! INPUTs
			dbg_dout			=>	dbg_dout,			--! Debug register data output
			dbg_rd_rdy		=>	r.dbg_rd_rdy,		--! Debug register data is ready for read
			mem_burst		=>	r.mem_burst,		--! Burst on going
			mem_burst_end	=>	mem_burst_end,		--! End TX/RX burst
			mem_burst_rd	=>	mem_burst_rd,		--! Start TX burst
			mem_burst_wr	=>	mem_burst_wr,		--! Start RX burst
			mem_bw			=>	mem_bw,				--! Burst byte width
			--! OUTPUTs
			dbg_addr			=>	d.dbg_addr_rs232,	--! Debug register address
			dbg_din			=>	d.dbg_din_rs232,	--! Debug register data input
			dbg_rd			=>	d.dbg_rd_rs232,	--! Debug register data read
			dbg_wr			=>	d.dbg_wr_rs232,	--! Debug register data write
			--! RS232 Interface
			dbg_uart_rxd	=>	dbg_uart_rxd,		--! Debug interface: UART RXD
			dbg_uart_txd	=>	dbg_uart_txd		--! Debug interface: UART TXD
		);
	end generate ADD_DEBUG_UART;

end RTL; --! fmsp_dbg
