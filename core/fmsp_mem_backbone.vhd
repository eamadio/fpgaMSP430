
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
--! @file fmsp_mem_backbone.vhd
--! 
--! @brief fpgaMSP430 Memory interface backbone (decoder + arbiter)
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

entity fmsp_mem_backbone is
generic (
	PMEM_SIZE		: integer := 32768;		--! Program Memory Size
	DMEM_SIZE		: integer := 16384;		--! Data Memory Size
	PER_SIZE			: integer := 16384;		--! Peripheral Memory Size
	DMA_IF_EN		: boolean := false		--! Wakeup condition from DMA interface
);
port (
	mclk				: in	std_logic;								--! Main system clock
	mrst			 	: in	std_logic;								--! Main system reset
	--! INPUTs
	cpu_halt_st		: in	std_logic;								--! Halt/Run status from CPU
	dbg_halt_cmd	: in	std_logic;								--! Debug interface Halt CPU command
	dbg_mem_addr	: in	std_logic_vector(15 downto 0);	--! Debug address for rd/wr access
	dbg_mem_dout	: in	std_logic_vector(15 downto 0);	--! Debug unit data output
	dbg_mem_en		: in	std_logic;								--! Debug unit memory enable
	dbg_mem_wr		: in	std_logic_vector(1 downto 0);		--! Debug unit memory write
	dmem_dout		: in	std_logic_vector(15 downto 0);	--! Data Memory data output
	eu_mab			: in	std_logic_vector(14 downto 0);	--! Execution Unit Memory address bus
	eu_mb_en			: in	std_logic;								--! Execution Unit Memory bus enable
	eu_mb_wr			: in	std_logic_vector(1 downto 0);		--! Execution Unit Memory bus write transfer
	eu_mdb_out		: in	std_logic_vector(15 downto 0);	--! Execution Unit Memory data bus output
	fe_mab			: in	std_logic_vector(14 downto 0);	--! Frontend Memory address bus
	fe_mb_en			: in	std_logic;								--! Frontend Memory bus enable
	dma_addr			: in	std_logic_vector(15 downto 0);	--! Direct Memory Access address
	dma_din			: in	std_logic_vector(15 downto 0);	--! Direct Memory Access data input
	dma_en			: in	std_logic;								--! Direct Memory Access enable (high active)
	dma_priority	: in	std_logic;								--! Direct Memory Access priority (0:low / 1:high)
	dma_we			: in	std_logic_vector(1 downto 0);		--! Direct Memory Access write byte enable (high active)
	per_dout			: in	std_logic_vector(15 downto 0);	--! Peripheral data output
	pmem_dout		: in	std_logic_vector(15 downto 0);	--! Program Memory data output
	--! OUTPUTs
	cpu_halt_cmd	: out	std_logic;													--! Halt CPU command
	dbg_mem_din		: out	std_logic_vector(15 downto 0);						--! Debug unit Memory data input
	dmem_addr		: out	std_logic_vector(f_log2(DMEM_SIZE)-2 downto 0);	--! Data Memory address
	dmem_cen			: out	std_logic;													--! Data Memory chip enable (low active)
	dmem_din			: out	std_logic_vector(15 downto 0);						--! Data Memory data input
	dmem_wen			: out	std_logic_vector(1 downto 0);							--! Data Memory write enable (low active)
	eu_mdb_in		: out	std_logic_vector(15 downto 0);						--! Execution Unit Memory data bus input
	fe_mdb_in		: out	std_logic_vector(15 downto 0);						--! Frontend Memory data bus input
	fe_pmem_wait	: out	std_logic;													--! Frontend wait for Instruction fetch
	dma_dout			: out	std_logic_vector(15 downto 0);						--! Direct Memory Access data output
	dma_ready		: out	std_logic;													--! Direct Memory Access is complete
	dma_resp			: out	std_logic;													--! Direct Memory Access response (0:Okay / 1:Error)
	per_addr			: out	std_logic_vector(13 downto 0);						--! Peripheral address
	per_din			: out	std_logic_vector(15 downto 0);						--! Peripheral data input
	per_we			: out	std_logic_vector(1 downto 0);							--! Peripheral write enable (high active)
	per_en			: out	std_logic;													--! Peripheral enable (high active)
	pmem_addr		: out	std_logic_vector(f_log2(PMEM_SIZE)-2 downto 0);	--! Program Memory address
	pmem_cen			: out	std_logic;													--! Program Memory chip enable (low active)
	pmem_din			: out	std_logic_vector(15 downto 0);						--! Program Memory data input (optional)
	pmem_wen			: out	std_logic_vector(1 downto 0)							--! Program Memory write enable (low active) (optional)
);
end entity fmsp_mem_backbone;

architecture RTL of fmsp_mem_backbone is 

--! Data Memory Base Adresses
	constant	DMEM_BASE		:integer := PER_SIZE;

--! Program & Data Memory most significant address bit (for 16 bit words)
	constant	PMEM_MSB		:integer := (f_log2(PMEM_SIZE)-2);
	constant	DMEM_MSB		:integer := (f_log2(DMEM_SIZE)-2);
	constant	PER_MSB		:integer := (f_log2(PER_SIZE)-2);
	
	constant	DMEM_END		:integer := (DMEM_BASE + DMEM_SIZE);
	constant	PMEM_OFFSET	:integer := (65535 - PMEM_SIZE + 1);

	signal	w_eu_dmem_cen			: std_logic;
	signal	w_cpu_halt_cmd			: std_logic;
	signal	w_dma_resp 				: std_logic;
	signal	w_dma_ready				: std_logic;
	signal	r_dma_ready_dly		: std_logic;
	signal	w_ext_mem_en			: std_logic;
	signal	w_ext_mem_wr			: std_logic_vector(1 downto 0);
	signal	w_ext_mem_addr			: std_logic_vector(15 downto 0);
	signal	w_ext_mem_dout			: std_logic_vector(15 downto 0);
	signal	w_dbg_mem_din			: std_logic_vector(15 downto 0);
	signal	w_dma_dout				: std_logic_vector(15 downto 0);
	signal	w_eu_dmem_sel			: std_logic;
	signal	w_eu_dmem_en			: std_logic;
	signal	w_eu_dmem_addr			: std_logic_vector(15 downto 0);
	signal	w_ext_dmem_sel			: std_logic;
	signal	w_ext_dmem_en			: std_logic;
	signal	w_ext_dmem_addr		: std_logic_vector(15 downto 0);
	signal	w_dmem_cen				: std_logic;
	signal	w_dmem_wen				: std_logic_vector(1 downto 0);
	signal	w_dmem_addr				: std_logic_vector(DMEM_MSB downto 0);
	signal	w_dmem_din				: std_logic_vector(15 downto 0);
	signal	w_eu_pmem_sel			: std_logic;
	signal	w_eu_pmem_en			: std_logic;
	signal	w_eu_pmem_addr			: std_logic_vector(15 downto 0);
	signal	w_fe_pmem_sel			: std_logic;
	signal	w_fe_pmem_en			: std_logic;
	signal	w_fe_pmem_addr			: std_logic_vector(15 downto 0);
	signal	w_ext_pmem_sel			: std_logic;
	signal	w_ext_pmem_en			: std_logic;
	signal	w_ext_pmem_addr		: std_logic_vector(15 downto 0);
	signal	w_pmem_cen				: std_logic;
	signal	w_pmem_wen				: std_logic_vector(1 downto 0);
	signal	w_pmem_addr				: std_logic_vector(PMEM_MSB downto 0);
	signal	w_pmem_din				: std_logic_vector(15 downto 0);
	signal	w_fe_pmem_wait			: std_logic;
	signal	w_eu_per_sel			: std_logic;
	signal	w_eu_per_en				: std_logic;
	signal	w_ext_per_sel			: std_logic;
	signal	w_ext_per_en			: std_logic;
	signal	w_per_en					: std_logic;
	signal	w_per_we					: std_logic_vector(1 downto 0);
	signal	w_per_addr_mux			: std_logic_vector(PER_MSB downto 0);
	signal	w_per_addr_ful			: std_logic_vector(14 downto 0);
	signal	w_per_addr				: std_logic_vector(13 downto 0);
	signal	w_per_din				: std_logic_vector(15 downto 0);
	signal	r_per_dout_val			: std_logic_vector(15 downto 0);
	signal	w_fe_pmem_en_dly		: std_logic;
	signal	w_fe_pmem_save			: std_logic;
	signal	w_fe_pmem_restore		: std_logic;
	signal	w_pmem_dout_bckup		: std_logic_vector(15 downto 0);
	signal	w_pmem_dout_bckup_sel: std_logic;
	signal	w_fe_mdb_in				: std_logic_vector(15 downto 0);
	signal	w_eu_mdb_in_sel		: std_logic_vector(1 downto 0);
	signal	w_eu_mdb_in				: std_logic_vector(15 downto 0);
	signal	w_ext_mem_din_sel		: std_logic_vector(1 downto 0);
	signal	w_ext_mem_din			: std_logic_vector(15 downto 0);

				--! Register peripheral data read path
	signal	r_fe_pmem_en_dly			: std_logic;
	signal	r_pmem_dout_bckup			: std_logic_vector(15 downto 0);
	signal	r_pmem_dout_bckup_sel	: std_logic;
	signal	r_eu_mdb_in_sel			: std_logic_vector(1 downto 0);
	signal	r_ext_mem_din_sel			: std_logic_vector(1 downto 0);


begin

		--=============================================================================
		--! 1)  DECODER
		--=============================================================================
											  
											  
	COMB : process(all)
	begin

		--------------------------------------------
		--! Arbiter between DMA and Debug interface
		--------------------------------------------
		if (DMA_IF_EN = true) then
			--! Debug-interface always stops the CPU
			--! Master interface stops the CPU in priority mode
			w_cpu_halt_cmd	<= dbg_halt_cmd or (dma_en and dma_priority);
			--! Return ERROR response if address lays outside the memory spaces (Peripheral, Data and Program memories)
			w_dma_resp		<= not(dbg_mem_en) and not(w_ext_dmem_sel or w_ext_pmem_sel or w_ext_per_sel) and dma_en;
			--! Master interface access is ready when the memory access occures
			w_dma_ready		<= not(dbg_mem_en) and (w_ext_dmem_en or w_ext_pmem_en or w_ext_per_en or dma_resp);
			--! Use delayed version of 'dma_ready' to mask the 'dma_dout' data output
			--! when not accessed and reduce toggle rate (thus power consumption)
			--! Mux between debug and master interface
			w_ext_mem_en	<= dbg_mem_en or dma_en;
			if (dbg_mem_en = '1') then
				w_ext_mem_wr	<= dbg_mem_wr;
				w_ext_mem_addr	<= dbg_mem_addr;
				w_ext_mem_dout	<= dbg_mem_dout;
			else
				w_ext_mem_wr	<= dma_we;
				w_ext_mem_addr	<= dma_addr;
				w_ext_mem_dout	<= dma_din;
			end if;
			--! External interface read data
			w_dbg_mem_din	<= w_ext_mem_din;
			if (r_dma_ready_dly = '1') then
				w_dma_dout		<= w_ext_mem_din;
			else
				w_dma_dout		<= x"0000";
			end if;
		else
			--! Debug-interface always stops the CPU
			w_cpu_halt_cmd	<= dbg_halt_cmd;
			--! Master interface access is always ready with error response when excluded
			w_dma_resp		<= '1';
			w_dma_ready		<= '1';
			--! Debug interface only
			w_ext_mem_en	<= dbg_mem_en;
			w_ext_mem_wr	<= dbg_mem_wr;
			w_ext_mem_addr	<= dbg_mem_addr;
			w_ext_mem_dout	<= dbg_mem_dout;
			--! External interface read data
			w_dbg_mem_din	<= w_ext_mem_din;
			w_dma_dout		<= x"0000";
		end if;

		--------------------------------------------
		--! DATA-MEMORY Interface
		--------------------------------------------

		--! Execution unit access
		--w_eu_dmem_sel	<= (eu_mab>=(`DMEM_BASE>>1)) and (eu_mab< ( DMEM_END >>1));
		if (	( UNSIGNED(eu_mab) >= TO_UNSIGNED((DMEM_BASE/2),15) ) and
				( UNSIGNED(eu_mab) < TO_UNSIGNED((DMEM_END/2),15) ) ) then
			w_eu_dmem_sel	<= '1';
		else
			w_eu_dmem_sel	<= '0';
		end if;
		w_eu_dmem_en	<= eu_mb_en and w_eu_dmem_sel;
		w_eu_dmem_addr	<= STD_LOGIC_VECTOR( (UNSIGNED(eu_mab)/2) - TO_UNSIGNED((DMEM_BASE/2),16) );

		--! Front-end access
		--! --! not allowed to execute from data memory --

		--! External Master/Debug interface access
		--w_ext_dmem_sel <= (ext_mem_addr[15:1]>=(`DMEM_BASE>>1)) an (ext_mem_addr[15:1]< ( DMEM_END >>1));
		if (	( UNSIGNED(w_ext_mem_addr(15 downto 1)) >= TO_UNSIGNED((DMEM_BASE/2),15) ) and
				( UNSIGNED(w_ext_mem_addr(15 downto 1)) <  TO_UNSIGNED((DMEM_END/2),15) ) ) then
			w_ext_dmem_sel	<= '1';
		else
			w_ext_dmem_sel	<= '0';
		end if;
		w_ext_dmem_en  <= w_ext_mem_en and  w_ext_dmem_sel and not(w_eu_dmem_en);
		--w_ext_dmem_addr<= {1'b0, ext_mem_addr[15:1]}-(`DMEM_BASE>>1);
		w_ext_dmem_addr	<= STD_LOGIC_VECTOR( (UNSIGNED(w_ext_mem_addr)/2) - TO_UNSIGNED((DMEM_BASE/2),16) );

		--! Data-Memory Interface
		w_dmem_cen     <= not(w_ext_dmem_en or w_eu_dmem_en);
		if (w_ext_dmem_en = '1') then
			w_dmem_wen	<= not(w_ext_mem_wr);
			w_dmem_addr	<= w_ext_dmem_addr(DMEM_MSB downto 0);
			w_dmem_din	<= w_ext_mem_dout;
		else
			w_dmem_wen	<= not(eu_mb_wr);
			w_dmem_addr	<= w_eu_dmem_addr(DMEM_MSB downto 0);
			w_dmem_din	<= eu_mdb_out;
		end if;

		--------------------------------------------
		--! PROGRAM-MEMORY Interface
		--------------------------------------------


		--! Execution unit access (only read access are accepted)
		--w_eu_pmem_sel	<= (eu_mab>=(PMEM_OFFSET>>1));
		if ( UNSIGNED(eu_mab) >= TO_UNSIGNED((PMEM_OFFSET/2),15) ) then
			w_eu_pmem_sel	<= '1';
		else
			w_eu_pmem_sel	<= '0';
		end if;
		w_eu_pmem_en	<= eu_mb_en and not(eu_mb_wr(1) or eu_mb_wr(0)) and w_eu_pmem_sel;
		--w_eu_pmem_addr	<= eu_mab-(PMEM_OFFSET>>1);
		w_eu_pmem_addr	<= STD_LOGIC_VECTOR( UNSIGNED(eu_mab) - TO_UNSIGNED((PMEM_OFFSET/2),16) );

		--! Front-end access
		--w_fe_pmem_sel	<= (fe_mab>=(PMEM_OFFSET>>1));
		if ( UNSIGNED(fe_mab) >= TO_UNSIGNED((PMEM_OFFSET/2),15) ) then
			w_fe_pmem_sel	<= '1';
		else
			w_fe_pmem_sel	<= '0';
		end if;
		w_fe_pmem_en	<= fe_mb_en and w_fe_pmem_sel;
		--w_fe_pmem_addr	<= fe_mab-(PMEM_OFFSET>>1);
		w_fe_pmem_addr	<= STD_LOGIC_VECTOR( UNSIGNED(fe_mab) - TO_UNSIGNED((PMEM_OFFSET/2),16) );

		--! External Master/Debug interface access
		--w_ext_pmem_sel		<= (ext_mem_addr[15:1]>=(PMEM_OFFSET>>1));
		if ( UNSIGNED(w_ext_mem_addr(15 downto 1)) >= TO_UNSIGNED((PMEM_OFFSET/2),15) ) then
			w_ext_pmem_sel	<= '1';
		else
			w_ext_pmem_sel	<= '0';
		end if;
		w_ext_pmem_en		<= w_ext_mem_en and w_ext_pmem_sel and not(w_eu_pmem_en) and not(w_fe_pmem_en);
		--w_ext_pmem_addr	<= {1'b0, ext_mem_addr[15:1]}-(PMEM_OFFSET>>1);
		w_ext_pmem_addr	<= STD_LOGIC_VECTOR( UNSIGNED(w_ext_mem_addr(15 downto 1)) - TO_UNSIGNED((PMEM_OFFSET/2),16) );



		--! Program-Memory Interface (Execution unit has priority over the Front-end)
		w_pmem_cen		<= not(w_fe_pmem_en or w_eu_pmem_en or w_ext_pmem_en);
		if (w_ext_pmem_en = '1') then
			w_pmem_wen	<= not(w_ext_mem_wr);
		else
			w_pmem_wen	<= "11";
		end if;
		if (w_ext_pmem_en = '1') then
			w_pmem_addr	<= w_ext_pmem_addr(PMEM_MSB downto 0);
		elsif (w_eu_pmem_en = '1') then
			w_pmem_addr	<= w_eu_pmem_addr(PMEM_MSB downto 0);
		else
			w_pmem_addr	<= w_fe_pmem_addr(PMEM_MSB downto 0);
		end if;
		w_pmem_din		<= w_ext_mem_dout;
		w_fe_pmem_wait	<= w_fe_pmem_en and w_eu_pmem_en;


		--------------------------------------------
		--! PERIPHERALS Interface
		--------------------------------------------

		--! Execution unit access
		--w_eu_per_sel	<= (eu_mab<(`PER_SIZE>>1));
		if ( UNSIGNED(eu_mab) < TO_UNSIGNED((PER_SIZE/2),15) ) then
			w_eu_per_sel	<= '1';
		else
			w_eu_per_sel	<= '0';
		end if;
		w_eu_per_en		<= eu_mb_en and w_eu_per_sel;

		--! Front-end access
		--! --! not allowed to execute from peripherals memory space --

		--! External Master/Debug interface access
		--w_ext_per_sel	<= (ext_mem_addr[15:1]<(`PER_SIZE>>1));
		if ( UNSIGNED(w_ext_mem_addr(15 downto 1)) < TO_UNSIGNED((PER_SIZE/2),15) ) then
			w_ext_per_sel	<= '1';
		else
			w_ext_per_sel	<= '0';
		end if;
		w_ext_per_en	<= w_ext_mem_en and w_ext_per_sel and not(w_eu_per_en);

		--! Peripheral Interface
		w_per_en			<= w_ext_per_en or w_eu_per_en;
		if (w_ext_per_en = '1') then
			w_per_we			<= w_ext_mem_wr;
			w_per_addr_mux	<= w_ext_mem_addr(PER_MSB+1 downto 1);
			w_per_din		<= w_ext_mem_dout;
		else
			w_per_we			<= eu_mb_wr;
			w_per_addr_mux	<= eu_mab(PER_MSB downto 0);
			w_per_din		<= eu_mdb_out;
		end if;
		--w_per_addr_ful	<= {{15-`PER_AWIDTH{1'b0}}, per_addr_mux};
		w_per_addr_ful	<= STD_LOGIC_VECTOR( resize( UNSIGNED(w_per_addr_mux), 15) );
		w_per_addr		<= w_per_addr_ful(13 downto 0);

		--------------------------------------------
		--! Frontend data Mux
		--------------------------------------------
		--! Whenever the frontend doesn't access the program memory,  backup the data

		--! Detect whenever the data should be backuped and restored

		w_fe_pmem_save   <= (not(w_fe_pmem_en) and  r_fe_pmem_en_dly) and not(cpu_halt_st);
		w_fe_pmem_restore<= ( w_fe_pmem_en and not(r_fe_pmem_en_dly)) or  cpu_halt_st;


		--! Mux between the Program memory data and the backup
		if (r_pmem_dout_bckup_sel = '1') then
			w_fe_mdb_in	<= r_pmem_dout_bckup;
		else
			w_fe_mdb_in	<= pmem_dout;
		end if;


		--------------------------------------------
		--! Execution-Unit data Mux
		--------------------------------------------

		--! Select between Peripherals, Program and Data memories
		--! Mux
		if (r_eu_mdb_in_sel(1) = '1') then
			w_eu_mdb_in	<= pmem_dout;
		elsif (r_eu_mdb_in_sel(0) = '1') then
			w_eu_mdb_in	<= r_per_dout_val;
		else
			w_eu_mdb_in	<= dmem_dout;
		end if;


		--------------------------------------------
		--! External Master/Debug interface data Mux
		--------------------------------------------

		--! Select between Peripherals, Program and Data memories
		--! Mux
		if (r_ext_mem_din_sel(1) = '1') then
			w_ext_mem_din	<= pmem_dout;
		elsif (r_ext_mem_din_sel(0) = '1') then
			w_ext_mem_din	<= r_per_dout_val;
		else
			w_ext_mem_din	<= dmem_dout;
		end if;
	end process COMB;

		
	REGS : process (mclk, mrst)
	begin
		if (mrst = '1') then
			r_dma_ready_dly			<= '0';
			r_per_dout_val				<=  x"0000";
			r_fe_pmem_en_dly			<= '0';
			r_pmem_dout_bckup_sel	<= '0';
			r_pmem_dout_bckup			<= x"0000";
			r_eu_mdb_in_sel			<= "00";
			r_ext_mem_din_sel			<= "00";
		elsif rising_edge(mclk) then
			--! Use delayed version of 'dma_ready' to mask the 'dma_dout' data output
			--! when not accessed and reduce toggle rate (thus power consumption)
			r_dma_ready_dly			<=  dma_ready;
			--! Register peripheral data read path
			r_per_dout_val				<=  per_dout;
			--! Detect whenever the data should be backuped and restored
			r_fe_pmem_en_dly			<=  w_fe_pmem_en;
			--! Mux between the Program memory data and the backup
			if (w_fe_pmem_save = '1') then
				r_pmem_dout_bckup_sel	<= '1';
			elsif (w_fe_pmem_restore = '1') then
				r_pmem_dout_bckup_sel	<= '0';
			end if;
			if (w_fe_pmem_save = '1') then
				r_pmem_dout_bckup	<= pmem_dout;
			end if;
			--! Select between Peripherals, Program and Data memories
			r_eu_mdb_in_sel	<= w_eu_pmem_en & w_eu_per_en;
			r_ext_mem_din_sel	<= w_ext_pmem_en & w_ext_per_en;

		end if;
	end process REGS;

	cpu_halt_cmd	<= w_cpu_halt_cmd;	--! Halt CPU command
	dbg_mem_din		<= w_dbg_mem_din;		--! Debug unit Memory data input
	dmem_addr		<= w_dmem_addr;		--! Data Memory address
	dmem_cen			<= w_dmem_cen;			--! Data Memory chip enable (low active)
	dmem_din			<= w_dmem_din;			--! Data Memory data input
	dmem_wen			<= w_dmem_wen;			--! Data Memory write enable (low active)
	eu_mdb_in		<= w_eu_mdb_in;		--! Execution Unit Memory data bus input
	fe_mdb_in		<= w_fe_mdb_in;		--! Frontend Memory data bus input
	fe_pmem_wait	<= w_fe_pmem_wait;	--! Frontend wait for Instruction fetch
	dma_dout			<= w_dma_dout;			--! Direct Memory Access data output
	dma_ready		<= w_dma_ready;		--! Direct Memory Access is complete
	dma_resp			<= w_dma_resp;			--! Direct Memory Access response (0:Okay / 1:Error)
	per_addr			<= w_per_addr;			--! Peripheral address
	per_din			<= w_per_din;			--! Peripheral data input
	per_we			<= w_per_we;			--! Peripheral write enable (high active)
	per_en			<= w_per_en;			--! Peripheral enable (high active)
	pmem_addr		<= w_pmem_addr;		--! Program Memory address
	pmem_cen			<= w_pmem_cen;			--! Program Memory chip enable (low active)
	pmem_din			<= w_pmem_din;			--! Program Memory data input (optional)
	pmem_wen			<= w_pmem_wen;			--! Program Memory write enable (low active) (optional)

end RTL;	--! fmsp_mem_backbone