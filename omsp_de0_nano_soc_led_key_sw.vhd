------------------------------------------------------------------------------
-- Copyright (C) 2001 Authors
--
-- This source file may be used and distributed without restriction provided
-- that this copyright statement is not removed from the file and that any
-- derivative work contains the original copyright notice and the associated
-- disclaimer.
--
-- This source file is free software; you can redistribute it and/or modify
-- it under the terms of the GNU Lesser General Public License as published
-- by the Free Software Foundation; either version 2.1 of the License, or
-- (at your option) any later version.
--
-- This source is distributed in the hope that it will be useful, but WITHOUT
-- ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
-- FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public
-- License for more details.
--
-- You should have received a copy of the GNU Lesser General Public License
-- along with this source; if not, write to the Free Software Foundation,
-- Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
--
------------------------------------------------------------------------------
--
-- *File Name: fmsp_de0_nano_soc_led_key_sw.vhd
--
-- *Module Description:
--                      Custom peripheral for the DE0 Nano SoC board
--                      for driving LEDs and reading SWITCHES and KEYs (i.e. buttons)
--
-- *Author(s):
--              - Olivier Girard,    olgirard@gmail.com
--! @author Emmanuel Amadio,   emmanuel.amadio@gmail.com (VHDL Rewrite)
--
------------------------------------------------------------------------------
-- $Rev$
-- $LastChangedBy$
-- $LastChangedDate$
------------------------------------------------------------------------------
library ieee;
	use ieee.std_logic_1164.all;	-- standard unresolved logic UX01ZWLH-
	use ieee.numeric_std.all;		-- for the signed, unsigned types and arithmetic ops
	use work.fmsp_functions.all;
	use work.fpga_package.all;

entity fmsp_de0_nano_soc_led_key_sw is 
	port (
		mclk		: in	std_logic;       						-- Main system clock
		puc_rst	: in	std_logic;       						-- Main system reset
		-- INPUTs
		key		: in	std_logic_vector(3 downto 0);		-- key/button inputs
		sw			: in	std_logic_vector(3 downto 0);		-- switches inputs
		per_addr	: in	std_logic_vector(13 downto 0);	-- Peripheral address
		per_din	: in	std_logic_vector(15 downto 0);	-- Peripheral data input
		per_en	: in	std_logic;								-- Peripheral enable (high active)
		per_we	: in	std_logic_vector(1 downto 0);		-- Peripheral write enable (high active)
		-- OUTPUTs
		irq_key	: out	std_logic;								-- Key/Button interrupt
		irq_sw	: out	std_logic;								-- Switch interrupt
		led		: out	std_logic_vector(7 downto 0);		-- LED output control
		per_dout	: out	std_logic_vector(15 downto 0)		-- Peripheral data output
	);
end entity fmsp_de0_nano_soc_led_key_sw;

architecture RTL of fmsp_de0_nano_soc_led_key_sw is 
	--=============================================================================
	-- 1)  PARAMETER DECLARATION
	--=============================================================================

	-- Register base address (must be aligned to decoder bit width)
	constant	BASE_ADDR	: std_logic_vector(14 downto 0) := "000000010010000";
	-- Decoder bit width (defines how many bits are considered for address decoding)
	constant	DEC_WD		: integer := 3;
	-- Register addresses offset
	constant	LED_CTRL				: integer := 0;	-- 'h0,
	constant	KEY_SW_VAL			: integer := 1;	-- 'h1,
	constant	KEY_SW_IRQ_EN		: integer := 2;	-- 'h2,
	constant	KEY_SW_IRQ_EDGE	: integer := 3;	-- 'h3,
	constant	KEY_SW_IRQ_VAL		: integer := 4;	-- 'h4;
	-- Register one-hot decoder utilities
	constant	DEC_SZ  				: integer :=  (2**DEC_WD);


	type fmsp_sfr_in_type is record
		key		: std_logic_vector(3 downto 0);	-- key/button inputs
		sw			: std_logic_vector(3 downto 0);	-- switches inputs
		per_addr	: std_logic_vector(13 downto 0);	-- Peripheral address
		per_din	: std_logic_vector(15 downto 0);	-- Peripheral data input
		per_en	: std_logic;							-- Peripheral enable (high active)
		per_we	: std_logic_vector(1 downto 0);	-- Peripheral write enable (high active)
		-- From sub modules
		key_deb	: std_logic_vector(3 downto 0);	-- key/button inputs
		sw_deb	: std_logic_vector(3 downto 0);	-- switches inputs
	end record;

	type reg_type is record
		-- To outside of module
		led_ctrl				: std_logic_vector(7 downto 0);	-- LED Control Register
		key_sw_irq_en		: std_logic_vector(7 downto 0);	-- KEY_SW_IRQ_EN Register
		key_sw_irq_edge	: std_logic_vector(7 downto 0);	-- KEY_SW_IRQ_EDGE Register
		key_sw_irq_val		: std_logic_vector(7 downto 0);	-- KEY_SW_IRQ_VAL Register
		key_sw_deb_dly		: std_logic_vector(7 downto 0);	-- Delay debounced signal for edge detection
	end record;

	signal	d		: fmsp_sfr_in_type;
	signal	r		: reg_type :=	(	led_ctrl				=> "00000000",
												key_sw_irq_en		=> "00000000",
												key_sw_irq_edge	=> "00000000",
												key_sw_irq_val		=> "00000000",
												key_sw_deb_dly		=> "00000000"
											);
	signal	rin	: reg_type;

begin

		d.key			<=	key;
		d.sw			<=	sw;
		d.per_addr	<=	per_addr;
		d.per_din	<=	per_din;
		d.per_en		<=	per_en;
		d.per_we		<=	per_we;

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
		variable	v_reg_wr				: std_logic_vector(DEC_SZ-1 downto 0);
		variable	v_reg_rd				: std_logic_vector(DEC_SZ-1 downto 0);
		-- LED Control Register
		variable	v_led_ctrl_wr			: std_logic;
		variable	v_led_ctrl_nxt			: std_logic_vector(7 downto 0);
		variable	v_key_sw_val			: std_logic_vector(7 downto 0);
		-- KEY_SW_IRQ_EN Register
		variable	v_key_sw_irq_en_wr		: std_logic;
		variable	v_key_sw_irq_en_nxt		: std_logic_vector(7 downto 0);
		-- KEY_SW_IRQ_EDGE Register
		variable	v_key_sw_irq_edge_wr		: std_logic;
		variable	v_key_sw_irq_edge_nxt	: std_logic_vector(7 downto 0);
		-- KEY_SW_IRQ_VAL Register
		variable	v_key_sw_irq_val_wr		: std_logic;
		variable	v_key_sw_irq_val_nxt		: std_logic_vector(7 downto 0);
		variable	v_key_sw_irq_clr			: std_logic_vector(7 downto 0);
		variable	v_key_sw_irq_set			: std_logic_vector(7 downto 0);
		variable	v_irq_key					: std_logic;
		variable	v_irq_sw						: std_logic;
		-- Data output mux
		variable	v_led_ctrl_rd				: std_logic_vector(15 downto 0);
		variable	v_key_sw_val_rd			: std_logic_vector(15 downto 0);
		variable	v_key_sw_irq_en_rd		: std_logic_vector(15 downto 0);
		variable	v_key_sw_irq_edge_rd		: std_logic_vector(15 downto 0);
		variable	v_key_sw_irq_val_rd		: std_logic_vector(15 downto 0);
		variable	v_per_dout					: std_logic_vector(15 downto 0);
		-- 5) IRQ GENERATION
		variable	v_key_sw_posedge			: std_logic_vector(7 downto 0);
		variable	v_key_sw_negedge			: std_logic_vector(7 downto 0);
		variable	v_key_sw_edge				: std_logic_vector(7 downto 0);
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
		v_reg_addr	:=	d.per_addr(DEC_WD-2 downto 0);
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


		-- LED Control Register
		------------------------
		v_led_ctrl_wr	:= v_reg_wr(LED_CTRL);
		v_led_ctrl_nxt	:= byte_per_select_din( LED_CTRL, d.per_din );
		if (v_led_ctrl_wr = '1') then
			v.led_ctrl	:=	v_led_ctrl_nxt;
		end if;

		-- KEY_SW_VAL Register
		-----------------------
		v_key_sw_val	:=	d.key_deb & d.sw_deb;


		-- KEY_SW_IRQ_EN Register
		------------------------
		v_key_sw_irq_en_wr	:= v_reg_wr(KEY_SW_IRQ_EN);
		v_key_sw_irq_en_nxt	:= byte_per_select_din( KEY_SW_IRQ_EN, d.per_din );
		if (v_key_sw_irq_en_wr = '1') then
			v.key_sw_irq_en	:=	v_key_sw_irq_en_nxt;
		end if;

		-- KEY_SW_IRQ_EDGE Register
		----------------------------
		v_key_sw_irq_edge_wr		:= v_reg_wr(KEY_SW_IRQ_EDGE);
		v_key_sw_irq_edge_nxt	:= byte_per_select_din( KEY_SW_IRQ_EDGE, d.per_din );
		if (v_key_sw_irq_edge_wr = '1') then
			v.key_sw_irq_edge		:=	v_key_sw_irq_edge_nxt;
		end if;


		-- KEY_SW_IRQ_VAL Register
		---------------------------
		v_key_sw_irq_val_wr	:= v_reg_wr(KEY_SW_IRQ_VAL);
		v_key_sw_irq_val_nxt	:= byte_per_select_din( KEY_SW_IRQ_VAL, d.per_din );
		-- Clear IRQ flag when 1 is writen
		v_key_sw_irq_clr		:= "00000000";
		if (v_key_sw_irq_val_wr = '1') then
			v_key_sw_irq_clr	:= v_key_sw_irq_val_nxt;
		end if;
		--v_key_sw_irq_set;

		--============================================================================
		-- 5) IRQ GENERATION
		--============================================================================

		-- Delay debounced signal for edge detection
		v.key_sw_deb_dly	:=	v_key_sw_val;

		v_key_sw_posedge	:= not(v_key_sw_val) and 	  r.key_sw_deb_dly;
		v_key_sw_negedge	:=		 v_key_sw_val  and not(r.key_sw_deb_dly);
		v_key_sw_edge		:=		(v_key_sw_posedge and	  r.key_sw_irq_edge)
									or	(v_key_sw_negedge and not(r.key_sw_irq_edge));

		v_key_sw_irq_set	:=  r.key_sw_irq_en and v_key_sw_edge;
		-- IRQ set has priority over clear
		v.key_sw_irq_val	:=	v_key_sw_irq_set or (not(v_key_sw_irq_clr) and r.key_sw_irq_val);

		v_irq_key	:=	r.key_sw_irq_val(4) or r.key_sw_irq_val(5) or r.key_sw_irq_val(6) or r.key_sw_irq_val(7);
		v_irq_sw		:=	r.key_sw_irq_val(0) or r.key_sw_irq_val(1) or r.key_sw_irq_val(2) or r.key_sw_irq_val(3);

		--============================================================================
		-- 4) DATA OUTPUT GENERATION
		--============================================================================

--		v_led_ctrl_rd			:= x"0000";
--		v_key_sw_val_rd		:= x"0000";
--		v_key_sw_irq_en_rd	:= x"0000";
--		v_key_sw_irq_edge_rd	:= x"0000";
--		v_key_sw_irq_val_rd	:= x"0000";

			-- Data output mux
		v_led_ctrl_rd			:= byte_per_select_dout( LED_CTRL,			v_reg_rd, r.led_ctrl				);
		v_key_sw_val_rd		:= byte_per_select_dout( KEY_SW_VAL,		v_reg_rd, v_key_sw_val			);
		v_key_sw_irq_en_rd	:= byte_per_select_dout( KEY_SW_IRQ_EN,	v_reg_rd, r.key_sw_irq_en		);
		v_key_sw_irq_edge_rd	:= byte_per_select_dout( KEY_SW_IRQ_EDGE, v_reg_rd, r.key_sw_irq_edge	);
		v_key_sw_irq_val_rd	:= byte_per_select_dout( KEY_SW_IRQ_VAL,	v_reg_rd, r.key_sw_irq_val		);

		v_per_dout	:=		v_led_ctrl_rd
							or v_key_sw_val_rd
							or v_key_sw_irq_en_rd
							or v_key_sw_irq_edge_rd
							or v_key_sw_irq_val_rd;



		-- drive register inputs
		rin <= v;
		-- drive module outputs
		
		per_dout	<=	v_per_dout;		-- Peripheral data output
		irq_key	<=	v_irq_key;		-- Key/Button interrupt
		irq_sw	<=	v_irq_sw;		-- Switch interrupt
		led		<=	r.led_ctrl;		-- LED output control

	end process COMB;

	REGS : process (mclk,puc_rst)
	begin
		if (puc_rst = '1') then
			r.led_ctrl			<= "00000000";
			r.key_sw_irq_en	<= "00000000";
			r.key_sw_irq_edge	<= "00000000";
			r.key_sw_irq_val	<= "00000000";
			r.key_sw_deb_dly	<= "00000000";
		elsif rising_edge(mclk) then
			r	<= rin;
		end if;
	end process REGS;


-- Synchronize and debounce the input signals
	key_debounce : for i in 0 to 3 generate
		sync_debouncer_10ms_key : sync_debouncer_10ms
		port map(
			clk_50mhz			=>	mclk,
			rst					=>	puc_rst,
			signal_async		=>	key(i),
			signal_debounced	=>	d.key_deb(i)
		);
	end generate;
	sw_debounce : for i in 0 to 3 generate
		sync_debouncer_10ms_sw : sync_debouncer_10ms
		port map(
			clk_50mhz			=>	mclk,
			rst					=>	puc_rst,
			signal_async		=>	sw(i),
			signal_debounced	=>	d.sw_deb(i)
		);
	end generate;


end RTL; -- fmsp_de0_nano_soc_led_key_sw

