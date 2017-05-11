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
--! @file fmsp_dbg_uart.vhd
--! 
--! @brief fpgaMSP430 Debug UART communication interface (8N1, Half-duplex)
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

entity fmsp_dbg_uart is 
	generic (
		DBG_UART_AUTO_SYNC	: boolean := true;					--! Debug UART interface auto data synchronization
		DBG_UART_BAUD			: integer := 9600;					--! Debug UART interface data rate
		DBG_DCO_FREQ			: integer := 20000000;				--! Debug DCO_CLK frequency
		DBG_HWBRK_RANGE		: boolean := true;					--! Enable/Disable the hardware breakpoint RANGE mode
		SYNC_DBG_UART_RXD		: boolean := true						--! Synchronize RXD inputs
	);
	port (
		dbg_clk			: in	std_logic;       						--! Debug unit clock
		dbg_rst			: in	std_logic;       						--! Debug unit reset
		--! INPUTs
		dbg_dout			: in	std_logic_vector(15 downto 0);	--! Debug register data output
		dbg_rd_rdy		: in	std_logic;       						--! Debug register data is ready for read
		dbg_uart_rxd	: in	std_logic;       						--! Debug interface: UART RXD
		mem_burst		: in	std_logic;       						--! Burst on going
		mem_burst_end	: in	std_logic;       						--! End TX/RX burst
		mem_burst_rd	: in	std_logic;       						--! Start TX burst
		mem_burst_wr	: in	std_logic;       						--! Start RX burst
		mem_bw			: in	std_logic;       						--! Burst byte width
		--! OUTPUTs
		dbg_addr			: out	std_logic_vector(5 downto 0);		--! Debug register address
		dbg_din			: out	std_logic_vector(15 downto 0);	--! Debug register data input
		dbg_rd			: out	std_logic;       						--! Debug register data read
		dbg_uart_txd	: out	std_logic;       						--! Debug interface: UART TXD
		dbg_wr			: out	std_logic       						--! Debug register data write
	);
end entity fmsp_dbg_uart;

architecture RTL of fmsp_dbg_uart is 
	--! Debug interface
	constant	DBG_UART_WR	: integer := 18;
	constant	DBG_UART_BW	: integer := 17;
--define 16 downto 11 16:11
	constant	DBG_UART_CNT_INT		: integer := ((DBG_DCO_FREQ/DBG_UART_BAUD)-1);
	--! Counter width for the debug interface UART
	constant	DBG_UART_XFER_CNT_W	: integer := 16;
	constant	DBG_UART_CNT			: std_logic_vector(DBG_UART_XFER_CNT_W+2 downto 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(DBG_UART_CNT_INT,DBG_UART_XFER_CNT_W+2));
	--! State machine definition
	constant	RX_SYNC			: std_logic_vector(2 downto 0) := "000";
	constant	RX_CMD			: std_logic_vector(2 downto 0) := "001";
	constant	RX_DATA1			: std_logic_vector(2 downto 0) := "010";
	constant	RX_DATA2			: std_logic_vector(2 downto 0) := "011";
	constant	TX_DATA1			: std_logic_vector(2 downto 0) := "100";
	constant	TX_DATA2			: std_logic_vector(2 downto 0) := "101";

	type fmsp_dbg_uart_in_type is record
		dbg_dout			: std_logic_vector(15 downto 0);	--! Debug register data output
		dbg_rd_rdy		: std_logic;       					--! Debug register data is ready for read
		dbg_uart_rxd	: std_logic;       					--! Debug interface: UART RXD
		mem_burst		: std_logic;       					--! Burst on going
		mem_burst_end	: std_logic;       					--! End TX/RX burst
		mem_burst_rd	: std_logic;       					--! Start TX burst
		mem_burst_wr	: std_logic;       					--! Start RX burst
		mem_bw			: std_logic;       					--! Burst byte width
		uart_rxd_n		: std_logic;
	end record;

	type reg_type is record
		rxd_buf			: std_logic_vector(1 downto 0);								--! RXD input buffer
		rxd_maj			: std_logic;														--! Majority decision
		uart_state		: std_logic_vector(2 downto 0);								--! Receive state
		xfer_buf			: std_logic_vector(19 downto 0);
		sync_busy		: std_logic;							
		sync_cnt			: std_logic_vector(DBG_UART_XFER_CNT_W+2 downto 0);
		xfer_bit			: std_logic_vector(3 downto 0);
		xfer_cnt			: std_logic_vector(DBG_UART_XFER_CNT_W-1 downto 0);	--! Transfer counter
		dbg_uart_txd	: std_logic;														--! Generate TXD output
		dbg_addr			: std_logic_vector(5 downto 0);
		dbg_bw			: std_logic;
	end record;

	signal	d		: fmsp_dbg_uart_in_type;
	signal	r		: reg_type :=	(			rxd_buf			=> "00",					--! RXD input buffer
														rxd_maj			=> '0',					--! Majority decision
														uart_state		=> "000",					--! Receive state
														xfer_buf			=> (Others => '0'),
														sync_busy		=> '0',							
														sync_cnt			=> (Others => '0'),
														xfer_bit			=> "0000",
														xfer_cnt			=> (Others => '0'),	--! Transfer counter
														dbg_uart_txd	=> '0',					--! Generate TXD output
														dbg_addr			=> "000000",
														dbg_bw			=> '0'
											);
	signal	rin	: reg_type;

begin

	d.dbg_dout			<=	dbg_dout;
	d.dbg_rd_rdy		<=	dbg_rd_rdy;
	d.dbg_uart_rxd		<=	dbg_uart_rxd;
	d.mem_burst			<=	mem_burst;
	d.mem_burst_end	<=	mem_burst_end;
	d.mem_burst_rd		<=	mem_burst_rd;
	d.mem_burst_wr		<=	mem_burst_wr;
	d.mem_bw				<=	mem_bw;

	COMB : process (d, r)
		variable	v						: reg_type;
		--=============================================================================
		--! 1)  UART RECEIVE LINE SYNCHRONIZTION & FILTERING
		--=============================================================================
		variable	v_uart_rxd			: std_logic;
		--! Majority decision
		variable	v_rxd_maj_nxt		: std_logic;
		variable	v_rxd_s				: std_logic;
		variable	v_rxd_fe				: std_logic;
		variable	v_rxd_re				: std_logic;
		variable	v_rxd_edge			: std_logic;
		--=============================================================================
		--! 2)  UART STATE MACHINE
		--=============================================================================
		--! Receive state
		variable	v_uart_state_nxt	: std_logic_vector(19 downto 0);
		variable	v_sync_done			: std_logic;
		variable	v_xfer_done			: std_logic;
		variable	v_xfer_buf_nxt		: std_logic_vector(19 downto 0);
		--! Utility signals
		variable	v_cmd_valid			: std_logic;
		variable	v_rx_active			: std_logic;
		variable	v_tx_active			: std_logic;
		--=============================================================================
		--! 3)  UART SYNCHRONIZATION
		--=============================================================================
		--! After DBG_RST, the host needs to fist send a synchronization character (0x80)
		--! If this feature doesn't work properly, it is possible to disable it by
		--! commenting the DBG_UART_AUTO_SYNC define in the openMSP430.inc file.

		variable	v_bit_cnt_max		: std_logic_vector(DBG_UART_XFER_CNT_W-1 downto 0);
		--=============================================================================
		--! 4)  UART RECEIVE / TRANSMIT
		--=============================================================================
		--! Transfer counter
		variable	v_txd_start			: std_logic;
		variable	v_rxd_start			: std_logic;
		variable	v_xfer_bit_inc		: std_logic;
		--=============================================================================
		--! 5) INTERFACE TO DEBUG REGISTERS
		--=============================================================================
		variable	v_dbg_din_bw		: std_logic;
		variable	v_dbg_din			: std_logic_vector(15 downto 0);
		variable	v_dbg_wr				: std_logic;
		variable	v_dbg_rd				: std_logic;

	begin
	
		--! default assignment
		v := r;
		--! overriding assignments

		--=============================================================================
		--! 1)  UART RECEIVE LINE SYNCHRONIZTION & FILTERING
		--=============================================================================

		--! Synchronize RXD input
		if	(SYNC_DBG_UART_RXD = true)	then
			v_uart_rxd	:=	not(d.uart_rxd_n);
		else
			v_uart_rxd	:=	d.dbg_uart_rxd;
		end if;

		--! RXD input buffer
		v.rxd_buf	:=	r.rxd_buf(0) & v_uart_rxd;

		--! Majority decision
		v_rxd_maj_nxt	:=		(v_uart_rxd and r.rxd_buf(0))
								or	(v_uart_rxd and r.rxd_buf(1))
								or	(r.rxd_buf(0) and r.rxd_buf(1));

		v.rxd_maj	:=	v_rxd_maj_nxt;

		v_rxd_s		:=	r.rxd_maj;
		v_rxd_fe		:=			r.rxd_maj	and	not(	v_rxd_maj_nxt);
		v_rxd_re		:=	not(	r.rxd_maj)	and			v_rxd_maj_nxt;
		v_rxd_edge	:=			r.rxd_maj	xor			v_rxd_maj_nxt;

		--=============================================================================
		--! 2)  UART STATE MACHINE
		--=============================================================================

		case (r.uart_state) is
			when RX_SYNC	=>
				v_uart_state_nxt			:=	RX_CMD;
			when RX_CMD	=>
				if (d.mem_burst_wr = '1') then
					if (d.mem_bw = '1') then
						v_uart_state_nxt	:=	RX_DATA2;
					else
						v_uart_state_nxt	:=	RX_DATA1;
					end if;
				elsif (d.mem_burst_rd = '1') then
					if (d.mem_bw = '1') then
						v_uart_state_nxt	:=	RX_DATA2;
					else
						v_uart_state_nxt	:=	RX_DATA1;
					end if;
				elsif (v_xfer_buf_nxt(DBG_UART_WR) = '1') then
					if (v_xfer_buf_nxt(DBG_UART_BW) = '1') then
						v_uart_state_nxt	:=	RX_DATA2;
					else
						v_uart_state_nxt	:=	RX_DATA1;
					end if;
				else
					if (v_xfer_buf_nxt(DBG_UART_BW) = '1') then
						v_uart_state_nxt	:=	TX_DATA2;
					else
						v_uart_state_nxt	:=	TX_DATA1;
					end if;
				end if;
			when RX_DATA1	=>
				v_uart_state_nxt			:=	RX_DATA2;
			when RX_DATA2	=>
				if ( (d.mem_burst and not(d.mem_burst_end)) = '1' ) then
					if (d.mem_bw = '1') then
						v_uart_state_nxt	:=	RX_DATA2;
					else
						v_uart_state_nxt	:=	RX_DATA1;
					end if;
				else
					v_uart_state_nxt		:=	RX_CMD;
				end if;
 			when TX_DATA1	=>
				v_uart_state_nxt			:=	TX_DATA2;
			when TX_DATA2	=>
				if ( (d.mem_burst and not(d.mem_burst_end)) = '1' ) then
					if (d.mem_bw = '1') then
						v_uart_state_nxt	:=	TX_DATA2;
					else
						v_uart_state_nxt	:=	TX_DATA1;
					end if;
				else
					v_uart_state_nxt		:=	RX_CMD;
				end if;
			when Others		=>
				v_uart_state_nxt			:=	RX_CMD;
		end case;

		--! State machine
		if ( (v_xfer_done or v_sync_done or d.mem_burst_wr or d.mem_burst_rd) = '1' ) then
			  v.uart_state	:=	v_uart_state_nxt;
		end if;

		--! Utility signals
		if (r.uart_state = RX_CMD) then
			v_cmd_valid	:=	v_xfer_done;
		else
			v_cmd_valid	:=	'0';
		end if;
		if (		(r.uart_state = RX_DATA1)
				or	(r.uart_state = RX_DATA2)
				or	(r.uart_state = RX_CMD)		) then
			v_rx_active	:=	'1';
		else
			v_rx_active	:=	'0';
		end if;
		if (		(r.uart_state = TX_DATA1)
				or	(r.uart_state = TX_DATA2)	) then
			v_tx_active	:=	'1';
		else
			v_tx_active	:=	'0';
		end if;

		--=============================================================================
		--! 3)  UART SYNCHRONIZATION
		--=============================================================================
		--! After DBG_RST, the host needs to fist send a synchronization character (0x80)
		--! If this feature doesn't work properly, it is possible to disable it by
		--! commenting the DBG_UART_AUTO_SYNC define in the openMSP430.inc file.

		if (		(r.uart_state = RX_SYNC)
				and	(v_rxd_fe = '1')	) then
			v.sync_busy	:=	'1';
		elsif (			(r.uart_state = RX_SYNC)
					and	(v_rxd_re = '1')	) then
			v.sync_busy	:=	'0';
		end if;

		v_sync_done	:=	'0';
		if (			(r.uart_state = RX_SYNC)
				and	(v_rxd_re = '1')
				and	(r.sync_busy = '1')	) then
			v_sync_done	:=	'1';
		end if;

		if	(DBG_UART_AUTO_SYNC = true)	then
			if ( (r.sync_busy or (not(r.sync_busy) and r.sync_cnt(2))) = '1') then 
				v.sync_cnt	:=	 STD_LOGIC_VECTOR( UNSIGNED(r.sync_cnt) + TO_UNSIGNED(1,DBG_UART_XFER_CNT_W+3) );
			end if;
			v_bit_cnt_max	:=	r.sync_cnt(DBG_UART_XFER_CNT_W+2 downto 3);
		else
			v_bit_cnt_max	:=	DBG_UART_CNT;
		end if;


		--=============================================================================
		--! 4)  UART RECEIVE / TRANSMIT
		--=============================================================================

		--! Transfer counter
		if (			(r.uart_state = TX_DATA1)
				and	(v_xfer_done = '1')	) then
			v_txd_start	:=	'1';
		elsif (dbg_rd_rdy = '1') then
			v_txd_start	:=	'1';
		else
			v_txd_start	:=	'0';
		end if;
		if (			(r.xfer_bit = "0000")
				and	(r.uart_state /= RX_SYNC)
				and	(v_rxd_fe = '1')	) then
			v_rxd_start	:=	'1';
		else
			v_rxd_start	:=	'0';
		end if;
		if (			(r.xfer_bit /= "0000")
				and	(r.xfer_cnt = STD_LOGIC_VECTOR(TO_UNSIGNED(0,DBG_UART_XFER_CNT_W)))	) then
			v_xfer_bit_inc	:=	'1';
		else
			v_xfer_bit_inc	:=	'0';
		end if;
		v_xfer_done	:=	'0';
		if (v_rx_active = '1') then
			if (r.xfer_bit = "1010") then
				v_xfer_done	:=	'1';
			end if;
		else
			if (r.xfer_bit = "1011") then
				v_xfer_done	:=	'1';
			end if;
		end if;

		if ( (v_txd_start or v_rxd_start) = '1') then
			v.xfer_bit	:=	"0001";
		elsif (v_xfer_done = '1') then
			v.xfer_bit	:=	"0000";
		elsif (v_xfer_bit_inc = '1') then
			v.xfer_bit	:=	STD_LOGIC_VECTOR( UNSIGNED(r.xfer_bit) + TO_UNSIGNED(1,4) );
		end if;

		if ( (v_rx_active and v_rxd_edge) = '1' ) then    
			v.xfer_cnt	:=	'0' & v_bit_cnt_max(DBG_UART_XFER_CNT_W-1 downto 1);
		elsif ( (v_txd_start or v_xfer_bit_inc) = '1' ) then 
			v.xfer_cnt	:=	v_bit_cnt_max;
		elsif ( r.xfer_cnt /= STD_LOGIC_VECTOR( TO_UNSIGNED(0,DBG_UART_XFER_CNT_W) ) ) then               
			v.xfer_cnt	:=	STD_LOGIC_VECTOR( UNSIGNED(r.xfer_cnt) - TO_UNSIGNED(1,DBG_UART_XFER_CNT_W) );
		end if;


		--! Receive/Transmit buffer
		v_xfer_buf_nxt	:=	v_rxd_s & r.xfer_buf(19 downto 1);

		if (dbg_rd_rdy = '1') then
			v.xfer_buf	:=	'1' & d.dbg_dout(15 downto 8) & "01" & dbg_dout(7 downto 0) & '0';
		elsif (v_xfer_bit_inc = '1') then
			v.xfer_buf	:=	v_xfer_buf_nxt;
		end if;


		--! Generate TXD output
		if ( (v_xfer_bit_inc and v_tx_active) = '1' ) then
			v.dbg_uart_txd	:=	r.xfer_buf(0);
		end if;


--=============================================================================
--! 5) INTERFACE TO DEBUG REGISTERS
--=============================================================================

		if (v_cmd_valid = '1') then
			v.dbg_addr	:=	v_xfer_buf_nxt(16 downto 11);
		end if;

		if (v_cmd_valid = '1') then
			v.dbg_bw	:=	v_xfer_buf_nxt(DBG_UART_BW);
		end if;

		if (d.mem_burst = '1') then
			v_dbg_din_bw	:=	d.mem_bw;
		else
			v_dbg_din_bw	:=	r.dbg_bw;
		end if;

		if (v_dbg_din_bw = '1') then
			v_dbg_din	:=	x"00" & v_xfer_buf_nxt(18 downto 11);
		else
			v_dbg_din	:=	v_xfer_buf_nxt(18 downto 11) & v_xfer_buf_nxt(9 downto 2);
		end if;
		v_dbg_wr	:=	'0';
		if (			(r.uart_state = RX_DATA2)
				and	(v_xfer_done = '1')	) then
			v_dbg_wr	:=	'1';
		end if;
		if (d.mem_burst = '1') then
			if (			(r.uart_state = TX_DATA2)
					and	(v_xfer_done = '1')	) then
				v_dbg_rd	:=	'1';
			else
				v_dbg_rd	:=	'0';
			end if;
		else
			v_dbg_rd	:=	( v_cmd_valid and not(v_xfer_buf_nxt(DBG_UART_WR)) ) or d.mem_burst_rd;
		end if;

		--! drive register inputs
		rin <= v;

		--! drive module outputs
		dbg_addr			<= r.dbg_addr;			--! Debug register address
		dbg_din			<= v_dbg_din;			--! Debug register data input
		dbg_rd			<= v_dbg_rd;			--! Debug register data read
		dbg_wr			<= v_dbg_wr;			--! Debug register data write
		dbg_uart_txd	<= r.dbg_uart_txd;	--! Debug interface: UART TXD

	end process COMB;

	REGS : process (dbg_clk,dbg_rst)
	begin
		if (dbg_rst = '1') then
			r	<=	(	rxd_buf			=> "11",					--! RXD input buffer
						rxd_maj			=> '1',					--! Majority decision
						uart_state		=> "000",					--! Receive state
						xfer_buf			=> (Others => '0'),
						sync_busy		=> '0',							
						sync_cnt			=> (0 => '0', 1 => '0', 2 => '0', Others => '1'),
						xfer_bit			=> "0000",
						xfer_cnt			=> (Others => '0'),	--! Transfer counter
						dbg_uart_txd	=> '1',					--! Generate TXD output
						dbg_addr			=> "000000",
						dbg_bw			=> '0'
					);
		elsif rising_edge(dbg_clk) then
			r	<= rin;
		end if;
	end process REGS;

	sync_cell_uart_rxd : fmsp_sync_cell
	port map(
		clk		=> dbg_clk,
		rst		=> dbg_rst,
		data_in	=> not(dbg_uart_rxd),
		data_out	=> d.uart_rxd_n
	);

end RTL;

