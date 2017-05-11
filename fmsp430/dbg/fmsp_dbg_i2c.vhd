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
--! @file fmsp_dbg_i2c.vhd
--! 
--! @brief fpgaMSP430 Debug I2C Slave communication interface
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

entity fmsp_dbg_i2c is 
	generic (
		DBG_I2C_BROADCAST_EN	: boolean := false		--! Enable the I2C broadcast address
	);
	port (
		dbg_clk				: in	std_logic;       						--! Debug unit clock
		dbg_rst				: in	std_logic;       						--! Debug unit reset
		--! INPUTs
		dbg_dout				: in	std_logic_vector(15 downto 0);	--! Debug register data output
		dbg_i2c_addr		: in	std_logic_vector(6 downto 0);		--! Debug interface: I2C ADDRESS
		dbg_i2c_broadcast	: in	std_logic_vector(6 downto 0);		--! Debug interface: I2C Broadcast Address (for multicore systems)
		dbg_i2c_scl			: in	std_logic;  							--! Debug interface: I2C SCL
		dbg_i2c_sda_in		: in	std_logic;       						--! Debug interface: I2C SDA IN
		mem_burst			: in	std_logic;       						--! Burst on going
		mem_burst_end		: in	std_logic;       						--! End TX/RX burst
		mem_burst_rd		: in	std_logic;       						--! Start TX burst
		mem_burst_wr		: in	std_logic;       						--! Start RX burst
		mem_bw				: in	std_logic;       						--! Burst byte width
		--! OUTPUTs
		dbg_addr				: out	std_logic_vector(5 downto 0);		--! Debug register address
		dbg_din				: out	std_logic_vector(15 downto 0);	--! Debug register data input
		dbg_i2c_sda_out	: out	std_logic;       						--! Debug interface: I2C SDA OUT
		dbg_rd				: out	std_logic;       						--! Debug register data read
		dbg_wr				: out	std_logic       						--! Debug register data write
	);
end entity fmsp_dbg_i2c;

architecture RTL of fmsp_dbg_i2c is 

	--! 3) I2C STATE MACHINE
	--! State machine definition
--	constant	RX_ADDR		: integer := 0;
--	constant	RX_ADDR_ACK	: integer := 1;
--	constant	RX_DATA		: integer := 2;
--	constant	RX_DATA_ACK	: integer := 3;
--	constant	TX_DATA		: integer := 4;
--	constant	TX_DATA_ACK	: integer := 5;
	type		I2C_STATE_TYPE is (	RX_ADDR,
											RX_ADDR_ACK,
											RX_DATA,
											RX_DATA_ACK,
											TX_DATA,
											TX_DATA_ACK
										);  --type of state machine.
	--! 6) DEBUG INTERFACE STATE MACHINE
	--! State machine definition
--	constant	RX_CMD		: integer := 0;
--	constant	RX_BYTE_LO	: integer := 1;
--	constant	RX_BYTE_HI	: integer := 2;
--	constant	TX_BYTE_LO	: integer := 3;
--	constant	TX_BYTE_HI	: integer := 4;
	type		DBG_STATE_TYPE is (	RX_CMD,
											RX_BYTE_LO,
											RX_BYTE_HI,
											TX_BYTE_LO,
											TX_BYTE_HI
										);  --type of state machine.
	--! 7) REGISTER READ/WRITE ACCESS
	constant	MEM_DATA		: std_logic_vector(5 downto 0) := "000110";

	type fmsp_dbg_i2c_in_type is record
		dbg_dout				: std_logic_vector(15 downto 0);	--! Debug register data output
		dbg_i2c_addr		: std_logic_vector(6 downto 0);	--! Debug interface: I2C ADDRESS
		dbg_i2c_broadcast	: std_logic_vector(6 downto 0);	--! Debug interface: I2C Broadcast Address (for multicore systems)
		dbg_i2c_scl			: std_logic;  							--! Debug interface: I2C SCL
		dbg_i2c_sda_in		: std_logic;     						--! Debug interface: I2C SDA IN
		mem_burst			: std_logic;     						--! Burst on going
		mem_burst_end		: std_logic;     						--! End TX/RX burst
		mem_burst_rd		: std_logic;     						--! Start TX burst
		mem_burst_wr		: std_logic;     						--! Start RX burst
		mem_bw				: std_logic;     						--! Burst byte width
		scl_sync_n			: std_logic;
		sda_in_sync_n		: std_logic;
	end record;

	type reg_type is record
		scl_buf				: std_logic_vector(1 downto 0);	--! SCL/SDA input buffers
		sda_in_buf			: std_logic_vector(1 downto 0);	--! SCL/SDA input buffers
		sda_in_dly			: std_logic;							--! SDA Edge detection
		scl_dly				: std_logic;							--! SCL Edge detection
		scl_re_dly			: std_logic_vector(1 downto 0);	--! Delayed SCL Rising-Edge for SDA data sampling
		i2c_active_seq		: std_logic;							--! I2C Slave Active
		i2c_state			: I2C_STATE_TYPE;						--! State register/wires
--		i2c_state_nxt		: I2C_STATE_TYPE;						--! State register/wires
		shift_buf			: std_logic_vector(8 downto 0);	--! Utility signals
		dbg_rd				: std_logic;							--! Utility signals
		dbg_i2c_sda_out	: std_logic;							--! 5) I2C TRANSMIT BUFFER
		dbg_state			: DBG_STATE_TYPE;						--! State register/wires
		dbg_addr				: std_logic_vector(5 downto 0);
		dbg_din_lo			: std_logic_vector(7 downto 0);
		dbg_din_hi			: std_logic_vector(7 downto 0);
		dbg_bw				: std_logic;							--! Utility signals
		dbg_wr				: std_logic;							--! Debug register data write command
	end record;

	signal	d		: fmsp_dbg_i2c_in_type;
	signal	r		: reg_type :=	(	scl_buf				=> "00",			--! SCL/SDA input buffers
												sda_in_buf			=> "00",			--! SCL/SDA input buffers
												sda_in_dly			=> '0',			--! SDA Edge detection
												scl_dly				=> '0',			--! SCL Edge detection
												scl_re_dly			=> "00",			--! Delayed SCL Rising-Edge for SDA data sampling
												i2c_active_seq		=> '0',			--! I2C Slave Active
												i2c_state			=> RX_ADDR,		--! State register/wires
												shift_buf			=> "000000000",--! Utility signals
												dbg_rd				=> '0',			--! Utility signals
												dbg_i2c_sda_out	=> '0',			--! I2C TRANSMIT BUFFER
												dbg_state			=> RX_CMD,		--! State register/wires
												dbg_addr				=>	"000000",
												dbg_din_lo			=>	"00000000",
												dbg_din_hi			=>	"00000000",
												dbg_bw				=> '0',			--! Utility signals
												dbg_wr				=> '0'			--! Debug register data write command
											);
	signal	rin	: reg_type;

begin

	d.dbg_dout				<=	dbg_dout;
	d.dbg_i2c_addr			<=	dbg_i2c_addr;
	d.dbg_i2c_broadcast	<=	dbg_i2c_broadcast;
	d.dbg_i2c_scl			<=	dbg_i2c_scl;
	d.dbg_i2c_sda_in		<=	dbg_i2c_sda_in;
	d.mem_burst				<=	mem_burst;
	d.mem_burst_end		<=	mem_burst_end;
	d.mem_burst_rd			<=	mem_burst_rd;
	d.mem_burst_wr			<=	mem_burst_wr;
	d.mem_bw					<=	mem_bw;

	COMB : process (d, r)
		variable	v						: reg_type;
		--! 1) I2C RECEIVE LINE SYNCHRONIZTION & FILTERING
		--! Synchronize SCL/SDA inputs
		variable	v_scl_sync_n					: std_logic;
		variable	v_scl_sync						: std_logic;
		variable	v_sda_in_sync_n				: std_logic;
		variable	v_sda_in_sync					: std_logic;
		--! SCL/SDA Majority decision
		variable	v_scl								: std_logic;
		variable	v_sda_in							: std_logic;
		--! SDA Edge detection
		variable	v_sda_in_fe						: std_logic;
		variable	v_sda_in_re						: std_logic;
--		variable	v_sda_in_edge					: std_logic;
		--! SCL Edge detection
		variable	v_scl_fe							: std_logic;
		variable	v_scl_re							: std_logic;
--		variable	v_scl_edge						: std_logic;
		--! Delayed SCL Rising-Edge for SDA data sampling
		variable	v_scl_sample					: std_logic;
		--! 2) I2C START & STOP CONDITION DETECTION
		--! Start condition
		variable	v_start_detect					: std_logic;
		--! Stop condition
		variable	v_stop_detect					: std_logic;
		--! I2C Slave Active
		variable	v_i2c_addr_not_valid			: std_logic;
		variable	v_i2c_active					: std_logic;
		variable	v_i2c_init						: std_logic;
		--! State machine
		variable	v_i2c_state_nxt				: I2C_STATE_TYPE;
		--! 4) I2C SHIFT REGISTER (FOR RECEIVING & TRANSMITING)
		variable	v_shift_rx_en					: std_logic;
		variable	v_shift_tx_en					: std_logic;
		variable	v_shift_tx_en_pre				: std_logic;
		variable	v_shift_rx_done				: std_logic;
		variable	v_shift_tx_done				: std_logic;
		variable	v_shift_buf_rx_init			: std_logic;
		variable	v_shift_buf_rx_en				: std_logic;
		variable	v_shift_buf_tx_init			: std_logic;
		variable	v_shift_buf_tx_en				: std_logic;
		variable	v_shift_tx_val					: std_logic_vector(7 downto 0);
		variable	v_shift_buf_nxt				: std_logic_vector(8 downto 0);
		variable	v_shift_buf						: std_logic;
		--! Detect when the received I2C device address is not valid
--		variable	v_i2c_addr_not_valid			: std_logic;
		variable	UNUSED_dbg_i2c_broadcast	: std_logic_vector(6 downto 0);
		--! Utility signals
		variable	v_shift_rx_data_done			: std_logic;
		variable	v_shift_tx_data_done			: std_logic;
		--! State machine
		variable	v_dbg_state_nxt				: DBG_STATE_TYPE;
		--! Utility signals
		variable	v_cmd_valid						: std_logic;
		variable	v_rx_lo_valid					: std_logic;
		variable	v_rx_hi_valid					: std_logic;
		--! Debug register address & bit width
		variable	v_dbg_addr						: std_logic_vector(5 downto 0);
		--! Debug register data input
		variable	v_dbg_din						: std_logic_vector(15 downto 0);
		--! Debug register data read command
--		variable	v_dbg_rd							: std_logic;
		--! Debug register data read value
--		variable	v_shift_tx_val					: std_logic;

	begin
	
		--! default assignment
		v := r;
		--! overriding assignments

		v_scl_sync		:= not(d.scl_sync_n);
		v_sda_in_sync	:= not(d.sda_in_sync_n);
		--! SCL/SDA input buffers
		v.scl_buf		:=	r.scl_buf(0) & v_scl_sync;
		v.sda_in_buf	:=	r.sda_in_buf(0) & v_sda_in_sync;



		--! SCL/SDA Majority decision
		v_scl	:=		(v_scl_sync		and	r.scl_buf(0))
					or	(v_scl_sync		and	r.scl_buf(0))
					or	(r.scl_buf(0)	and	r.scl_buf(1));

		v_sda_in	:=		(v_sda_in_sync		and	r.sda_in_buf(0))
						or	(v_sda_in_sync		and	r.sda_in_buf(1))
						or	(r.sda_in_buf(0)	and	r.sda_in_buf(1));


		--! SCL/SDA Edge detection
		--------------------------------

		--! SDA Edge detection
		v.sda_in_dly	:=  v_sda_in;

		v_sda_in_fe		:=			r.sda_in_dly	and not(	v_sda_in);
		v_sda_in_re		:=	not(	r.sda_in_dly)	and		v_sda_in;
--		v_sda_in_edge	:=			r.sda_in_dly	xor		v_sda_in;

		--! SCL Edge detection
		v.scl_dly	:=  v_scl;

		v_scl_fe		:=			r.scl_dly	and not(	v_scl);
		v_scl_re		:=	not(	r.scl_dly)	and		v_scl;
--		v_scl_edge	:=			r.scl_dly	xor		v_scl;


		--! Delayed SCL Rising-Edge for SDA data sampling
		v.scl_re_dly	:=	r.scl_re_dly(0) & v_scl_re;

		v_scl_sample	:=	r.scl_re_dly(1);

--=============================================================================
--! 4) I2C SHIFT REGISTER (FOR RECEIVING)
--=============================================================================
		v_shift_rx_en		:=	'0';
		if	(		(r.i2c_state = RX_ADDR) 
				or	(r.i2c_state = RX_DATA)
				or	(r.i2c_state = RX_DATA_ACK)
			)	then
			v_shift_rx_en	:=	'1';
		end if;
		v_shift_rx_done	:= v_shift_rx_en and v_scl_fe and r.shift_buf(8);
		v_shift_buf_rx_init	:=	'0';
		if	(		(v_i2c_init = '1')
				or	(			(r.i2c_state = RX_ADDR_ACK) 
						and	(v_scl_fe = '1')
						and	(not(r.shift_buf(8)) = '1')
					)
				or	(			(r.i2c_state = RX_DATA_ACK) 
						and	(v_scl_fe = '1')
					)
			)	then
			v_shift_buf_rx_init	:=	'1';
		end if;
		v_shift_buf_rx_en		:= v_shift_rx_en	and	v_scl_sample;
		--! Detect when the received I2C device address is not valid
		v_i2c_addr_not_valid	:=	'0';
		if	(DBG_I2C_BROADCAST_EN = true)	then
			if	(			(r.i2c_state = RX_ADDR) 
					and	(v_shift_rx_done = '1')
					and	(r.shift_buf(7 downto 1) /= d.dbg_i2c_broadcast(6 downto 0))
					and	(r.shift_buf(7 downto 1) /= d.dbg_i2c_addr(6 downto 0))
				)	then
					v_i2c_addr_not_valid	:=	'1';
			end if;
 		else
			if	(			(r.i2c_state = RX_ADDR) 
					and	(v_shift_rx_done = '1')
					and	(r.shift_buf(7 downto 1) /= d.dbg_i2c_addr(6 downto 0))
				)	then
					v_i2c_addr_not_valid	:=	'1';
			end if;
		end if;
--=============================================================================
--! 4) I2C SHIFT REGISTER (FOR RECEIVING & TRANSMITING)
--=============================================================================
		v_shift_tx_en		:=	'0';
		if	(		(r.i2c_state = TX_DATA)
				or	(r.i2c_state = TX_DATA_ACK)
			)	then
			v_shift_tx_en	:=	'1';
		end if;
		if	(r.shift_buf = "100000000")	then
			v_shift_tx_done	:=	v_shift_tx_en and v_scl_fe;
		else
			v_shift_tx_done	:=	'0';
		end if;

--=============================================================================
--! 2) I2C START & STOP CONDITION DETECTION
--=============================================================================

		--! Start condition
		v_start_detect	:=	v_sda_in_fe and v_scl;
		--! Stop condition
		v_stop_detect	:=	v_sda_in_re and v_scl;

		-------------------
		--! I2C Slave Active
		-------------------
		--! The I2C logic will be activated whenever a start condition
		--! is detected and will be disactivated if the slave address
		--! doesn't match or if a stop condition is detected.
		if (v_start_detect = '1') then
			v.i2c_active_seq	:= '1';
		elsif ((v_stop_detect = '1') or (v_i2c_addr_not_valid = '1')) then
			v.i2c_active_seq := '0';
		end if;

		v_i2c_active	:=	r.i2c_active_seq and not(v_stop_detect);
		v_i2c_init		:= not(v_i2c_active) or v_start_detect;


		--=============================================================================
		--! 3) I2C STATE MACHINE
		--=============================================================================

		--! State transition
		case (r.i2c_state) is
			when RX_ADDR	=>
				if (v_i2c_init = '1') then
					v_i2c_state_nxt	:=	RX_ADDR;
				elsif (not(v_shift_rx_done) = '1') then
					v_i2c_state_nxt	:=	RX_ADDR;
				elsif (not(v_i2c_addr_not_valid) = '1') then
					v_i2c_state_nxt	:=	RX_ADDR;
				else
					v_i2c_state_nxt	:=	RX_ADDR_ACK;
				end if;
			when RX_ADDR_ACK	=>
				if (v_i2c_init = '1') then
					v_i2c_state_nxt	:=	RX_ADDR;
				elsif (not(v_scl_fe) = '1') then
					v_i2c_state_nxt	:=	RX_ADDR_ACK;
				elsif (r.shift_buf(0) = '1') then
					v_i2c_state_nxt	:=	TX_DATA;
				else
					v_i2c_state_nxt	:=	RX_DATA;
				end if;
			when RX_DATA	=>
				if (v_i2c_init = '1') then
					v_i2c_state_nxt	:=	RX_ADDR;
				elsif (not(v_shift_rx_done) = '1') then
					v_i2c_state_nxt	:=	RX_DATA;
				else
					v_i2c_state_nxt	:=	RX_DATA_ACK;
				end if;
			when RX_DATA_ACK	=>
				if (v_i2c_init = '1') then
					v_i2c_state_nxt	:=	RX_ADDR;
				elsif (not(v_scl_fe) = '1') then
					v_i2c_state_nxt	:=	RX_DATA_ACK;
				else
					v_i2c_state_nxt	:=	RX_DATA;
				end if;
			when TX_DATA	=>
				if (v_i2c_init = '1') then
					v_i2c_state_nxt	:=	RX_ADDR;
				elsif (not(v_shift_tx_done) = '1') then
					v_i2c_state_nxt	:=	TX_DATA;
				else
					v_i2c_state_nxt	:=	TX_DATA_ACK;
				end if;
			when TX_DATA_ACK	=>
				if (v_i2c_init = '1') then
					v_i2c_state_nxt	:=	RX_ADDR;
				elsif (not(v_scl_fe) = '1') then
					v_i2c_state_nxt	:=	TX_DATA_ACK;
				elsif (not(v_sda_in) = '1') then
					v_i2c_state_nxt	:=	TX_DATA;
				else
					v_i2c_state_nxt	:=	RX_ADDR;
				end if;
			when Others	=>
				v_i2c_state_nxt	:=	RX_ADDR;
		end case;

		--! State machine
		v.i2c_state	:=	v_i2c_state_nxt;



--=============================================================================
--! 4) I2C SHIFT REGISTER (FOR RECEIVING & TRANSMITING)
--=============================================================================

		v_shift_tx_en_pre	:=	'0';
		if	(		(v_i2c_state_nxt = TX_DATA)
				or	(v_i2c_state_nxt = TX_DATA_ACK)
			)	then
			v_shift_tx_en_pre	:=	'1';
		end if;

		v_shift_buf_tx_init	:=	'0';
		if	(		(			(r.i2c_state = RX_ADDR_ACK) 
						and	(v_scl_re = '1')
						and	(not(r.shift_buf(0)) = '1')
					)
				or	(			(r.i2c_state = TX_DATA_ACK) 
						and	(v_scl_re = '1')
					)
			)	then
			v_shift_buf_tx_init	:=	'1';
		end if;

		if	(r.shift_buf /= "100000000")	then
			v_shift_buf_tx_en	:=	v_shift_tx_en_pre and v_scl_fe;
		else
			v_shift_buf_tx_en	:=	'0';
		end if;

		--! Debug register data read value
		if (r.dbg_state = TX_BYTE_HI) then
			v_shift_tx_val	:=	d.dbg_dout(15 downto 8);
		else
			v_shift_tx_val	:=	d.dbg_dout(7 downto 0);
		end if;

		if (v_shift_buf_rx_init = '1') then		--! RX Init
			v_shift_buf_nxt	:=	"000000001";
		elsif (v_shift_buf_tx_init = '1') then	--! TX Init
			v_shift_buf_nxt	:=	v_shift_tx_val & '1';
		elsif (v_shift_buf_rx_en = '1') then	--! RX Shift
			v_shift_buf_nxt	:=	r.shift_buf(7 downto 0) & v_sda_in;
		elsif (v_shift_buf_tx_en = '1') then	--! TX Shift
			v_shift_buf_nxt	:=	r.shift_buf(7 downto 0) & '0';
		else												--! Hold
			v_shift_buf_nxt	:=	r.shift_buf(8 downto 0);
		end if;

		v.shift_buf	:=	v_shift_buf_nxt;

		UNUSED_dbg_i2c_broadcast	:= d.dbg_i2c_broadcast;


		--! Utility signals
		v_shift_rx_data_done	:=	'0';
		if	(			(r.i2c_state = RX_DATA) 
				and	(v_shift_rx_done = '1')
			)	then
				v_shift_rx_data_done	:=	'1';
			end if;
		v_shift_tx_data_done := v_shift_tx_done;


		--=============================================================================
		--! 5) I2C TRANSMIT BUFFER
		--=============================================================================
		if (v_scl_fe) then
			if	(		(v_i2c_state_nxt = RX_ADDR_ACK) 
					or	(v_i2c_state_nxt = RX_ADDR_ACK) 
					or	(			(v_shift_buf_tx_en = '1') 
							and	(not(r.shift_buf(8)) = '1')
						)
				)	then
				v.dbg_i2c_sda_out	:=	'1';
			else
				v.dbg_i2c_sda_out	:=	'1';
			end if;
		end if;

		--=============================================================================
		--! 6) DEBUG INTERFACE STATE MACHINE
		--=============================================================================
		--! State transition
		case (r.dbg_state) is
			when RX_CMD	=>
				if (d.mem_burst_wr = '1') then
					v_dbg_state_nxt	:=	RX_BYTE_LO;
				elsif (d.mem_burst_rd = '1') then
					v_dbg_state_nxt	:=	TX_BYTE_LO;
				elsif (not(v_shift_rx_data_done) = '1') then
					v_dbg_state_nxt	:=	RX_CMD;
				elsif (r.shift_buf(7) = '1') then
					v_dbg_state_nxt	:=	RX_BYTE_LO;
				else
					v_dbg_state_nxt	:=	TX_BYTE_LO;
				end if;
 			when RX_BYTE_LO	=>
				if ( (d.mem_burst = '1') and (d.mem_burst_end = '1') ) then
					v_dbg_state_nxt	:=	RX_CMD;
				elsif (not(v_shift_rx_data_done) = '1') then
					v_dbg_state_nxt	:=	RX_BYTE_LO;
				elsif ( (d.mem_burst = '1') and (not(d.mem_burst_end) = '1') ) then
					if (not(d.mem_bw) = '1') then
						v_dbg_state_nxt	:=	RX_BYTE_LO;
					else
						v_dbg_state_nxt	:=	RX_BYTE_HI;
					end if;
				elsif (not(r.dbg_bw) = '1') then
					v_dbg_state_nxt	:=	RX_CMD;
				else
					v_dbg_state_nxt	:=	RX_BYTE_HI;
				end if;
			when RX_BYTE_HI	=>
				if (not(v_shift_rx_data_done) = '1') then
					v_dbg_state_nxt	:=	RX_BYTE_HI;
				elsif ( (d.mem_burst_wr = '1') and (not(d.mem_burst_end) = '1') ) then
					v_dbg_state_nxt	:=	RX_BYTE_LO;
				else
					v_dbg_state_nxt	:=	RX_CMD;
				end if;
			when TX_BYTE_LO	=>
				if (not(v_shift_tx_data_done) = '1') then
					v_dbg_state_nxt	:=	TX_BYTE_LO;
				elsif ( (d.mem_burst = '1') and (d.mem_bw = '1') ) then
					v_dbg_state_nxt	:=	TX_BYTE_LO;
				elsif ( (d.mem_burst = '1') and (not(d.mem_bw) = '1') ) then
					v_dbg_state_nxt	:=	TX_BYTE_HI;
				elsif (not(r.dbg_bw) = '1') then
					v_dbg_state_nxt	:=	TX_BYTE_HI;
				else
					v_dbg_state_nxt	:=	RX_CMD;
				end if;
			when TX_BYTE_HI	=>
				if (not(v_shift_rx_data_done) = '1') then
					v_dbg_state_nxt	:=	TX_BYTE_HI;
				elsif (d.mem_burst = '1') then
					v_dbg_state_nxt	:=	TX_BYTE_LO;
				else
					v_dbg_state_nxt	:=	RX_CMD;
				end if;
			when Others	=>
				v_dbg_state_nxt	:=	RX_CMD;
		end case;

		--! State machine
		v.dbg_state	:=	v_dbg_state_nxt;

		--! Utility signals
		if (			(r.dbg_state = RX_CMD)
				and	(v_shift_rx_data_done = '1')
			) then
			v_cmd_valid	:= '1';
		else
			v_cmd_valid	:= '0';
		end if;
		if (			(r.dbg_state = RX_BYTE_LO)
				and	(v_shift_rx_data_done = '1')
			) then
			v_rx_lo_valid	:= '1';
		else
			v_rx_lo_valid	:= '0';
		end if;
		if (			(r.dbg_state = RX_BYTE_HI)
				and	(v_shift_rx_data_done = '1')
			) then
			v_rx_hi_valid	:= '1';
		else
			v_rx_hi_valid	:= '0';
		end if;

		--=============================================================================
		--! 7) REGISTER READ/WRITE ACCESS
		--=============================================================================
		if (v_cmd_valid = '1') then
			v.dbg_bw		:= r.shift_buf(6);
			v.dbg_addr	:= r.shift_buf(5 downto 0);
		elsif (d.mem_burst = '1') then
			v.dbg_bw		:= d.mem_bw;
			v.dbg_addr	:= MEM_DATA;
		end if;


		--! Debug register data input
		if (v_rx_lo_valid = '1') then
			v.dbg_din_lo	:= r.shift_buf(7 downto 0);
		end if;
		if (v_rx_lo_valid = '1') then
			v.dbg_din_hi	:= x"00";
		elsif (v_rx_hi_valid = '1') then
 			v.dbg_din_hi	:= r.shift_buf(7 downto 0);
		end if;

		v_dbg_din := r.dbg_din_hi & r.dbg_din_lo;


		--! Debug register data write command
		if ((d.mem_burst and d.mem_bw) = '1') then
			v.dbg_wr	:= v_rx_lo_valid;
		elsif ((d.mem_burst and not(d.mem_bw)) = '1') then
 			v.dbg_wr	:= v_rx_hi_valid;
		elsif (r.dbg_bw = '1') then
 			v.dbg_wr	:= v_rx_lo_valid;
		else
 			v.dbg_wr	:= v_rx_hi_valid;
		end if;

		--! Debug register data read command
		if ((d.mem_burst and d.mem_bw) = '1') then
			if (			(r.dbg_state = TX_BYTE_LO)
					and	(v_shift_tx_data_done = '1')
				) then
				v.dbg_rd	:= '1';
			else
				v.dbg_rd	:= '0';
			end if;
		elsif ((d.mem_burst and not(d.mem_bw)) = '1') then
 			if (			(r.dbg_state = TX_BYTE_HI)
					and	(v_shift_tx_data_done = '1')
				) then
				v.dbg_rd	:= '1';
			else
				v.dbg_rd	:= '0';
			end if;
		elsif (v_cmd_valid = '1') then
 			v.dbg_rd	:= not(r.shift_buf(7));
		else
 			v.dbg_rd	:= '0';
		end if;

		--! drive register inputs
		rin <= v;

		--! drive module outputs
		dbg_addr				<= r.dbg_addr;	--! Debug register address
		dbg_din				<= v_dbg_din;	--! Debug register data input
		dbg_i2c_sda_out	<= r.dbg_i2c_sda_out;	--! Debug interface: I2C SDA OUT
		dbg_rd				<= r.dbg_rd;	--! Debug register data read
		dbg_wr				<= r.dbg_wr;	--! Debug register data write

	end process COMB;

	REGS : process (dbg_clk,dbg_rst)
	begin
		if (dbg_rst = '1') then
			r	<=	(	scl_buf				=> "11",			--! SCL input buffer
						sda_in_buf			=> "11",			--! SDA input buffer
						sda_in_dly			=> '0',			--! SDA Edge detection
						scl_dly				=> '0',			--! SCL Edge detection
						scl_re_dly			=> "00",			--! Delayed SCL Rising-Edge for SDA data sampling
						i2c_active_seq		=> '0',			--! I2C Slave Active
						i2c_state			=> RX_ADDR,		--! State register/wires
						shift_buf			=> "000000000",--! Utility signals
						dbg_rd				=> '0',			--! Utility signals
						dbg_i2c_sda_out	=> '0',			--! I2C TRANSMIT BUFFER
						dbg_state			=> RX_CMD,		--! State register/wires
						dbg_addr				=>	"000000",
						dbg_din_lo			=>	"00000000",
						dbg_din_hi			=>	"00000000",
						dbg_bw				=> '0',			--! Utility signals
						dbg_wr				=> '0'			--! Debug register data write command
					);
		elsif rising_edge(dbg_clk) then
			r	<= rin;
		end if;
	end process REGS;

	sync_cell_i2c_scl : fmsp_sync_cell
	port map(
		clk		=> dbg_clk,
		rst		=> dbg_rst,
		data_in	=> not(dbg_i2c_scl),
		data_out	=> d.scl_sync_n
	);
	sync_cell_i2c_sda : fmsp_sync_cell
	port map(
		clk		=> dbg_clk,
		rst		=> dbg_rst,
		data_in	=> not(dbg_i2c_sda_in),
		data_out	=> d.sda_in_sync_n
	);

end RTL;
