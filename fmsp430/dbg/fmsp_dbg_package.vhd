------------------------------------------------------------------------------
--! Copyright (C) 2017 , Emmanuel Amadio
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
--! @file fmsp_dbg_package.vhd
--! 
--! @brief fpgaMSP430 Debug Package
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

package fmsp_dbg_package is

	component fmsp_dbg_uart is 
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
	end component fmsp_dbg_uart;

	component fmsp_dbg_i2c is 
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
	end component fmsp_dbg_i2c;

	component fmsp_dbg is 
	generic (
		DBG_UART					: boolean := false;		--	Enable UART (8N1) debug interface
		DBG_I2C					: boolean := true;		--	Enable I2C debug interface
		DBG_I2C_BROADCAST_EN	: boolean := false;		--! Enable the I2C broadcast address
		DBG_RST_BRK_EN			: boolean := false;		--! CPU break on PUC reset
		DBG_HWBRK_0_EN			: boolean := false;		--	Include hardware breakpoints unit 
		DBG_HWBRK_1_EN			: boolean := false;		--	Include hardware breakpoints unit 
		DBG_HWBRK_2_EN			: boolean := false;		--	Include hardware breakpoints unit 
		DBG_HWBRK_3_EN			: boolean := false;		--	Include hardware breakpoints unit 
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
		dbg_i2c_sda_out	: out	std_logic;       						--! Debug interface: I2C SDA OUT
		dbg_mem_addr		: out	std_logic_vector(15 downto 0);	--! Debug address for rd/wr access
		dbg_mem_dout		: out	std_logic_vector(15 downto 0);	--! Debug unit data output
		dbg_mem_en			: out	std_logic;       						--! Debug unit memory enable
		dbg_mem_wr			: out	std_logic_vector(1 downto 0);		--! Debug unit memory write
		dbg_reg_wr			: out	std_logic;       						--! Debug unit CPU register write
		dbg_uart_txd		: out	std_logic       						--! Debug interface: UART TXD
	);
end component fmsp_dbg;

component fmsp_dbg_hwbrk is 
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
end component fmsp_dbg_hwbrk;

--! Debug interface
	constant	C_DBG_UART_WR	: integer := 18;
	constant	C_DBG_UART_BW	: integer := 17;
--	constant	C_DBG_UART_ADDR	: integer := 16:11

--! Debug interface CPU_CTL register
	constant	C_HALT			: integer := 0;
	constant	C_RUN				: integer := 1;
	constant	C_ISTEP			: integer := 2;
	constant	C_SW_BRK_EN		: integer := 3;
	constant	C_FRZ_BRK_EN	: integer := 4;
	constant	C_RST_BRK_EN	: integer := 5;
	constant	C_CPU_RST		: integer := 6;

--! Debug interface CPU_STAT register
	constant	C_HALT_RUN		: integer := 0;
	constant	C_PUC_PND		: integer := 1;
	constant	C_SWBRK_PND		: integer := 3;
	constant	C_HWBRK0_PND	: integer := 4;
	constant	C_HWBRK1_PND	: integer := 5;

--! Debug interface BRKx_CTL register
	constant	C_BRK_MODE_RD	: integer := 0;
	constant	C_BRK_MODE_WR	: integer := 1;
--	constant	C_BRK_MODE		: integer := 1:0
	constant	C_BRK_EN			: integer := 2;
	constant	C_BRK_I_EN		: integer := 3;
	constant	C_BRK_RANGE		: integer := 4;

	
--! Basic clock module: BCSCTL1 Control Register
--	constant	C_DIVAx       5:4
	constant	C_DMA_CPUOFF	: integer := 0;
	constant	C_DMA_OSCOFF	: integer := 1;
	constant	C_DMA_SCG0		: integer := 2;
	constant	C_DMA_SCG1		: integer := 3;

--! Basic clock module: BCSCTL2 Control Register
	constant	C_SELMx			: integer := 7;
	constant	C_SELS			: integer := 3;
--	constant	C_DIVSx       2:1


--
--! DEBUG INTERFACE EXTRA CONFIGURATION
--======================================

--! Debug interface: CPU version
--!   1 - FPGA support only (Pre-BSD licence era)
--!   2 - Add ASIC support
--!   3 - Add DMA interface support
	constant	C_CPU_VERSION	: integer	range 0 to 7 := 1;

--! Debug interface: Software breakpoint opcode
	constant	C_DBG_SWBRK_OP : std_logic_vector(15 downto 0) := x"4343";

--! Debug UART interface auto data synchronization
--! If the following define is commented out, then
--! the DBG_UART_BAUD and DBG_DCO_FREQ need to be properly
--! defined.
--	constant	C_DBG_UART_AUTO_SYNC

--! Debug UART interface data rate
--!      In order to properly setup the UART debug interface, you
--!      need to specify the DCO_CLK frequency (DBG_DCO_FREQ) and
--!      the chosen BAUD rate from the UART interface.
--
--	constant	C_DBG_UART_BAUD    9600
--	constant	C_DBG_UART_BAUD   19200
--	constant	C_DBG_UART_BAUD   38400
--	constant	C_DBG_UART_BAUD   57600
--	constant	C_DBG_UART_BAUD  115200
--	constant	C_DBG_UART_BAUD  230400
--	constant	C_DBG_UART_BAUD  460800
--	constant	C_DBG_UART_BAUD  576000
--	constant	C_DBG_UART_BAUD  921600
--	constant	C_DBG_UART_BAUD 2000000
--	constant	C_DBG_DCO_FREQ  20000000
--	constant	C_DBG_UART_CNT ((`DBG_DCO_FREQ/`DBG_UART_BAUD)-1)

--! Debug interface selection
--!             	constant	C_DBG_UART -> Enable UART (8N1) debug interface
--!             	constant	C_DBG_JTAG -> DON'T UNCOMMENT, NOT SUPPORTED
--
--	constant	C_DBG_UART
--	constant	C_DBG_JTAG


end fmsp_dbg_package; --! fmsp_package

