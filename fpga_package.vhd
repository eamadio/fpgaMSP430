------------------------------------------------------------------------------
-- Copyright (C) 2009 , Emmanuel Amadio
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
-- *File Name: fpga_package.vhd
-- 
-- *Module Description:
--                       
--
-- *Author(s):
--! @author Emmanuel Amadio,   emmanuel.amadio@gmail.com (VHDL Rewrite)
--
------------------------------------------------------------------------------
-- $Rev: 103 $
-- $LastChangedBy: olivier.girard $
-- $LastChangedDate: 2011-03-05 15:44:48 +0100 (Sat, 05 Mar 2011) $
------------------------------------------------------------------------------
library ieee;
	use ieee.std_logic_1164.all;	-- standard unresolved logic UX01ZWLH-
	use ieee.numeric_std.all;		-- for the signed, unsigned types and arithmetic ops
	use ieee.math_real.all;

package fpga_package is


component ram_16x8k is
	port(
		-- INPUTs
		address	: in	std_logic_vector(12 downto 0);
		byteena	: in	std_logic_vector(1 downto 0);	
		clken		: in	std_logic;
		clock		: in	std_logic;
		data		: in	std_logic_vector(15 downto 0);
		wren		: in	std_logic;
		-- OUTPUTs
		q			: out	std_logic_vector(15 downto 0)
	);
end component ram_16x8k;

component ram_16x16k is
	port(
		-- INPUTs
		address	: in	std_logic_vector(13 downto 0);
		byteena	: in	std_logic_vector(1 downto 0);	
		clken		: in	std_logic;
		clock		: in	std_logic;
		data		: in	std_logic_vector(15 downto 0);
		wren		: in	std_logic;
		-- OUTPUTs
		q			: out	std_logic_vector(15 downto 0)
	);
end component ram_16x16k;

component sync_debouncer_10ms is
	port (
		-- INPUTs
		clk_50mhz			: in	std_logic;	-- 50MHz clock
		rst					: in	std_logic;	-- reset
		signal_async		: in	std_logic;	-- Asynchonous signal
		-- OUTPUTs
		signal_debounced	: out	std_logic	-- Synchronized and 10ms debounced signal
	);
end component sync_debouncer_10ms;

component fmsp_de0_nano_soc_led_key_sw is 
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
end component fmsp_de0_nano_soc_led_key_sw;

end fpga_package;