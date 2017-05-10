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
--! @file fmsp_misc_package.vhd
--! 
--! @brief fpgaMSP430 Miscellaneous package
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

package fmsp_misc_package is


	component fmsp_and_gate is 
	port (
		--! INPUTs
		a	: in	std_logic;	--! AND gate input A
		b	: in	std_logic;	--! AND gate input B
		--! OUTPUTs
		y	: out	std_logic	--! AND gate outputt
	);
	end component fmsp_and_gate;

	component fmsp_sync_reset is 
	port (
		--! INPUTs
		clk		: in	std_logic;	--! Receiving clock
		rst_a		: in	std_logic;	--! Asynchronous reset
		--! OUTPUTs
		rst_s		: out	std_logic	--! Synchronized resett
	);
	end component fmsp_sync_reset;

	component fmsp_sync_cell is 
	generic (
		SYNC_EN	: boolean := true		--! Synchronize input
	);
	port (
		--! INPUTs
		clk		: in	std_logic;	--! Receiving clock
		data_in	: in	std_logic;	--! Asynchronous data input
		rst		: in	std_logic;	--! Receiving reset (active high)
		--! OUTPUTs
		data_out	: out	std_logic	--! Synchronized data output
	);
	end component fmsp_sync_cell;

end fmsp_misc_package; --! fmsp_misc_package



