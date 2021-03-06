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
--!
--! @file fmsp_scan_mux.vhd
--! 
--! @brief fpgaMSP430 Generic mux for scan mode
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

entity fmsp_scan_mux is 
port (
	--! INPUTs
	data_in_scan	: in	std_logic;		--! Selected data input for scan mode
	data_in_func	: in	std_logic;		--! Selected data input for functional mode
	scan_mode		: in	std_logic;		--! Scan mode
	--! OUTPUTs
	data_out			: out	std_logic		--! Scan mux data output
);
end entity fmsp_scan_mux;

architecture RTL of fmsp_scan_mux is 

begin

--=============================================================================
--! 1)  SCAN MUX
--=============================================================================

	DATA_MUX : process(scan_mode,data_in_scan,data_in_func)
	begin
		if (scan_mode = '1') then
			data_out	<= data_in_scan;
		else
			data_out	<= data_in_func;
		end if;
	end process DATA_MUX;

end RTL; --! fmsp_scan_mux
