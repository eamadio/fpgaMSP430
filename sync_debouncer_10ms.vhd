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
-- *File Name: sync_debouncer_10ms.vhd
--
-- *Module Description:
--                      Super basic 10ms debouncer.
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

entity sync_debouncer_10ms is
	port (
		-- INPUTs
		clk_50mhz			: in	std_logic;	-- 50MHz clock
		rst					: in	std_logic;	-- reset
		signal_async		: in	std_logic;	-- Asynchonous signal
		-- OUTPUTs
		signal_debounced	: out	std_logic	-- Synchronized and 10ms debounced signal
	);
end entity sync_debouncer_10ms;

architecture RTL of sync_debouncer_10ms is 

	signal	sync_stage					: std_logic_vector(1 downto 0);
	signal	debounce_counter			: unsigned(18 downto 0);

begin

	REGS : process (clk_50mhz, rst)
	begin
		if (rst = '1') then
			sync_stage			<= "00";
			debounce_counter	<= TO_UNSIGNED( 0, 19);
			signal_debounced	<= '0';
		elsif rising_edge(clk_50mhz) then
			-- Synchronize signal
			sync_stage	<= sync_stage(0) & signal_async;
			-- Debouncer (10.48ms = 0x7ffff x 50MHz clock cycles)
			if (signal_debounced = sync_stage(1)) then
				debounce_counter <= TO_UNSIGNED(0,19);
			else
				debounce_counter <= debounce_counter+1;
			end if;
			-- Output signal
			if (debounce_counter = TO_UNSIGNED( ((2**19)-1), 19)) then
				signal_debounced <= not(signal_debounced);
			end if;

		end if;
	end process REGS;

end RTL;	-- sync_debouncer_10ms