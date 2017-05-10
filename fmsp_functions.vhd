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
--! @file fmsp_functions.vhd
--! 
--! @brief fpgaMSP430 Functions package
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

package fmsp_functions is

--	function reverse_vector (a: in std_logic_vector) return std_logic_vector; -- function reverse_any_vector
--	function bit_to_signed (a: in std_logic; w: integer) return signed; -- function reverse_any_vector
--	function carry_in_adder_b (a: in signed; b: in signed; c: std_logic) return signed; -- function reverse_any_vector
--	function carry_in_adder (a: in std_logic_vector; b: in std_logic_vector; c: std_logic) return std_logic_vector;
	function	bcd_add( X: std_logic_vector(3 downto 0); Y: std_logic_vector(3 downto 0); C: std_logic) return std_logic_vector;
	function	one_hot64( X: std_logic_vector(5 downto 0)) return std_logic_vector;
	function	one_hot16( X: std_logic_vector(3 downto 0)) return std_logic_vector;
	function	one_hot8( X: std_logic_vector(2 downto 0)) return std_logic_vector;
	function	get_irq_num( irq_all: std_logic_vector(62 downto 0)) return std_logic_vector;
	function f_log2(X: integer) return integer;
	function onehot(value : std_logic_vector) return std_logic_vector;
--	function int_is_even(value : integer) return boolean;
--	function int_is_odd(value : integer) return boolean;
--	function	numpar_bit_select(value : integer; B: std_logic; A: std_logic) return std_logic;
--	function	numpar_byte_select(value : integer; B: std_logic_vector(7 downto 0); A: std_logic_vector(7 downto 0)) return std_logic_vector;
--	function	byte_per_select_wr(value : integer; hi_wr: std_logic_vector; lo_wr: std_logic_vector) return std_logic;
	function	byte_per_select_din(value : integer; din: std_logic_vector(15 downto 0)) return std_logic_vector;
	function	byte_per_select_dout(value : integer; reg_rd:std_logic_vector; din: std_logic_vector(7 downto 0)) return std_logic_vector;
--	function	word_per_select_wr(value : integer; reg_wr: std_logic_vector) return std_logic;
	function	word_per_select_dout(value : integer; reg_rd:std_logic_vector; din: std_logic_vector(15 downto 0)) return std_logic_vector;

end package fmsp_functions;

package body fmsp_functions is

--function reverse_vector (a: in std_logic_vector) return std_logic_vector is
--  variable result: std_logic_vector(a'RANGE);
--  alias aa: std_logic_vector(a'REVERSE_RANGE) is a;
--begin
--  for i in aa'RANGE loop
--    result(i) := aa(i);
--  end loop;
--  return result;
--end; -- function reverse_any_vector
--
--function bit_to_signed (a: in std_logic; w: integer) return signed is
--  variable result	: signed(w-1 downto 0);
--  variable aa		: std_logic_vector(w-1 downto 0);
--begin
--  aa(0) := a;
--  for i in 1 to w-1 loop
--    aa(i) := '0';
--  end loop;
--  result := SIGNED(aa);
--  return result;
--end; -- function bit_to_signed
--
--function carry_in_adder (a: in std_logic_vector; b: in std_logic_vector; c: std_logic) return std_logic_vector is
--  variable add_r	: unsigned(a'LENGTH downto 0);
--  variable result	: std_logic_vector(a'LENGTH downto 0);
--  variable aa		: std_logic_vector(a'LENGTH downto 0);
--  variable bb		: std_logic_vector(a'LENGTH downto 0);
--  variable cc		: std_logic_vector(a'LENGTH downto 0);
--begin
--  aa := '0' & STD_LOGIC_VECTOR(a);
--  bb := '0' & STD_LOGIC_VECTOR(b);
--  cc(0) := c;
--  for i in 1 to cc'high loop
--    cc(i) := '0';
--  end loop;
--  add_r	:= UNSIGNED(aa) + UNSIGNED(bb) + UNSIGNED(cc);
--  result	:= STD_LOGIC_VECTOR(add_r);
--  return result;
--end; -- function carry_in_adder
--
--function carry_in_adder_b (a: in signed; b: in signed; c: std_logic) return signed is
--  variable result	: signed(a'LENGTH downto 0);
--  variable aa		: std_logic_vector(a'LENGTH downto 0);
--  variable bb		: std_logic_vector(a'LENGTH downto 0);
--  variable cc		: std_logic_vector(a'LENGTH downto 0);
--begin
--  aa := '0' & STD_LOGIC_VECTOR(a);
--  bb := '0' & STD_LOGIC_VECTOR(b);
--  cc(0) := c;
--  for i in 1 to cc'high loop
--    cc(i) := '0';
--  end loop;
--  result := SIGNED(aa) + SIGNED(bb) + SIGNED(cc);
--  return result;
--end; -- function carry_in_adder

	function	bcd_add( X: std_logic_vector(3 downto 0); Y: std_logic_vector(3 downto 0); C: std_logic) return std_logic_vector is
		variable result	: std_logic_vector(4 downto 0);
		variable	XX			: std_logic_vector(4 downto 0);
		variable	YY			: std_logic_vector(4 downto 0);
		variable	CC			: std_logic_vector(4 downto 0);
		variable	ZZ			: unsigned(4 downto 0);
   begin
		XX	:= '0' & X;	
		YY	:= '0' & Y;	
		CC	:= "0000" & C;	
		ZZ := UNSIGNED(XX)+UNSIGNED(YY)+UNSIGNED(CC);
      if ( ZZ < TO_UNSIGNED( 10, 5) ) then
			result := STD_LOGIC_VECTOR(ZZ);
      else
			result := STD_LOGIC_VECTOR(ZZ + TO_UNSIGNED( 6, 5));
		end if;
		return result;
	end function bcd_add;

	--! 64 bits one-hot decoder
	function	one_hot64( X: std_logic_vector(5 downto 0)) return std_logic_vector is
		variable result	: std_logic_vector(63 downto 0);
		variable	XX			: integer range 63 downto 0;
   begin
		XX	:= TO_INTEGER(UNSIGNED(X));
 		result := x"0000000000000000";
      result(XX) := '1';
		return result;
	end function one_hot64;

	--! 16 bits one-hot decoder
	function	one_hot16( X: std_logic_vector(3 downto 0)) return std_logic_vector is
		variable result	: std_logic_vector(15 downto 0);
		variable	XX			: integer range 15 downto 0;
   begin
		XX	:= TO_INTEGER(UNSIGNED(X));
 		result := x"0000";
      result(XX) := '1';
		return result;
	end function one_hot16;
   
	--! 8 bits one-hot decoder
	function	one_hot8( X: std_logic_vector(2 downto 0)) return std_logic_vector is
		variable result	: std_logic_vector(7 downto 0);
		variable	XX			: integer range 7 downto 0;
   begin
		XX	:= TO_INTEGER(UNSIGNED(X));
 		result := x"00";
      result(XX) := '1';
		return result;
	end function one_hot8;

	--! Get IRQ number
	function	get_irq_num( irq_all: std_logic_vector(62 downto 0)) return std_logic_vector is
		variable result	: std_logic_vector(5 downto 0);
   begin
		result := "111111";
		looping : for i in 62 downto 0 loop
			if ( (result = "111111") and (irq_all(i) = '1') ) then
				result	:= STD_LOGIC_VECTOR(TO_UNSIGNED(i,6));
			end if;
		end loop looping;		
		return result;
	end function get_irq_num;
	
	function f_log2(X: integer) return integer is
		variable result : integer;
	begin
		result := INTEGER(CEIL(LOG2(REAL(X-1))));
		return result;
	end function f_log2;

	function onehot(value : std_logic_vector) return std_logic_vector is
		variable result : std_logic_vector(2**value'length - 1 downto 0) := (others => '0');
	begin
		result(to_integer(unsigned(value))) := '1';
		return result;
	end function onehot;
	
	function int_is_even(value : integer) return boolean is
		variable result : boolean := false;
	begin
		if ( (value/2) = ((value+1)/2) ) then
			result := true;
		end if;
		return result;
	end function int_is_even;

	function int_is_odd(value : integer) return boolean is
		variable result : boolean := true;
	begin
		if ( (value/2) = ((value+1)/2) ) then
			result := false;
		end if;
		return result;
	end function int_is_odd;

	function	numpar_bit_select(value : integer; B: std_logic; A: std_logic) return std_logic is
		variable result : std_logic;
	begin
		if ( (value/2) = ((value+1)/2) ) then
			result := A;
		else
			result := B;
		end if;
		return result;
	end function numpar_bit_select;

	function	numpar_byte_select(value : integer; B: std_logic_vector(7 downto 0); A: std_logic_vector(7 downto 0)) return std_logic_vector is
		variable result : std_logic_vector(7 downto 0);
	begin
		if ( (value/2) = ((value+1)/2) ) then
			result := A;
		else
			result := B;
		end if;
		return result;
	end function numpar_byte_select;

--	function	byte_per_select_wr(value : integer; hi_wr: std_logic_vector; lo_wr: std_logic_vector) return std_logic is
--		variable result : std_logic;
--	begin
--		if ( (value/2) = ((value+1)/2) ) then
--			result := lo_wr(value/2);
--		else
--			result := hi_wr(value/2);
--		end if;
--		return result;
--	end function byte_per_select_wr;

	function	byte_per_select_din(value : integer; din: std_logic_vector(15 downto 0)) return std_logic_vector is
		variable result : std_logic_vector(7 downto 0);
	begin
		if ( (value/2) = ((value+1)/2) ) then
			result := din(7 downto 0);
		else
			result := din(15 downto 8);
		end if;
		return result;
	end function byte_per_select_din;

	function	byte_per_select_dout(value : integer; reg_rd:std_logic_vector; din: std_logic_vector(7 downto 0)) return std_logic_vector is
		variable result : std_logic_vector(15 downto 0);
	begin
		result := x"0000";
		if ( reg_rd(value) = '1') then
			if ( (value/2) = ((value+1)/2) ) then
				result := x"00" & din;
			else
				result := din & x"00";
			end if;
		end if;
		return result;
	end function byte_per_select_dout;
	
--	function	word_per_select_wr(value : integer; reg_wr: std_logic_vector) return std_logic is
--		variable result : std_logic;
--	begin
--		result := reg_wr(value/2);
--		return result;
--	end function word_per_select_wr;

	function	word_per_select_dout(value : integer; reg_rd:std_logic_vector; din: std_logic_vector(15 downto 0)) return std_logic_vector is
		variable result : std_logic_vector(15 downto 0);
	begin
		result := x"0000";
		if ( reg_rd(value/2) = '1') then
			result := din;
		end if;
		return result;
	end function word_per_select_dout;

end package body;


