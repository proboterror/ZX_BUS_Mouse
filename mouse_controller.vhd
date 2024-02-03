-- ZX BUS Kempston Mouse controller
-- Target: EPM3032ALC44 / EPM3064ALC44

library IEEE;
use IEEE.std_logic_1164.all;

-- Register with 3 state output.
-- Input data stored on C clock rising edge.
-- If OE then stored data set to output Q (async?) else Q set to Z state.
-- Do not pass through input data to output / do not set output on C clock (?).
entity register_generic_3_state is
generic(data_width : integer := 8);
port(
		D: in bit_vector(data_width - 1 downto 0);
		Q: out std_logic_vector(data_width - 1 downto 0) := (others => 'Z');
		C: in std_ulogic := '0'; -- clock (active high)
		OE: in bit := '0' -- output enable (active high)
	);
end register_generic_3_state;

architecture structure of register_generic_3_state is
begin
	process(C, OE)
	variable DATA: bit_vector(data_width - 1 downto 0); -- := (others => '0'); -- does it need to be initialized? 1?
	begin
		if (OE = '0') then
			Q <= (others => 'Z');
		else
			Q <= to_stdlogicvector(DATA);
		end if;

		if rising_edge(C) then
			DATA := D;
		end if;
	end process;
end structure;

library IEEE;
use IEEE.std_logic_1164.all;

-- Kempston mouse ports:
-- #FADF 1111 1010 1101 1111 BUTTONS
-- #FBDF 1111 1011 1101 1111 MX
-- #FFDF 1111 1111 1101 1111 MY

entity mouse_controller is
port(
		A0: in bit; -- 1
		A1: in bit; -- 1
		A7: in bit; -- 1
		M1: in bit; -- 1
		A5: in bit; -- 0
		RD: in bit; -- active low
		IORQ: in bit; -- active low
		A8: in bit;
		A10: in bit;
		IORQGE: out std_logic := 'Z'; -- 1 when address lower bits and M1 == 1 (address partial match), Z state otherwise.

		MX: in bit; -- init?
		MY: in bit; -- init?
		MKEY: in bit; -- init?
		DI: in bit_vector(7 downto 0);

		D: out std_logic_vector(7 downto 0) := (others => 'Z')
	);
end mouse_controller;
	
architecture structure of mouse_controller is
	
component register_generic_3_state
generic(data_width : integer := 8);
port(
		D: in bit_vector(data_width - 1 downto 0);
		Q: out std_logic_vector(data_width - 1 downto 0);
		C: in std_ulogic;
		OE: in bit
	);
end component;

signal MX_SEL, MY_SEL, MKEY_SEL: bit;
signal address_partial_match: bit;
signal enable: bit;

begin
	address_partial_match <= not(A0 and A1 and A7 and M1 and not A5);
	IORQGE <= '1' when address_partial_match = '0' else 'Z';
	enable <= not(address_partial_match or RD or IORQ);
	
	MKEY_SEL <= enable and not A8 and not A10;
	MX_SEL <= enable and A8 and not A10;
	MY_SEL <= enable and A8 and A10;

	register_x: register_generic_3_state
	port map(DI, D, C => to_stdulogic(MX), OE => MX_SEL);

	register_y: register_generic_3_state
	port map(DI, D, C => to_stdulogic(MY), OE => MY_SEL);

	-- 8 bit register required for mouse wheel support (currently requires 35 macrocells and thus EPM3064ALC44).
	-- ToDo: check port #FADF 3-7 bits state.
	register_key: register_generic_3_state
	generic map(data_width => 3)
	port map(DI(2 downto 0), D(2 downto 0), C => to_stdulogic(MKEY), OE => MKEY_SEL);
	-- Note:
	-- https://velesoft.speccy.cz/kmouse/km-doc/kempston_mouse_turbo_interface/km-t_2011/k-mouse2011-doc.pdf:
	-- If wheel is off then D4-D7 on button port return 1111.
	-- https://www.benophetinternet.nl/hobby/kmt/index.htm
	-- Bit 4-7 (D4-D7) return the position of the mouse wheel = %1111 is default
end structure;