library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity system_tick_gen is
	port (
		clk       : in  std_logic;
		rst       : in  std_logic;
		tick_20ms : out std_logic;
		tick_cnt  : out std_logic_vector(20 downto 0)
	);
end entity;

architecture rtl of system_tick_gen is
	constant PERIOD : unsigned(20 downto 0) := to_unsigned(1999999, 21);
	signal counter  : unsigned(20 downto 0) := (others => '0');
begin

	process(clk)
	begin
		if rising_edge(clk) then
			if rst = '1' then
				counter <= (others => '0');
			elsif counter = PERIOD then
				counter <= (others => '0');
			else
				counter <= counter + 1;
			end if;
		end if;
	end process;

	tick_20ms <= '1' when counter = PERIOD else '0';
	tick_cnt  <= std_logic_vector(counter);

end architecture;
