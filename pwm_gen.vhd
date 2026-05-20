library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity pwm_gen is
	port (
		clk     : in  std_logic;
		rst     : in  std_logic;
		angle   : in  std_logic_vector(7 downto 0);
		pwm_out : out std_logic
	);
end entity;

architecture rtl of pwm_gen is
	signal counter   : unsigned(20 downto 0) := (others => '0');
	signal threshold : unsigned(20 downto 0);
begin

	-- 0 deg = 1.0ms = 100000 counts, 180 deg = 2.0ms = 200000 counts
	-- threshold = 100000 + angle * 556 (556 ~= 100000/180)
	threshold <= to_unsigned(100000, 21) +
	             resize(unsigned(angle) * to_unsigned(556, 10), 21);

	process(clk)
	begin
		if rising_edge(clk) then
			if rst = '1' then
				counter <= (others => '0');
			elsif counter = to_unsigned(1999999, 21) then
				counter <= (others => '0');
			else
				counter <= counter + 1;
			end if;
		end if;
	end process;

	pwm_out <= '1' when counter < threshold else '0';

end architecture;
