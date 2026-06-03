library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity divider is
	port (
		clk      : in  std_logic;
		start    : in  std_logic;
		dividend : in  signed(63 downto 0);
		divisor  : in  signed(63 downto 0);
		quotient : out signed(31 downto 0);
		done     : out std_logic
	);
end entity;

-- Restoring divider: one bit per clock cycle.
-- 64 iterations for 64-bit dividend, result taken as lower 32 bits.
-- Total latency: 66 cycles (1 setup + 64 iterations + 1 output).

architecture rtl of divider is

	type state_t is (D_IDLE, D_CALC, D_DONE);
	signal state : state_t := D_IDLE;

	signal count    : unsigned(5 downto 0) := (others => '0');
	signal neg_res  : std_logic := '0';
	signal dvd_reg  : unsigned(63 downto 0) := (others => '0');
	signal dvs_reg  : unsigned(63 downto 0) := (others => '0');
	signal rem_reg  : unsigned(63 downto 0) := (others => '0');
	signal quot_reg : unsigned(63 downto 0) := (others => '0');

begin

	process(clk)
		variable new_rem : unsigned(63 downto 0);
	begin
		if rising_edge(clk) then
			done <= '0';

			case state is

				when D_IDLE =>
					if start = '1' then
						if divisor = to_signed(0, 64) then
							quotient <= to_signed(65536, 32);
							done     <= '1';
						else
							neg_res <= dividend(63) xor divisor(63);
							if dividend(63) = '1' then
								dvd_reg <= unsigned(-dividend);
							else
								dvd_reg <= unsigned(dividend);
							end if;
							if divisor(63) = '1' then
								dvs_reg <= unsigned(-divisor);
							else
								dvs_reg <= unsigned(divisor);
							end if;
							rem_reg  <= (others => '0');
							quot_reg <= (others => '0');
							count    <= to_unsigned(63, 6);
							state    <= D_CALC;
						end if;
					end if;

				when D_CALC =>
					new_rem := rem_reg(62 downto 0) & dvd_reg(63);
					dvd_reg <= dvd_reg(62 downto 0) & '0';

					if new_rem >= dvs_reg then
						rem_reg  <= new_rem - dvs_reg;
						quot_reg <= quot_reg(62 downto 0) & '1';
					else
						rem_reg  <= new_rem;
						quot_reg <= quot_reg(62 downto 0) & '0';
					end if;

					if count = 0 then
						state <= D_DONE;
					else
						count <= count - 1;
					end if;

				when D_DONE =>
					done <= '1';
					if neg_res = '1' then
						quotient <= -signed(quot_reg(31 downto 0));
					else
						quotient <= signed(quot_reg(31 downto 0));
					end if;
					state <= D_IDLE;

			end case;
		end if;
	end process;

end architecture;
