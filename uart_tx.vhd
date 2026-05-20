library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity uart_tx is
	port (
		clk       : in  std_logic;
		rst       : in  std_logic;
		tx_data   : in  std_logic_vector(7 downto 0);
		tx_start  : in  std_logic;
		tx_serial : out std_logic;
		tx_busy   : out std_logic
	);
end entity;

architecture rtl of uart_tx is
	constant CLKS_PER_BIT : unsigned(9 downto 0) := to_unsigned(867, 10);

	type state_t is (S_IDLE, S_START, S_DATA, S_STOP);
	signal state     : state_t := S_IDLE;
	signal clk_count : unsigned(9 downto 0) := (others => '0');
	signal bit_idx   : unsigned(2 downto 0) := (others => '0');
	signal data_reg  : std_logic_vector(7 downto 0) := (others => '0');
begin

	process(clk)
	begin
		if rising_edge(clk) then
			if rst = '1' then
				state     <= S_IDLE;
				tx_serial <= '1';
				clk_count <= (others => '0');
			else
				case state is
					when S_IDLE =>
						tx_serial <= '1';
						if tx_start = '1' then
							data_reg  <= tx_data;
							state     <= S_START;
							clk_count <= (others => '0');
						end if;

					when S_START =>
						tx_serial <= '0';
						if clk_count = CLKS_PER_BIT then
							state     <= S_DATA;
							clk_count <= (others => '0');
							bit_idx   <= (others => '0');
						else
							clk_count <= clk_count + 1;
						end if;

					when S_DATA =>
						tx_serial <= data_reg(to_integer(bit_idx));
						if clk_count = CLKS_PER_BIT then
							clk_count <= (others => '0');
							if bit_idx = 7 then
								state <= S_STOP;
							else
								bit_idx <= bit_idx + 1;
							end if;
						else
							clk_count <= clk_count + 1;
						end if;

					when S_STOP =>
						tx_serial <= '1';
						if clk_count = CLKS_PER_BIT then
							state <= S_IDLE;
						else
							clk_count <= clk_count + 1;
						end if;
				end case;
			end if;
		end if;
	end process;

	tx_busy <= '0' when state = S_IDLE else '1';

end architecture;
