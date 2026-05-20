library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity uart_rx is
	port (
		clk       : in  std_logic;
		rst       : in  std_logic;
		rx_serial : in  std_logic;
		rx_data   : out std_logic_vector(7 downto 0);
		rx_valid  : out std_logic
	);
end entity;

architecture rtl of uart_rx is
	constant CLKS_PER_BIT : unsigned(9 downto 0) := to_unsigned(867, 10);
	constant HALF_BIT     : unsigned(9 downto 0) := to_unsigned(433, 10);

	type state_t is (S_IDLE, S_START, S_DATA, S_STOP);
	signal state     : state_t := S_IDLE;
	signal clk_count : unsigned(9 downto 0) := (others => '0');
	signal bit_idx   : unsigned(2 downto 0) := (others => '0');
	signal shift_reg : std_logic_vector(7 downto 0) := (others => '0');

	signal rx_sync1 : std_logic := '1';
	signal rx_sync2 : std_logic := '1';
begin

	-- metastability protection
	process(clk)
	begin
		if rising_edge(clk) then
			rx_sync1 <= rx_serial;
			rx_sync2 <= rx_sync1;
		end if;
	end process;

	process(clk)
	begin
		if rising_edge(clk) then
			rx_valid <= '0';

			if rst = '1' then
				state     <= S_IDLE;
				clk_count <= (others => '0');
				bit_idx   <= (others => '0');
			else
				case state is
					when S_IDLE =>
						if rx_sync2 = '0' then
							state     <= S_START;
							clk_count <= (others => '0');
						end if;

					when S_START =>
						if clk_count = HALF_BIT then
							if rx_sync2 = '0' then
								state     <= S_DATA;
								clk_count <= (others => '0');
								bit_idx   <= (others => '0');
							else
								state <= S_IDLE;
							end if;
						else
							clk_count <= clk_count + 1;
						end if;

					when S_DATA =>
						if clk_count = CLKS_PER_BIT then
							clk_count <= (others => '0');
							shift_reg(to_integer(bit_idx)) <= rx_sync2;
							if bit_idx = 7 then
								state <= S_STOP;
							else
								bit_idx <= bit_idx + 1;
							end if;
						else
							clk_count <= clk_count + 1;
						end if;

					when S_STOP =>
						if clk_count = CLKS_PER_BIT then
							state    <= S_IDLE;
							rx_valid <= '1';
							rx_data  <= shift_reg;
						else
							clk_count <= clk_count + 1;
						end if;
				end case;
			end if;
		end if;
	end process;

end architecture;
