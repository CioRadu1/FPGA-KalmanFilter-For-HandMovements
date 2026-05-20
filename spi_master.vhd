library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- SPI mode 3: CPOL=1, CPHA=1
-- SCLK idles high
-- data changed on falling edge, sampled on rising edge
-- SCLK = 100MHz / 20 = 5MHz

entity spi_master is
	port (
		clk      : in  std_logic;
		rst      : in  std_logic;
		start    : in  std_logic;
		tx_data  : in  std_logic_vector(7 downto 0);
		rx_data  : out std_logic_vector(7 downto 0);
		busy     : out std_logic;
		done     : out std_logic;
		sclk_out : out std_logic;
		mosi     : out std_logic;
		miso     : in  std_logic;
		cs_n     : out std_logic
	);
end entity;

architecture rtl of spi_master is
	constant CLK_DIV : integer := 9;

	type state_t is (S_IDLE, S_LEADING_EDGE, S_TRAILING_EDGE, S_DONE);
	signal state     : state_t := S_IDLE;
	signal div_cnt   : unsigned(3 downto 0) := (others => '0');
	signal bit_cnt   : unsigned(2 downto 0) := (others => '0');
	signal sclk_reg  : std_logic := '1';
	signal mosi_reg  : std_logic := '0';
	signal shift_out : std_logic_vector(7 downto 0) := (others => '0');
	signal shift_in  : std_logic_vector(7 downto 0) := (others => '0');
	signal cs_reg    : std_logic := '1';

	signal miso_s1 : std_logic := '1';
	signal miso_s2 : std_logic := '1';
begin

	process(clk)
	begin
		if rising_edge(clk) then
			miso_s1 <= miso;
			miso_s2 <= miso_s1;
		end if;
	end process;

	process(clk)
	begin
		if rising_edge(clk) then
			done <= '0';

			if rst = '1' then
				state    <= S_IDLE;
				sclk_reg <= '1';
				cs_reg   <= '1';
				mosi_reg <= '0';
			else
				case state is
					when S_IDLE =>
						sclk_reg <= '1';
						if start = '1' then
							shift_out <= tx_data;
							cs_reg    <= '0';
							mosi_reg  <= tx_data(7);
							bit_cnt   <= (others => '0');
							div_cnt   <= (others => '0');
							state     <= S_LEADING_EDGE;
						end if;

					-- wait then generate falling edge, change MOSI
					when S_LEADING_EDGE =>
						if div_cnt = CLK_DIV then
							div_cnt  <= (others => '0');
							sclk_reg <= '0';
							state    <= S_TRAILING_EDGE;
						else
							div_cnt <= div_cnt + 1;
						end if;

					-- wait then generate rising edge, sample MISO
					when S_TRAILING_EDGE =>
						if div_cnt = CLK_DIV then
							div_cnt  <= (others => '0');
							sclk_reg <= '1';
							shift_in <= shift_in(6 downto 0) & miso_s2;

							if bit_cnt = 7 then
								state <= S_DONE;
							else
								bit_cnt   <= bit_cnt + 1;
								shift_out <= shift_out(6 downto 0) & '0';
								mosi_reg  <= shift_out(6);
								state     <= S_LEADING_EDGE;
							end if;
						else
							div_cnt <= div_cnt + 1;
						end if;

					when S_DONE =>
						rx_data  <= shift_in;
						done     <= '1';
						sclk_reg <= '1';
						state    <= S_IDLE;
				end case;
			end if;
		end if;
	end process;

	sclk_out <= sclk_reg;
	mosi     <= mosi_reg;
	cs_n     <= cs_reg;
	busy     <= '0' when state = S_IDLE else '1';

end architecture;
