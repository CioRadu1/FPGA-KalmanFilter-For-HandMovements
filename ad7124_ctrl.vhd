library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity ad7124_ctrl is
	port (
		clk         : in  std_logic;
		rst         : in  std_logic;
		tick_20ms   : in  std_logic;
		-- SPI master interface
		spi_start   : out std_logic;
		spi_tx_data : out std_logic_vector(7 downto 0);
		spi_rx_data : in  std_logic_vector(7 downto 0);
		spi_busy    : in  std_logic;
		spi_done    : in  std_logic;
		spi_cs_n    : out std_logic;
		-- results: 9 measured angles (0-180), embedded conversion
		meas_angles : out std_logic_vector(71 downto 0);
		meas_valid  : out std_logic;
		config_done : out std_logic
	);
end entity;

architecture rtl of ad7124_ctrl is

	type main_state_t is (
		S_RESET_SEND, S_RESET_WAIT, S_RESET_DELAY,
		S_INIT_CFG, S_INIT_SEND_CMD, S_INIT_SEND_DATA, S_INIT_WAIT,
		S_READ_POLL, S_READ_CMD, S_READ_DATA, S_READ_STATUS, S_READ_STORE,
		S_DONE_CYCLE
	);
	signal state : main_state_t := S_RESET_SEND;

	-- reset: send 8 bytes of 0xFF
	signal reset_cnt : unsigned(2 downto 0) := (others => '0');

	-- delay counter for post-reset wait (4ms = 400000 clocks)
	signal delay_cnt : unsigned(18 downto 0) := (others => '0');
	constant DELAY_4MS : unsigned(18 downto 0) := to_unsigned(399999, 19);

	-- init config sequence
	type init_entry_t is record
		addr     : std_logic_vector(5 downto 0);
		data_len : unsigned(1 downto 0);
		data     : std_logic_vector(23 downto 0);
	end record;
	type init_array_t is array (0 to 11) of init_entry_t;

	-- AD7124 register addresses from no-OS driver
	-- Channel regs: enable + setup0 + AINP=N + AINM=17(AVSS)
	constant INIT_SEQ : init_array_t := (
		-- ADC_Control (0x01): DATA_STATUS=1, REF_EN=1, POWER=full, MODE=cont
		0  => (addr => "000001", data_len => "01", data => x"000DC0"),
		-- Config_0 (0x19): unipolar, buffers, internal ref, PGA=1
		1  => (addr => "011001", data_len => "01", data => x"000070"),
		-- Filter_0 (0x21): sinc3, FS=384 -> 50Hz ODR
		2  => (addr => "100001", data_len => "10", data => x"060180"),
		-- Channel 0: enable, setup0, AINP=0, AINM=17
		3  => (addr => "001001", data_len => "01", data => x"008011"),
		-- Channel 1: enable, setup0, AINP=1, AINM=17
		4  => (addr => "001010", data_len => "01", data => x"008031"),
		-- Channel 2: enable, setup0, AINP=2, AINM=17
		5  => (addr => "001011", data_len => "01", data => x"008051"),
		-- Channel 3: enable, setup0, AINP=3, AINM=17
		6  => (addr => "001100", data_len => "01", data => x"008071"),
		-- Channel 4: enable, setup0, AINP=4, AINM=17
		7  => (addr => "001101", data_len => "01", data => x"008091"),
		-- Channel 5: enable, setup0, AINP=5, AINM=17
		8  => (addr => "001110", data_len => "01", data => x"0080B1"),
		-- Channel 6: enable, setup0, AINP=6, AINM=17
		9  => (addr => "001111", data_len => "01", data => x"0080D1"),
		-- Channel 7: enable, setup0, AINP=7, AINM=17
		10 => (addr => "010000", data_len => "01", data => x"0080F1"),
		-- Channel 8: enable, setup0, AINP=8, AINM=17
		11 => (addr => "010001", data_len => "01", data => x"008111")
	);

	signal init_idx    : unsigned(3 downto 0) := (others => '0');
	signal byte_idx    : unsigned(1 downto 0) := (others => '0');
	signal cfg_done_r  : std_logic := '0';

	-- read cycle
	signal read_byte_cnt : unsigned(1 downto 0) := (others => '0');
	signal adc_raw       : std_logic_vector(23 downto 0) := (others => '0');
	signal status_byte   : std_logic_vector(7 downto 0) := (others => '0');
	signal channels_read : std_logic_vector(8 downto 0) := (others => '0');

	-- stored angles per channel
	type angle_array_t is array (0 to 8) of std_logic_vector(7 downto 0);
	signal angles : angle_array_t := (others => (others => '0'));

	signal cs_held_low : std_logic := '1';

begin

	config_done <= cfg_done_r;

	-- CS control: held low during read polling, otherwise controlled per-transaction
	spi_cs_n <= '0' when cs_held_low = '0' else '1';

	process(clk)
		variable adc_val    : unsigned(23 downto 0);
		variable angle_calc : unsigned(31 downto 0);
		variable ch_num     : integer range 0 to 15;
	begin
		if rising_edge(clk) then
			spi_start  <= '0';
			meas_valid <= '0';

			if rst = '1' then
				state        <= S_RESET_SEND;
				reset_cnt    <= (others => '0');
				cfg_done_r   <= '0';
				cs_held_low  <= '1';
				channels_read <= (others => '0');
			else
				case state is

					-- send 8 bytes of 0xFF to reset AD7124
					when S_RESET_SEND =>
						cs_held_low <= '0';
						if spi_busy = '0' then
							spi_tx_data <= x"FF";
							spi_start   <= '1';
							state       <= S_RESET_WAIT;
						end if;

					when S_RESET_WAIT =>
						if spi_done = '1' then
							if reset_cnt = 7 then
								state     <= S_RESET_DELAY;
								delay_cnt <= (others => '0');
								cs_held_low <= '1';
							else
								reset_cnt <= reset_cnt + 1;
								state     <= S_RESET_SEND;
							end if;
						end if;

					-- wait 4ms after reset
					when S_RESET_DELAY =>
						if delay_cnt = DELAY_4MS then
							state    <= S_INIT_CFG;
							init_idx <= (others => '0');
						else
							delay_cnt <= delay_cnt + 1;
						end if;

					-- init config: send command byte then data bytes
					when S_INIT_CFG =>
						cs_held_low <= '0';
						if spi_busy = '0' then
							-- command byte: WR=0, addr
							spi_tx_data <= "00" & INIT_SEQ(to_integer(init_idx)).addr;
							spi_start   <= '1';
							byte_idx    <= INIT_SEQ(to_integer(init_idx)).data_len;
							state       <= S_INIT_SEND_CMD;
						end if;

					when S_INIT_SEND_CMD =>
						if spi_done = '1' then
							state <= S_INIT_SEND_DATA;
						end if;

					when S_INIT_SEND_DATA =>
						if spi_busy = '0' then
							-- send data MSB first
							case byte_idx is
								when "10" =>
									spi_tx_data <= INIT_SEQ(to_integer(init_idx)).data(23 downto 16);
								when "01" =>
									spi_tx_data <= INIT_SEQ(to_integer(init_idx)).data(15 downto 8);
								when others =>
									spi_tx_data <= INIT_SEQ(to_integer(init_idx)).data(7 downto 0);
							end case;
							spi_start <= '1';
							state     <= S_INIT_WAIT;
						end if;

					when S_INIT_WAIT =>
						if spi_done = '1' then
							if byte_idx = 0 then
								cs_held_low <= '1';
								if init_idx = 11 then
									cfg_done_r <= '1';
									state      <= S_READ_POLL;
									cs_held_low <= '0';
									channels_read <= (others => '0');
								else
									init_idx <= init_idx + 1;
									state    <= S_INIT_CFG;
								end if;
							else
								byte_idx <= byte_idx - 1;
								state    <= S_INIT_SEND_DATA;
							end if;
						end if;

					-- poll DOUT/RDY (MISO goes low when data ready, CS held low)
					when S_READ_POLL =>
						cs_held_low <= '0';
						-- in mode 3 with DATA_STATUS, we just send the read command
						-- and clock in data+status. The AD7124 will hold DOUT/RDY low
						-- when ready. For simplicity, just continuously read.
						if spi_busy = '0' then
							-- send read data register command (0x42 = RD | DATA_REG)
							spi_tx_data <= x"42";
							spi_start   <= '1';
							read_byte_cnt <= (others => '0');
							state         <= S_READ_CMD;
						end if;

					when S_READ_CMD =>
						if spi_done = '1' then
							state <= S_READ_DATA;
						end if;

					-- read 3 data bytes
					when S_READ_DATA =>
						if spi_busy = '0' then
							spi_tx_data <= x"00";
							spi_start   <= '1';
							state       <= S_READ_STATUS;
						end if;

					when S_READ_STATUS =>
						if spi_done = '1' then
							case read_byte_cnt is
								when "00" =>
									adc_raw(23 downto 16) <= spi_rx_data;
								when "01" =>
									adc_raw(15 downto 8) <= spi_rx_data;
								when "10" =>
									adc_raw(7 downto 0) <= spi_rx_data;
								when others =>
									status_byte <= spi_rx_data;
							end case;

							if read_byte_cnt = 3 then
								state <= S_READ_STORE;
							else
								read_byte_cnt <= read_byte_cnt + 1;
								state         <= S_READ_DATA;
							end if;
						end if;

					-- convert raw ADC to angle and store
					when S_READ_STORE =>
						adc_val    := unsigned(adc_raw);
						-- angle = (adc_raw * 180) >> 24
						-- this maps full-scale 0xFFFFFF -> 180
						angle_calc := resize(adc_val * to_unsigned(180, 8), 32);
						ch_num     := to_integer(unsigned(status_byte(3 downto 0)));

						if ch_num < 9 then
							angles(ch_num) <= std_logic_vector(angle_calc(31 downto 24));
							channels_read(ch_num) <= '1';
						end if;

						if channels_read = "111111111" then
							state <= S_DONE_CYCLE;
						else
							state <= S_READ_POLL;
						end if;

					when S_DONE_CYCLE =>
						-- output all 9 angles
						meas_angles(71 downto 64) <= angles(0);
						meas_angles(63 downto 56) <= angles(1);
						meas_angles(55 downto 48) <= angles(2);
						meas_angles(47 downto 40) <= angles(3);
						meas_angles(39 downto 32) <= angles(4);
						meas_angles(31 downto 24) <= angles(5);
						meas_angles(23 downto 16) <= angles(6);
						meas_angles(15 downto  8) <= angles(7);
						meas_angles( 7 downto  0) <= angles(8);
						meas_valid    <= '1';
						channels_read <= (others => '0');
						state         <= S_READ_POLL;

				end case;
			end if;
		end if;
	end process;

end architecture;
