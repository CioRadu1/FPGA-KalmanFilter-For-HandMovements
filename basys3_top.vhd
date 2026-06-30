library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity basys3_top is
	port (
		clk      : in  std_logic;
		sw0      : in  std_logic;
		led      : out std_logic_vector(15 downto 0);
		btnC     : in  std_logic;
		RsRx     : in  std_logic;
		RsTx     : out std_logic;
		-- Pmod JA: PWM servos 0-3 (pinky, ring, middle, index)
		JA0      : out std_logic;
		JA1      : out std_logic;
		JA2      : out std_logic;
		JA3      : out std_logic;
		-- Pmod JB: PWM servos 4-7 (thumb-palm, thumb-tension, wrist, elbow)
		JB0      : out std_logic;
		JB1      : out std_logic;
		JB2      : out std_logic;
		JB3      : out std_logic;
		-- Pmod JC: SPI to AD7124 through MAX14850
		JC0_cs_n : out std_logic;
		JC1_mosi : out std_logic;
		JC2_miso : in  std_logic;
		JC3_sclk : out std_logic;
		-- Pmod JA upper half (unused, active drive low)
		JA4      : out std_logic;
		JA5      : out std_logic;
		JA6      : out std_logic;
		JA7      : out std_logic;
		-- Pmod JB upper half (unused)
		JB4      : out std_logic;
		JB5      : out std_logic;
		JB6      : out std_logic;
		JB7      : out std_logic;
		-- Pmod JC upper half (unused)
		JC4      : out std_logic;
		JC5      : out std_logic;
		JC6      : out std_logic;
		JC7      : out std_logic;
		-- 7-segment display (active-low, driven off)
		seg      : out std_logic_vector(6 downto 0);
		dp       : out std_logic;
		an       : out std_logic_vector(3 downto 0);
		-- VGA (driven low)
		vgaRed   : out std_logic_vector(3 downto 0);
		vgaGreen : out std_logic_vector(3 downto 0);
		vgaBlue  : out std_logic_vector(3 downto 0);
		Hsync    : out std_logic;
		Vsync    : out std_logic;
		-- PS/2 (high-Z)
		PS2Clk   : inout std_logic;
		PS2Data  : inout std_logic;
		-- JXADC (high-Z)
		JXADC    : inout std_logic_vector(7 downto 0);
		-- unused buttons
		btnU     : in std_logic;
		btnD     : in std_logic;
		btnL     : in std_logic;
		btnR     : in std_logic;
		-- unused switches
		sw       : in std_logic_vector(15 downto 1)
	);
end basys3_top;

architecture Behavioral of basys3_top is

	signal rst : std_logic;

	signal tick_20ms : std_logic;
	signal tick_cnt  : std_logic_vector(20 downto 0);

	signal uart_rx_data  : std_logic_vector(7 downto 0);
	signal uart_rx_valid : std_logic;
	signal uart_tx_busy  : std_logic;

	signal frame_dir        : std_logic_vector(7 downto 0);
	signal frame_angles     : std_logic_vector(63 downto 0);
	signal frame_valid      : std_logic;
	signal frame_chksum_err : std_logic;

	signal frame_tx_busy : std_logic;
	signal ftx_data      : std_logic_vector(7 downto 0);
	signal ftx_start     : std_logic;

	signal mb_angles   : std_logic_vector(63 downto 0);
	signal mb_new      : std_logic;
	signal mb_wr_valid : std_logic;

	signal demo_angles   : std_logic_vector(63 downto 0);
	signal target_angles : std_logic_vector(63 downto 0);

	signal spi_start_sig  : std_logic;
	signal spi_tx_data_s  : std_logic_vector(7 downto 0);
	signal spi_rx_data    : std_logic_vector(7 downto 0);
	signal spi_busy_sig   : std_logic;
	signal spi_done_sig   : std_logic;

	signal meas_angles  : std_logic_vector(63 downto 0);
	signal meas_valid   : std_logic;
	signal adc_cfg_done : std_logic;

	signal kalman_out   : std_logic_vector(63 downto 0);
	signal kalman_valid : std_logic;
	signal kalman_busy  : std_logic;

	signal pwm_out       : std_logic_vector(7 downto 0);
	signal send_readback : std_logic;

	-- unghiurile efectiv trimise la PWM: identice cu kalman_out, exceptand
	-- canalul 4 (police-palma), al carui servomotor are sens invers montat.
	-- Acest servo trebuie sa mearga in sens invers, de la 90 la 0 grade:
	-- comanda 0 -> iesire 90, comanda 180 -> iesire 0.
	signal pwm_angles : std_logic_vector(63 downto 0);

begin

	-- remapare canal 4: inversare + scalare, iesire = 90 - (comanda / 2)
	-- canalele 0-3 ocupa bitii 63..32, canalul 4 bitii 31..24, 5-7 bitii 23..0
	pwm_angles(63 downto 32) <= kalman_out(63 downto 32);
	pwm_angles(31 downto 24) <=
		std_logic_vector(to_unsigned(90, 8) -
		                 shift_right(unsigned(kalman_out(31 downto 24)), 1));
	pwm_angles(23 downto 0)  <= kalman_out(23 downto 0);

	rst <= btnC;

	-- unused outputs driven to safe states
	JA4      <= '0';
	JA5      <= '0';
	JA6      <= '0';
	JA7      <= '0';
	JB4      <= '0';
	JB5      <= '0';
	JB6      <= '0';
	JB7      <= '0';
	JC4      <= '0';
	JC5      <= '0';
	JC6      <= '0';
	JC7      <= '0';
	seg      <= (others => '1');
	dp       <= '1';
	an       <= (others => '1');
	vgaRed   <= (others => '0');
	vgaGreen <= (others => '0');
	vgaBlue  <= (others => '0');
	Hsync    <= '0';
	Vsync    <= '0';
	PS2Clk   <= 'Z';
	PS2Data  <= 'Z';
	JXADC    <= (others => 'Z');

	-- PWM to Pmod JA
	JA0 <= pwm_out(0);
	JA1 <= pwm_out(1);
	JA2 <= pwm_out(2);
	JA3 <= pwm_out(3);

	-- PWM to Pmod JB
	JB0 <= pwm_out(4);
	JB1 <= pwm_out(5);
	JB2 <= pwm_out(6);
	JB3 <= pwm_out(7);

	-- LED status
	led(0)  <= adc_cfg_done;
	led(1)  <= meas_valid;
	led(2)  <= frame_valid;
	led(3)  <= frame_chksum_err;
	led(4)  <= kalman_busy;
	led(5)  <= sw0;
	led(6)  <= uart_tx_busy;
	led(7)  <= spi_busy_sig;
	led(15 downto 8) <= kalman_out(63 downto 56);

	------------------------------------------------------------------------
	-- INSTANTIATIONS
	------------------------------------------------------------------------

	u_tick : entity work.system_tick_gen
		port map (
			clk       => clk,
			rst       => rst,
			tick_20ms => tick_20ms,
			tick_cnt  => tick_cnt
		);

	u_uart_rx : entity work.uart_rx
		port map (
			clk       => clk,
			rst       => rst,
			rx_serial => RsRx,
			rx_data   => uart_rx_data,
			rx_valid  => uart_rx_valid
		);

	u_uart_tx : entity work.uart_tx
		port map (
			clk       => clk,
			rst       => rst,
			tx_data   => ftx_data,
			tx_start  => ftx_start,
			tx_serial => RsTx,
			tx_busy   => uart_tx_busy
		);

	u_frame_rx : entity work.frame_rx
		port map (
			clk          => clk,
			rst          => rst,
			rx_data      => uart_rx_data,
			rx_valid     => uart_rx_valid,
			frame_dir    => frame_dir,
			frame_angles => frame_angles,
			frame_valid  => frame_valid,
			checksum_err => frame_chksum_err
		);

	u_frame_tx : entity work.frame_tx
		port map (
			clk        => clk,
			rst        => rst,
			angles_in  => kalman_out,
			send_start => send_readback,
			tx_data    => ftx_data,
			tx_start   => ftx_start,
			tx_busy    => uart_tx_busy,
			frame_busy => frame_tx_busy
		);

	mb_wr_valid <= frame_valid and (not frame_chksum_err);

	u_mailbox : entity work.mailbox
		port map (
			clk       => clk,
			rst       => rst,
			wr_angles => frame_angles,
			wr_valid  => mb_wr_valid,
			tick_20ms => tick_20ms,
			rd_angles => mb_angles,
			rd_new    => mb_new
		);

	u_demo : entity work.demo_rom
		port map (
			clk        => clk,
			rst        => rst,
			enable     => sw0,
			tick_20ms  => tick_20ms,
			angles_out => demo_angles
		);

	u_mux : entity work.source_mux
		port map (
			demo_mode      => sw0,
			mailbox_angles => mb_angles,
			demo_angles    => demo_angles,
			target_angles  => target_angles
		);

	u_spi : entity work.spi_master
		port map (
			clk      => clk,
			rst      => rst,
			start    => spi_start_sig,
			tx_data  => spi_tx_data_s,
			rx_data  => spi_rx_data,
			busy     => spi_busy_sig,
			done     => spi_done_sig,
			sclk_out => JC3_sclk,
			mosi     => JC1_mosi,
			miso     => JC2_miso,
			cs_n     => open
		);

	u_adc : entity work.ad7124_ctrl
		port map (
			clk         => clk,
			rst         => rst,
			tick_20ms   => tick_20ms,
			spi_start   => spi_start_sig,
			spi_tx_data => spi_tx_data_s,
			spi_rx_data => spi_rx_data,
			spi_busy    => spi_busy_sig,
			spi_done    => spi_done_sig,
			spi_cs_n    => JC0_cs_n,
			meas_angles => meas_angles,
			meas_valid  => meas_valid,
			config_done => adc_cfg_done
		);

	u_kalman : entity work.kalman_engine
		port map (
			clk           => clk,
			rst           => rst,
			tick_20ms     => tick_20ms,
			target_angles => target_angles,
			output_angles => kalman_out,
			output_valid  => kalman_valid,
			busy          => kalman_busy
		);

	gen_pwm : for i in 0 to 7 generate
		u_pwm : entity work.pwm_gen
			port map (
				clk     => clk,
				rst     => rst,
				angle   => pwm_angles((7-i)*8+7 downto (7-i)*8),
				pwm_out => pwm_out(i)
			);
	end generate;

	process(clk)
	begin
		if rising_edge(clk) then
			send_readback <= '0';
			if frame_valid = '1' and frame_dir = x"FE" then
				send_readback <= '1';
			end if;
		end if;
	end process;

end Behavioral;
