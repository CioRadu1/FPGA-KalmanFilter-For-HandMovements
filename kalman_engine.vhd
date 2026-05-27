library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity kalman_engine is
	port (
		clk           : in  std_logic;
		rst           : in  std_logic;
		tick_20ms     : in  std_logic;
		target_angles : in  std_logic_vector(71 downto 0);
		meas_angles   : in  std_logic_vector(71 downto 0);
		meas_valid    : in  std_logic;
		output_angles : out std_logic_vector(71 downto 0);
		output_valid  : out std_logic;
		busy          : out std_logic
	);
end entity;

-- 2-state Kalman filter: x = [position, velocity]
-- Q16.16 fixed-point (32-bit signed)
-- time-multiplexed over 9 servo channels

architecture rtl of kalman_engine is

	constant ONE_Q16  : signed(31 downto 0) := to_signed(65536, 32);
	constant DT_Q16   : signed(31 downto 0) := to_signed(1311, 32);   -- 0.02s
	constant Q00      : signed(31 downto 0) := to_signed(655, 32);    -- 0.01 deg^2
	constant Q11      : signed(31 downto 0) := to_signed(6554, 32);   -- 0.1 (deg/s)^2
	constant R_NOISE  : signed(31 downto 0) := to_signed(65536, 32);  -- 1.0 deg^2
	constant P_INIT   : signed(31 downto 0) := to_signed(655360, 32); -- 10.0

	-- state RAM: 9 channels x 5 words (x0, x1, P00, P01, P11)
	type ram_t is array (0 to 44) of signed(31 downto 0);
	signal state_ram : ram_t := (others => (others => '0'));

	signal initialized : std_logic := '0';
	signal init_addr   : unsigned(5 downto 0) := (others => '0');

	type fsm_t is (
		S_IDLE, S_INIT_RAM,
		S_LOAD, S_PREDICT_X,
		S_PREDICT_P00, S_PREDICT_P01, S_PREDICT_P11,
		S_UPDATE_S, S_UPDATE_K0, S_UPDATE_K1,
		S_UPDATE_X, S_UPDATE_P,
		S_STORE, S_NEXT_CH, S_OUTPUT
	);
	signal fsm : fsm_t := S_INIT_RAM;

	signal channel : unsigned(3 downto 0) := (others => '0');

	signal x0, x1           : signed(31 downto 0) := (others => '0');
	signal p00, p01, p11    : signed(31 downto 0) := (others => '0');
	signal x0_pred, x1_pred : signed(31 downto 0) := (others => '0');
	signal pp00, pp01, pp11 : signed(31 downto 0) := (others => '0');

	signal s_val   : signed(31 downto 0) := (others => '0');
	signal k0, k1  : signed(31 downto 0) := (others => '0');
	signal y_innov : signed(31 downto 0) := (others => '0');
	signal z_meas  : signed(31 downto 0) := (others => '0');

	signal meas_reg : std_logic_vector(71 downto 0) := (others => '0');
	signal out_reg  : std_logic_vector(71 downto 0) := (others => '0');

	function mul_q16(a : signed(31 downto 0); b : signed(31 downto 0))
		return signed is
		variable product : signed(63 downto 0);
	begin
		product := a * b;
		return product(47 downto 16);
	end function;

	function ch_base(ch : unsigned(3 downto 0)) return integer is
	begin
		return to_integer(ch) * 5;
	end function;

	function get_angle_q16(vec : std_logic_vector(71 downto 0);
	                       ch  : unsigned(3 downto 0))
		return signed is
		variable idx : integer;
		variable ang : unsigned(7 downto 0);
	begin
		idx := (8 - to_integer(ch)) * 8;
		ang := unsigned(vec(idx+7 downto idx));
		return to_signed(to_integer(ang) * 65536, 32);
	end function;

begin

	process(clk)
	begin
		if rising_edge(clk) then
			if rst = '1' then
				meas_reg <= (others => '0');
			elsif meas_valid = '1' then
				meas_reg <= meas_angles;
			end if;
		end if;
	end process;

	process(clk)
		variable base         : integer;
		variable tmp64        : signed(63 downto 0);
		variable result_angle : integer;
	begin
		if rising_edge(clk) then
			output_valid <= '0';

			if rst = '1' then
				fsm         <= S_INIT_RAM;
				channel     <= (others => '0');
				initialized <= '0';
				init_addr   <= (others => '0');
			else
				case fsm is

					when S_INIT_RAM =>
						case to_integer(init_addr mod 5) is
							when 2     => state_ram(to_integer(init_addr)) <= P_INIT;
							when 4     => state_ram(to_integer(init_addr)) <= P_INIT;
							when others => state_ram(to_integer(init_addr)) <= (others => '0');
						end case;
						if init_addr = 44 then
							initialized <= '1';
							fsm         <= S_IDLE;
						else
							init_addr <= init_addr + 1;
						end if;

					when S_IDLE =>
						if tick_20ms = '1' and initialized = '1' then
							channel <= (others => '0');
							fsm     <= S_LOAD;
						end if;

					when S_LOAD =>
						base := ch_base(channel);
						x0  <= state_ram(base + 0);
						x1  <= state_ram(base + 1);
						p00 <= state_ram(base + 2);
						p01 <= state_ram(base + 3);
						p11 <= state_ram(base + 4);
						z_meas <= get_angle_q16(target_angles, channel);
						fsm <= S_PREDICT_X;

					when S_PREDICT_X =>
						x0_pred <= x0 + mul_q16(DT_Q16, x1);
						x1_pred <= x1;
						fsm <= S_PREDICT_P00;

					when S_PREDICT_P00 =>
						pp00 <= p00 + mul_q16(DT_Q16, p01) +
						        mul_q16(DT_Q16, p01) +
						        mul_q16(DT_Q16, mul_q16(DT_Q16, p11)) + Q00;
						fsm <= S_PREDICT_P01;

					when S_PREDICT_P01 =>
						pp01 <= p01 + mul_q16(DT_Q16, p11);
						fsm  <= S_PREDICT_P11;

					when S_PREDICT_P11 =>
						pp11 <= p11 + Q11;
						fsm  <= S_UPDATE_S;

					when S_UPDATE_S =>
						s_val   <= pp00 + R_NOISE;
						y_innov <= z_meas - x0_pred;
						fsm     <= S_UPDATE_K0;

					when S_UPDATE_K0 =>
						if s_val /= 0 then
							tmp64 := shift_left(resize(pp00, 64), 16);
							k0 <= resize(tmp64 / resize(s_val, 64), 32);
						else
							k0 <= ONE_Q16;
						end if;
						fsm <= S_UPDATE_K1;

					when S_UPDATE_K1 =>
						if s_val /= 0 then
							tmp64 := shift_left(resize(pp01, 64), 16);
							k1 <= resize(tmp64 / resize(s_val, 64), 32);
						else
							k1 <= (others => '0');
						end if;
						fsm <= S_UPDATE_X;

					when S_UPDATE_X =>
						x0 <= x0_pred + mul_q16(k0, y_innov);
						x1 <= x1_pred + mul_q16(k1, y_innov);
						fsm <= S_UPDATE_P;

					when S_UPDATE_P =>
						p00 <= mul_q16(ONE_Q16 - k0, pp00);
						p01 <= mul_q16(ONE_Q16 - k0, pp01);
						p11 <= mul_q16(-k1, pp01) + pp11;
						fsm <= S_STORE;

					when S_STORE =>
						base := ch_base(channel);
						state_ram(base + 0) <= x0;
						state_ram(base + 1) <= x1;
						state_ram(base + 2) <= p00;
						state_ram(base + 3) <= p01;
						state_ram(base + 4) <= p11;

						result_angle := to_integer(shift_right(x0, 16));
						if result_angle < 0 then
							result_angle := 0;
						elsif result_angle > 180 then
							result_angle := 180;
						end if;

						out_reg((8 - to_integer(channel))*8 + 7 downto
						        (8 - to_integer(channel))*8) <=
							std_logic_vector(to_unsigned(result_angle, 8));

						fsm <= S_NEXT_CH;

					when S_NEXT_CH =>
						if channel = 8 then
							fsm <= S_OUTPUT;
						else
							channel <= channel + 1;
							fsm     <= S_LOAD;
						end if;

					when S_OUTPUT =>
						output_angles <= out_reg;
						output_valid  <= '1';
						fsm           <= S_IDLE;

				end case;
			end if;
		end if;
	end process;

	busy <= '0' when fsm = S_IDLE else '1';

end architecture;
