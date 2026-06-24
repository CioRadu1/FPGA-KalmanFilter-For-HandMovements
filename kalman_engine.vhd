library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity kalman_engine is
	port (
		clk           : in  std_logic;
		rst           : in  std_logic;
		tick_20ms     : in  std_logic;
		target_angles : in  std_logic_vector(63 downto 0);
		output_angles : out std_logic_vector(63 downto 0);
		output_valid  : out std_logic;
		busy          : out std_logic
	);
end entity;

architecture rtl of kalman_engine is

	constant ONE_Q16       : signed(31 downto 0) := to_signed(65536, 32);
	constant DT_Q16        : signed(31 downto 0) := to_signed(1311, 32);
	constant Q00           : signed(31 downto 0) := to_signed(655, 32);
	constant Q11           : signed(31 downto 0) := to_signed(6554, 32);
	constant R_NOISE       : signed(31 downto 0) := to_signed(65536, 32);
	constant P_INIT        : signed(31 downto 0) := to_signed(655360, 32);
	constant BYPASS_THRESH : signed(31 downto 0) := to_signed(1966080, 32);
	constant MAX_VEL       : signed(31 downto 0) := to_signed(65536000, 32);
	constant MIN_X0        : signed(31 downto 0) := to_signed(0, 32);
	constant MAX_X0        : signed(31 downto 0) := to_signed(11796480, 32);

	-- state stored as 5 separate per-channel RAMs (8 entries each).
	-- Splitting avoids a single 40-deep RAM that won't map to BRAM and
	-- forces a huge read/write mux (timing killer).
	type ch_ram_t is array (0 to 7) of signed(31 downto 0);
	signal x0_ram  : ch_ram_t := (others => (others => '0'));
	signal x1_ram  : ch_ram_t := (others => (others => '0'));
	signal p00_ram : ch_ram_t := (others => (others => '0'));
	signal p01_ram : ch_ram_t := (others => (others => '0'));
	signal p11_ram : ch_ram_t := (others => (others => '0'));

	signal initialized : std_logic := '0';
	signal first_run   : std_logic := '1';
	signal init_addr   : unsigned(3 downto 0) := (others => '0');

	type fsm_t is (
		S_IDLE, S_INIT_RAM, S_LOAD,
		S_PREDICT_X,
		S_PREDICT_DT, S_PREDICT_P00A, S_PREDICT_P00B, S_PREDICT_P00C,
		S_UPDATE_S,
		S_DIV_K0_START, S_DIV_K0_WAIT,
		S_DIV_K1_START, S_DIV_K1_WAIT,
		S_UPDATE_X0A, S_UPDATE_X0B, S_CLAMP_X0,
		S_UPDATE_X1A, S_UPDATE_X1B, S_CLAMP_X1,
		S_UPDATE_P00, S_CLAMP_P00,
		S_UPDATE_P01,
		S_UPDATE_P11A, S_UPDATE_P11B, S_CLAMP_P11,
		S_STORE, S_NEXT_CH, S_OUTPUT
	);
	signal fsm : fsm_t := S_INIT_RAM;

	signal channel : unsigned(2 downto 0) := (others => '0');

	signal x0, x1           : signed(31 downto 0) := (others => '0');
	signal p00, p01, p11    : signed(31 downto 0) := (others => '0');
	signal x0_pred, x1_pred : signed(31 downto 0) := (others => '0');
	signal pp00, pp01, pp11 : signed(31 downto 0) := (others => '0');

	signal s_val   : signed(31 downto 0) := (others => '0');
	signal k0, k1  : signed(31 downto 0) := (others => '0');
	signal y_innov : signed(31 downto 0) := (others => '0');
	signal z_meas  : signed(31 downto 0) := (others => '0');

	signal dt_p01  : signed(31 downto 0) := (others => '0');
	signal dt_p11  : signed(31 downto 0) := (others => '0');
	signal tmp     : signed(31 downto 0) := (others => '0');
	signal mul_res : signed(31 downto 0) := (others => '0');

	signal out_reg : std_logic_vector(63 downto 0) := (others => '0');

	signal div_start    : std_logic := '0';
	signal div_dividend : signed(63 downto 0) := (others => '0');
	signal div_divisor  : signed(63 downto 0) := (others => '0');
	signal div_quotient : signed(31 downto 0);
	signal div_done     : std_logic;

	component divider is
		port (
			clk      : in  std_logic;
			start    : in  std_logic;
			dividend : in  signed(63 downto 0);
			divisor  : in  signed(63 downto 0);
			quotient : out signed(31 downto 0);
			done     : out std_logic
		);
	end component;

	function mul_q16(a : signed(31 downto 0); b : signed(31 downto 0))
		return signed is
		variable product : signed(63 downto 0);
	begin
		product := a * b;
		return product(47 downto 16);
	end function;

	function get_angle_q16(vec : std_logic_vector(63 downto 0);
	                       ch  : unsigned(2 downto 0))
		return signed is
		variable idx     : integer;
		variable ang     : unsigned(7 downto 0);
		variable ang_int : integer;
	begin
		idx := (7 - to_integer(ch)) * 8;
		ang := unsigned(vec(idx+7 downto idx));
		ang_int := to_integer(ang);
		if ang_int > 180 then
			ang_int := 180;
		end if;
		return to_signed(ang_int * 65536, 32);
	end function;

begin

	u_div : divider
		port map (
			clk      => clk,
			start    => div_start,
			dividend => div_dividend,
			divisor  => div_divisor,
			quotient => div_quotient,
			done     => div_done
		);

	process(clk)
		variable result_angle : integer;
	begin
		if rising_edge(clk) then
			output_valid <= '0';
			div_start    <= '0';

			if rst = '1' then
				fsm         <= S_INIT_RAM;
				channel     <= (others => '0');
				initialized <= '0';
				first_run   <= '1';
				init_addr   <= (others => '0');
			else
				case fsm is

					when S_INIT_RAM =>
						-- init all 8 channels: x0=x1=0, P00=P11=P_INIT, P01=0
						x0_ram(to_integer(init_addr(2 downto 0)))  <= (others => '0');
						x1_ram(to_integer(init_addr(2 downto 0)))  <= (others => '0');
						p00_ram(to_integer(init_addr(2 downto 0))) <= P_INIT;
						p01_ram(to_integer(init_addr(2 downto 0))) <= (others => '0');
						p11_ram(to_integer(init_addr(2 downto 0))) <= P_INIT;
						if init_addr = 7 then
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
						z_meas <= get_angle_q16(target_angles, channel);
						if first_run = '1' then
							x0  <= get_angle_q16(target_angles, channel);
							x1  <= (others => '0');
							p00 <= P_INIT;
							p01 <= (others => '0');
							p11 <= P_INIT;
							fsm <= S_STORE;
						else
							x0  <= x0_ram(to_integer(channel));
							x1  <= x1_ram(to_integer(channel));
							p00 <= p00_ram(to_integer(channel));
							p01 <= p01_ram(to_integer(channel));
							p11 <= p11_ram(to_integer(channel));
							fsm <= S_PREDICT_X;
						end if;

					-- x0_pred = x0 + dt*x1, x1_pred = x1
					when S_PREDICT_X =>
						x0_pred <= x0 + mul_q16(DT_Q16, x1);
						x1_pred <= x1;
						fsm     <= S_PREDICT_DT;

					-- pre-compute dt*p01 and dt*p11
					when S_PREDICT_DT =>
						dt_p01 <= mul_q16(DT_Q16, p01);
						dt_p11 <= mul_q16(DT_Q16, p11);
						fsm    <= S_PREDICT_P00A;

					-- pp00 = p00 + 2*dt*p01 + dt^2*p11 + Q00
					-- step A: pp00 = p00 + dt_p01 + dt_p01 (just adds, no multiply)
					when S_PREDICT_P00A =>
						pp00 <= p00 + dt_p01 + dt_p01;
						pp01 <= p01 + dt_p11;
						fsm  <= S_PREDICT_P00B;

					-- step B: tmp = dt*dt_p11 (multiply only, store result)
					when S_PREDICT_P00B =>
						tmp <= mul_q16(DT_Q16, dt_p11);
						fsm <= S_PREDICT_P00C;

					-- step C: pp00 += tmp + Q00 (additions only)
					when S_PREDICT_P00C =>
						pp00 <= pp00 + tmp + Q00;
						pp11 <= p11 + Q11;
						fsm  <= S_UPDATE_S;

					-- innovation and bypass check
					when S_UPDATE_S =>
						s_val   <= pp00 + R_NOISE;
						y_innov <= z_meas - x0_pred;
						if (z_meas - x0_pred) > BYPASS_THRESH or
						   (z_meas - x0_pred) < -BYPASS_THRESH then
							x0  <= z_meas;
							x1  <= (others => '0');
							p00 <= P_INIT;
							p01 <= (others => '0');
							p11 <= P_INIT;
							fsm <= S_STORE;
						else
							fsm <= S_DIV_K0_START;
						end if;

					-- K0 = pp00 / s_val (Q16.16 division)
					when S_DIV_K0_START =>
						div_dividend <= shift_left(resize(pp00, 64), 16);
						div_divisor  <= resize(s_val, 64);
						div_start    <= '1';
						fsm          <= S_DIV_K0_WAIT;

					when S_DIV_K0_WAIT =>
						if div_done = '1' then
							k0  <= div_quotient;
							fsm <= S_DIV_K1_START;
						end if;

					-- K1 = pp01 / s_val
					when S_DIV_K1_START =>
						div_dividend <= shift_left(resize(pp01, 64), 16);
						div_divisor  <= resize(s_val, 64);
						div_start    <= '1';
						fsm          <= S_DIV_K1_WAIT;

					when S_DIV_K1_WAIT =>
						if div_done = '1' then
							k1  <= div_quotient;
							fsm <= S_UPDATE_X0A;
						end if;

					-- x0 = x0_pred + K0*y : split multiply and add
					when S_UPDATE_X0A =>
						mul_res <= mul_q16(k0, y_innov);
						fsm     <= S_UPDATE_X0B;

					when S_UPDATE_X0B =>
						x0  <= x0_pred + mul_res;
						fsm <= S_CLAMP_X0;

					-- clamp x0 to 0..180 (comparisons only, no multiply)
					when S_CLAMP_X0 =>
						if x0 < MIN_X0 then
							x0 <= MIN_X0;
						elsif x0 > MAX_X0 then
							x0 <= MAX_X0;
						end if;
						fsm <= S_UPDATE_X1A;

					-- x1 = x1_pred + K1*y : split multiply and add
					when S_UPDATE_X1A =>
						mul_res <= mul_q16(k1, y_innov);
						fsm     <= S_UPDATE_X1B;

					when S_UPDATE_X1B =>
						x1  <= x1_pred + mul_res;
						fsm <= S_CLAMP_X1;

					-- clamp x1 to +/- MAX_VEL
					when S_CLAMP_X1 =>
						if x1 > MAX_VEL then
							x1 <= MAX_VEL;
						elsif x1 < -MAX_VEL then
							x1 <= -MAX_VEL;
						end if;
						fsm <= S_UPDATE_P00;

					-- p00 = (1-K0)*pp00
					when S_UPDATE_P00 =>
						p00 <= mul_q16(ONE_Q16 - k0, pp00);
						fsm <= S_CLAMP_P00;

					when S_CLAMP_P00 =>
						if p00 < to_signed(1, 32) then
							p00 <= to_signed(1, 32);
						end if;
						fsm <= S_UPDATE_P01;

					-- p01 = (1-K0)*pp01
					when S_UPDATE_P01 =>
						p01 <= mul_q16(ONE_Q16 - k0, pp01);
						fsm <= S_UPDATE_P11A;

					-- p11 = -K1*pp01 + pp11 : split multiply and add
					when S_UPDATE_P11A =>
						mul_res <= mul_q16(-k1, pp01);
						fsm     <= S_UPDATE_P11B;

					when S_UPDATE_P11B =>
						p11 <= mul_res + pp11;
						fsm <= S_CLAMP_P11;

					when S_CLAMP_P11 =>
						if p11 < to_signed(1, 32) then
							p11 <= to_signed(1, 32);
						end if;
						fsm <= S_STORE;

					-- store results to RAM, convert to 8-bit angle
					when S_STORE =>
						x0_ram(to_integer(channel))  <= x0;
						x1_ram(to_integer(channel))  <= x1;
						p00_ram(to_integer(channel)) <= p00;
						p01_ram(to_integer(channel)) <= p01;
						p11_ram(to_integer(channel)) <= p11;

						result_angle := to_integer(shift_right(x0, 16));
						if result_angle < 0 then
							result_angle := 0;
						elsif result_angle > 180 then
							result_angle := 180;
						end if;

						out_reg((7 - to_integer(channel))*8 + 7 downto
						        (7 - to_integer(channel))*8) <=
							std_logic_vector(to_unsigned(result_angle, 8));

						fsm <= S_NEXT_CH;

					when S_NEXT_CH =>
						if channel = 7 then
							fsm <= S_OUTPUT;
						else
							channel <= channel + 1;
							fsm     <= S_LOAD;
						end if;

					when S_OUTPUT =>
						output_angles <= out_reg;
						output_valid  <= '1';
						first_run     <= '0';
						fsm           <= S_IDLE;

				end case;
			end if;
		end if;
	end process;

	busy <= '0' when fsm = S_IDLE else '1';

end architecture;
