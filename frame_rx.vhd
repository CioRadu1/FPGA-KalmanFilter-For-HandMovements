library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity frame_rx is
	port (
		clk          : in  std_logic;
		rst          : in  std_logic;
		rx_data      : in  std_logic_vector(7 downto 0);
		rx_valid     : in  std_logic;
		frame_dir    : out std_logic_vector(7 downto 0);
		frame_angles : out std_logic_vector(63 downto 0);
		frame_valid  : out std_logic;
		checksum_err : out std_logic
	);
end entity;

architecture rtl of frame_rx is
	type state_t is (S_WAIT_DIR, S_RECV_DATA, S_RECV_CHKSUM);
	signal state      : state_t := S_WAIT_DIR;
	signal byte_count : unsigned(2 downto 0) := (others => '0');
	signal dir_reg    : std_logic_vector(7 downto 0) := (others => '0');
	signal angles_reg : std_logic_vector(63 downto 0) := (others => '0');
	signal xor_acc    : std_logic_vector(7 downto 0) := (others => '0');

	constant TIMEOUT_MAX : unsigned(16 downto 0) := to_unsigned(99999, 17);
	signal timeout_cnt   : unsigned(16 downto 0) := (others => '0');
begin

	process(clk)
	begin
		if rising_edge(clk) then
			frame_valid  <= '0';
			checksum_err <= '0';

			if rst = '1' then
				state       <= S_WAIT_DIR;
				byte_count  <= (others => '0');
				timeout_cnt <= (others => '0');
			else
				if state /= S_WAIT_DIR then
					if timeout_cnt = TIMEOUT_MAX then
						state       <= S_WAIT_DIR;
						timeout_cnt <= (others => '0');
					else
						timeout_cnt <= timeout_cnt + 1;
					end if;
				end if;

				case state is
					when S_WAIT_DIR =>
						if rx_valid = '1' then
							if rx_data = x"FF" or rx_data = x"FE" then
								dir_reg     <= rx_data;
								xor_acc     <= rx_data;
								byte_count  <= (others => '0');
								timeout_cnt <= (others => '0');
								state       <= S_RECV_DATA;
							end if;
						end if;

					when S_RECV_DATA =>
						if rx_valid = '1' then
							timeout_cnt <= (others => '0');
							angles_reg(63 - to_integer(byte_count)*8 downto
							           56 - to_integer(byte_count)*8) <= rx_data;
							xor_acc <= xor_acc xor rx_data;
							if byte_count = 7 then
								state <= S_RECV_CHKSUM;
							else
								byte_count <= byte_count + 1;
							end if;
						end if;

					when S_RECV_CHKSUM =>
						if rx_valid = '1' then
							if (xor_acc xor rx_data) = x"00" then
								frame_valid  <= '1';
								frame_dir    <= dir_reg;
								frame_angles <= angles_reg;
							else
								checksum_err <= '1';
							end if;
							state <= S_WAIT_DIR;
						end if;
				end case;
			end if;
		end if;
	end process;

end architecture;
