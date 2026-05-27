library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity frame_tx is
	port (
		clk        : in  std_logic;
		rst        : in  std_logic;
		-- frame to send
		angles_in  : in  std_logic_vector(71 downto 0);
		send_start : in  std_logic;
		-- uart_tx interface
		tx_data    : out std_logic_vector(7 downto 0);
		tx_start   : out std_logic;
		tx_busy    : in  std_logic;
		-- status
		frame_busy : out std_logic
	);
end entity;

architecture rtl of frame_tx is
	type state_t is (S_IDLE, S_SEND_DIR, S_WAIT_DIR, S_SEND_DATA, S_WAIT_DATA,
	                 S_SEND_CHKSUM, S_WAIT_CHKSUM);
	signal state      : state_t := S_IDLE;
	signal byte_idx   : unsigned(3 downto 0) := (others => '0');
	signal angles_reg : std_logic_vector(71 downto 0) := (others => '0');
	signal xor_acc    : std_logic_vector(7 downto 0) := (others => '0');
begin

	process(clk)
	begin
		if rising_edge(clk) then
			tx_start <= '0';

			if rst = '1' then
				state <= S_IDLE;
			else
				case state is
					when S_IDLE =>
						if send_start = '1' then
							angles_reg <= angles_in;
							state      <= S_SEND_DIR;
						end if;

					when S_SEND_DIR =>
						if tx_busy = '0' then
							tx_data  <= x"FE";
							tx_start <= '1';
							xor_acc  <= x"FE";
							state    <= S_WAIT_DIR;
						end if;

					when S_WAIT_DIR =>
						if tx_busy = '0' then
							byte_idx <= (others => '0');
							state    <= S_SEND_DATA;
						end if;

					when S_SEND_DATA =>
						if tx_busy = '0' then
							tx_data  <= angles_reg(71 - to_integer(byte_idx)*8 downto
							                       64 - to_integer(byte_idx)*8);
							tx_start <= '1';
							xor_acc  <= xor_acc xor
							            angles_reg(71 - to_integer(byte_idx)*8 downto
							                       64 - to_integer(byte_idx)*8);
							state    <= S_WAIT_DATA;
						end if;

					when S_WAIT_DATA =>
						if tx_busy = '0' then
							if byte_idx = 8 then
								state <= S_SEND_CHKSUM;
							else
								byte_idx <= byte_idx + 1;
								state    <= S_SEND_DATA;
							end if;
						end if;

					when S_SEND_CHKSUM =>
						if tx_busy = '0' then
							tx_data  <= xor_acc;
							tx_start <= '1';
							state    <= S_WAIT_CHKSUM;
						end if;

					when S_WAIT_CHKSUM =>
						if tx_busy = '0' then
							state <= S_IDLE;
						end if;
				end case;
			end if;
		end if;
	end process;

	frame_busy <= '0' when state = S_IDLE else '1';

end architecture;
