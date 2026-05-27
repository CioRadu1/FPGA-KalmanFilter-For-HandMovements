library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity mailbox is
	port (
		clk       : in  std_logic;
		rst       : in  std_logic;
		-- write side (from frame_rx)
		wr_angles : in  std_logic_vector(71 downto 0);
		wr_valid  : in  std_logic;
		-- read side (servo cycle)
		tick_20ms : in  std_logic;
		rd_angles : out std_logic_vector(71 downto 0);
		rd_new    : out std_logic
	);
end entity;

architecture rtl of mailbox is
	signal buf_write     : std_logic_vector(71 downto 0) := (others => '0');
	signal buf_read      : std_logic_vector(71 downto 0) := (others => '0');
	signal new_pending   : std_logic := '0';
begin

	process(clk)
	begin
		if rising_edge(clk) then
			rd_new <= '0';

			if rst = '1' then
				buf_write   <= (others => '0');
				buf_read    <= (others => '0');
				new_pending <= '0';
			else
				if wr_valid = '1' then
					buf_write   <= wr_angles;
					new_pending <= '1';
				end if;

				if tick_20ms = '1' then
					if new_pending = '1' then
						buf_read    <= buf_write;
						new_pending <= '0';
						rd_new      <= '1';
					end if;
				end if;
			end if;
		end if;
	end process;

	rd_angles <= buf_read;

end architecture;
