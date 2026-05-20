library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity source_mux is
	port (
		demo_mode      : in  std_logic;
		mailbox_angles : in  std_logic_vector(63 downto 0);
		demo_angles    : in  std_logic_vector(63 downto 0);
		target_angles  : out std_logic_vector(63 downto 0)
	);
end entity;

architecture rtl of source_mux is
begin
	target_angles <= demo_angles when demo_mode = '1' else mailbox_angles;
end architecture;
