## Clock (100 MHz)
set_property -dict { PACKAGE_PIN W5   IOSTANDARD LVCMOS33 } [get_ports clk]
create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5} [get_ports clk]

## Switch 0 (demo mode)
set_property -dict { PACKAGE_PIN V17  IOSTANDARD LVCMOS33 } [get_ports sw0]

## Unused switches
set_property -dict { PACKAGE_PIN V16  IOSTANDARD LVCMOS33 } [get_ports {sw[1]}]
set_property -dict { PACKAGE_PIN W16  IOSTANDARD LVCMOS33 } [get_ports {sw[2]}]
set_property -dict { PACKAGE_PIN W17  IOSTANDARD LVCMOS33 } [get_ports {sw[3]}]
set_property -dict { PACKAGE_PIN W15  IOSTANDARD LVCMOS33 } [get_ports {sw[4]}]
set_property -dict { PACKAGE_PIN V15  IOSTANDARD LVCMOS33 } [get_ports {sw[5]}]
set_property -dict { PACKAGE_PIN W14  IOSTANDARD LVCMOS33 } [get_ports {sw[6]}]
set_property -dict { PACKAGE_PIN W13  IOSTANDARD LVCMOS33 } [get_ports {sw[7]}]
set_property -dict { PACKAGE_PIN V2   IOSTANDARD LVCMOS33 } [get_ports {sw[8]}]
set_property -dict { PACKAGE_PIN T3   IOSTANDARD LVCMOS33 } [get_ports {sw[9]}]
set_property -dict { PACKAGE_PIN T2   IOSTANDARD LVCMOS33 } [get_ports {sw[10]}]
set_property -dict { PACKAGE_PIN R3   IOSTANDARD LVCMOS33 } [get_ports {sw[11]}]
set_property -dict { PACKAGE_PIN W2   IOSTANDARD LVCMOS33 } [get_ports {sw[12]}]
set_property -dict { PACKAGE_PIN U1   IOSTANDARD LVCMOS33 } [get_ports {sw[13]}]
set_property -dict { PACKAGE_PIN T1   IOSTANDARD LVCMOS33 } [get_ports {sw[14]}]
set_property -dict { PACKAGE_PIN R2   IOSTANDARD LVCMOS33 } [get_ports {sw[15]}]

## Buttons
set_property -dict { PACKAGE_PIN U18  IOSTANDARD LVCMOS33 } [get_ports btnC]
set_property -dict { PACKAGE_PIN T18  IOSTANDARD LVCMOS33 } [get_ports btnU]
set_property -dict { PACKAGE_PIN U17  IOSTANDARD LVCMOS33 } [get_ports btnD]
set_property -dict { PACKAGE_PIN W19  IOSTANDARD LVCMOS33 } [get_ports btnL]
set_property -dict { PACKAGE_PIN T17  IOSTANDARD LVCMOS33 } [get_ports btnR]

## LEDs
set_property -dict { PACKAGE_PIN U16  IOSTANDARD LVCMOS33 } [get_ports {led[0]}]
set_property -dict { PACKAGE_PIN E19  IOSTANDARD LVCMOS33 } [get_ports {led[1]}]
set_property -dict { PACKAGE_PIN U19  IOSTANDARD LVCMOS33 } [get_ports {led[2]}]
set_property -dict { PACKAGE_PIN V19  IOSTANDARD LVCMOS33 } [get_ports {led[3]}]
set_property -dict { PACKAGE_PIN W18  IOSTANDARD LVCMOS33 } [get_ports {led[4]}]
set_property -dict { PACKAGE_PIN U15  IOSTANDARD LVCMOS33 } [get_ports {led[5]}]
set_property -dict { PACKAGE_PIN U14  IOSTANDARD LVCMOS33 } [get_ports {led[6]}]
set_property -dict { PACKAGE_PIN V14  IOSTANDARD LVCMOS33 } [get_ports {led[7]}]
set_property -dict { PACKAGE_PIN V13  IOSTANDARD LVCMOS33 } [get_ports {led[8]}]
set_property -dict { PACKAGE_PIN V3   IOSTANDARD LVCMOS33 } [get_ports {led[9]}]
set_property -dict { PACKAGE_PIN W3   IOSTANDARD LVCMOS33 } [get_ports {led[10]}]
set_property -dict { PACKAGE_PIN U3   IOSTANDARD LVCMOS33 } [get_ports {led[11]}]
set_property -dict { PACKAGE_PIN P3   IOSTANDARD LVCMOS33 } [get_ports {led[12]}]
set_property -dict { PACKAGE_PIN N3   IOSTANDARD LVCMOS33 } [get_ports {led[13]}]
set_property -dict { PACKAGE_PIN P1   IOSTANDARD LVCMOS33 } [get_ports {led[14]}]
set_property -dict { PACKAGE_PIN L1   IOSTANDARD LVCMOS33 } [get_ports {led[15]}]

## 7-Segment Display
set_property -dict { PACKAGE_PIN W7   IOSTANDARD LVCMOS33 } [get_ports {seg[0]}]
set_property -dict { PACKAGE_PIN W6   IOSTANDARD LVCMOS33 } [get_ports {seg[1]}]
set_property -dict { PACKAGE_PIN U8   IOSTANDARD LVCMOS33 } [get_ports {seg[2]}]
set_property -dict { PACKAGE_PIN V8   IOSTANDARD LVCMOS33 } [get_ports {seg[3]}]
set_property -dict { PACKAGE_PIN U5   IOSTANDARD LVCMOS33 } [get_ports {seg[4]}]
set_property -dict { PACKAGE_PIN V5   IOSTANDARD LVCMOS33 } [get_ports {seg[5]}]
set_property -dict { PACKAGE_PIN U7   IOSTANDARD LVCMOS33 } [get_ports {seg[6]}]
set_property -dict { PACKAGE_PIN V7   IOSTANDARD LVCMOS33 } [get_ports dp]
set_property -dict { PACKAGE_PIN U2   IOSTANDARD LVCMOS33 } [get_ports {an[0]}]
set_property -dict { PACKAGE_PIN U4   IOSTANDARD LVCMOS33 } [get_ports {an[1]}]
set_property -dict { PACKAGE_PIN V4   IOSTANDARD LVCMOS33 } [get_ports {an[2]}]
set_property -dict { PACKAGE_PIN W4   IOSTANDARD LVCMOS33 } [get_ports {an[3]}]

## USB-UART
set_property -dict { PACKAGE_PIN B18  IOSTANDARD LVCMOS33 } [get_ports RsRx]
set_property -dict { PACKAGE_PIN A18  IOSTANDARD LVCMOS33 } [get_ports RsTx]

## VGA
set_property -dict { PACKAGE_PIN G19  IOSTANDARD LVCMOS33 } [get_ports {vgaRed[0]}]
set_property -dict { PACKAGE_PIN H19  IOSTANDARD LVCMOS33 } [get_ports {vgaRed[1]}]
set_property -dict { PACKAGE_PIN J19  IOSTANDARD LVCMOS33 } [get_ports {vgaRed[2]}]
set_property -dict { PACKAGE_PIN N19  IOSTANDARD LVCMOS33 } [get_ports {vgaRed[3]}]
set_property -dict { PACKAGE_PIN J17  IOSTANDARD LVCMOS33 } [get_ports {vgaGreen[0]}]
set_property -dict { PACKAGE_PIN H17  IOSTANDARD LVCMOS33 } [get_ports {vgaGreen[1]}]
set_property -dict { PACKAGE_PIN G17  IOSTANDARD LVCMOS33 } [get_ports {vgaGreen[2]}]
set_property -dict { PACKAGE_PIN D17  IOSTANDARD LVCMOS33 } [get_ports {vgaGreen[3]}]
set_property -dict { PACKAGE_PIN N18  IOSTANDARD LVCMOS33 } [get_ports {vgaBlue[0]}]
set_property -dict { PACKAGE_PIN L18  IOSTANDARD LVCMOS33 } [get_ports {vgaBlue[1]}]
set_property -dict { PACKAGE_PIN K18  IOSTANDARD LVCMOS33 } [get_ports {vgaBlue[2]}]
set_property -dict { PACKAGE_PIN J18  IOSTANDARD LVCMOS33 } [get_ports {vgaBlue[3]}]
set_property -dict { PACKAGE_PIN P19  IOSTANDARD LVCMOS33 } [get_ports Hsync]
set_property -dict { PACKAGE_PIN R19  IOSTANDARD LVCMOS33 } [get_ports Vsync]

## PS/2
set_property -dict { PACKAGE_PIN C17  IOSTANDARD LVCMOS33  PULLUP true } [get_ports PS2Clk]
set_property -dict { PACKAGE_PIN B17  IOSTANDARD LVCMOS33  PULLUP true } [get_ports PS2Data]

## Pmod JA (pins 0-3 = PWM servos 0-3, pins 4-7 = unused)
set_property -dict { PACKAGE_PIN J1   IOSTANDARD LVCMOS33 } [get_ports JA0]
set_property -dict { PACKAGE_PIN L2   IOSTANDARD LVCMOS33 } [get_ports JA1]
set_property -dict { PACKAGE_PIN J2   IOSTANDARD LVCMOS33 } [get_ports JA2]
set_property -dict { PACKAGE_PIN G2   IOSTANDARD LVCMOS33 } [get_ports JA3]
set_property -dict { PACKAGE_PIN H1   IOSTANDARD LVCMOS33 } [get_ports JA4]
set_property -dict { PACKAGE_PIN K2   IOSTANDARD LVCMOS33 } [get_ports JA5]
set_property -dict { PACKAGE_PIN H2   IOSTANDARD LVCMOS33 } [get_ports JA6]
set_property -dict { PACKAGE_PIN G3   IOSTANDARD LVCMOS33 } [get_ports JA7]

## Pmod JB (pins 0-3 = PWM servos 4-7, pins 4-7 = unused)
set_property -dict { PACKAGE_PIN A14  IOSTANDARD LVCMOS33 } [get_ports JB0]
set_property -dict { PACKAGE_PIN A16  IOSTANDARD LVCMOS33 } [get_ports JB1]
set_property -dict { PACKAGE_PIN B15  IOSTANDARD LVCMOS33 } [get_ports JB2]
set_property -dict { PACKAGE_PIN B16  IOSTANDARD LVCMOS33 } [get_ports JB3]
set_property -dict { PACKAGE_PIN A15  IOSTANDARD LVCMOS33 } [get_ports JB4]
set_property -dict { PACKAGE_PIN A17  IOSTANDARD LVCMOS33 } [get_ports JB5]
set_property -dict { PACKAGE_PIN C15  IOSTANDARD LVCMOS33 } [get_ports JB6]
set_property -dict { PACKAGE_PIN C16  IOSTANDARD LVCMOS33 } [get_ports JB7]

## Pmod JC (pin 0 = CS, pin 1 = MOSI, pin 2 = MISO, pin 3 = SCLK, pins 4-7 = unused)
set_property -dict { PACKAGE_PIN K17  IOSTANDARD LVCMOS33 } [get_ports JC0_cs_n]
set_property -dict { PACKAGE_PIN M18  IOSTANDARD LVCMOS33 } [get_ports JC1_mosi]
set_property -dict { PACKAGE_PIN N17  IOSTANDARD LVCMOS33 } [get_ports JC2_miso]
set_property -dict { PACKAGE_PIN P18  IOSTANDARD LVCMOS33 } [get_ports JC3_sclk]
set_property -dict { PACKAGE_PIN L17  IOSTANDARD LVCMOS33 } [get_ports JC4]
set_property -dict { PACKAGE_PIN M19  IOSTANDARD LVCMOS33 } [get_ports JC5]
set_property -dict { PACKAGE_PIN P17  IOSTANDARD LVCMOS33 } [get_ports JC6]
set_property -dict { PACKAGE_PIN R18  IOSTANDARD LVCMOS33 } [get_ports JC7]

## Pmod JXADC
set_property -dict { PACKAGE_PIN J3   IOSTANDARD LVCMOS33 } [get_ports {JXADC[0]}]
set_property -dict { PACKAGE_PIN L3   IOSTANDARD LVCMOS33 } [get_ports {JXADC[1]}]
set_property -dict { PACKAGE_PIN M2   IOSTANDARD LVCMOS33 } [get_ports {JXADC[2]}]
set_property -dict { PACKAGE_PIN N2   IOSTANDARD LVCMOS33 } [get_ports {JXADC[3]}]
set_property -dict { PACKAGE_PIN K3   IOSTANDARD LVCMOS33 } [get_ports {JXADC[4]}]
set_property -dict { PACKAGE_PIN M3   IOSTANDARD LVCMOS33 } [get_ports {JXADC[5]}]
set_property -dict { PACKAGE_PIN M1   IOSTANDARD LVCMOS33 } [get_ports {JXADC[6]}]
set_property -dict { PACKAGE_PIN N1   IOSTANDARD LVCMOS33 } [get_ports {JXADC[7]}]

## Configuration
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
