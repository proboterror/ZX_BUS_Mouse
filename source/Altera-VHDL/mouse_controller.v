/*
 ZX BUS Kempston Mouse controller
 Target: EPM3032ALC44 / EPM3064ALC44

 Kempston mouse ports:
 #FADF 1111 1010 1101 1111 BUTTONS
 #FBDF 1111 1011 1101 1111 MX
 #FFDF 1111 1111 1101 1111 MY

 Port #FADF: buttons and mouse wheel.
  D0: left button (0 = pressed)
  D1: right button (0 = pressed)
  D2: middle button (0 = pressed)
  D3: reserved (default 1)
  D4-D7: mouse wheel axis

   Note on mouse wheel:
   https://velesoft.speccy.cz/kmouse/km-doc/kempston_mouse_turbo_interface/km-t_2011/k-mouse2011-doc.pdf:
   If wheel is off then D4-D7 on button port return 1111.
   https://www.benophetinternet.nl/hobby/kmt/index.htm
   Bit 4-7 (D4-D7) return the position of the mouse wheel = %1111 is default

 Port #FBDF: X axis (increases from left to right)
 Port #FFDF: Y axis (increases from down to up)
*/
module mouse_controller
(
	input wire MX, MY, MKEY,
	input wire[7:0] DI,

	input wire A0, // 1
	input wire A1, // 1
	input wire A7, // 1
	input wire M1, // 1
	input wire A5, // 0
	input wire RD, // active low
	input wire IORQ, // active low
	input wire A8,
	input wire A10,

	// IORQGE = 0 when address lower bits and M1 == 1 (address partial match) else = Z.
	output wire IORQGE,

	output wire[7:0] D
);

	reg[7:0] register_x, register_y, register_key;

	// Data written to registers on rising_edge (see source/ATmega/main.c).
	always @(posedge MX) register_x = DI;
	always @(posedge MY) register_y = DI;
	always @(posedge MKEY) register_key = DI;

	wire address_partial_match = ~(A0 & A1 & A7 & M1 & ~A5);
	assign IORQGE = address_partial_match ? 1'hZ : 1; // CPLD IORQGE pin directly connected to ZX-BUS IORQGE pin.
	wire enable = ~(address_partial_match | RD | IORQ);

	wire MKEY_SEL = enable & ~A8 & ~A10;
	wire MX_SEL = enable & A8 & ~A10;
	wire MY_SEL = enable & A8 & A10;

	assign D = MX_SEL ? register_x : 8'hZ;
	assign D = MY_SEL ? register_y : 8'hZ;

	// Example 3 buttons mouse support with mouse wheel (35 macrocells)
	//assign D = MKEY_SEL ? register_key : 8'hZ;
	
	// Example 2 buttons mouse support with mouse wheel (33 macrocells)
	// Exclude 3rd mouse button and reserved bit from output in order to fit to EPM3032.
	//assign D = MKEY_SEL ? {register_key[7:4], 2'b11, register_key[1:0]} : 8'hZ;

	// Example 3 buttons mouse support with mouse wheel turned off (31 macrocells)
	assign D = MKEY_SEL ? {4'b1111, register_key[3:0]} : 8'hZ;
endmodule