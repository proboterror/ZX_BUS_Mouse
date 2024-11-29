/*
	PS/2 mouse support for ZX BUS Kempston Mouse Controller.
	Read PS/2 port mouse state from PS2_DATA_PIN / PS2_CLK_PIN
	and set Kempston mouse controller registers via DI_PORT and MX_PIN, MY_PIN, MKEY_PIN strobes.
	Support 3 mouse buttons
	Mouse wheel not supported.

	Uses the PS2Mouse library available from http://github.com/kristopher/PS2-Mouse-Arduino/
	Compiled using https://github.com/MCUdude/MiniCore and Arduino 1.8.19.
*/

#include <PS2Mouse.h>

#define MOUSE_DATA 3
#define MOUSE_CLOCK 2

/*
#define PS2_DATA_PORT D
#define PS2_DATA_PIN 3
#define PS2_CLK_PORT D
#define PS2_CLK_PIN 2
*/
#define DI_PORT PORTB
#define MX_PORT PORTC
#define MX_PIN 0
#define MY_PORT PORTC
#define MY_PIN 1
#define MKEY_PORT PORTC
#define MKEY_PIN 2

#define DI_BUS_SET_DELAY 2
#define REGISTER_SET_DELAY 10

#define low(PORT_, PIN_) PORT_ &= ~(1 << PIN_)
#define high(PORT_, PIN_) PORT_ |= (1 << PIN_)

// Kempston mouse interface axis data, increased by reported PS/2 mouse axis coordinate delta.
// 8-bit values are wrapped and not clamped.
uint8_t mouse_x = 128;
uint8_t mouse_y = 96;
// Value read from PS/2 mouse data packet. Should be inverted for Kempston mouse.
// Bit 2: middle button, bit 1: right button, bit 0: left button.
// Unlike PS/2 mouse, Kempston mouse can have 4 buttons.
uint8_t mouse_buttons = 0;

PS2Mouse mouse(MOUSE_CLOCK, MOUSE_DATA, STREAM);

void setup()
{
	mouse.initialize();

	// Set CPLD ports to output.
	DDRB = 0xFF; // DI_PORT
	DDRC = 0xFF; // MX_PORT / MY_PORT / MKEY_PORT

	low(MX_PORT, MX_PIN);
	low(MY_PORT, MY_PIN);
	low(MKEY_PORT, MKEY_PIN);

	// Set initial MX, MY, MKEY values to CPLD controller registers. Changed after first PS/2 mouse event packet received.
	// Data written to registers on rising_edge.
	// Questionable, some code attempting to detect Kempston mouse presence by checking X=Y=0xFF, some other by checking X!=Y or buttons=0b111.
	// Actually there is no reliable method to detect Kempston mouse presence.

	DI_PORT = 0xFF;
	delayMicroseconds(DI_BUS_SET_DELAY);
	high(MX_PORT, MX_PIN);
	delayMicroseconds(REGISTER_SET_DELAY);
	low(MX_PORT, MX_PIN);

	DI_PORT = 0x80;
	delayMicroseconds(DI_BUS_SET_DELAY);
	high(MY_PORT, MY_PIN);
	delayMicroseconds(REGISTER_SET_DELAY);
	low(MY_PORT, MY_PIN);

	DI_PORT = 0x60;
	delayMicroseconds(DI_BUS_SET_DELAY);
	high(MKEY_PORT, MKEY_PIN);
	delayMicroseconds(REGISTER_SET_DELAY);
	low(MKEY_PORT, MKEY_PIN);
}

void loop()
{ 
	int16_t mouse_data[3];
	mouse.report(mouse_data);

	mouse_buttons = (uint8_t)mouse_data[0];

	int8_t dx = (int8_t)mouse_data[1];
	int8_t dy = (int8_t)mouse_data[2];

	mouse_x += dx;
	mouse_y += dy;

	// Set MX, MY, MKEY values to controller registers.
	// Data written to registers on rising_edge.
	DI_PORT = mouse_x;
	delayMicroseconds(DI_BUS_SET_DELAY);
	high(MX_PORT, MX_PIN);
	delayMicroseconds(REGISTER_SET_DELAY);
	low(MX_PORT, MX_PIN);

	DI_PORT = mouse_y;
	delayMicroseconds(DI_BUS_SET_DELAY);
	high(MY_PORT, MY_PIN);
	delayMicroseconds(REGISTER_SET_DELAY);
	low(MY_PORT, MY_PIN);

	DI_PORT = (~mouse_buttons & 0b00000111) | (0b11111000); // Unused port bits set to 1 (mouse_z axis, 4th mouse button)
	delayMicroseconds(DI_BUS_SET_DELAY);
	high(MKEY_PORT, MKEY_PIN);
	delayMicroseconds(REGISTER_SET_DELAY);
	low(MKEY_PORT, MKEY_PIN);
}