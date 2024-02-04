#if DEBUG
#include "lcd.h"
#endif

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include <avr/wdt.h>

#include <util/delay.h>

/*
	Reference:
	PS/2 Mouse/Keyboard Protocol (C) 1999, Adam Chapweske
	The PS/2 Mouse Interface (C) 2003 Adam Chapweske
	Keyboard/Auxiliary Device Controller - October 1990 (C) IBM Corp. 1990
*/

/*
Source: The PS/2 Mouse Interface by Adam Chapweske, 04/01/03

Movement Data Packet:

The standard PS/2 mouse sends movement/button information to the host using the following 3-byte packet (4): 

		Bit 7      Bit 6      Bit 5      Bit 4      Bit 3    Bit 2      Bit 1     Bit 0 
Byte 1  Y overflow X overflow Y sign bit X sign bit Always 1 Middle Btn Right Btn Left Btn
Byte 2                               X Movement
Byte 3                               Y Movement

Modes of Operation:

Data reporting is handled according to the mode in which the mouse is operating.  There are four standard modes of operation: 

- Reset - The mouse enters Reset mode at power-up or after receiving the "Reset" (0xFF) command. 
- Stream - This is the default mode (after Reset finishes executing) and is the mode in which most software uses the mouse.  If the host has previously set the mouse to Remote mode, it may re-enter Stream mode by sending the "Set Stream Mode" (0xEA) command to the mouse.
- Remote 
- Wrap 

Reset Mode:

The mouse enters reset mode at power-on or in response to the "Reset" (0xFF) command. After entring this mode, the mouse performs a diagnostic self-test referred to as BAT (Basic Assurance Test) and sets the following default values:

Sample Rate = 100 samples/sec
Resolution = 4 counts/mm
Scaling = 1:1
Data Reporting Disabled

It then sends a BAT completion code of either AAh (BAT successful) or FCh (Error). The host's response to a completion code other than AAh is undefined.
Following the BAT completion code (AAh or FCh), the mouse sends its device ID of 00h. This distinguishes it from a keyboard, or a mouse in an extended mode. I have read documents saything the host is not supposed to transmit any data until it receives a device ID.  However I've found that some BIOS's will send the "Reset" command immediately following the 0xAA received after a power-on reset.

After the mouse has sent its device ID to the host, it will enter Stream Mode.  Note that one of the default values set by the mouse is "Data Reporting Disabled".  This means the mouse will not issue any movement data packets until it receives the "Enable Data Reporting" command.

Stream Mode:

In stream mode, the mouse sends movement data when it detects movement or a change in state of one or more mouse buttons. The maximum rate at which this data reporting may occur is known as the sample rate.  This parameter ranges from 10 samples/sec to 200 samples/sec. Its default value is 100 samples/sec and the host may change that value by using the "Set Sample Rate" command.  Stream mode is the default mode of operation following reset.

Remote Mode:

In this mode the mouse reads its inputs and updates its counters/flags at the current sample rate, but it does not automatically issue data packets when movement has occured.  Instead, the host must poll the mouse using the "Read Data" command.  Upon receiving this command the mouse will send a single movement data packet and reset its movement counters.

Command Set:

The following is the set of command accepted by the standard PS/2 mouse.  If the mouse is in Stream mode, the host should disable data reporting (command F5h) before sending any other commands.

FFh (Reset) - The mouse responds to this command with "acknowledge" (FAh) then enters Reset Mode.
FEh (Resend) - The host sends this command whenever it receives invalid data from the mouse. The mouse responds by resending the last(2) packet(3) it sent to the host.   If the mouse responds to the "Resend" command with another invalid packet, the host may either issue another "Resend" command, issue an "Error" command, cycle the mouse's power supply to reset the mouse, or it may inhibit communication (by bringing the Clock line low).  The action taken depends on the host.
F6h (Set Defaults) - The mouse responds with "acknowledge" (FAh) then loads the following values:  Sampling rate = 100, Resolution = 4 counts/mm, Scaling = 1:1, Disable Data Reporting.  The mouse then resets its movement counters and enters stream mode.
F5h (Disable Data Reporting) - The mouse responds with "acknowledge" (FAh) then disables data reporting and resets its movement counters.  This only effects data reporting in Stream mode and does not disable sampling.  Disabled stream mode funcions the same as remote mode.
F4h (Enable Data Reporting) - The mouse responds with "acknowledge" (FAh) then enables data reporting and resets its movement counters.  This command may be issued while the mouse is in Remote Mode (or Stream mode), but it will only effect data reporting in Stream mode.
F3h (Set Sample Rate) - The mouse responds with "acknowledge" (FAh) then reads one more byte from the host.  The mouse saves this byte as the new sample rate. After receiving the sample rate, the mouse again responds with "acknowledge" (0xFA) and resets its movement counters.  Valid sample rates are 10, 20, 40, 60, 80, 100, and 200 samples/sec.
F2h (Get Device ID) - The mouse responds with "acknowledge" (FAh) followed by its device ID (00h for the standard PS/2 mouse.)  The mouse should also reset its movement counters.
F0h (Set Remote Mode) - The mouse responds with "acknowledge" (FAh) then resets its movement counters and enters remote mode.
EEh (Set Wrap Mode) - The mouse responds with "acknowledge" (FAh) then resets its movement counters and  enters wrap mode.
ECh (Reset Wrap Mode) - The mouse responds with "acknowledge" (FAh) then resets its movement counters and enters the mode it was in prior to wrap mode (Stream Mode or Remote Mode.)
EBh (Read Data) - The mouse responds with acknowledge (FAh) then sends a movement data packet. This is the only way to read data in Remote Mode.  After the data packets has been successfully sent, it resets its movement counters.
EAh (Set Stream Mode) - The mouse responds with "acknowledge" then resets its movement counters and enters steram mode.
E9h (Status Request) - The mouse responds with "acknowledge" then sends the following 3-byte status packet (then resets its movement counters.): 

	   Bit 7     Bit 6 Bit 5  Bit 4   Bit 3    Bit 2    Bit 1      Bit 0
Byte 1 Always 0  Mode  Enable Scaling Always 0 Left Btn Middle Btn Right Btn
Byte 2                               Resolution
Byte 3                              Sample Rate

Right, Middle, Left Btn = 1 if button pressed; 0 if button is not pressed.
Scaling = 1 if scaling is 2:1; 0 if scaling is 1:1. (See commands E7h and E6h)
Enable = 1 if data reporting is enabled; 0 if data reporting is disabled. (See commands F5h and F4h)
Mode = 1 if Remote Mode is enabled; 0 if Stream mode is enabled. (See commands F0h and EAh)

E8h (Set Resolution) - The mouse responds with acknowledge (FAh) then reads one byte from the host and again responds with acknowledge (FAh) then resets its movement counters.  The byte read from the host determines the resolution as follows: 
 
Byte Read from Host  Resolution
0x00                 1 count/mm
0x01                 2 count/mm
0x02                 4 count/mm
0x03                 8 count/mm

E7h (Set Scaling 2:1) - The mouse responds with acknowledge (FAh) then enables 2:1 scaling
E6h (Set Scaling 1:1) - The mouse responds with acknowledge (FAh) then enables 1:1 scaling

The only commands the standard PS/2 mouse will send to the host are the "Resend" (FEh) and "Error" (FCh).  They both work the same as they do as host-to-device commands.

Intellimouse Extensions:

After power-on or reset the Microsoft Intellimouse operates just like a standard PS/2 mouse (ie, it uses a 3-byte movement data packet, responds to all commands in the same way as a standard PS/2 mouse, and reports a device ID of 00h.)  To enter "scrolling wheel" mode, the host sends the following command sequence:

Set sample rate 200
Set sample rate 100
Set sample rate 80

The host then issues the "Get device ID" command and waits for a response.  If a standard PS/2 mouse (ie, non-Intellimouse) is attached, it will respond with a device ID of 00h.  In this case, the host will recognize the fact that the mouse does have a scrolling wheel and will continue to treat it as a standard PS/2 mouse.  However, if a Microsoft Intellimouse is attached, it will respond with an ID of 03h.  This tells the host that the attached pointing device has a scrolling wheel and the host will then expect the mouse to use the following 4-byte movement data packet: 

		Bit 7      Bit 6      Bit 5      Bit 4      Bit 3    Bit 2      Bit 1     Bit 0 
Byte 1  Y overflow X overflow Y sign bit X sign bit Always 1 Middle Btn Right Btn Left Btn
Byte 2                               X Movement
Byte 3                               Y Movement
Byte 4                               Z Movement

Z Movement is a 2's complement number that represents the scrolling wheel's movement since the last data report.  Valid values are in the range of -8 to +7. This means the number is actually represented only by the least significant four bits.
*/

enum mouse_commands
{
	MOUSE_RESET = 0xFF, // Reset mouse
	MOUSE_RESEND = 0xFE, // Resend last data packet
	MOUSE_SET_DEFAULTS = 0xF6, // Set defaults
	MOUSE_DISABLE_DATA_REPORT = 0xF5, // Disable data reporting
	MOUSE_ENABLE_DATA_REPORT = 0xF4, // Enable data reporting
	MOUSE_SET_SAMPLE_RATE =  0xF3, // Set sample rate: mouse responds ACK, then reads one byte arg
	MOUSE_GET_DEVICE_ID = 0xF2, // Get device id
	MOUSE_SET_REMOTE_MODE = 0xF0, // Set remote mode
	MOUSE_SET_WRAP = 0xEE, // Set wrap mode
	MOUSE_RESET_WRAP = 0xEC, // Switch back to previous (remote or stream) mode
	MOUSE_REQUEST_DATA = 0xEB, // Request data in remote mode
	MOUSE_SET_STREAM = 0xEA, // Set stream mode (continuous reporting)
	MOUSE_STATUS_REQUEST = 0xE9, // Status request
	MOUSE_SET_RESOLUTION = 0xE8, // Set resolution
	MOUSE_SET_SCALE_21 = 0xE7, // Set scaling 2:1
	MOUSE_SET_SCALE_11 = 0xE6, // Set scaling 1:1
};

enum mouse_response
{
	MOUSE_ACK = 0xFA, // Reset acknowledge OK
	MOUSE_NAK = 0xFE, // Reset acknowledge fail
	MOUSE_ERROR = 0xFC, // Mouse error
	MOUSE_RESET_OK = 0xAA, // Post-reset self test passed
};

enum mouse_resolution
{
	RESOLUTION_1_COUNTS_PER_MM = 0,
	RESOLUTION_2_COUNTS_PER_MM = 1,
	RESOLUTION_4_COUNTS_PER_MM = 2,
	RESOLUTION_8_COUNTS_PER_MM = 3
};

#define GLUE(a, b) a##b
#define DDR(p) GLUE(DDR, p)
#define PORT(p) GLUE(PORT, p)
#define PIN(p) GLUE(PIN, p)

#define DEFAULT_MOUSE_DEVICE_ID 0
#define INTELLIMOUSE_DEVICE_ID 3

#define ENABLE_WHEEL 1
// Valid sample rates are 10, 20, 40, 60, 80, 100, and 200 samples/sec.
#define PS2_SAMPLES_PER_SEC 60

#define PS2_DATA_PORT D
#define PS2_DATA_PIN 4
#define PS2_CLK_PORT D
#define PS2_CLK_PIN 2

#define DI_PORT B
#define MX_PORT C
#define MX_PIN 0
#define MY_PORT C
#define MY_PIN 1
#define MKEY_PORT C
#define MKEY_PIN 2

// DDRx Register: 1 makes the corresponding pin an output, and a 0 makes the corresponding pin an input.
#define input(PORT_, PIN_) DDR(PORT_) &= ~(1 << PIN_);
#define output(PORT_, PIN_) DDR(PORT_) |= (1 << PIN_)

/*
The PORTx Register
The PORTx register functions differently depending on whether a pin is set to input or output. The simpler case is if a pin is set to output. Then, the PORTC register, for example, controls the value at the physical IO pins on Port C.
When a pin is set to be an input, PORTx register DOES NOT contain the logic values applied to the pins. We use the PINx register for that. If a pin is an input, a 1 in the PORTx register sets a pull-up resistor.
*/
#define low(PORT_, PIN_) PORT(PORT_) &= ~(1 << PIN_)
#define high(PORT_, PIN_) PORT(PORT_) |= (1 << PIN_)
/*
typedef struct bits_t
{
	uint8_t bit0 : 1;
	uint8_t bit1 : 1;
	uint8_t bit2 : 1;
	uint8_t bit3 : 1;
	uint8_t bit4 : 1;
	uint8_t bit5 : 1;
	uint8_t bit6 : 1;
	uint8_t bit7 : 1;
} bits;

#define PORTD_BITS (*((volatile bits *)&PORTD))
#define DDRD_BITS (*((volatile bits *)&DDRD))
#define PIND_BITS (*((volatile bits *)&PIND))
*/
//#define KBD_CLK  PORTD_BITS.bit2
//#define KBD_DATA PORTD_BITS.bit4
//#define KBD_CLK_DIR  DDRD_BITS.bit2
//#define KBD_DATA_DIR DDRD_BITS.bit4
#define MCLK 2 // ToDo: replace with PS2_CLK_PORT / PS2_CLK_PIN
#define MDATA 4 // ToDo: replace with PS2_DATA_PORT / PS2_DATA_PIN
//#define CLK_READ PIND_BITS.bit2
//#define DATA_READ PIND_BITS.bit4
#define CLK_READ ((PIN(PS2_CLK_PORT) >> PS2_CLK_PIN) & 0x1)
#define DATA_READ ((PIN(PS2_DATA_PORT) >> PS2_DATA_PIN) & 0x1)

#define ps2_data() ((PIN(PS2_DATA_PORT) >> PS2_DATA_PIN) & 0x1)
#define ps2_clk() ((PIN(PS2_CLK_PORT) >> PS2_CLK_PIN) & 0x1)

/*uint8_t*/int16_t mouse_x = 128;
/*uint8_t*/int16_t mouse_y = 96;
uint8_t mouse_buttons = 0; // ToDo: check PS/2 and Kempston buttons bits. Bit 2: Middle Btn bit 1: Right Btn bit 0: Left Btn

#if ENABLE_WHEEL
bool mouse_wheel_enabled = false; // Mouse wheel present flag.
int8_t mouse_z = 0;
#endif

enum ps2_state_t
{
	PS2_STATE_ERROR,
	PS2_STATE_OK
};

uint8_t ps2_state = PS2_STATE_OK;

void gohi(int pin)
{
	switch (pin)
	{
	case MDATA:
		high(PS2_DATA_PORT, PS2_DATA_PIN);
		input(PS2_DATA_PORT, PS2_DATA_PIN);
		break;
	case MCLK:
		high(PS2_CLK_PORT, PS2_CLK_PIN);
		input(PS2_CLK_PORT, PS2_CLK_PIN);
		break;
	};
}

void golo(int pin)
{
	switch (pin)
	{
	case MDATA:
		low(PS2_DATA_PORT, PS2_DATA_PIN);
		output(PS2_DATA_PORT, PS2_DATA_PIN);
		break;
	case MCLK:
		low(PS2_CLK_PORT, PS2_CLK_PIN);
		output(PS2_CLK_PORT, PS2_CLK_PIN);

		break;
	};
}

void mouse_write_bit(const uint8_t bit)
{
	if (bit)
	{
		gohi(MDATA);
	}
	else
	{
		golo(MDATA);
	}
	/* wait for clock cycle */
	while (CLK_READ == 0) {}
	while (CLK_READ == 1) {}
}

void mouse_write_byte(uint8_t data)
{
	uint8_t parity = 1;

	/* put pins in output mode */
	gohi(MDATA); // ?
	gohi(MCLK); // ?
	_delay_us(300);

	golo(MCLK); // Inhibit communication with CLK = low.
	_delay_us(300);

	// start bit = 0
	golo(MDATA);
	_delay_us(10);
	gohi(MCLK);

	/* wait for mouse to take control of clock); */
	while (CLK_READ == 1) {}
	/* clock is low, and we are clear to send data */

	for (uint8_t i = 0; i < 8; i++)
	{
		mouse_write_bit(data & 0x01);

		parity = parity ^ (data & 0x01);
		data = data >> 1;
	}

	mouse_write_bit(parity);

	// stop bit = 1
	gohi(MDATA);
	_delay_us(50);

	while (CLK_READ == 1) {}
	/* wait for mouse to switch modes */
	while ((CLK_READ == 0) || (DATA_READ == 0)) {} // Device should confirm with ACK bit = 0.

	// put a hold on the incoming data / inhibit the auxiliary device
	golo(MCLK);
}

uint8_t mouse_read_bit(void)
{
	// The Data line changes state when Clock is high and that data is valid when Clock is low.
	// Wait CLK down
	while (CLK_READ == 1) {}

	const uint8_t bit = DATA_READ;

	// Wait CLK up
	while (CLK_READ == 0) {}

	return bit;
}

uint8_t mouse_read_byte(void)
{
	uint8_t data = 0;
	uint8_t parity = 1;

	/* start the clock */
	gohi(MCLK); // ?
	gohi(MDATA); // ?
	_delay_us(50);

	const uint8_t start_bit = mouse_read_bit(); // Start bit should be 0
	if(start_bit == 1)
		ps2_state = PS2_STATE_ERROR;

	// Skip start bit
//	while (CLK_READ == 1) {}
//	_delay_us(5); /* not sure why */
//	while (CLK_READ == 0) {}

	for (uint8_t i = 0; i < 8; i++)
	{
		data >>= 1;

		const uint8_t bit = mouse_read_bit();
		if(bit)
			data |= 0x80;
		parity = parity ^ (bit & 0x01);
	}

	const uint8_t parity_bit = mouse_read_bit(); // Parity bit (odd parity)
	if (parity != (parity_bit != 0))
		ps2_state = PS2_STATE_ERROR;

	const uint8_t stop_bit = mouse_read_bit(); // Stop bit should be 1
	if(stop_bit == 0)
		ps2_state = PS2_STATE_ERROR;

	/* put a hold on the incoming data. */
	golo(MCLK);

	return data;
}

void mouse_write_byte_read_ack(const uint8_t data)
{
	mouse_write_byte(data);

	if (mouse_read_byte() != MOUSE_ACK)
	{ 
		ps2_state = PS2_STATE_ERROR; 
	}
}

void mouse_init(void)
{
	gohi(MCLK); //?
	gohi(MDATA); //?

	mouse_write_byte_read_ack(MOUSE_RESET);

	if (mouse_read_byte() != MOUSE_RESET_OK)
	{ 
		ps2_state = PS2_STATE_ERROR;
	}

	if (mouse_read_byte() != DEFAULT_MOUSE_DEVICE_ID)
	{ 
		ps2_state = PS2_STATE_ERROR; 
	}

#if ENABLE_WHEEL
	// Microsoft Intellimouse scrolling wheel enable sequence. Sample rate set to 80 as side effect.
	mouse_write_byte_read_ack(MOUSE_SET_SAMPLE_RATE);
	mouse_write_byte_read_ack(200);
	mouse_write_byte_read_ack(MOUSE_SET_SAMPLE_RATE);
	mouse_write_byte_read_ack(100);
	mouse_write_byte_read_ack(MOUSE_SET_SAMPLE_RATE);
	mouse_write_byte_read_ack(80);

	// Check if connected device is Microsoft Intellimouse compatible.
	mouse_write_byte_read_ack(MOUSE_GET_DEVICE_ID);
	// Standard PS/2 mouse will respond with device ID = 00h.
	// Microsoft Intellimouse will respond with an ID of 03h.
	mouse_wheel_enabled = (mouse_read_byte() == INTELLIMOUSE_DEVICE_ID);
#endif
	// Set mouse resolution.
	mouse_write_byte_read_ack(MOUSE_SET_RESOLUTION);
	mouse_write_byte_read_ack(RESOLUTION_8_COUNTS_PER_MM);

	// Set mouse reports count per second.
	mouse_write_byte_read_ack(MOUSE_SET_SAMPLE_RATE);
	mouse_write_byte_read_ack(PS2_SAMPLES_PER_SEC);

	mouse_write_byte_read_ack(MOUSE_SET_SCALE_11);

	// Enable mouse data streaming.
	mouse_write_byte_read_ack(MOUSE_ENABLE_DATA_REPORT);

	_delay_us(100); //?
}

int main(void)
{
#if DEBUG
	lcd_init(LCD_DISP_ON); // init lcd and turn on
#endif
	mouse_init();

	DDR(DI_PORT) = 0xFF;
	output(MX_PORT, MX_PIN);
	output(MY_PORT, MY_PIN);
	output(MKEY_PORT, MKEY_PIN);

	PORT(DI_PORT) = 0x00;
	low(MX_PORT, MX_PIN);
	low(MY_PORT, MY_PIN);
	low(MKEY_PORT, MKEY_PIN);

#if ENABLE_WHEEL
		lcd_gotoxy(0, 3);

		if (mouse_wheel_enabled)
		{
			lcd_puts("Intellimouse mode");
		}
		else
		{
			lcd_puts("Standard mode");
		}
#endif

	while (1)
	{
		mouse_buttons = mouse_read_byte();
		int8_t dx = (int8_t)mouse_read_byte();
		int8_t dy = (int8_t)mouse_read_byte();
		mouse_x += dx;
		mouse_y += dy;
#if ENABLE_WHEEL
		if (mouse_wheel_enabled)
		{
			mouse_z = (int8_t)mouse_read_byte();
		}
#endif
		PORT(DI_PORT) = mouse_x;
		_delay_us(1);
		high(MX_PORT, MX_PIN);
		_delay_us(10);
		low(MX_PORT, MX_PIN);

		PORT(DI_PORT) = mouse_y;
		_delay_us(1);
		high(MY_PORT, MY_PIN);
		_delay_us(10);
		low(MY_PORT, MY_PIN);

#if ENABLE_WHEEL
		PORT(DI_PORT) = (mouse_buttons & 0b00000111) | (mouse_z << 4);
#else
		PORT(DI_PORT) = (mouse_buttons & 0b00000111);
#endif
		_delay_us(1);
		high(MKEY_PORT, MKEY_PIN);
		_delay_us(10);
		low(MKEY_PORT, MKEY_PIN);

#if DEBUG
		char buf[5];
		lcd_gotoxy(0, 0);
		itoa(mouse_x, buf, 10);
		lcd_puts(buf);
		lcd_puts(" ");
		itoa(mouse_y, buf, 10);
		lcd_puts(buf);
		lcd_puts(" ");
#if ENABLE_WHEEL
		itoa(mouse_z, buf, 10);
		lcd_puts(buf);
		lcd_puts(" ");
#endif
#endif // DEBUG
		if (ps2_state == PS2_STATE_ERROR)
		{
			wdt_enable(WDTO_1S); // reboot controller in case of error
		}
	};

	return 0;
}