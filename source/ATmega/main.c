/*
	PS/2 mouse support for ZX BUS Kempston Mouse Controller.
	Read PS/2 port mouse state from PS2_DATA_PIN / PS2_CLK_PIN
	and set Kempston mouse controller registers via DI_PORT and MX_PIN, MY_PIN, MKEY_PIN strobes.
	3 mouse buttons and 4 bit mouse wheel axis supported.

	Interrupt processing based on avr-mouse-ps2-to-serial firmware source by Oleg Trifonov
	https://github.com/trol73/avr-mouse-ps2-to-serial

	Partially based on "Arduino/Wiring Library for interfacing with a PS2 mouse"
	https://github.com/kristopher/PS2-Mouse-Arduino

	Notable mention:
	https://github.com/svofski/mouse1351
	https://github.com/rucek/arduino-ps2-mouse
	https://github.com/trol73/avr-mouse-ps2-to-serial
	ZX Evolution ATmega128 peripheral controller avr source, http://svn.zxevo.ru/listing.php?repname=pentevo
*/

#if DEBUG
//Uses "OLED for AVR mikrocontrollers" library, https://github.com/Sylaina/oled-display
#include "lcd.h"
#endif

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include <avr/interrupt.h>

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

#define DEFAULT_MOUSE_DEVICE_ID 0
#define INTELLIMOUSE_DEVICE_ID 3

// Valid sample rates are 10, 20, 40, 60, 80, 100, and 200 samples/sec.
#define PS2_SAMPLES_PER_SEC 60

#define PS2_DATA_PORT D
#define PS2_DATA_PIN 3
#define PS2_CLK_PORT D
#define PS2_CLK_PIN 2

#define DI_PORT B
#define MX_PORT C
#define MX_PIN 0
#define MY_PORT C
#define MY_PIN 1
#define MKEY_PORT C
#define MKEY_PIN 2

#define GLUE(a, b) a##b
#define DDR(p) GLUE(DDR, p)
#define PORT(p) GLUE(PORT, p)
#define PIN(p) GLUE(PIN, p)

// DDRx Register: 1 makes the corresponding pin an output, and a 0 makes the corresponding pin an input.
#define input(PORT_, PIN_) DDR(PORT_) &= ~(1 << PIN_)
#define output(PORT_, PIN_) DDR(PORT_) |= (1 << PIN_)

/*
The PORTx Register
The PORTx register functions differently depending on whether a pin is set to input or output. The simpler case is if a pin is set to output. Then, the PORTC register, for example, controls the value at the physical IO pins on Port C.
When a pin is set to be an input, PORTx register DOES NOT contain the logic values applied to the pins. We use the PINx register for that. If a pin is an input, a 1 in the PORTx register sets a pull-up resistor.
*/
#define low(PORT_, PIN_) PORT(PORT_) &= ~(1 << PIN_)
#define high(PORT_, PIN_) PORT(PORT_) |= (1 << PIN_)

#define CLK_READ ((PIN(PS2_CLK_PORT) >> PS2_CLK_PIN) & 0x1)
#define DATA_READ ((PIN(PS2_DATA_PORT) >> PS2_DATA_PIN) & 0x1)

#define DI_BUS_SET_DELAY 1
#define REGISTER_SET_DELAY 1

// Kempston mouse interface axis data, increased by reported PS/2 mouse axis coordinate delta.
// 8-bit values are wrapped and not clamped.
uint8_t mouse_x = 128;
uint8_t mouse_y = 96;
// Value read from PS/2 mouse data packet. Should be inverted for Kempston mouse.
// Bit 2: middle button, bit 1: right button, bit 0: left button.
// Unlike PS/2 mouse, Kempston mouse can have 4 buttons.
uint8_t mouse_buttons = 0;

#if ENABLE_WHEEL
bool mouse_wheel_enabled = false; // Mouse wheel present flag.
/*
Absolute mouse wheel axis value.
0b0000 by default if mouse wheel presented else 0b1111.
Increased / decreased by reported mouse wheel delta.
Matched Unreal Speccy 0.39.0 behaviour.
References:
https://groups.google.com/g/fido7.real.speccy/c/Qeid4aFhjRg
https://groups.google.com/g/fido7.zx.spectrum/c/vpr8vh2X2sA
https://zxpress.ru/article.php?id=6538 (DonNews #19)
https://velesoft.speccy.cz/kmouse/km-doc/kempston_mouse_turbo_interface/km-t_2011/k-mouse2011-doc.pdf
*/
int8_t mouse_z = 0b00000000;
#endif

typedef enum
{
	PS2_STATE_ERROR,
	PS2_STATE_READ,
	PS2_STATE_WRITE,
	PS2_STATE_OK
} ps2_state_t;

#define PS2_BUF_SIZE 255 // PS/2 receive buffer size

volatile ps2_state_t ps2_state = PS2_STATE_OK;
volatile uint8_t ps2_bitcount; // send / receive data handler remaining bits counter
volatile uint8_t ps2_data; // send / receive one byte buffer
volatile uint8_t ps2_parity; // sending byte parity

volatile uint8_t ps2_rx_buf[PS2_BUF_SIZE]; // PS/2 receive ring buffer
volatile uint8_t ps2_rx_buf_w; // ring buffer write head
volatile uint8_t ps2_rx_buf_r; // ring buffer read tail
volatile uint8_t ps2_rx_buf_count; // bytes count in receive buffer

typedef enum
{
	MCLK,
	MDATA
} mouse_pin_t;

// PS/2 interface data and clock lines are both open collector.
// Configuring port as input (DDRx register) switches it to high impendance mode,
// writing high to PORTx register enables internal weak pull-up resistor.
void gohi(mouse_pin_t pin)
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

// Port configured as output and connected to ground.
void golo(mouse_pin_t pin)
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

// Enable INT0 external interrupt on clock change.
void enable_int0(void)
{
	GIFR = _BV(INTF0);
	GICR |= _BV(INT0);
	MCUCR = (MCUCR & 0xFC) | 2; // The falling edge of INT0 generates an interrupt request.
}

// Disable INT0 external interrupt on clock change.
void disable_int0(void)
{
	GIFR = _BV(INTF0);
	GICR &= ~_BV(INT0);
}

// Store byte received from PS/2 mouse device in receive buffer.
// Called only from interrupt handler.
void ps2_rx_push(const uint8_t data)
{
	// Reboot / restart controller in case of receive buffer overflow.
	if (ps2_rx_buf_count >= sizeof(ps2_rx_buf))
	{
		ps2_state = PS2_STATE_ERROR;
		return;
	}

	// Store in receive buffer
	ps2_rx_buf[ps2_rx_buf_w] = data;
	ps2_rx_buf_count++;

	if (++ps2_rx_buf_w == sizeof(ps2_rx_buf))
		ps2_rx_buf_w = 0;
}

// Non-blocking get byte from receive buffer.
// Returns 0 if buffer empty.
uint8_t mouse_read_byte_async(void) // ps2_aread
{
	uint8_t data;

	// Return 0 if buffer empty.
	if (ps2_rx_buf_count == 0)
	{
		data = 0;
	}
	else
	{
		disable_int0(); // Disable clock change interrupt because of receive buffer change.

		// Read byte from receive buffer.
		data = ps2_rx_buf[ps2_rx_buf_r];
		ps2_rx_buf_count--;

		if (++ps2_rx_buf_r == sizeof(ps2_rx_buf))
			ps2_rx_buf_r = 0;

		enable_int0(); // Enable clock change interrupt.
	}

	return data;
}

uint8_t parity(uint8_t p)
{
	p = p ^ (p >> 4 | p << 4);
	p = p ^ (p >> 2);
	p = p ^ (p >> 1);

	return (p ^ 1) & 1;
}

ISR (INT0_vect)
{
	if (ps2_state == PS2_STATE_ERROR)
		return;

	// Data sent from the host to the device is read on the rising edge of the clock signal.
	if (ps2_state == PS2_STATE_WRITE)
	{
		switch (ps2_bitcount)
		{
			default: // Data byte
				if (ps2_data & 1)
					//gohi(MDATA);
					input(PS2_DATA_PORT, PS2_DATA_PIN);
				else
					//golo(MDATA);
					output(PS2_DATA_PORT, PS2_DATA_PIN);

				ps2_data >>= 1;
				break;
			case 3: // Parity bit
				if (ps2_parity)
					//gohi(MDATA);
					input(PS2_DATA_PORT, PS2_DATA_PIN);
				else
					//golo(MDATA);
					output(PS2_DATA_PORT, PS2_DATA_PIN);

				break;
			case 2: // Stop bit
				//gohi(MDATA); // Stop bit should be 1
				input(PS2_DATA_PORT, PS2_DATA_PIN);
				break;
			case 1: // Receive acknowledge bit
				if (DATA_READ)
					ps2_state = PS2_STATE_ERROR;
				else
					ps2_state = PS2_STATE_READ; 

				ps2_bitcount = 12;
				break;
		}
	}
	// Data sent from the device to the host is read on the falling edge of the clock signal.
	else // PS2_STATE_READ
	{
		// The Data line changes state when Clock is high and that data is valid when Clock is low.
		switch(ps2_bitcount)
		{
			case 11: // Start bit
				if (DATA_READ) // Start bit should be 0
					ps2_state = PS2_STATE_ERROR;

				break;
			default: // Data byte
				ps2_data >>= 1;

				if(DATA_READ)
					ps2_data |= 0x80;

				break;
			case 2: // Parity bit
				if (parity(ps2_data) != (DATA_READ != 0)) // Parity bit (odd parity)
					ps2_state = PS2_STATE_ERROR;

				break;
			case 1: // Stop bit
				if (DATA_READ) // Stop bit should be 1
					ps2_rx_push(ps2_data);
				else
					 ps2_state = PS2_STATE_ERROR;

				ps2_bitcount = 12;
		}
	}

	ps2_bitcount--;
}

void ps2_init_port(void)
{
	disable_int0();

	// Switch PS/2 port to input.
	input(PS2_CLK_PORT, PS2_CLK_PIN);
	input(PS2_DATA_PORT, PS2_DATA_PIN);
	// Release clock and data lines.
	//gohi(MCLK);
	//gohi(MDATA);

	// Reset receive biffer.
	ps2_rx_buf_w = 0;
	ps2_rx_buf_r = 0;
	ps2_rx_buf_count = 0;

	// Set clock interrupt handler state.
	ps2_state = PS2_STATE_READ;
	ps2_bitcount = 11;

	enable_int0();

	sei(); // Enable global interrupts.
}

/*
The PS/2 Mouse/Keyboard Protocol
http://www.computer-engineering.org/ps2protocol/

Host-to-Device Communication:

PS/2 device always generates the clock signal. If the host wants to send data, it must first put the Clock
and Data lines in a "Request-to-send" state as follows:

* Inhibit communication by pulling Clock low for at least 100 microseconds.
* Apply "Request-to-send" by pulling Data low, then release Clock.

The device should check for this state at intervals not to exceed 10 milliseconds. When the device detects this state, it will begin generating Clock signals and clock in eight data bits and one stop bit. The host changes the Data line only when the Clock line is low, and data is read by the device when Clock is high.

After the stop bit is received, the device will acknowledge the received byte by bringing the Data line low and generating one last clock pulse. 

Steps the host must follow to send data to a PS/2 device:
 1) Bring the Clock line low for at least 100 microseconds.
 2) Bring the Data line low.
 3) Release the Clock line.
 4) Wait for the device to bring the Clock line low.
 5) Set/reset the Data line to send the first data bit
 6) Wait for the device to bring Clock high.
 7) Wait for the device to bring Clock low.
 8) Repeat steps 5-7 for the other seven data bits and the parity bit
 9) Release the Data line.
 10) Wait for the device to bring Data low.
 11) Wait for the device to bring Clock low.
 12) Wait for the device to release Data and Clock
*/

// Non-blocking send byte to PS/2 device without acknowledge.
void mouse_write_byte_async(uint8_t data) // ps2_write
{
	disable_int0();

	// Inhibit host to device communication by bringing clock low for 100 us.
	golo(MCLK);
	_delay_us(100);

	golo(MDATA); // This is start bit = 0

	_delay_us(10); // Required rise time for data line.

	// Release clock line.
	// Data sent from the host to the device is read on the rising edge of the clock signal.
	input(PS2_CLK_PORT, PS2_CLK_PIN);
	//gohi(MCLK);

	// Reset receive buffer.
	ps2_rx_buf_count = 0;
	ps2_rx_buf_w = 0;
	ps2_rx_buf_r = 0;

	// Set interrupt handler state to send.
	ps2_state = PS2_STATE_WRITE;
	ps2_bitcount = 11;
	ps2_data = data;
	ps2_parity = parity(data);

	enable_int0();
}

// Blocking read byte from PS/2 port.
uint8_t mouse_read_byte_sync(void) // ps2_recv
{
	while (ps2_rx_buf_count == 0);

	return mouse_read_byte_async();
}

uint8_t mouse_init_state = 0; // ToDo: reset on init restart

// Returns true in byte read from device and matches passed value. 
// Increases mouse_init_state if value matches else set ps2_state to PS2_STATE_ERROR.
bool mouse_read_compare_value_async(uint8_t value)
{
	if(ps2_rx_buf_count)
	{
		if (mouse_read_byte_sync() != value)
		{
			ps2_state = PS2_STATE_ERROR;

			return false;
		}

		mouse_init_state++; // Intended to be called from mouse_init_protocol.

		return true;
	}

	return false;
}

// Async mouse init with state machine.
// Execution time not exceed 110 us (send inhibit + set start bit) time.
// Returns true if init sequence succesfully completed.
bool mouse_init_protocol(void)
{
	switch (mouse_init_state)
	{
	case 0:
		mouse_write_byte_async(MOUSE_RESET);
		mouse_init_state++;
	case 1:
		if(!mouse_read_compare_value_async(MOUSE_ACK))
			return false;
	case 2:
		if(!mouse_read_compare_value_async(MOUSE_RESET_OK))
			return false;
	case 3:
		if(!mouse_read_compare_value_async(DEFAULT_MOUSE_DEVICE_ID))
		{
			return false;
		}
		else
		{
#if !ENABLE_WHEEL
			mouse_init_state = 19;
#endif
		}

#if ENABLE_WHEEL
	// Microsoft Intellimouse scrolling wheel enable sequence: set sample rate 200, 100, 80.
	// Sample rate set to 80 as side effect.
	case 4:
		mouse_write_byte_async(MOUSE_SET_SAMPLE_RATE);
		mouse_init_state++;
	case 5:
		if(!mouse_read_compare_value_async(MOUSE_ACK))
			return false;
	case 6:
		mouse_write_byte_async(200);
		mouse_init_state++;
	case 7:
		if(!mouse_read_compare_value_async(MOUSE_ACK))
			return false;

	case 8:
		mouse_write_byte_async(MOUSE_SET_SAMPLE_RATE);
		mouse_init_state++;
	case 9:	
		if(!mouse_read_compare_value_async(MOUSE_ACK))
			return false;
	case 10:
		mouse_write_byte_async(100);
		mouse_init_state++;
	case 11:
		if(!mouse_read_compare_value_async(MOUSE_ACK))
			return false;

	case 12:
		mouse_write_byte_async(MOUSE_SET_SAMPLE_RATE);
		mouse_init_state++;
	case 13:
		if(!mouse_read_compare_value_async(MOUSE_ACK))
			return false;
	case 14:
		mouse_write_byte_async(80);
		mouse_init_state++;
	case 15:
		if(!mouse_read_compare_value_async(MOUSE_ACK))
			return false;

	// Check if connected device is Microsoft Intellimouse compatible.
	case 16:
		mouse_write_byte_async(MOUSE_GET_DEVICE_ID);
		mouse_init_state++;
	case 17:
		if(!mouse_read_compare_value_async(MOUSE_ACK))
			return false;
	case 18:
		if(ps2_rx_buf_count)
		{
			// Standard PS/2 mouse will respond with device ID = 00h.
			// Microsoft Intellimouse will respond with an ID of 03h.
			if (mouse_read_byte_sync() == INTELLIMOUSE_DEVICE_ID)
			{
				mouse_wheel_enabled = true;
			}
			else
			{
				// If mouse wheel not presented then bits 7-4 on buttons port returns 1111.
				if(!mouse_wheel_enabled)
					mouse_z = 0b00001111;
			}

			mouse_init_state++;
		}
		else
			return false;
#endif // ENABLE_WHEEL

	// Set mouse resolution.
	case 19:
		mouse_write_byte_async(MOUSE_SET_RESOLUTION);
		mouse_init_state++;
	case 20:
		if(!mouse_read_compare_value_async(MOUSE_ACK))
			return false;
	case 21:
		mouse_write_byte_async(RESOLUTION_8_COUNTS_PER_MM);
		mouse_init_state++;
	case 22:
		if(!mouse_read_compare_value_async(MOUSE_ACK))
			return false;

	// Set mouse reports count per second.
	case 23:
		mouse_write_byte_async(MOUSE_SET_SAMPLE_RATE);
		mouse_init_state++;
	case 24:
		if(!mouse_read_compare_value_async(MOUSE_ACK))
			return false;
	case 25:
		mouse_write_byte_async(PS2_SAMPLES_PER_SEC);
		mouse_init_state++;
	case 26:
		if(!mouse_read_compare_value_async(MOUSE_ACK))
			return false;

	case 27:
		mouse_write_byte_async(MOUSE_SET_SCALE_11);
		mouse_init_state++;
	case 28:
		if(!mouse_read_compare_value_async(MOUSE_ACK))
			return false;

	// Enable mouse data streaming.
	case 29:
		mouse_write_byte_async(MOUSE_ENABLE_DATA_REPORT);
		mouse_init_state++;
	case 30:
		if(!mouse_read_compare_value_async(MOUSE_ACK))
			return false;
	}

	return true;
}

// Process received mouse state packet: x, y axis delta and buttons / wheel axis.
// Returns false if no data received.
bool mouse_read_state_async(void)
{
	uint8_t bytes_expected = 3;
#if ENABLE_WHEEL
	if (mouse_wheel_enabled)
		bytes_expected++;
#endif
	if(ps2_rx_buf_count >= bytes_expected)
	{
		mouse_buttons = mouse_read_byte_async() & 7;
		mouse_x += (int8_t)mouse_read_byte_async();
		mouse_y += (int8_t)mouse_read_byte_async();
#if ENABLE_WHEEL
		if (mouse_wheel_enabled)
		{
			mouse_z -= (int8_t)mouse_read_byte_async(); // Axis direction matched Unreal Speccy 0.39.0 behaviour.
			mouse_z &= 0b00001111;
		}
#endif
		return true;
	}

	return false;
}

int main(void)
{
	// Set CPLD ports to output.
	DDR(DI_PORT) = 0xFF;
	output(MX_PORT, MX_PIN);
	output(MY_PORT, MY_PIN);
	output(MKEY_PORT, MKEY_PIN);

	low(MX_PORT, MX_PIN);
	low(MY_PORT, MY_PIN);
	low(MKEY_PORT, MKEY_PIN);

	// Set initial MX, MY, MKEY values to CPLD controller registers. Changed after first PS/2 mouse event packet received.
	// Data written to registers on rising_edge.
	// Questionable, some code attempting to detect Kempston mouse presence by checking X=Y=0xFF, some other by checking X!=Y or buttons=0b111.
	// Actually there is no reliable method to detect Kempston mouse presence.

	PORT(DI_PORT) = 0x80;
	_delay_us(DI_BUS_SET_DELAY);
	high(MX_PORT, MX_PIN);
	_delay_us(REGISTER_SET_DELAY);
	low(MX_PORT, MX_PIN);

	PORT(DI_PORT) = 0x60;
	_delay_us(DI_BUS_SET_DELAY);
	high(MY_PORT, MY_PIN);
	_delay_us(REGISTER_SET_DELAY);
	low(MY_PORT, MY_PIN);

	PORT(DI_PORT) = 0xFF;
	_delay_us(DI_BUS_SET_DELAY);
	high(MKEY_PORT, MKEY_PIN);
	_delay_us(REGISTER_SET_DELAY);
	low(MKEY_PORT, MKEY_PIN);

#if DEBUG
	lcd_init(LCD_DISP_ON); // init lcd and turn on

	lcd_gotoxy(0, 2);
	lcd_puts("Init");
#endif

	bool ps2_port_inited = false;
	bool mouse_inited = false;

	while (1)
	{
		if(ps2_port_inited == false)
		{
			lcd_puts(".");
			ps2_init_port(); 
			ps2_port_inited = true;

			// Start mouse protocol init timeout timer.
			TCNT1 = 0; // Initial timer value.
			TCCR1B = 0b00000101; // Set timer 1 clock prescaler to F_CPU/1024.
		}

		if(mouse_inited == false)
		{
			mouse_inited = mouse_init_protocol();

			if(mouse_inited)
			{
			#if DEBUG
				lcd_puts("Done");

				lcd_gotoxy(0, 3);
			#if ENABLE_WHEEL
				if (mouse_wheel_enabled)
				{
					lcd_puts("Intellimouse mode");
				}
				else
			#endif
				{
					lcd_puts("Standard mode");
				}
			#endif // DEBUG
			}
			else
			{
				// Reboot controller in case of init lock.
				// Microsoft Mouse Port Compatible Mouse 2.1A (FCC ID: C3KKS8, Part No 92841): takes more than 1s to init after power on.
				// Logitech M-SBF96 (P/N 852209-A000): requires 1s delay after power on before init.

				// if mouse protocol init timeout timer elapsed, restart init
				if(TCNT1 > (F_CPU >> 10 << 1)) // timer set to 2s
				{
					ps2_state = PS2_STATE_ERROR;
				}
			}
		}
		else
		{
			if(mouse_read_state_async())
			{
				// Set MX, MY, MKEY values to controller registers.
				// Data written to registers on rising_edge.
				PORT(DI_PORT) = mouse_x;
				_delay_us(DI_BUS_SET_DELAY);
				high(MX_PORT, MX_PIN);
				_delay_us(REGISTER_SET_DELAY);
				low(MX_PORT, MX_PIN);

				PORT(DI_PORT) = mouse_y;
				_delay_us(DI_BUS_SET_DELAY);
				high(MY_PORT, MY_PIN);
				_delay_us(REGISTER_SET_DELAY);
				low(MY_PORT, MY_PIN);

#if ENABLE_WHEEL
				PORT(DI_PORT) = (~mouse_buttons & 0b00000111) | (mouse_z << 4) | (0b00001000); // Bit 4 is 4th mouse button (not pressed)
#else
				PORT(DI_PORT) = (~mouse_buttons & 0b00000111) | (0b11111000); // Unused port bits set to 1 (mouse_z axis, 4th mouse button)
#endif
				_delay_us(DI_BUS_SET_DELAY);
				high(MKEY_PORT, MKEY_PIN);
				_delay_us(REGISTER_SET_DELAY);
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
				itoa((mouse_buttons & 0b00000111), buf, 10);
				lcd_puts(buf);
#endif // DEBUG
			}
		}

		if (ps2_state == PS2_STATE_ERROR)
		{
			// Reset PS/2 port and mouse protocol state in case of error.
			ps2_port_inited = false;
			mouse_inited = false;

			mouse_init_state = 0;
		}
	};

	return 0;
}