# ZX-BUS PS/2 Kempston Mouse Controller

![photo_1](images/photo_1.jpg)
![photo_2](images/photo_2.jpg)
![photo_3](images/photo_3.jpg)
![photo_4](images/photo_4.jpg)
![pcb_front](images/pcb_front.png)
![pcb_back](images/pcb_back.png)
![pcb](images/pcb.png)
![scheme](images/scheme.png)

## ������� ������
**V.1.0**
* ������ IORQGE/ ����� ������� 3.3V (LVTTL). ����� �� �������� ��������� � ZX-BUS XTRaiser.
* ��� ��������� ��������� ������ ���� ����� CPLD EPM3064.
* ��� ��������� �������� ����������� MX, MY, MKEY ���������������� ����������� � 0xFF, 0x80, 0x60: ������ ���� 0x80, 0x60, 0xFF (� �������� MKEY ������ ������ ���� ������).

**V.1.1**
* ������ IORQGE/ ������������� ����� 3-state buffer 74LVC1G125 �� ������ TTL (5V) ��� ����������� ��������� � ZX-BUS XTRaiser, ����� ������������ � �������������� ��������� ���� � ������ ���� ����� IO ����� CPLD. ������� ����������� � ������ � ZX-Multisound � ZXM-GeneralSound.
* �������� ������ ��� ����������� �������� ������� PS/2 (���������������� ���������� PC AT 486/Pentium ����������� ����)
* �������� ������ UART ��� �������
* ������ �������� ������� ZX-BUS ��������� � 80 �� 81 ��
* �������� ����� � CPLD ���������� �� Verilog � �������������� ��� ��������� ������ ���� �� EPM3032 ��� ������ �� ������� ������ ���� (32 macrocells). �������� ������ ��� ��������� ������ ���� (31 macrocells) � � ������ ���������� (34 macrocells).
* �� ��������� �������� � ������� ��������� Scorpion (����, �� �������� ��������� �� ROM ����� ��� �������������� ����). ����� ������ A15 � ���������� ������.

**V.1.2**
* ����������� �������� � ROM ����� � ������� Scorpion ZS 256. � ���������� ������ � ������������ ������� IORQGE �������� ������ A15.

## ������� ����������:
* ����������� ����������� ���������� ������� � ���������� ��������� PS/2 Kempston Mouse ��� ���� ZX BUS.
* ��������� �������� ������������������ / CPLD, ����� � �������� ����� ������ ���� ��������� �������.
* ������ WAIT ��������������� �� ������ ��������������.

�� ������ � ����������:
* ��������� USB ����. ����� ���������� � ����������� ��������� USB ����� � ���������������� �� ������������� ��������� ������.

## �������� ���������� Kempston Mouse
���������� �������� ��� 3 8-��������� �������� ������ � ���� X/Y MKEY, MX, MY �� ��������� ������� ������:
<pre>
#FADF 1111 1010 1101 1111 BUTTONS
#FBDF 1111 1011 1101 1111 MX
#FFDF 1111 1111 1101 1111 MY
</pre>

* ���� 0-2 �������� MKEY ������������� ��������������� ��������� �����, ������, ������� ������ ����.<br>
* ��� 3 �������� MKEY �������������� ��� ��������� ������ ���� � ������ ����� 1.<br>
* ���� 4-7 �������� MKEY ������ �������� ��� mouse wheel, ���� ��� ���������� (��������� �������� %0000). ���� ��� mouse wheel �� ����������, ������ �������� %1111.

��� ������������ 8-bit �������� ��������� ���� ���������� ������� ��������. ���������� �������� ���� 8-��� ����� �� ������ � �������������� ����.

������������ ��� mouse wheel ���� �������� ������������� � [��������� � Fido �������������� REAL SPECCY](https://groups.google.com/g/fido7.real.speccy/c/Qeid4aFhjRg)

�������������� ����������: [https://speccy.info/Kempston_Mouse](https://speccy.info/Kempston_Mouse)

## ��������� ������������ ���������� � ����������
[Original Kempston Mouse Interface (1986)](https://k1.spdns.de/Vintage/Sinclair/82/Peripherals/Mouse%20Interfaces/Kempston%20Mouse%20Interface/)

[ZX-���� 1995 #1](history/zx-review_1995-01.txt)
[[�����]](history/zx-review_1995-01.png)<br>
KEMPSTON-MOUSE<br>
(�) �.������� (�����), �.������� (���������) 1995

����� Scorpion & �.�. (�����-��������� 1995-97)<br>
[���������� IBM PC-���������� � ����](https://web.archive.org/web/20010308054936/http://scorpion.ru/spectrum/hard/kontroller_IBM.htm)<br>
[�����](history/scorpion_keyboard_mouse_controller_scheme.png) [�������� V2.5](history/scorpion_keyboard_mouse_controller_2.5_firmware.zip)<br>
���������� �� ������ � �����������: V2.1: [1](history/scorpion_keyboard_mouse_controller_2.1_1.png) [2](history/scorpion_keyboard_mouse_controller_2.1_2.png) 
V2.5: [1](history/scorpion_keyboard_mouse_controller_2.5_1.jpg) [2](history/scorpion_keyboard_mouse_controller_2.5_2.jpg)<br>
����������:
[ZX Format #01 (31 ������� 1995)](https://zxpress.ru/article.php?id=319)
[On-Line #15 (17 ������� 1995)](https://zxpress.ru/article.php?id=1989)
[On-Line #16 (24 ������� 1995)](https://zxpress.ru/article.php?id=1994)

[����� �� ������� ����������� (17 ������ 1995)](history/kempmous.txt)

��������� Kempston Mouse �� Create Soft (������ �������, Mad Max & RML) (�����-���������)<br>
[ZX-Format #5 (12 ������� 1996)](https://zxdn.narod.ru/hardware/zf5kempm.htm)
[[screenshot]](history/zx-format_5.png)<br>
����a KEMPSTON MOUSE<br>
(�) Mad Max

[Spectrofon #20 (12 ������� 1996)](https://zxpress.ru/article.php?id=4708)
[[text]](history/spectrofon_20.txt)
[[screenshot]](history/spectrofon_20.png)
[[�����]](history/spectrofon_20_kempston_mouse.png)<br>
ZX-SPECTRUM � ����<br>
(C) �������p ��p����, 1996, �����-���������

AY Mouse<br>
[ZX Power #1 (12.10.1996)](https://zxdn.narod.ru/hardware/zpw1ayms.htm)<br>
� ������ ������ �����<br>
(C) V.M.G., �������

[Optron #13 (12 ���� 1998)](https://zxdn.narod.ru/hardware/op13kemp.htm)
[[screenshot]](history/optron_13.png)<br>
Kempston mouse<br>
(C) �������, ������ ���������

[ZX Club #09 (31 ������� 1998)](https://zxpress.ru/article.php?id=14937)
[[screenshot]](history/zx-club_9.png)<br>
Kempston mouse<br>
(�) SERGIUS PUZZLER / PROXIMA

[Adventurer #11 (31 ���� 2000)](https://zxdn.narod.ru/hardware/ad11kmse.htm)
[[screenshot]](history/adventurer_11.png)<br>
�������� �� ����������� Kempston MOUSE (������� � ��������� �����)<br>
(C) �������� XN0ByS ����/CPU

[Deja Vu #0A (30 �������� 2000)](https://zxdn.narod.ru/hardware/dv0akemp.htm)
[[text]](https://zxpress.ru/article.php?id=11221)
[[screenshot]](history/dejavu_0a.png)<br>
����� Kempston Mouse Interface (v2.1) �� ��� KP580BB55A<br>
(C) Felix Knajzev

��������� ��� mouse wheel<br>
[DonNews #19 (26 ������� 2003)](https://zxpress.ru/article.php?id=6538)
[[screenshot]](history/donnews_19.png)<br>
KEMPSTON MOUSE INTERFACE<br>
(C) Bit/XXL

[����� ����������� ���� �� AT89C2051 (MSC-51)](history/kmsps2.txt)<br>
������������ � ZX-Net �������������� HARDWARE.ZX (21 Aug 2000)<br>
(C) Eugene Stahov 1999-2000<br>
��������������� AT89C1051/AT89C2051, �������� ��� �������� ������.<br>
���������� ������ WAIT ��� ������ ��������� ����������.

[PS/2 -> Kempston Mouse Adapter v2.0](history/atmelps2.txt)<br>
������������ � Fido �������������� ZX.SPECTRUM (2003-01-10)<br>
(C) 1999-2000 Studio STALL (Eugene Stahov)<br>
(C) 2003 Kamil Karimov<br>
��������������� AT90S2313 (��� AT90S1200), �������� ��� �������� ������.<br>
���������� ������ WAIT ��� ������ ��������� ����������.

[PS/2 -> Kempston Mouse Adapter v4.0](history/zxkmv40.zip)
[[�����]](history/zxkm4.png)<br>
(C)2010 Kamil Karimov<br>
�� ���������� ������ WAIT.<br>
��� ����������� ������ ����������� ��� ���������� Z80 ��������������� ATTiny2313 �������� �� ������� 20 ���.<br>
�������� ��� �������� ���������������� ������.

The Kempston Mouse Turbo Interface [2006/2008](https://velesoft.speccy.cz/kmturbo-cz.htm)/[2011](https://velesoft.speccy.cz/kmouse/km-doc/kempston_mouse_turbo_interface/km-t_2011/k-mouse2011-doc.pdf) by Velesoft and Ben Versteeg<br>
[PC Mouse and Kempston joystick Interface for the Sinclair ZX Spectrum](https://www.benophetinternet.nl/hobby/kmt/)<br>
��������� master/slave mouse, mouse wheel.<br>
��������������� PIC16F84A + CPLD XC9572X<br>
�������� ��� �������� ���������������� ������.

[Z-Controller](http://pentagon.nedopc.com/info.htm)<br>
(C) ������� ����� KingOfEvil 2007<br>
��������� ���������� � ZX Spectrum ����������� ���������� � ���� � ����������� PS/2, IDE-����������, ����� ����-������ ��������� SD. ���������� ������������� ��� ��������� � ���� ZX Bus.<br>
[������������](http://svn.zxevo.ru/filedetails.php?repname=pentevo&path=/docs/ZC.pdf)<br>
��������������� ��1878��1 + CPLD EPM7128SLC84<br>
[�������� ��� �������� CPLD � ���������������� ������ � 2024 ����](https://github.com/koe1234/Z-controller)

[ZX Evolution](http://www.nedopc.com/zxevo/zxevo.php)<br>
(C) NedoPC 2010<br>
������������ ���������� ATMEGA128<br>
PS/2 ��������� ���������� � ����<br>
[Open source](http://svn.zxevo.ru/listing.php?repname=pentevo)

[ZX_Multi_Card ZXMC1/ZXMC2](https://zx-pk.ru/threads/609-zx_multi_card.html)<br>
(C) caro Kamil Karimov 2005<br>
������������:<br>
- IBM AT ���������� - ��������� ����������� ���������� ���������;<br>
- PS/2 ����� - ��������� �������� ����� (� �������);<br>
- RS232 - ��������� �������� ������ � ���������� �� �����������.<br>
- ���� ��������� ������� (RTC)

Closed source

[����� ����������� Kempston Mouse ��� Nemo-BUS](https://zxbyte.ru/kempston_mouse_for_zx.htm) (����������� ����� �� Spectrofon #20)<br>
(C) Prusak 2016

[���������� ��� USB ���������� � ����](https://zx-pk.ru/threads/34912-kontroller-dlya-usb-klaviatury-i-myshi.html)
[[github]](https://github.com/Igor-azx987sa/ZXKM)<br>
(�) ����� ������� @azx987sa 2023<br>
��������������� CH9350<br>
Closed source

## ������ ����������:
��� ���������� ���� ������� ������ ATmega8 + EPM3032.<br>
����� ���� ����������� Kempston Mouse �� ATmega+EPM3032, [����� ��������� � ����� ����� ������ ��� ���������� ������](images/deltagon_1.5_mouse_controller.png) ���� ����� �� [Deltagon 1.5](https://t.me/Pentagon_ZX_Chat/126403) (C) ������� ������ @djking26, ����� ������� @azx987sa.

�������� ATmega ��� ��������� ��������� �������� �� ����� C. � ��������� ����� ���� ���������� �������� �� ����������� Arduino.<br>
���������� ���������� ATmega �� ������� �������� PS/2 MDATA/MCLK � ������� �� �������������� ��� ������� ���������� ����. ��� �� �����, ������� MDATA/MCLK ���������� � ������ INT1/INT0 ����������������, ���� [�������������� ��������](https://github.com/proboterror/ZX_BUS_Mouse/tree/interrupt_processing), ������������ ���������� ���������� �� ������� PS/2 clock. �������������� �������� ������������� � ������ ������� ��� �������������.

���������� ��������� ����������, ����� ���������� ������� � ������������ ������� IORQGE ��� CPLD EPM3032 ��������� �� ����� Verilog. ����� ���� ���������� ����� �������� ���������� �� VHDL.

������� ���������� ���������� 31 �� 32 macrocells � CPLD EPM3032 c ������������ ������ ��� mouse wheel � �������� ������ ����, 33 macrocells ��� ��������� ����� ��� mouse wheel.<br>
����������� �������� ������������� ����� ��������� � ����� ������� CPLD EPM3064 ��� ����������� ������ � Verilog � ������������� �������� � CPLD.<br>
�� ��������� �������� ��������� ���� ������ � mouse wheel, ������������ 32 macrocells.

��� ������ ������� �� ���������� �� ������� ������� ������ ������� �� PS/2 ���� � �������� MKEY, MX, MY ������ ������������ �������� 0xFF, 0x80, 0x60.<br>
��� ����������� ��������� ��� mouse wheel � ������� 4 ����� �������� MKEY %1111, ��� ���������� ��������� ��������� �������� ��� %0000.

������������ ������� IORQGE/ ���������� ������ �� ����� ����� ������� ������ A0,A1,A7,A5/ � ������� M1/.

������� ���������� ����������� �� [Pentagon 2024](https://t.me/Pentagon_ZX_Chat/125656) � ��������� � ���������� Unreal Speccy 0.39.0, ��������� ��� �������� ����������.
������������ ������ �� Scorpion ZS 256, Scorpion ZS 1024, Pentagon 1024ZP Crocodile Edition.

�������� ������������� ����� ������������ � CPLD:
![scheme-decompose](images/scheme-decompose.png)

## ��������� ��������:
�� Scorpion ZS 256 � ���� ���������� �������� ���� �������� �����������, ��� ���� ����, ����������� � RAM, �������� ���������.
�� �������� �������� � ROM ����� � ������� ��� ���������� ����� ����������� �������������.
��������������� ����������� ���������� ������� A15 � ����� ����������, ������������������ ���������� ����� ���� ���. ����� �������� ������������ �������������� ������� /DOS.

���� �������� ������������ ������ ��������� ���� � CPLD: ��������� ��� ������������ ���� �����. �������� ��������������� �������� ����� � ��������� CPLD.

�� ��������� ������ � ������������ ����������� IORQGE ������������ �������� �������� � ������ Kempston Joystick.

## �����������:
PS/2 ���� ����� ��������� �������� ����� �������������� ����� ������ �������. ������ ������������� ����������� ����� 2 ������ �������� ������������� ���������� PS/2.

Gluk Reset Service ��������� ������� ���� ����� ����� ��������� (24 ms ��� Pentagon 2024: 10K+10uF � ���� ������ Z80: ������������ RESET 10 ms).<br>
��������:<br>
- � ����� ������� ������������ �����������, ��� �������� ��� X �� ��������
- �����������, ��� ������� KEY �� ����� �������� X
- �����������, ��� ������� 3 ���� �������� KEY ����� 1

��������:<br>
���� ��� ��������� ������������� �� ������������, �� ������������ ����� ������. ����� ������������� �������� ���� ����� ������� [8].<br>
���� �������� � ���, ��� ��������������� ATmega �� �������� ������������������� PS/2 ���� (1-2 s) � ��������� �������� ��������� ����������� �� ��������� (~6 us).<br>

�������� �������:<br>
- ��� ����������� ������� �������������� � ����������� ���� ��������� �� 1-2 ������, � ������� ������� ������ /RESET Z80 ������������ ������������, ��� ��� �������� ������� � ����������� ZXMC.
- ���������������� �������� ����������� ����� ����� ���������, �� ��������� �������� ������� PS/2 ����. ��� ��� ����� ��������� ������� �������� ������ Z80. ��� �� ��������� Gluk ���� ������� ���� ����� ���������, ���� ���� PS/2 ���� �� ������������.

� ���� ���������� ������ �������.

## Bill of materials
|Ref|Qnty|Value|Cmp name|Footprint|Description|
|-----|-----|-----|-----|-----|-----|
|C1, C2, C3, C4, C5, C6, C7, C8|8|0.1u|C|Capacitor_SMD:C_1206_3216Metric|Unpolarized capacitor|
|C9|1|10u|C_Polarized|Capacitor_Tantalum_SMD:CP_EIA-6032-28_Kemet-C_Pad2.25x2.35mm|Polarized capacitor|
|C10|1|22u|C_Polarized|Capacitor_Tantalum_SMD:CP_EIA-6032-28_Kemet-C_Pad2.25x2.35mm|Polarized capacitor|
|J4|1|Mini-DIN-6|Mini-DIN-6|Connector_Mini-DIN_Female_6Pin_2rows|6-pin Mini-DIN connector|
|J5|1|Conn_02x03_Odd_Even ISP|Conn_02x03_Odd_Even|Connector_PinHeader_2x03_P2.54mm_Vertical||
|J7|1|Conn_02x05_Odd_Even JTAG|Conn_02x05_Odd_Even|Connector_IDC-Header_2x05_P2.54mm_Vertical||
|R1|1|2.2K|R|Resistor_SMD:R_1206_3216Metric|Resistor|
|R2, R3|2|4.7K|R|Resistor_SMD:R_1206_3216Metric|Resistor|
|R4, R5, R6, R7|4|10K|R|Resistor_SMD:R_1206_3216Metric|Resistor|
|U1|1|ATmega8A-U|ATmega8A-U|Package_QFP:TQFP-32_7x7mm_P0.8mm|16MHz, 8kB Flash, 1kB SRAM, 512B EEPROM, TQFP-32|
|U2|1|EPM3032ALC44|EPM3032ALC44|Package_LCC:PLCC-44_THT-Socket|CPLD MAX3000A MCs:32 IOs:34 [PLCC-44]|
|U3|1|AMS1117-3.3|AMS1117-3.3|Package_TO_SOT_SMD:SOT-223-3_TabPin2|1A Low Dropout regulator, positive, 3.3V fixed output, SOT-223|
|U4|1|74LVC1G125DB|74LVC1G125|Package_TO_SOT_SMD:SOT-23-5|Single Buffer Gate Tri-State, Low-Voltage CMOS|

IC Socket PLCC-44 THT DIP

������ J4 PS/2 ������� � ������� ������.<br>
������� �������������� J5, J7 - ����� pin header 2x03, 2x05 2.54mm. ��������� �� �� �����������.<br>
������������ C9, C10 ��������� ��������������� ���������� ���������� �������� [datasheet](http://www.advanced-monolithic.com/pdf/ds1117.pdf).

���������� ����������:
|Ref|Qnty|Value|Cmp name|Footprint|Description|
|-----|-----|-----|-----|-----|-----|
|R8, R9|2|2.2K|R|Resistor_SMD:R_1206_3216Metric|Resistor|
|J6|1|Conn_01x04 Debug|Conn_01x04|Connector_PinHeader_1x04_P2.54mm_Vertical||

## ������ ��������

### ATmega8
����� ���������� AVR SDK: WinAVR-20100110<br>
����� ��� GNU ������ ����� ������������� ���������� [MSYS2](https://www.msys2.org/). ���� � msys64/usr/bin ������ ���� � PATH.<br>

Compile:
<pre>
make clean
make all
</pre>

[��������� ��������](source/ATmega/main.hex) � ���������� ���������� mouse wheel � ����������� ���������� ������� �� OLED �������.

### Altera EPM3032
��� ������ � ������� �������� CPLD ���������� ���������� ������ �� Quartus II Web Edition ������ 9.1 SP2 / 13.0 SP1<br>
������������������ ���������� � ��������:<br>
- File/Open Project<br>
- Processing/Start Compilation<br>
- Tools/Programmer<br>
- ��� ������������� ������� ������������ USB Blaster ����� Hardware Setup<br>
- ��� ����� ���������������� �������� mouse_controller.pof ������� Program/Configure � Verify.<br>
- Start.

## ������� ��������

### ATmega8
��� �������� ATmega ����� ������������ ������������ CH341A � �� NeoProgrammer V2.2.<br>
������������������ ��������:<br>
- ���������, ��� ��������������� �������� � ������������ ����� SPI: ������ Detect.<br>
- ���� ���������� �� ������������, �� �������� ������ ���������� �������� � ��������������, ���������� �������� � ������� ISP, ������ ������, � ���������������� ����� ���� ������� fuses ��� ������������� �������� ���������� ����������.<br>
- ��������� Fuses ����� ��������������� ������, ������ Write.

[Fuses ��� �������� ATmega8:](source/ATmega/fuses.png)
<pre>
Low     High
0xC4    0xD9
</pre>

- ������� ���� .hex �������� ����� Open File.<br>
- ����� Write IC ������� Erase/Write/Verify.

��� ��������� �������� �������� �������� ISP � JTAG ����� �� ����������, � ������� ��� ����� �� ����� ��������.

### Altera EPM3032
��� �������� �� JTAG ����� ���� ����������� ������� ������ Altera USB Blaster.<br>
��������� ����� ������� ������� USB Blaster �� FT245+CPLD �������� ����� ����������  � ��������� � ������ (�� �����������).

��������� ������� ������� Altera USB Blaster Rev.c �� CH552G ������� ��������������, ����� ���������� �������� �������������.<br>
��������� WCHISPTool � �������� [CH552_Blaster_v22.2.27.hex](https://github.com/xjtuecho/CH552Nano/blob/main/FW/CH552_Blaster_v22.2.27.hex)<br>
���������: [�������� ���������� USB-Blaster �� ������ CH552G](https://ua3reo.ru/proshivka-kitajskogo-usb-blaster-na-osnove-ch552g/)<br>
������� ������������� ��������������� �� ����� ��������� quartus/drivers/usb-blaster

����� ������� � ����� ���� ����������� ������ CPLD � ������� JTAG, � ������ ������� ��������������� ������� 5V=>3.3V, ��� ������ ������� 3.3V �� ������ JTAG � ������������� ���������� ������� ������ �� ~2V.<br>
CPLD ������������ � Quartus, �� �� �����������.<br>
����� ������ �� ����� ������� ������� 5V ����� ������ ISP ��� ���� ZX-BUS.<br>
�� �������� ��� ������� ����� ������� ������������ �� ���� �� ���������� Pentagon 2024.<br>
��������� ������ ������������ ��� ����������� �������������� ����� USB, ���������� ��������� �� ����������� ����������.

������ � ������������ ���� EPM3032 ����� ���� �������� � �� ����� ����� JTAG ���������. ����� [�������� ����� � ��������� unlock](https://zx-pk.ru/threads/25998-programmator-mikroskhem-programmiruemoj-logiki(cpld).html?p=1043136&viewfull=1#post1043136), ��� �������� � ������� ����������.

### ��������� mouse wheel
��������� mouse wheel � �������� ������ ����������� �������� �� ��������� � ����������� ������� ������� ����. ��� ���������� ���������� 32 macrocells � ���������� � EPM3032 (source/Altera-VHDL/mouse_controller.v):
```
	// Example 2 buttons mouse support with mouse wheel (32 macrocells)
	// Exclude 3rd mouse button and reserved bit from output in order to fit to EPM3032.
	assign D = MKEY_SEL ? {register_key[7:4], 2'b11, register_key[1:0]} : 8'hZ;
```
��� ��������� ������� ������ ���� ��� ������ ��������� �������� ��:
```
	// Example 3 buttons mouse support with mouse wheel turned off (31 macrocells)
	assign D = MKEY_SEL ? {4'b1111, register_key[3:0]} : 8'hZ;
```
��� ������ ��������� �������� ��:
```
	// Example 3 buttons mouse support with mouse wheel (34 macrocells)
	assign D = MKEY_SEL ? register_key : 8'hZ;
```
������� ����������� ������� EPM3064.

������ � EPM3064 ������������.

## �������
�� ��� ������������ ����������: [MousTest (C) 2012 Patrik Rak - Raxoft](test/Test%20Mouse.trd.zip)

��� ������� ���������� PS/2 ����� ������������ OLED I2C Display 128x32. ������ ���� ������� ��������� �������� I2C SDA/SCL R8, R9.<br>
���������� ������� �� �����, ���������� �������� � ������� I2C ��� ����������� ����� �������.<br>
� Makefile ����� �������� ���������� ��� � ����������� � ����������� [OLED for AVR mikrocontrollers](https://github.com/Sylaina/oled-display):
```c
CFLAGS += -DDEBUG=1
```
����� ��������� ���� ������������� ���������� PS/2, ��� ����������, ��������� ����, ������ � ������ ����.

�������� ������������� � ������ PS/2 ���� ����������������� ATmega8:<br>
������� 1:<br>
- �������� OLED ������� 128x132 � ������ I2C.
- ������� ���������� �������� [ZX_BUS_Mouse_ATmega8_Fuses_C4_D9_Debug_OLED_128x32.hex](source/ATmega/Debug/ZX_BUS_Mouse_ATmega8_Fuses_C4_D9_Debug_OLED_128x32.hex)

��� ��������� ��������� ������� Init... Done � ����� ��� �������� ���� �� ���������� � ��������� ��� wheel / ������.

������� 2:<br>
- ������������ ��������� � ��������� ������� I2C GND � SDA. �������� ��� ���������� � GND.
- ������� ���������� �������� [ZX_BUS_Mouse_ATmega8_Fuses_C4_D9_Debug_LED_SDA_GND.hex](source/ATmega/Debug/ZX_BUS_Mouse_ATmega8_Fuses_C4_D9_Debug_LED_SDA_GND.hex)

��� ��������� ����� �������� ������������� PS/2 ���� ��������� ����������. ��� ������� ������ ������.

## Credits
* ����� ���� ����������� Kempston Mouse �� ������ ATmega+EPM3032, ����� ��������� � ����� ����� ������ ��� ���������� ������: [Deltagon 1.5](https://t.me/Pentagon_ZX_Chat/126403) (C) ������� ������ @djking26, ����� ������� @azx987sa 
* @Alex_Eburg - [ZX BUS PCB Prototype Board](https://github.com/AlexEkb4ever/ZX_BUS)
* Black_Cat - [BC Info Guide #7. �������������� ZXBus ����������� � ���.](https://zx.clan.su/forum/7-82-1) (c) Black_Cat 2011-2020
* Adam Chapweske - PS/2 Mouse/Keyboard Protocol (C) 1999, The PS/2 Mouse Interface (C) 2003
* Kris Chambers - [Arduino/Wiring Library for interfacing with a PS2 mouse](https://github.com/kristopher/PS2-Mouse-Arduino)
* [Patrik Rak raxoft](https://github.com/raxoft) - MousTest
* Sylaina - [OLED for AVR mikrocontrollers](https://github.com/Sylaina/oled-display) library
* Velesoft - [RUSKA VERZE KEMPSTON MOUSE](https://velesoft.speccy.cz/kmrus-cz.htm) - ��������� ������ �� ����������� ��������
* Zxdn - [�����������/����� ��������� - ����������, ����](https://zxdn.narod.ru/hardware.htm#inputdev)
* ������� ��������/newart - [zxpress.ru](https://zxpress.ru)
* @AMM - ��������� � ��������� �������
* @Michael_Kaa - ������ � ������������ �� Scorpion ZS 256
* Denis (����� �������) - ������������ � ������� �� Scorpion ZS 256
* @djking26 - ����������� �������� � Gluk Reset Service