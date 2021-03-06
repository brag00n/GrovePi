#!/usr/bin/env bash
update_grovepi_firmware(){
	avrdude -c gpio -p m328p -U lfuse:w:0xFF:m
	avrdude -c gpio -p m328p -U hfuse:w:0xDA:m
	avrdude -c gpio -p m328p -U efuse:w:0x05:m
	avrdude -c gpio -p m328p -U flash:w:grove_pi_v1_3_0_OCO_MPU.ino.hex
}
