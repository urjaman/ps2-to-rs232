# AVR-GCC Makefile
PROJECT=ps2-to-rs232
SOURCES=main.c USI_UART.c ps2host.c timer.c ps2mouse.c msmouse_out.c
DEPS=Makefile USI_UART.h main.h ps2host.h timer.h ps2mouse.h msmouse.h
CC=avr-gcc
OBJCOPY=avr-objcopy
MMCU=attiny85
#AVRBINDIR=~/avr-tools/bin/
AVRDUDEMCU=t85
AVRDUDECMD=sudo avrdude -p $(AVRDUDEMCU) -c avrispmkII -P usb
DFLAGS=-DF_CPU=8000000
CFLAGS=-mmcu=$(MMCU) -Os -g -Wall -W -pipe -std=gnu99 -Wno-main $(DFLAGS)

$(PROJECT).hex: $(PROJECT).out
	$(AVRBINDIR)$(OBJCOPY) -j .text -j .data -O ihex $(PROJECT).out $(PROJECT).hex

$(PROJECT).out: $(SOURCES)
	$(AVRBINDIR)$(CC) $(CFLAGS) -flto -fwhole-program -flto-partition=none -mrelax -I./ -o $(PROJECT).out  $(SOURCES) -lc -lm


asm: $(SOURCES)
	$(AVRBINDIR)$(CC) -S $(CFLAGS) -I./ -o $(PROJECT).S $(SOURCES)

objdump: $(PROJECT).out
	$(AVRBINDIR)avr-objdump -xd $(PROJECT).out | less

program: $(PROJECT).hex
	cp $(PROJECT).hex /tmp && cd /tmp && $(AVRBINDIR)$(AVRDUDECMD) -U flash:w:$(PROJECT).hex


size: $(PROJECT).out
	$(AVRBINDIR)avr-size $(PROJECT).out

clean:
	-rm -f *.o
	-rm -f $(PROJECT).out
	-rm -f $(PROJECT).hex
	-rm -f $(PROJECT).S
