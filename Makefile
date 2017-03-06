# Processor frequency.
#     This will define a symbol, F_CPU, in all source code files equal to the
#     processor frequency. You can then use this symbol in your source code to
#     calculate timings. Do NOT tack on a 'UL' at the end, this will be done
#     automatically to create a 32-bit value in your source code.
#     Typical values are:
#         F_CPU =  8000000
#         F_CPU = 16000000
#         F_CPU = 20000000

# Baby orangutan works at 20 MHZ
F_CPU = 20000000

# Place -D or -U options here for C sources
CDEFS = -DF_CPU=$(F_CPU)UL

DEVICE = atmega328p
MCU = atmega328p
AVRDUDE_DEVICE = m328p
DEVICE ?= atmega168
MCU ?= atmega168
AVRDUDE_DEVICE ?= m168

CFLAGS=-g -Wall -mcall-prologues -mmcu=$(MCU) $(DEVICE_SPECIFIC_CFLAGS) -Os
CFLAGS += $(CDEFS)
LIBS = -lprintf_flt -lm
CC=avr-gcc
OBJ2HEX=avr-objcopy
#LDFLAGS=-Wl,-gc-sections -lpololu_$(DEVICE) -Wl,-relax
LDFLAGS=-Wl,-gc-sections -Wl,-relax


PORT ?= /dev/ttyACM0
AVRDUDE=avrdude

TARGET=Main
#OBJECT_FILES=rogerbot.o

# List C source files here. (C dependencies are automatically generated.)
SRC = $(TARGET).c USART_and_telemetry.c Motor.c PIDfollower.c 
# Object files directory
#     To put object files in current directory, use a dot (.), do NOT make
#     this an empty or blank macro!
OBJDIR = .
# Define all object files.
OBJECT_FILES = $(SRC:%.c=$(OBJDIR)/%.o)


all: $(TARGET).hex

clean:
	rm -f *.o *.hex *.obj *.hex

%.hex: %.obj
	$(OBJ2HEX) -R .eeprom -O ihex $< $@

test.o: test.c

%.obj: $(OBJECT_FILES)
	$(CC) $(CFLAGS) $(OBJECT_FILES) $(LDFLAGS) $(LIBS) -o $@

program: $(TARGET).hex
	$(AVRDUDE) -p $(AVRDUDE_DEVICE) -c avrisp2 -P $(PORT) -U flash:w:$(TARGET).hex
