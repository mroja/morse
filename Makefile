#name of program
NAME = morse

SOURCES = $(NAME).c

# cpu frequency
F_CPU = 16000000L

#####################################################################

.PHONY: all clean upload

MCU_TARGET = atmega32
CFLAGS = -std=gnu99 -mmcu=$(MCU_TARGET) -DF_CPU=$(F_CPU) -O2
CXXFLAGS = $(CFLAGS)
LDFLAGS = -mmcu=$(MCU_TARGET)
CC  = avr-gcc
CXX = avr-gcc
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
KAMPROG = kamprogavrc -d -f

all: $(SOURCES) $(NAME).hex upload

$(NAME): $(NAME).hex

$(NAME).elf: $(SOURCES)
	$(CC) $(CFLAGS) -o $(NAME).elf $(SOURCES)
	
$(NAME).hex: $(NAME).elf
	$(OBJCOPY) -O ihex $(NAME).elf $(NAME).hex

upload: $(NAME).hex
	$(KAMPROG) $(NAME).hex

# listing
lst: $(NAME).lst

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

clean:
	rm -f *.hex *.o *.cof *.s *.lst *.elf
