# ATmega128 스펙트럼 녹음기 빌드 (avr-gcc)
MCU     = atmega128
F_CPU   = 16000000UL
TARGET  = spectrum_recorder

SRC     = src/main.c src/diskio.c \
	      lib/fatfs/ff.c lib/fatfs/ffsystem.c lib/fatfs/ffunicode.c

CC      = avr-gcc
OBJCOPY = avr-objcopy
CFLAGS  = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -std=gnu99 -Wall \
	      -ffunction-sections -fdata-sections -Isrc -Ilib/fatfs
LDFLAGS = -mmcu=$(MCU) -Wl,--gc-sections
LIBS    = -lm

BUILD   = build
OBJ     = $(patsubst %.c,$(BUILD)/%.o,$(SRC))

all: $(BUILD)/$(TARGET).hex size

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJ)
	$(CC) $(LDFLAGS) $^ -o $@ $(LIBS)

$(BUILD)/$(TARGET).hex: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

size: $(BUILD)/$(TARGET).elf
	avr-size --mcu=$(MCU) -C $<

# 예: make flash PORT=/dev/ttyUSB0 (ISP 장비에 맞게 PROGRAMMER 수정)
PROGRAMMER ?= avrisp2
PORT       ?= usb
flash: $(BUILD)/$(TARGET).hex
	avrdude -c $(PROGRAMMER) -P $(PORT) -p m128 -U flash:w:$<:i

clean:
	rm -rf $(BUILD)

.PHONY: all size flash clean
