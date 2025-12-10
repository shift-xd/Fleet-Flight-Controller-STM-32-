CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
CFLAGS = -mcpu=cortex-m4 -mthumb -O2 -g -Wall
LDFLAGS = -T stm32f405_linker.ld -nostartfiles -Wl,-Map=firmware.map
SRCS = main.c hal_init.c mpu6000.c madgwick.c pid.c pwm_out.c uart.c
OBJS = $(SRCS:.c=.o)
TARGET = fleet-fw.elf

all: $(TARGET) $(TARGET:.elf=.bin)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) $(LDFLAGS) -o $@

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

clean:
	rm -f $(OBJS) $(TARGET) *.bin *.map

.PHONY: all clean
