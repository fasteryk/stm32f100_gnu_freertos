CC = arm-linux-gnueabi-gcc
LD = arm-linux-gnueabi-ld
AS = arm-linux-gnueabi-as
OBJDUMP = arm-linux-gnueabi-objdump
OBJCOPY = arm-linux-gnueabi-objcopy

INCLUDEPATH = -I./ -Iinclude -Iinclude/cmsis -Iinclude/stm32-stdperiph -Iinclude/freertos

CFLAGS = -mcpu=cortex-m3 -mtune=cortex-m3 -mthumb -O0 -g -DSTM32F10X_LD_VL -DUSE_STDPERIPH_DRIVER \
         -ffunction-sections -fdata-sections $(INCLUDEPATH)
LDFLAGS = -T cortex_m3.lds -Map $(TARGET).map --gc-sections 
ASFLAGS = -mcpu=cortex-m3 -mthumb -mimplicit-it=always

TARGET = pm

DIRS = ./ lib/ freertos/ system/ system/stm32-stdperiph/

OBJS := $(foreach dir,$(DIRS),$(wildcard $(dir)*.c))
OBJS := $(patsubst %.c,%.o,$(OBJS))
OBJS += system/start.o


all : $(TARGET)

$(TARGET) : $(OBJS)
	$(LD) $(LDFLAGS) -o $@ $^
	$(OBJDUMP) -t $(TARGET) > $(TARGET).tab
	$(OBJCOPY) -O binary $(TARGET) $(TARGET).bin

%.o : %.S
	$(AS) $(ASFLAGS) -o $@ -c $<
	
%.o : %.c 
	$(CC) $(CFLAGS) -o $@ -c $<

print : 
	echo $(OBJS)
		
.PHONY: clean
clean :
	@rm -f $(foreach dir,$(DIRS),$(wildcard $(dir)*.o))
	@rm -f *.bin
	@rm -f *.map
	@rm -f *.tab
	@rm -f $(TARGET)
