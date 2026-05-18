# ==========================================
# UESTCHN3018 微缩车模 STM32 GCC 编译脚本
# ==========================================

# 1. 目标输出名称
TARGET = TDPS_MiniCar

# 2. 指定 GCC 交叉编译工具链
CC = arm-none-eabi-gcc
AS = arm-none-eabi-gcc -x assembler-with-cpp
CP = arm-none-eabi-objcopy
SZ = arm-none-eabi-size

# 3. 单片机内核参数 (Cortex-M3)
MCU = -mcpu=cortex-m3 -mthumb

# 4. 全局宏定义 (配合标准外设库)
C_DEFS = -DUSE_STDPERIPH_DRIVER -DSTM32F10X_MD

# 5. 头文件包含路径 (根据你的目录结构动态匹配)
C_INCLUDES = \
-IUser \
-ISystem \
-ILibrary \
-IStart

# 6. C 语言源文件 (利用 wildcard 自动遍历文件夹下的所有 .c，以后加新文件无需修改此处)
C_SOURCES = \
$(wildcard User/*.c) \
$(wildcard System/*.c) \
$(wildcard Library/*.c) \
$(wildcard Start/*.c)

# 7. 汇编源文件 (指向你刚刚放进去的 GCC 专属启动文件)
ASM_SOURCES = Start/startup_stm32f10x_md_gcc.s

# 8. 链接脚本路径
LDSCRIPT = STM32F103XB_FLASH.ld

# 9. 编译和链接参数配置
CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) -O2 -Wall -fdata-sections -ffunction-sections
LDFLAGS = $(MCU) -specs=nano.specs -specs=nosys.specs -T$(LDSCRIPT) -Wl,-Map=$(TARGET).map,--cref -Wl,--gc-sections -lm

# ==========================================
# 编译执行规则 (Build Rules)
# ==========================================
all: $(TARGET).elf $(TARGET).hex $(TARGET).bin size

# 将 .c 编译为 .o 对象文件
%.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

# 将 .s 编译为 .o 对象文件
%.o: %.s
	$(AS) -c $(CFLAGS) $< -o $@

# 链接所有 .o 文件生成 .elf 可执行文件
OBJS = $(C_SOURCES:.c=.o) $(ASM_SOURCES:.s=.o)
$(TARGET).elf: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $@

# 生成下载用的 hex 和 bin 文件
$(TARGET).hex: $(TARGET).elf
	$(CP) -O ihex $< $@

$(TARGET).bin: $(TARGET).elf
	$(CP) -O binary -S $< $@

# 打印最终占用空间大小 (Flash 和 RAM)
size: $(TARGET).elf
	$(SZ) $<

# 清理编译垃圾指令
clean:
	rm -f $(OBJS) $(TARGET).elf $(TARGET).hex $(TARGET).bin $(TARGET).map