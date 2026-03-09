/* STM32F103C8T6 / STM32F103R8T6 Linker Script for GCC */
/* Flash: 64KB, SRAM: 20KB */

/* 最高可用 RAM 地址，用于设置初始栈指针 */
_estack = ORIGIN(RAM) + LENGTH(RAM);    /* end of RAM */

/* 规定堆 (Heap) 和栈 (Stack) 的最小保留大小 */
_Min_Heap_Size = 0x200;      /* required amount of heap  */
_Min_Stack_Size = 0x400;     /* required amount of stack */

/* 物理内存映射定义 */
MEMORY
{
RAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 20K
FLASH (rx)     : ORIGIN = 0x08000000, LENGTH = 64K
}

/* 输出段 (Sections) 布局定义 */
SECTIONS
{
  /* 中断向量表放在 Flash 的最前面 */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* 确保不被编译器优化掉 */
    . = ALIGN(4);
  } >FLASH

  /* 核心程序代码和只读数据 (如 const 变量) 放在 Flash */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >FLASH

  /* 记录初始化数据在 Flash 中的起始地址 */
  _sidata = LOADADDR(.data);

  /* 已初始化的全局变量和静态变量，运行在 RAM，但初始值存在 Flash 中 */
  .data : 
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */
    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
  } >RAM AT> FLASH

  /* 未初始化的全局变量 (默认清零)，直接分配在 RAM 中 */
  .bss :
  {
    . = ALIGN(4);
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAM

  /* 校验堆栈空间是否足够 */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM
}