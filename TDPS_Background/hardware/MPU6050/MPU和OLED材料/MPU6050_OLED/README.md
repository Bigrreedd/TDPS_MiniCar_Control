# MPU6050 与 OLED 模块化驱动参考材料

本文件夹为从主工程 `HARDWARE/` 目录**原样复制**的参考代码，内容与工程内源文件一致，仅供查阅、移植或学习使用，不参与主工程编译。

**来源工程：** STM32F103C8（Keil MDK）  
**通信方式：** 两套独立的软件 I2C（GPIO 模拟），OLED 与 MPU6050 各用一组引脚。

---

## 目录结构

```
MPU6050_OLED/
├── README.md
└── HARDWARE/
    ├── OLED/
    │   ├── oled.c / oled.h      SSD1306 驱动（初始化、显示、软件 I2C）
    │   ├── oledfont.h           字库
    │   └── bmp.h                图片点阵
    └── MPU6050/
        ├── mpuiic.c / mpuiic.h  底层软件 I2C
        ├── mpu6050.c / mpu6050.h  寄存器读写、传感器初始化
        └── eMPL/                InvenSense DMP 姿态融合库
            ├── inv_mpu.c / inv_mpu.h
            ├── inv_mpu_dmp_motion_driver.c / .h
            ├── dmpKey.h
            └── dmpmap.h
```

---

## 硬件引脚（本工程配置）

| 模块 | 信号 | STM32 引脚 | I2C 地址 |
|------|------|-----------|----------|
| OLED (SSD1306) | SCL | **PA15** | 0x78 |
| OLED (SSD1306) | SDA | **PB12** | |
| MPU6050 | SCL | **PB8** | 0x68（AD0 接地） |
| MPU6050 | SDA | **PB9** | |

引脚宏定义位于主工程 `Core/Inc/main.h`（`OLED_SCL_Pin`、`OLED_SDA_Pin`、`SCL_6050_Pin`、`SDA_6050_Pin`），GPIO 初始化在 `Core/Src/gpio.c` 的 `MX_GPIO_Init()` 中完成。

---

## 模块分层说明

### OLED

- `oled.h` 中通过 `HAL_GPIO_WritePin` 宏实现 SCL/SDA 位操作。
- 常用接口：`OLED_Init()`、`OLED_Clear()`、`OLED_ShowString()`、`OLED_ShowNum()`、`OLED_ShowCHinese()` 等。

### MPU6050

1. **mpuiic** — I2C 起始/停止、单字节读写、ACK。
2. **mpu6050** — `MPU_Init()`、量程/采样率设置、原始加速度/陀螺仪/温度读取。
3. **eMPL (DMP)** — `mpu_dmp_init()`、`mpu_dmp_get_data(&pitch, &roll, &yaw)` 输出欧拉角。

---

## 移植到其他工程时需补充

1. 将 `HARDWARE/OLED` 与 `HARDWARE/MPU6050` 加入工程并添加头文件路径。
2. 在 `main.h`（或等价头文件）中定义上述引脚宏，并在 GPIO 初始化中配置为推挽输出。
3. 依赖 STM32 HAL（`stm32f1xx_hal.h`、`main.h`）。
4. 主程序示例（见主工程 `Core/Src/main.c`）：

```c
OLED_Init();
OLED_Clear();

while (MPU_Init() != 0);
while (mpu_dmp_init() != 0);

mpu_dmp_get_data(&pitch, &roll, &yaw);
OLED_ShowString(0, 0, (uint8_t *)"Hello", 12);
```

---

## 说明

- 代码注释中的旧引脚（如 OLED 的 PD6/PD7、MPU 的 PB10/PB11）为原始例程说明，**以本工程 `main.h` 为准**。
- 本文件夹仅作参考备份；修改驱动请回到主工程 `HARDWARE/` 目录进行。
