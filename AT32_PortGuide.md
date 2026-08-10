# OpenIMU AT32 集成移植指南（AT32_PortGuide）

本文档详细描述将 **OpenIMU BNO086 协议驱动（libOpenIMU）** 集成到 AT32（ArteryTek Cortex-M4F）工程中的完整步骤。**参考实现**为 `Project/bsp/libOpenIMU_portable.c` / `libOpenIMU_portable.h`（本工程 MC1507 的 AT32 移植层）。

---

## 1. 概述

- **libOpenIMU** 是 OpenIMU BNO086 模组（UART6，3Mbps）的协议驱动，用纯 C 编写，**不含任何平台相关代码**，可移植到任意 MCU/RTOS。
- 平台能力通过函数指针结构体 **`libOpenIMU_IO`** 注入（时间、串口读写等）。
- 运行状态 `libOpenIMU_TypeDef` 实例由**移植层**持有，通过 `libOpenIMU_Init( io, inst, uploadFormat )` 传入协议逻辑（含上传格式）。
- 本工程 AT32 移植层位于 `Project/bsp/libOpenIMU_portable.c`，是移植到其它 AT32 工程的**直接参考模板**。

### 目录结构

| 文件 | 作用 | 平台相关？ |
|------|------|-----------|
| `libOpenIMU/include/libopenimu.h` | 公共头：`libOpenIMU_Frame`、`XFPK_Type`、`libOpenIMU_State`、`libOpenIMU_IO`、`libOpenIMU_TypeDef`、API | 否 |
| `libOpenIMU/src/libopenimu.c` | 协议逻辑：初始化状态机、AT 指令、帧解析、超时重试 | **否（纯算法）** |
| `Project/bsp/libOpenIMU_portable.h` | 移植层接口（`libOpenIMU_Portable_Init`） | 是 |
| `Project/bsp/libOpenIMU_portable.c` | 移植层实现：实例化 `libOpenIMU_IO` + 状态，实现平台函数 | 是 |

---

## 2. 平台抽象接口（libOpenIMU_IO）

`libOpenIMU` 只通过 `libOpenIMU_IO` 访问平台，移植时**必须**实现以下成员：

```c
typedef struct
{
    uint32_t ( *getTickMs )( void );                     /* 获取当前毫秒（支持 32 位回绕） */
    uint32_t ( *getUs )( void );                         /* 获取当前微秒（自由运行计数器原始值） */
    uint32_t ( *rxAvailable )( void );                   /* 串口可读字节数 */
    uint32_t ( *read )( uint8_t *pBuf, uint32_t len );   /* 串口读，返回实际读取字节数 */
    uint32_t ( *write )( const uint8_t *pData, uint32_t len ); /* 串口写 */
    uint32_t fullCycleUs;                                /* 微秒计数器满周期（us），用于回绕计算 */
    XFPK_Type ( *getXfpkType )( void );                  /* 读取节点配置的算法滤波类型（可选） */
}libOpenIMU_IO;
```

| 成员 | 语义要求 | AT32 参考实现（`libOpenIMU_portable.c`） |
|------|----------|------------------------------------------|
| `getTickMs` | 返回毫秒计数；用于状态机超时/启动延时 | `HAL_GetTick()` |
| `getUs` | 返回**自由运行计数器原始值**（1MHz）；用于帧等待 3ms 超时 | `(uint32_t)MAIN_TMR->CNT`（TMR3） |
| `fullCycleUs` | 微秒计数器满周期（us），做 16 位回绕修正 | `(uint32_t)MAIN_TMR_DESC.fullCycleUs`（= 65536） |
| `rxAvailable` | 返回接收缓冲当前可读字节数 | `UART6_RxAvailable()` |
| `read` | 从接收缓冲读取最多 len 字节，返回实际读取数 | `UART6_Read( pBuf, len )` |
| `write` | 发送 len 字节 | `UART6_SendFrame( pData, len )` |
| `getXfpkType` | 返回算法滤波类型（`XFPK_Base`/`XFPK_Additional`）；不需要时返回 `XFPK_Additional` | `StaConfig_getXfpkType()`（节点配置缓存） |

> ⚠️ `getUs()` 返回的是**计数器原始值**，不是"自系统启动以来的微秒数"；回绕由协议逻辑用 `fullCycleUs` 修正。

---

## 3. 集成步骤（以 IAR EWARM 为例）

### 3.1 添加源码与头文件路径

把 `libOpenIMU` 目录复制到工程内（或加入源码路径），并在工程中：

1. **加入源文件**（编译列表）：
   - `libOpenIMU/src/libopenimu.c`（协议逻辑）
   - 你的移植层 `libOpenIMU_portable.c`（参考本工程）

   本工程 `.ewp` 对应条目：
   ```xml
   <file>
       <name>$PROJ_DIR$\..\..\libOpenIMU\src\libopenimu.c</name>
   </file>
   <file>
       <name>$PROJ_DIR$\..\bsp\libOpenIMU_portable.c</name>
   </file>
   ```

2. **加入头文件搜索路径**：
   ```xml
   <state>$PROJ_DIR$\..\..\libOpenIMU\include</state>
   ```
   （`libOpenIMU_portable.h` 通常放在你的 bsp 目录，与其它 bsp 头同路径即可。）

3. `libOpenIMU` 仅依赖标准 C 头（`stdint.h`/`stdbool.h`/`string.h` 等），**不依赖** AT32 头文件，因此跨平台复制时无需改动 `libopenimu.c`/`libopenimu.h`。

### 3.2 实现移植层（新建 `libOpenIMU_portable.c/.h`）

参考 `Project/bsp/libOpenIMU_portable.c`，模板如下：

```c
/* libOpenIMU_portable.h */
void libOpenIMU_Portable_Init( libOpenIMU_UploadFormat uploadFormat );  /* 绑定 IO + 状态并调用 libOpenIMU_Init */
```

```c
/* libOpenIMU_portable.c */
#include "config.h"          /* 含 MAIN_TMR 宏（如需要） */
#include "uart.h"            /* 你的串口驱动 */
#include "timer.h"           /* 含 MAIN_TMR_DESC（如需要） */
#include "at32f4xx.h"        /* HAL_GetTick 等 */
#include "libopenimu.h"

/* 平台函数 */
static uint32_t libOpenIMU_Portable_GetTickMs( void ) { return HAL_GetTick(); }
static uint32_t libOpenIMU_Portable_GetUs( void )     { return (uint32_t)MAIN_TMR->CNT; }
static uint32_t libOpenIMU_Portable_RxAvailable( void ) { return UART6_RxAvailable(); }
static uint32_t libOpenIMU_Portable_Read( uint8_t *pBuf, uint32_t len )
{ return UART6_Read( pBuf, len ); }
static uint32_t libOpenIMU_Portable_Write( const uint8_t *pData, uint32_t len )
{ return UART6_SendFrame( pData, len ); }
static XFPK_Type libOpenIMU_Portable_GetXfpkType( void )
{ return XFPK_Additional; }   /* 有节点配置时改为 StaConfig_getXfpkType() */

/* 实例化 IO（fullCycleUs 在 Init 时填充，因 MAIN_TMR_DESC.fullCycleUs 非编译期常量） */
static libOpenIMU_IO gLibOpenIMU_IO =
{
    .getTickMs   = libOpenIMU_Portable_GetTickMs,
    .getUs       = libOpenIMU_Portable_GetUs,
    .rxAvailable = libOpenIMU_Portable_RxAvailable,
    .read        = libOpenIMU_Portable_Read,
    .write       = libOpenIMU_Portable_Write,
    .getXfpkType = libOpenIMU_Portable_GetXfpkType,
};
static libOpenIMU_TypeDef gLibOpenIMU;

void libOpenIMU_Portable_Init( libOpenIMU_UploadFormat uploadFormat )
{
    gLibOpenIMU_IO.fullCycleUs = (uint32_t)MAIN_TMR_DESC.fullCycleUs;
    libOpenIMU_Init( &gLibOpenIMU_IO, &gLibOpenIMU, uploadFormat );
}
```

> 💡 上传格式（`LIBOPENIMU_UPLOAD_FORMAT_STRING` / `LIBOPENIMU_UPLOAD_FORMAT_HEX`）不再用编译期宏写死，而是作为 `libOpenIMU_Init` 的参数传入，同一份固件可在运行时选择字符串或二进制解析。

> 💡 `fullCycleUs` **不能**放在静态初始化器里（`MAIN_TMR_DESC.fullCycleUs` 是全局数组的运行时值，IAR 报 `Error[Pe028]`），必须在 `Init()` 运行时赋值——这就是参考实现放在 `libOpenIMU_Portable_Init()` 中的原因。

### 3.3 初始化与周期轮询

1. **上电后**（对应本工程 `main.c` 的 IMUSample_task 初始化段）调用一次：
   ```c
   libOpenIMU_Portable_Init( LIBOPENIMU_UPLOAD_FORMAT_HEX );
   ```
   - 内部完成：`fullCycleUs` 填充 → `libOpenIMU_Init( &gLibOpenIMU_IO, &gLibOpenIMU, uploadFormat )`
   - `libOpenIMU_Init` 会读取 `getXfpkType()` 存到 `algFilterType`、保存 `uploadFormat`，并把状态机复位到 `LIBOPENIMU_STATE_INIT`（等待模组启动 ~600ms 后开始自动配置）。

2. **周期任务中**反复调用（本工程在 IMUSample_task 主循环）：
   ```c
   libOpenIMU_Poll();
   ```
   - 内部是**非阻塞状态机**：开机自动完成 config 模式 → LED 关闭 → 设置滤波类型 → 设置上传格式 → 校验 → 进入 requestMeasurement 稳态；
   - 稳态下每次 `Poll` 发送 `AT+requestFrame` 并等待 ≤3ms 解析一帧。
   - 无需额外延时/定时器调度，直接放进你的采样循环即可。

### 3.4 读取数据

```c
libOpenIMU_Frame frame;
if ( libOpenIMU_GetFrame( &frame ) )
{
    /* frame.quat_wxyz[4] / accel_g[3] / gyro_dps[3] / mag_uT[3] */
    /* frame.timestampMs 为收到该帧的 ms 时间戳 */
}
/* 调试打印：libOpenIMU_PrintFrame(); */
```

- `libOpenIMU_GetFrame` 返回 `true` 表示取到最新有效帧（拷贝语义）；无有效帧返回 `false`。

---

## 4. 串口要求（UART6）

| 项 | 要求 | 本工程参考 |
|----|------|-----------|
| 外设 | 任意可用的 UART | USART6（PA4=TX，PA5=RX） |
| 波特率 | **3 Mbps** | `UART6_SetBR` |
| 接收 | 必须提供 `rxAvailable`/`read`；推荐 DMA 循环 + IDLE 截帧入环形缓冲，避免丢字节 | lwrb 环形缓冲（256B） |
| 发送 | 必须提供 `write`；推荐 DMA/中断发送 | lwrb_tx + DMA1_Ch1 |
| 语义 | `read` 返回实际读取字节数；接收侧不可丢帧（模组响应为 `\r\n<data>\r\n\r\nOK\r\n`，二进制帧可能含 `0x0A`） | `UART6_Read` |

> 详见 `openspec/specs/uart6-dma-rx` 与 `libopenimu.c` 的行解析逻辑（`libOpenIMU_RxReadLine`）。

---

## 5. 时间要求

| 项 | 要求 | 参考 |
|----|------|------|
| 毫秒 | 1ms tick，支持 32 位回绕 | `HAL_GetTick()` |
| 微秒 | **1MHz 自由运行计数器**，读原始值 | `MAIN_TMR`（TMR3）→ `CNT` |
| 满周期 | 计数器回绕周期（us）；16 位 1MHz → 65536 | `MAIN_TMR_DESC.fullCycleUs` |

如果目标平台没有独立的 1MHz 计数器，可用任意自由运行的定时器替代，只要：
- `getUs()` 返回计数原始值；
- `fullCycleUs` 填对应满周期。

---

## 6. 可配置项

| 配置 | 位置 | 说明 |
|------|------|------|
| 上传格式 | `libOpenIMU_Init`/`libOpenIMU_Portable_Init` 的 `uploadFormat` 参数（`LIBOPENIMU_UPLOAD_FORMAT_STRING` / `LIBOPENIMU_UPLOAD_FORMAT_HEX`） | 字符串 13 浮点 CSV，或二进制 52 字节 JustFloat；运行时选择 |
| 滤波类型 | `getXfpkType()` 返回值 | `XFPK_Base`（游戏旋转矢量）/ `XFPK_Additional`（标准，默认）；配置阶段发 `AT+CONFIG=algFilterType,<value>` |
| 状态 LED | 自动 | config 模式发 `AT+SETLED=OFF` 关闭 |
| 调试打印 | `LIBOPENIMU_DEBUG_PRINT`（`libopenimu.c`） | 置 1 打印原始 RX/响应行/超时状态 |

---

## 7. 协议状态机（简介）

初始化自动流程（`libOpenIMU_State`）：

```
INIT(等待启动600ms)
  → SET_CONFIG_MODE    (AT+MODE=config)
  → SET_LED_OFF        (AT+SETLED=OFF)
  → SET_ALG_FILTER     (AT+CONFIG=algFilterType,<value>)
  → SET_UPLOADFORMAT   (AT+UPLOADFORMAT=...)
  → VERIFY_UPLOADFORMAT(AT+UPLOADFORMAT=? 校验)
  → SET_REQUEST_MEASUREMENT (AT+MODE=requestMeasurement)
  → MEASUREMENT(稳态：AT+requestFrame 逐帧请求并解析)
```

- 每个状态等待 `OK`，超时按重试策略重发；超过上限记录错误并重新初始化，**不阻塞任务**。

---

## 8. 常见问题（FAQ）

| 现象 | 原因/排查 |
|------|-----------|
| 一直 `cmd ERROR, retry.` / `init retry exceeded` | 波特率不是 3Mbps、接线反/断、模组未上电或处于测量模式 |
| 一直超时无响应 | 检查 `rxAvailable`/`read` 是否真的把数据读出来了（先开 `LIBOPENIMU_DEBUG_PRINT` 看原始 RX） |
| 二进制帧解析乱 | 确保 `libOpenIMU_Init` 的 `uploadFormat` 与模组实际 `AT+UPLOADFORMAT` 一致（HEX 走固定帧长 52B，不走行解析） |
| 编译报 `fullCycleUs` 非常量 | 不能在静态初始化器中赋值，改到 `Init()` 运行时填充 |
| 换 MCU 后代码要改？ | 不需要——只需重写移植层 `libOpenIMU_portable.c`（实现 `libOpenIMU_IO` 即可） |

---

## 9. 相关文件索引

| 位置 | 内容 |
|------|------|
| `libOpenIMU/include/libopenimu.h` | 公共头/API/类型定义 |
| `libOpenIMU/src/libopenimu.c` | 协议逻辑（纯算法） |
| `Project/bsp/libOpenIMU_portable.c/.h` | **AT32 参考移植层** |
| `Project/main.c`（约 939/958 行） | `libOpenIMU_Portable_Init()` 与 `libOpenIMU_Poll()` 调用点 |
| `Project/bsp/uart.c/.h` | UART6 驱动（`UART6_*`） |
| `Project/bsp/timer.c/.h`、`config.h` | `MAIN_TMR`/`MAIN_TMR_DESC` |
| `Project/Proj/ATMC1507APP_OS.ewp` | IAR 工程（include 路径 + 源文件条目） |

---

*注：本文档与 `openspec/specs/openimu-uart6`、`openspec/specs/libopenimu-portable` 规格对应；移植时以 `libOpenIMU` 公共头与参考移植层为准。*
