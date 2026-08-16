# OpenIMU AT32 集成移植指南（AT32_PortGuide）

本文档详细描述将 **OpenIMU BNO086 协议驱动（libOpenIMU）** 集成到 AT32（ArteryTek Cortex-M4F）工程中的完整步骤。**参考实现**为 `Project/bsp/libOpenIMU_portable.c` / `libOpenIMU_portable.h`（本工程 MC1507 的 AT32 移植层）。

---

## 1. 概述

- **libOpenIMU** 是 OpenIMU BNO086 模组（UART6，3Mbps）的协议驱动，用纯 C 编写，**不含任何平台相关代码**，可移植到任意 MCU/RTOS。
- 平台能力通过函数指针结构体 **`libOpenIMU_IO`** 注入（时间、串口读写、波特率切换、电源控制等）。
- 运行状态 `libOpenIMU_TypeDef` 实例由**移植层**持有，通过 `libOpenIMU_Init( io, inst, uploadFormat, IMU_rawType, targetBaud )` 传入协议逻辑（含上传格式、上传内容组合与目标波特率）。
- 本工程 AT32 移植层位于 `Project/bsp/libOpenIMU_portable.c`，是移植到其它 AT32 工程的**直接参考模板**。

### 目录结构

| 文件 | 作用 | 平台相关？ |
|------|------|-----------|
| `libOpenIMU/include/libopenimu.h` | 公共头：`libOpenIMU_Frame`、`XFPK_Type`、`IMU_rawType`、`libOpenIMU_State`、`libOpenIMU_IO`、`libOpenIMU_TypeDef`、API | 否 |
| `libOpenIMU/src/libopenimu.c` | 协议逻辑：初始化状态机（含波特率探测）、AT 指令、帧解析、超时重试 | **否（纯算法）** |
| `Project/bsp/libOpenIMU_portable.h` | 移植层接口（`libOpenIMU_Portable_Init`） | 是 |
| `Project/bsp/libOpenIMU_portable.c` | 移植层实现：实例化 `libOpenIMU_IO` + 状态，实现平台函数 | 是 |

---

## 2. 平台抽象接口（libOpenIMU_IO）

`libOpenIMU` 只通过 `libOpenIMU_IO` 访问平台。**必须**实现的成员：时间（`getTickMs`/`getUs`/`fullCycleUs`/`delayMs`）与串口（`rxAvailable`/`read`/`write`）；**可选**成员：`setBaudrate`（波特率探测）、`powerOn`/`powerOff`（电源控制）、`bootInitDelayMs`（上电等待）、`getXfpkType`（滤波类型）。

```c
typedef struct
{
    uint32_t ( *getTickMs )( void );                     /* 获取当前毫秒（支持 32 位回绕） */
    uint32_t ( *getUs )( void );                         /* 获取当前微秒（自由运行计数器原始值） */
    void ( *delayMs )( uint32_t ms );                    /* 忙等延时（ms），基于 getTickMs 计时 */
    uint32_t ( *rxAvailable )( void );                   /* 串口可读字节数 */
    uint32_t ( *read )( uint8_t *pBuf, uint32_t len );   /* 串口读，返回实际读取字节数 */
    uint32_t ( *write )( const uint8_t *pData, uint32_t len ); /* 串口写 */
    void ( *setBaudrate )( uint32_t baud );              /* 主机串口波特率切换（波特率探测用，可选） */
    void ( *powerOn )( void );                           /* 模组电源开启（可选） */
    void ( *powerOff )( void );                          /* 模组电源关闭（可选） */
    uint32_t fullCycleUs;                                /* 微秒计数器满周期（us），用于回绕计算 */
    uint32_t bootInitDelayMs;                            /* 上电后等待模组初始化完成延时（ms），默认 300（可选） */
    XFPK_Type ( *getXfpkType )( void );                  /* 读取节点配置的算法滤波类型（可选） */
}libOpenIMU_IO;
```

| 成员 | 语义要求 | AT32 参考实现（`libOpenIMU_portable.c`） |
|------|----------|------------------------------------------|
| `getTickMs` | 返回毫秒计数；用于状态机超时/启动延时 | `HAL_GetTick()` |
| `getUs` | 返回**自由运行计数器原始值**（1MHz）；用于帧等待 3ms 超时 | `(uint32_t)MAIN_TMR->CNT`（TMR3） |
| `delayMs` | 忙等延时 `ms` 毫秒，基于 `getTickMs` 计时（可替代 `HAL_Delay`） | `libOpenIMU_Portable_DelayMs`（`getTickMs()` 差值忙等） |
| `fullCycleUs` | 微秒计数器满周期（us），做 16 位回绕修正 | `(uint32_t)MAIN_TMR_DESC.fullCycleUs`（= 65536） |
| `rxAvailable` | 返回接收缓冲当前可读字节数 | `UART6_RxAvailable()` |
| `read` | 从接收缓冲读取最多 len 字节，返回实际读取数 | `UART6_Read( pBuf, len )` |
| `write` | 发送 len 字节 | `UART6_SendFrame( pData, len )` |
| `setBaudrate` | 主机串口波特率切换（波特率探测用）；为 NULL 时跳过探测，按既有波特率继续 | `UART6_SetBR( baud )` + 排空 RX |
| `powerOn` | 模组电源开启（开机流程用） | `OpenIMU_PwrCtl_PowerOn()` |
| `powerOff` | 模组电源关闭（开机流程用） | `OpenIMU_PwrCtl_PowerOff()` |
| `bootInitDelayMs` | 上电后等待模组初始化完成延时（ms，默认 `LIBOPENIMU_BOOT_INIT_DELAY_MS`=300） | 初始化器中填 `LIBOPENIMU_BOOT_INIT_DELAY_MS` |
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
void libOpenIMU_Portable_Init( libOpenIMU_UploadFormat uploadFormat, IMU_rawType IMU_rawType, libOpenIMU_BaudRate targetBaud );  /* 绑定 IO + 状态并调用 libOpenIMU_Init */
```

```c
/* libOpenIMU_portable.c */
#include "config.h"          /* 含 MAIN_TMR、UART6_TASK_EN 宏（如需要） */
#include "uart.h"            /* 你的串口驱动（UART6_*） */
#include "timer.h"           /* 含 MAIN_TMR_DESC（如需要） */
#include "at32f4xx.h"        /* HAL_GetTick 等 */
#include "libopenimu.h"
#include "gpio.h"            /* 模组电源控制（如需要） */

/* 平台函数 */
static uint32_t libOpenIMU_Portable_GetTickMs( void ) { return HAL_GetTick(); }
static uint32_t libOpenIMU_Portable_GetUs( void )     { return (uint32_t)MAIN_TMR->CNT; }
static void libOpenIMU_Portable_DelayMs( uint32_t ms )
{
    uint32_t startMs = gLibOpenIMU_IO.getTickMs();
    while ( gLibOpenIMU_IO.getTickMs() - startMs < ms ) {}
}
static uint32_t libOpenIMU_Portable_RxAvailable( void ) { return UART6_RxAvailable(); }
static uint32_t libOpenIMU_Portable_Read( uint8_t *pBuf, uint32_t len )
{ return UART6_Read( pBuf, len ); }
static uint32_t libOpenIMU_Portable_Write( const uint8_t *pData, uint32_t len )
{ return UART6_SendFrame( pData, len ); }
static void libOpenIMU_Portable_SetBaudrate( uint32_t baud )  /* 波特率探测用 */
{
    UART6_SetBR( baud );
    while ( UART6_RxAvailable() > 0 ) { uint8_t t[16]; UART6_Read( t, sizeof( t ) ); }
}
static void libOpenIMU_Portable_PowerOn( void )  { OpenIMU_PwrCtl_PowerOn(); }   /* 模组电源开 */
static void libOpenIMU_Portable_PowerOff( void ) { OpenIMU_PwrCtl_PowerOff(); }  /* 模组电源关 */
static XFPK_Type libOpenIMU_Portable_GetXfpkType( void )
{ return XFPK_Additional; }   /* 有节点配置时改为 StaConfig_getXfpkType() */

/* 实例化 IO（fullCycleUs 在 Init 时填充，因 MAIN_TMR_DESC.fullCycleUs 非编译期常量） */
static libOpenIMU_IO gLibOpenIMU_IO =
{
    .getTickMs   = libOpenIMU_Portable_GetTickMs,
    .getUs       = libOpenIMU_Portable_GetUs,
    .delayMs     = libOpenIMU_Portable_DelayMs,
    .rxAvailable = libOpenIMU_Portable_RxAvailable,
    .read        = libOpenIMU_Portable_Read,
    .write       = libOpenIMU_Portable_Write,
    .setBaudrate = libOpenIMU_Portable_SetBaudrate,
    .powerOn     = libOpenIMU_Portable_PowerOn,
    .powerOff    = libOpenIMU_Portable_PowerOff,
    .bootInitDelayMs = LIBOPENIMU_BOOT_INIT_DELAY_MS,   /* 默认 300ms */
    .getXfpkType = libOpenIMU_Portable_GetXfpkType,
};
static libOpenIMU_TypeDef gLibOpenIMU;

void libOpenIMU_Portable_Init( libOpenIMU_UploadFormat uploadFormat, IMU_rawType IMU_rawType, libOpenIMU_BaudRate targetBaud )
{
    /* 开机流程（若需电源控制）：断电 → 100ms → 打开 UART6 → 上电 → 等待模组初始化 */
    gLibOpenIMU_IO.powerOff();
    gLibOpenIMU_IO.delayMs( 100 );
#if (UART6_TASK_EN == 1)
    UART6_Open();
#endif
    gLibOpenIMU_IO.powerOn();
    gLibOpenIMU_IO.delayMs( gLibOpenIMU_IO.bootInitDelayMs );

    gLibOpenIMU_IO.fullCycleUs = (uint32_t)MAIN_TMR_DESC.fullCycleUs;
    libOpenIMU_Init( &gLibOpenIMU_IO, &gLibOpenIMU, uploadFormat, IMU_rawType, targetBaud );
}
```

> 💡 上传格式（`LIBOPENIMU_UPLOAD_FORMAT_STRING` / `LIBOPENIMU_UPLOAD_FORMAT_HEX`）、**上传内容组合**（`IMU_rawType` 位域）与**目标波特率**（`targetBaud`）均作为 `libOpenIMU_Init` 的参数传入：同一份固件可在运行时选择字符串或二进制解析、按需组合四元数/加速度/角速度/磁力计，并可选地把模组波特率配置为目标值（参照 `ATCMD.md` §1.2）。

> 💡 `setBaudrate` / `powerOn` / `powerOff` / `bootInitDelayMs` 为**可选**：`setBaudrate==NULL` 时 `libOpenIMU_Init` 跳过波特率探测（按既有波特率继续）；无电源控制需求时可将 `powerOn`/`powerOff` 置空，并在 `libOpenIMU_Portable_Init` 中去掉对应开机时序。

> 💡 `fullCycleUs` **不能**放在静态初始化器里（`MAIN_TMR_DESC.fullCycleUs` 是全局数组的运行时值，IAR 报 `Error[Pe028]`），必须在 `Init()` 运行时赋值——这就是参考实现放在 `libOpenIMU_Portable_Init()` 中的原因。

### 3.3 初始化与周期轮询

1. **上电后**（对应本工程 `main.c` 的 IMUSample_task 初始化段）调用一次：
   ```c
   libOpenIMU_Portable_Init( LIBOPENIMU_UPLOAD_FORMAT_HEX, IMU_RAW_ALL, LIBOPENIMU_BAUD_3000000 );  /* IMU_RAW_ALL=四组全量；targetBaud 传具体值（如 LIBOPENIMU_BAUD_3000000）即把模组配置为目标波特率，传 LIBOPENIMU_BAUD_UNKNOWN 保持探测值 */
   ```
   - 内部完成开机流程：**断电 → 等待 100ms → 打开 UART6 → 上电 → 等待模组初始化（`bootInitDelayMs`，默认 300ms）** → `fullCycleUs` 填充 → `libOpenIMU_Init( &gLibOpenIMU_IO, &gLibOpenIMU, uploadFormat, IMU_rawType, targetBaud )`；
   - `libOpenIMU_Init` 会先调用 `libOpenIMU_DetectBaudrate()` 探测模组当前波特率（遍历 `libOpenIMU_BaudRate`，逐档经 `setBaudrate` 切换主机串口、发 `AT\r\n` 等待 `\r\nOK\r\n`），命中后主机串口即保持在该波特率；随后读取 `getXfpkType()` 存到 `algFilterType`、保存 `uploadFormat`、`IMU_rawType` 与 `targetBaud`，并按 `IMU_rawType` 动态生成上传格式命令/校验串与期望帧长（`libOpenIMU_BuildUploadFormat`），把状态机复位到 `LIBOPENIMU_STATE_INIT`（该状态不再延时，开机等待已由移植层 `bootInitDelayMs` 完成，进入后先进入 config 模式、再按需配置目标波特率，然后继续配置）；
   - **开机/电源时序已内置于本函数**，`IMUSample_task` 无需再写断电/上电/开串口代码。

2. **周期任务中**反复调用（本工程在 IMUSample_task 主循环）：
   ```c
   libOpenIMU_Poll();
   ```
   - 内部是**非阻塞状态机**：开机自动完成 config 模式 → （可选）设置目标波特率 → LED 关闭 → 设置滤波类型 → 设置上传格式 → 校验 → 进入 requestMeasurement 稳态；
   - 稳态下每次 `Poll` 发送 `AT+requestFrame` 并等待 ≤3ms 解析一帧。
   - 初始化阶段（非 MEASUREMENT）命令发送受 **100ms 最小发送间隔**（`LIBOPENIMU_CMD_SEND_INTERVAL_MS`）节流，给模组反应时间；重试/状态间不会立即连发。
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

### 3.5 更换模块波特率（目标波特率配置）

默认情况下模组波特率为 115200；`libOpenIMU` 可在初始化时把模组波特率配置为调用方指定的目标值（参考 `ATCMD.md` §1.2「修改模块波特率」）。

**调用方式**：通过 `libOpenIMU_Init` / `libOpenIMU_Portable_Init` 的 `targetBaud` 参数指定：
- 传具体枚举值（如 `LIBOPENIMU_BAUD_3000000`）→ 探测当前波特率后，把模组配置为该目标波特率；
- 传 `LIBOPENIMU_BAUD_UNKNOWN` → 不更改波特率，保持探测值（既有行为）。

**内部流程**（初始化状态机中的 `SET_BAUDRATE` 状态，位于进入 config 模式之后）：

```text
SET_CONFIG_MODE（AT+MODE=config，进入 config 模式——AT+UARTCFG 仅 config 模式有效）
  → SET_BAUDRATE：
      跳过条件（满足其一直接进 SET_LED_OFF）：
        · targetBaud == LIBOPENIMU_BAUD_UNKNOWN（未指定）
        · targetBaud == 探测到的波特率
        · libOpenIMU_IO.setBaudrate == NULL（无主机波特率切换能力）
      否则按子步执行（每子步等待 OK，超时重试，子步间间隔 ≥100ms）：
        子步0：发 AT\r\n                  → 当前波特率下确认通信
        子步1：发 AT+UARTCFG=<target>\r\n  → 模组切换波特率；随后调用 setBaudrate(目标值) 切换主机串口
        子步2：发 AT\r\n                  → 目标波特率下验证通信 → 更新 baud → 进入 SET_LED_OFF
```

> ⚠️ **注意事项**：
> - `AT+UARTCFG` **仅在 config 模式有效**，因此 `SET_BAUDRATE` 必须排在 `SET_CONFIG_MODE` 之后；若跳过 config 模式直接改波特率会得到 `ERROR` 并陷入重试/重初始化循环。
> - 模组返回 `AT+UARTCFG=...` 的 `OK` 后，主机必须**立即**经 `setBaudrate` 切到同一波特率，否则后续通信失败；子步间 ≥100ms 间隔已为模组留出切换稳定时间。
> - 若 `targetBaud == 探测值`（如模组已在目标波特率），`SET_BAUDRATE` 自动跳过，无需重配。

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
| 上传格式 | `libOpenIMU_Init`/`libOpenIMU_Portable_Init` 的 `uploadFormat` 参数（`LIBOPENIMU_UPLOAD_FORMAT_STRING` / `LIBOPENIMU_UPLOAD_FORMAT_HEX`） | 字符串 CSV 或二进制 JustFloat（帧内容/长度随 `IMU_rawType` 变化）；运行时选择 |
| 上传内容组合 | `libOpenIMU_Init`/`libOpenIMU_Portable_Init` 的 `IMU_rawType` 参数（位域：`IMU_RAW_QUAT`/`IMU_RAW_ACCEL`/`IMU_RAW_GYRO`/`IMU_RAW_MAG`，可按位或；`IMU_RAW_ALL`=全量） | 决定上传哪几组数据：上传命令串、校验串、期望 float 个数与二进制帧长均按位掩码运行时生成；未选中组在 `libOpenIMU_Frame` 中保持 0 |
| 波特率探测 | `libOpenIMU_IO.setBaudrate`（`libopenimu.c` 的 `libOpenIMU_DetectBaudrate`） | 遍历 `libOpenIMU_BaudRate` 探测模组当前波特率；`setBaudrate==NULL` 时跳过探测，按既有波特率继续 |
| 目标波特率 | `libOpenIMU_Init`/`libOpenIMU_Portable_Init` 的 `targetBaud` 参数（`libOpenIMU_BaudRate` 枚举） | 探测后把模组波特率配置为目标值（`SET_BAUDRATE` 状态，参照 `ATCMD.md` §1.2）；传 `LIBOPENIMU_BAUD_UNKNOWN` 或目标==探测值时跳过 |
| 发送间隔 | `LIBOPENIMU_CMD_SEND_INTERVAL_MS`（`libopenimu.c`，默认 100） | 初始化阶段（非 MEASUREMENT）两次 AT 命令发送的最小间隔 ms，给模组反应时间，避免立即重试/连发 |
| 开机等待延时 | `libOpenIMU_IO.bootInitDelayMs` / `LIBOPENIMU_BOOT_INIT_DELAY_MS`（`libopenimu.h`） | 上电后等待模组初始化完成（默认 300ms），`libOpenIMU_Portable_Init` 开机流程使用 |
| 模组电源 | `libOpenIMU_IO.powerOn` / `powerOff` | 开机流程断电/上电；无电源控制需求时可置空并去掉对应时序 |
| 滤波类型 | `getXfpkType()` 返回值 | `XFPK_Base`（游戏旋转矢量）/ `XFPK_Additional`（标准，默认）；配置阶段发 `AT+CONFIG=algFilterType,<value>` |
| 状态 LED | 自动 | config 模式发 `AT+SETLED=OFF` 关闭 |
| 调试打印 | `LIBOPENIMU_DEBUG_PRINT`（`libopenimu.c`） | 置 1 打印原始 RX/响应行/超时状态 |

---

## 7. 协议状态机（简介）

初始化自动流程：`libOpenIMU_Init` 先执行波特率探测，再进入状态机（`libOpenIMU_State`）：

```
波特率探测（libOpenIMU_DetectBaudrate：遍历候选 → setBaudrate → 发 AT\r\n → 等 \r\nOK\r\n）
  → INIT(直接开始配置，开机等待已由移植层 bootInitDelayMs 完成)
  → SET_CONFIG_MODE    (AT+MODE=config)
  → SET_BAUDRATE       (可选：目标波特率配置 AT+UARTCFG；targetBaud==UNKNOWN/探测值 或 setBaudrate==NULL 时跳过)
  → SET_LED_OFF        (AT+SETLED=OFF)
  → SET_ALG_FILTER     (AT+CONFIG=algFilterType,<value>)
  → SET_UPLOADFORMAT   (AT+UPLOADFORMAT=<string|hex>,<按 IMU_rawType 动态生成的内容列表>)
  → VERIFY_UPLOADFORMAT(AT+UPLOADFORMAT=? 校验，期望串按 IMU_rawType 动态生成)
  → SET_REQUEST_MEASUREMENT (AT+MODE=requestMeasurement)
  → MEASUREMENT(稳态：AT+requestFrame 逐帧请求并解析)
```

- 每个状态等待 `OK`，超时按重试策略重发；超过上限记录错误并重新初始化，**不阻塞任务**。
- 非 MEASUREMENT 状态向模组发送 AT 命令受 **100ms 最小间隔**（`LIBOPENIMU_CMD_SEND_INTERVAL_MS`）节流，给模组反应时间。

---

## 8. 常见问题（FAQ）

| 现象 | 原因/排查 |
|------|-----------|
| 一直 `cmd ERROR, retry.` / `init retry exceeded` | 接线反/断、模组未上电、处于测量模式；波特率已由 `libOpenIMU_DetectBaudrate` 自动探测（日志可见 `baud detected` / `baud probe failed`），若出现 `baud detect failed` 说明所有候选波特率均无响应，先查接线/供电 |
| 反复出现 `target baud == detected baud, skip.` + `cmd ERROR` 循环 | 每次重初始化都卡在 `SET_BAUDRATE` 之后的 config 命令上：模组**未进入 config 模式**（`AT+UARTCFG`/`AT+SETLED` 等仅 config 模式有效，需先发 `AT+MODE=config`）。检查 `libOpenIMU_Poll` 中 `INIT` 是否先进入 `SET_CONFIG_MODE` 再进 `SET_BAUDRATE` |
| 一直超时无响应 | 检查 `rxAvailable`/`read` 是否真的把数据读出来了（先开 `LIBOPENIMU_DEBUG_PRINT` 看原始 RX） |
| 二进制帧解析乱 | 确保 `libOpenIMU_Init` 的 `uploadFormat` 与 `IMU_rawType` 和模组实际 `AT+UPLOADFORMAT` 一致（HEX 帧长 = 选中 float 数 × 4，随 `IMU_rawType` 动态变化，不走行解析） |
| 某些帧字段恒为 0 | 正常：`IMU_rawType` 未选中的组不会被上传/解析，`libOpenIMU_Frame` 对应字段保持 0；如需该组数据，请把对应 bit 加入 `IMU_rawType` |
| 编译报 `fullCycleUs` 非常量 | 不能在静态初始化器中赋值，改到 `Init()` 运行时填充 |
| 换 MCU 后代码要改？ | 不需要——只需重写移植层 `libOpenIMU_portable.c`（实现 `libOpenIMU_IO` 即可） |

---

## 9. 相关文件索引

| 位置 | 内容 |
|------|------|
| `libOpenIMU/include/libopenimu.h` | 公共头/API/类型定义 |
| `libOpenIMU/src/libopenimu.c` | 协议逻辑（纯算法） |
| `Project/bsp/libOpenIMU_portable.c/.h` | **AT32 参考移植层** |
| `Project/main.c`（IMUSample_task） | `libOpenIMU_Portable_Init()` 与 `libOpenIMU_Poll()` 调用点（开机/电源时序已内置于 `libOpenIMU_Portable_Init`） |
| `Project/bsp/uart.c/.h` | UART6 驱动（`UART6_*`） |
| `Project/bsp/timer.c/.h`、`config.h` | `MAIN_TMR`/`MAIN_TMR_DESC` |
| `Project/Proj/ATMC1507APP_OS.ewp` | IAR 工程（include 路径 + 源文件条目） |

---

*注：本文档与 `openspec/specs/openimu-uart6`、`openspec/specs/libopenimu-portable` 规格对应；移植时以 `libOpenIMU` 公共头与参考移植层为准。*
