/**
 * @file    libopenimu.c
 * @brief   OpenIMU BNO086 module (UART6) protocol driver | OpenIMU BNO086 模组（UART6）协议驱动
 * @details Implementation of the OpenIMU BNO086 protocol driver over UART6 | OpenIMU BNO086模组通过UART6的协议驱动实现
 * @author  MaoxiaoHu
 * @date    2016-01-01
 * @version 1.0
 * @copyright Copyright (C), 2015-2020, BEIJING FOHEART Tech. Co., Ltd.
 *
 * @par 修订历史
 * <table>
 * <tr><th>作者</th><th>时间</th><th>版本</th><th>描述</th></tr>
 * <tr><td>MaoxiaoHu</td><td>16/01/01</td><td>1.0</td><td>build this moudle</td></tr>
 * </table>
 */

/* Includes ------------------------------------------------------------------*/
#include "libopenimu.h"

/** @name AT commands (all end with \r\n) | AT指令（均以\r\n结尾）
 * @{
 */
/**
 * @brief AT command: enter config mode | AT指令：进入配置模式
 */
#define LIBOPENIMU_CMD_SET_CONFIG_MODE "AT+MODE=config\r\n"
/**
 * @brief AT command: turn off the status LED | AT指令：关闭状态LED
 */
#define LIBOPENIMU_CMD_SET_LED_OFF "AT+SETLED=OFF\r\n"
/**
 * @brief AT command: query the upload format | AT指令：查询上传格式
 */
#define LIBOPENIMU_CMD_QUERY_UPLOADFORMAT "AT+UPLOADFORMAT=?\r\n"
/**
 * @brief AT command: set request-measurement mode | AT指令：设置为测量模式（requestMeasurement）
 */
#define LIBOPENIMU_CMD_SET_REQUEST_MEASUREMENT "AT+MODE=requestMeasurement\r\n"
/**
 * @brief AT command: request one data frame | AT指令：请求一帧数据
 */
#define LIBOPENIMU_CMD_REQUEST_FRAME "AT+requestFrame\r\n"
/** @} */

/* 上传格式（字符串/二进制）由 libOpenIMU_Init 参数传入，经 libOpenIMU_TypeDef.uploadFormat 运行时选择；
 * 上传内容命令/校验串与期望 float 个数由 IMU_rawType 在 libOpenIMU_BuildUploadFormat 运行时生成 */

/** @name Timing / retry configuration | 时序与重试配置
 * @{
 */
/**
 * @brief Response timeout per state during initialization, in ms | 初始化阶段每状态等待响应超时（单位：ms）
 * @note Valid range: positive integer; default 200 | 有效范围：正整数；默认200
 */
#define LIBOPENIMU_RESP_TIMEOUT_MS (200)
/**
 * @brief Maximum retry count per state | 每状态最大重试次数
 * @note When exceeded, the init state machine restarts from INIT | 超过后初始化状态机从INIT重新开始
 */
#define LIBOPENIMU_MAX_RETRY (3)
/**
 * @brief Max wait after a frame request in steady state, in ms | 稳态请求帧后最多等待时间（单位：ms）
 * @note Default 3 | 默认3
 */
#define LIBOPENIMU_FRAME_TIMEOUT_MS (3)
/**
 * @brief Min interval between AT commands to the module during init, in ms | 初始化阶段向模组发送AT命令的最小间隔（单位：ms）
 * @details Gives the module reaction time and avoids immediate retry/back-to-back sends | 给模组反应时间，避免立即重试/连发
 */
#define LIBOPENIMU_CMD_SEND_INTERVAL_MS (100)
/**
 * @brief Max expected float count (all 10 tokens: quat4+accel/gyro/mag raw+cali 6*3=22), for the local parse buffer | 最大期望float个数（全10 token：quat4+accel/gyro/mag的raw+cali共6*3=22），局部解析缓冲用
 * @note The actual expected count is generated at runtime from the IMU_rawType bitmask in libOpenIMU_BuildUploadFormat and stored in frameFloatCount | 实际期望个数由IMU_rawType位掩码在libOpenIMU_BuildUploadFormat运行时生成并存入frameFloatCount
 */
#define LIBOPENIMU_FRAME_FLOAT_CNT_MAX (22)
/**
 * @brief Caller line buffer size (one AT response / data-frame line) | 调用方行缓冲大小（AT响应/数据帧一行）
 * @note Full 10-token string frame is at most ~176 chars + \r\n + NUL, so 192 is safe | 全10 token字符串帧最长约176字节+\r\n+NUL，192足够
 */
#define LIBOPENIMU_LINE_BUF_SIZE (192)
/** @} */

/**
 * @brief Debug switch: print raw bytes and response lines received on UART6 | 调试开关：打印UART6收到的原始数据与响应行
 * @details Used to localize initialization failures | 用于定位初始化失败问题
 * @note 1=enable, 0=disable | 1=使能，0=关闭
 */
#define LIBOPENIMU_DEBUG_PRINT (0)

/** @name Baudrate detection | 波特率探测
 * @{
 */
/**
 * @brief Number of baudrate candidates (excluding the LIBOPENIMU_BAUD_UNKNOWN sentinel) | 波特率探测候选数（不含LIBOPENIMU_BAUD_UNKNOWN哨兵）
 */
#define LIBOPENIMU_BAUD_COUNT ((int)(LIBOPENIMU_BAUD_UNKNOWN - LIBOPENIMU_BAUD_115200))
/**
 * @brief Response wait timeout per baudrate candidate, in ms | 波特率探测每档等待响应超时（单位：ms）
 */
#define LIBOPENIMU_BAUD_DETECT_TIMEOUT_MS (100)
/**
 * @brief AT online-test command used for baudrate detection | 波特率探测用的AT在线测试命令
 */
#define LIBOPENIMU_CMD_AT "AT\r\n"
/** @} */

/**
 * @brief Pointer to the module runtime-state instance | 模块运行状态实例指针
 * @details Instance is provided by libOpenIMU_portable and passed via libOpenIMU_Init | 实例由libOpenIMU_portable提供，经libOpenIMU_Init传入
 * @note NULL until libOpenIMU_Init() is called | 调用libOpenIMU_Init()前为NULL
 */
static libOpenIMU_TypeDef *sLibOpenIMU;
/**
 * @brief Pointer to the platform IO abstraction | 平台IO抽象指针
 * @details Time, UART read/write and baudrate callbacks; provided by libOpenIMU_portable | 时间、串口读写与波特率回调，由libOpenIMU_portable提供
 * @note NULL until libOpenIMU_Init() is called | 调用libOpenIMU_Init()前为NULL
 */
static const libOpenIMU_IO *sLibOpenIMU_IO;

/* 私有函数声明 */
static void libOpenIMU_SetState(libOpenIMU_State state);
static void libOpenIMU_Retry(void);
static void libOpenIMU_DrainRx(void);
static void libOpenIMU_SendCmd(const char *cmd);
static void libOpenIMU_RxAccumulateBytes(void);
static bool libOpenIMU_RxReadLine(char *outLine, uint16_t outMax);
static const char *libOpenIMU_CmdForState(libOpenIMU_State state);
static void libOpenIMU_InitStep(void);
static void libOpenIMU_InitStepSetBaudrate(void);
static void libOpenIMU_BuildUploadFormat(IMU_rawType rawType, libOpenIMU_UploadFormat fmt);
static void libOpenIMU_RequestFrame(void);
static uint32_t libOpenIMU_TmrElapsedUs(uint32_t startUs);
/* 字符串与二进制解析函数均编译，运行时按 sLibOpenIMU->uploadFormat 选择 */
static bool libOpenIMU_TryParseFrame(void);
static bool libOpenIMU_ParseFrame(const char *line);
static float libOpenIMU_BytesToFloatLE(const uint8_t *p);
static void libOpenIMU_ParseHexFrameData(const uint8_t *data);
static bool libOpenIMU_TryParseHexFrame(void);

/**
 * @brief Baudrate enum → numeric value lookup table | 波特率枚举→数值映射表
 * @details Index matches the libOpenIMU_BaudRate enum order; excludes LIBOPENIMU_BAUD_UNKNOWN | 下标与libOpenIMU_BaudRate枚举序一致，不含LIBOPENIMU_BAUD_UNKNOWN
 * @note Units: baud; valid values: 115200..3000000 | 单位：波特；有效范围：115200..3000000
 */
static const uint32_t libOpenIMU_BaudValue[LIBOPENIMU_BAUD_COUNT] = {
    115200, 230400, 256000, 460800, 921600,
    1000000, 1500000, 2000000, 3000000};

/**
 * @brief Switch state and reset state-related fields | 切换状态并重置该状态相关字段
 * @param state New state to enter | 要进入的新状态
 * @return void
 * @note Resets cmdPending/retryCount/formatSeen/rxLen/baudStep and records stateStartMs | 重置cmdPending/retryCount/formatSeen/rxLen/baudStep并记录stateStartMs
 */
static void libOpenIMU_SetState(libOpenIMU_State state)
{
    sLibOpenIMU->state = state;
    sLibOpenIMU->cmdPending = false;
    sLibOpenIMU->retryCount = 0;
    sLibOpenIMU->formatSeen = false;
    sLibOpenIMU->rxLen = 0;
    sLibOpenIMU->baudStep = 0;
    sLibOpenIMU->stateStartMs = sLibOpenIMU_IO->getTickMs();
}

/**
 * @brief Current-state timeout/failure handling | 当前状态超时/失败处理
 * @details Retries by resending the command, or re-initializes when the retry limit is reached | 重发命令重试，达到重试上限时重新初始化
 * @return void
 * @note On reaching LIBOPENIMU_MAX_RETRY it drains RX and restarts from the INIT state | 达到LIBOPENIMU_MAX_RETRY时清空RX并从INIT状态重新开始
 */
static void libOpenIMU_Retry(void)
{
    sLibOpenIMU->retryCount++;
    if (sLibOpenIMU->retryCount >= LIBOPENIMU_MAX_RETRY)
    {
        printf("[OpenIMU] init retry exceeded, re-init.\r\n");
        libOpenIMU_DrainRx();
        libOpenIMU_SetState(LIBOPENIMU_STATE_INIT);
    }
    else
    {
        sLibOpenIMU->cmdPending = false;
        sLibOpenIMU->formatSeen = false;
        sLibOpenIMU->rxLen = 0;
        sLibOpenIMU->stateStartMs = sLibOpenIMU_IO->getTickMs();
    }
}

/**
 * @brief Discard residual data in the UART6 RX | 丢弃UART6 RX中残留数据
 * @details Drains all pending bytes, e.g. module startup messages | 清空所有待读字节（如模组启动消息）
 * @return void
 * @note Resets rxLen to 0 after draining | 清空后rxLen复位为0
 */
static void libOpenIMU_DrainRx(void)
{
    uint8_t tmp[16];
    uint32_t avail;

    while ((avail = sLibOpenIMU_IO->rxAvailable()) > 0)
    {
        sLibOpenIMU_IO->read(tmp, avail > sizeof(tmp) ? sizeof(tmp) : avail);
    }
    sLibOpenIMU->rxLen = 0;
}

/**
 * @brief Send an AT command over UART6 | 通过UART6发送一条AT指令
 * @param cmd Null-terminated AT command string | 以'\0'结尾的AT指令字符串
 * @return void
 */
static void libOpenIMU_SendCmd(const char *cmd)
{
    sLibOpenIMU_IO->write((const uint8_t *)cmd, strlen(cmd));
}

/**
 * @brief Debug: print received raw bytes as hex + printable chars | 调试：以十六进制+可读字符打印收到的原始字节
 * @param tag Tag prefix for the log line | 日志行的标签前缀
 * @param buf Pointer to the raw byte buffer | 原始字节缓冲区指针
 * @param len Number of bytes to print | 要打印的字节数
 * @return void
 * @note Only active when LIBOPENIMU_DEBUG_PRINT == 1 | 仅当LIBOPENIMU_DEBUG_PRINT == 1时生效
 */
static void libOpenIMU_DebugDumpHex(const char *tag, const uint8_t *buf, uint32_t len)
{
    printf("[OpenIMU][%s] len=%lu:", tag, (unsigned long)len);
    for (uint32_t i = 0; i < len; i++)
    {
        printf(" %02X", buf[i]);
    }
    printf(" | ");
    for (uint32_t i = 0; i < len; i++)
    {
        uint8_t c = buf[i];
        if (c >= 0x20 && c < 0x7F)
        {
            printf("%c", (char)c);
        }
        else if (c == '\r')
        {
            printf("\\r");
        }
        else if (c == '\n')
        {
            printf("\\n");
        }
        else
        {
            printf(".");
        }
    }
    printf("\r\n");
}

/**
 * @brief Move all UART6 RX bytes into the internal accumulation buffer | 把UART6 RX中所有数据搬入内部累积缓冲
 * @details Preserves data that has not yet formed a complete line/frame | 保留未成行/未成帧的数据
 * @return void
 * @note When the buffer is full, the oldest byte is discarded to make room | 缓冲满时丢弃最早一个字节腾出空间
 */
static void libOpenIMU_RxAccumulateBytes(void)
{
    uint8_t tmp[16];
    uint32_t avail;

    while ((avail = sLibOpenIMU_IO->rxAvailable()) > 0)
    {
        uint32_t n = sLibOpenIMU_IO->read(tmp, avail > sizeof(tmp) ? sizeof(tmp) : avail);
#if (LIBOPENIMU_DEBUG_PRINT == 1)
        libOpenIMU_DebugDumpHex("rx", tmp, n);
#endif
        for (uint32_t i = 0; i < n; i++)
        {
            if (sLibOpenIMU->rxLen < LIBOPENIMU_RX_BUF_SIZE)
            {
                sLibOpenIMU->rxBuf[sLibOpenIMU->rxLen++] = tmp[i];
            }
            else
            {
                /* 累积缓冲满：丢弃最早一个字节，腾出空间 */
                memmove(sLibOpenIMU->rxBuf, sLibOpenIMU->rxBuf + 1, LIBOPENIMU_RX_BUF_SIZE - 1);
                sLibOpenIMU->rxBuf[LIBOPENIMU_RX_BUF_SIZE - 1] = tmp[i];
            }
        }
    }
}

/**
 * @brief Read UART6 RX into the line buffer and extract one line | 读取UART6 RX到行缓冲，取出一行
 * @details Lines end with '\n'; tolerates leading CRLF and fragmented arrival | 以'\n'结尾的行，容忍前导CRLF与分片
 * @param outLine Output buffer for the extracted line | 输出行缓冲区
 * @param outMax Size of the output buffer | 输出缓冲区大小
 * @return true = one line obtained (outLine contains no newline); false = no complete line yet | true=已得到一行（outLine不含换行符）；false=暂无完整行
 * @note The consumed line (including '\n') is removed from the accumulation buffer | 已消费的一行（含'\n'）会从累积缓冲中移除
 */
static bool libOpenIMU_RxReadLine(char *outLine, uint16_t outMax)
{
    uint16_t lineEnd;

    /* 1) 把 UART6 RX 中所有数据搬入内部累积缓冲（保留未成行数据） */
    libOpenIMU_RxAccumulateBytes();

    /* 2) 从累积缓冲中取出以 \n 结尾的一行 */
    lineEnd = 0;
    while (lineEnd < sLibOpenIMU->rxLen && sLibOpenIMU->rxBuf[lineEnd] != '\n')
    {
        lineEnd++;
    }
    if (lineEnd < sLibOpenIMU->rxLen)
    {
        uint16_t len = lineEnd;
        if (len > 0 && sLibOpenIMU->rxBuf[len - 1] == '\r')
        {
            len--;
        }
        if (len < outMax)
        {
            memcpy(outLine, sLibOpenIMU->rxBuf, len);
            outLine[len] = 0;
        }
        else
        {
            outLine[0] = 0;
        }
        /* 移除已消费的一行（含 \n） */
        sLibOpenIMU->rxLen -= (lineEnd + 1);
        memmove(sLibOpenIMU->rxBuf, sLibOpenIMU->rxBuf + lineEnd + 1, sLibOpenIMU->rxLen);
        return true;
    }
    return false;
}

/**
 * @brief Return the AT command to send for the current state | 返回当前状态应发送的AT指令
 * @param state Current initialization state | 当前初始化状态
 * @return Command string, or NULL for states with no command | 指令字符串；无指令的状态返回NULL
 * @note SET_UPLOADFORMAT command depends on the runtime-configured uploadFormat/IMU_rawType | SET_UPLOADFORMAT指令取决于运行时配置的uploadFormat/IMU_rawType
 */
static const char *libOpenIMU_CmdForState(libOpenIMU_State state)
{
    switch (state)
    {
    case LIBOPENIMU_STATE_SET_CONFIG_MODE:
        return LIBOPENIMU_CMD_SET_CONFIG_MODE;
    case LIBOPENIMU_STATE_SET_LED_OFF:
        return LIBOPENIMU_CMD_SET_LED_OFF;
    case LIBOPENIMU_STATE_SET_UPLOADFORMAT:
        return (sLibOpenIMU->uploadFormat == LIBOPENIMU_UPLOAD_FORMAT_STRING)
                   ? sLibOpenIMU->uploadFormatCmdStr
                   : sLibOpenIMU->uploadFormatCmdHex;
    case LIBOPENIMU_STATE_VERIFY_UPLOADFORMAT:
        return LIBOPENIMU_CMD_QUERY_UPLOADFORMAT;
    case LIBOPENIMU_STATE_SET_REQUEST_MEASUREMENT:
        return LIBOPENIMU_CMD_SET_REQUEST_MEASUREMENT;
    default:
        return NULL;
    }
}

/**
 * @brief Single-step advance of the init state machine | 初始化状态机单步推进
 * @details Sends the state command, then waits for the response with timeout/retry | 发送状态命令，然后等待响应（超时重试）
 * @return void
 * @note Command sending is throttled by LIBOPENIMU_CMD_SEND_INTERVAL_MS | 命令发送受LIBOPENIMU_CMD_SEND_INTERVAL_MS间隔节流
 */
static void libOpenIMU_InitStep(void)
{
    const char *cmd;
    char line[LIBOPENIMU_LINE_BUF_SIZE];

    /* 首次进入该状态：发送命令（受发送间隔节流，给模组反应时间） */
    if (!sLibOpenIMU->cmdPending)
    {
        if (sLibOpenIMU_IO->getTickMs() - sLibOpenIMU->lastCmdSendMs < LIBOPENIMU_CMD_SEND_INTERVAL_MS)
        {
            return; /* 未到发送间隔：等待下一轮再发 */
        }
        cmd = libOpenIMU_CmdForState(sLibOpenIMU->state);
        if (cmd == NULL)
        {
            libOpenIMU_SetState(LIBOPENIMU_STATE_INIT);
            return;
        }
        libOpenIMU_SendCmd(cmd);
        sLibOpenIMU->lastCmdSendMs = sLibOpenIMU_IO->getTickMs();
        sLibOpenIMU->cmdPending = true;
        sLibOpenIMU->stateStartMs = sLibOpenIMU_IO->getTickMs();
#if (LIBOPENIMU_DEBUG_PRINT == 1)
        printf("[OpenIMU] send: %s", cmd);
#endif
    }

    /* 轮询读取响应 */
    while (libOpenIMU_RxReadLine(line, sizeof(line)))
    {
#if (LIBOPENIMU_DEBUG_PRINT == 1)
        printf("[OpenIMU] line='%s'\r\n", line);
#endif
        if (strcmp(line, "ERROR") == 0)
        {
            printf("[OpenIMU] cmd ERROR, retry.\r\n");
            libOpenIMU_Retry();
            return;
        }

        if (sLibOpenIMU->state == LIBOPENIMU_STATE_VERIFY_UPLOADFORMAT)
        {
            /* 校验：需先看到期望的 +UploadFormat 行（按 IMU_rawType 动态生成的校验串），再看到 OK */
            if (strncmp(line, "+UploadFormat:", strlen("+UploadFormat:")) == 0 && strstr(line, sLibOpenIMU->expectedUploadFormat) != NULL)
            {
                sLibOpenIMU->formatSeen = true;
                printf("[OpenIMU] upload format verified: %s\r\n", line);
            }
            if (strcmp(line, "OK") == 0 && sLibOpenIMU->formatSeen)
            {
                libOpenIMU_SetState(LIBOPENIMU_STATE_SET_REQUEST_MEASUREMENT);
                return;
            }
        }
        else
        {
            if (strcmp(line, "OK") == 0)
            {
                libOpenIMU_SetState((libOpenIMU_State)(sLibOpenIMU->state + 1));
                return;
            }
        }
    }

    /* 超时重试 */
    if (sLibOpenIMU_IO->getTickMs() - sLibOpenIMU->stateStartMs >= LIBOPENIMU_RESP_TIMEOUT_MS)
    {
#if (LIBOPENIMU_DEBUG_PRINT == 1)
        printf("[OpenIMU] timeout: rxBuf='%.*s' rxLen=%u rxAvail=%lu\r\n",
               (int)sLibOpenIMU->rxLen, (const char *)sLibOpenIMU->rxBuf,
               (unsigned int)sLibOpenIMU->rxLen,
               (unsigned long)sLibOpenIMU_IO->rxAvailable());
#endif
        printf("[OpenIMU] cmd timeout, retry.\r\n");
        libOpenIMU_Retry();
    }
}

/**
 * @brief SET_BAUDRATE state handling | SET_BAUDRATE状态处理
 * @details Switches the module/host to the target baudrate in baudStep sub-steps and verifies it | 按baudStep子步将模组/主机切换到目标波特率并验证
 * @return void
 * @note Sub-step 0 = AT confirms current baud; sub-step 1 = AT+UARTCFG=<target> plus host baud switch;
 *       sub-step 2 = AT verifies target baud. Skip conditions: UNKNOWN target, target == detected, or setBaudrate == NULL | 子步0=AT确认当前波特率；子步1=AT+UARTCFG=<target>并同步切换主机波特率；子步2=AT验证目标波特率。跳过条件：目标为UNKNOWN、目标==探测值或setBaudrate==NULL
 */
static void libOpenIMU_InitStepSetBaudrate(void)
{
    char line[LIBOPENIMU_LINE_BUF_SIZE];

    /* 首次进入该状态：跳过判定 */
    if (!sLibOpenIMU->cmdPending && sLibOpenIMU->baudStep == 0)
    {
        if (sLibOpenIMU->targetBaud == LIBOPENIMU_BAUD_UNKNOWN)
        {
            /* 未指定目标波特率：保持探测值 */
            libOpenIMU_SetState(LIBOPENIMU_STATE_SET_LED_OFF);
            return;
        }
        if (sLibOpenIMU->targetBaud == sLibOpenIMU->baud)
        {
            printf("[OpenIMU] target baud == detected baud, skip.\r\n");
            libOpenIMU_SetState(LIBOPENIMU_STATE_SET_LED_OFF);
            return;
        }
        if (sLibOpenIMU_IO->setBaudrate == NULL)
        {
            printf("[OpenIMU] setBaudrate not provided, skip baud change.\r\n");
            libOpenIMU_SetState(LIBOPENIMU_STATE_SET_LED_OFF);
            return;
        }
    }

    /* 首次进入当前子步：发送该子步命令（受发送间隔节流，给模组反应时间） */
    if (!sLibOpenIMU->cmdPending)
    {
        char cmd[LIBOPENIMU_LINE_BUF_SIZE];

        if (sLibOpenIMU_IO->getTickMs() - sLibOpenIMU->lastCmdSendMs < LIBOPENIMU_CMD_SEND_INTERVAL_MS)
        {
            return; /* 未到发送间隔：等待下一轮再发 */
        }
        if (sLibOpenIMU->baudStep == 1)
        {
            snprintf(cmd, sizeof(cmd), "AT+UARTCFG=%lu\r\n",
                     (unsigned long)libOpenIMU_BaudValue[sLibOpenIMU->targetBaud]);
        }
        else
        {
            /* 子步 0 / 2：当前波特率与目标波特率下的 AT 在线确认 */
            strcpy(cmd, LIBOPENIMU_CMD_AT);
        }
        libOpenIMU_SendCmd(cmd);
        sLibOpenIMU->lastCmdSendMs = sLibOpenIMU_IO->getTickMs();
        sLibOpenIMU->cmdPending = true;
        sLibOpenIMU->stateStartMs = sLibOpenIMU_IO->getTickMs();
#if (LIBOPENIMU_DEBUG_PRINT == 1)
        printf("[OpenIMU] send len: %zu, cmd: %s", strlen(cmd), cmd);
#endif
    }

    /* 轮询读取响应 */
    while (libOpenIMU_RxReadLine(line, sizeof(line)))
    {
#if (LIBOPENIMU_DEBUG_PRINT == 1)
        printf("[OpenIMU] line='%s'\r\n", line);
#endif
        if (strcmp(line, "ERROR") == 0)
        {
            printf("[OpenIMU] baud cmd ERROR, retry.\r\n");
            libOpenIMU_Retry();
            return;
        }
        if (strcmp(line, "OK") == 0)
        {
            if (sLibOpenIMU->baudStep == 0)
            {
                /* 当前波特率确认通过 → 进入设置波特率子步 */
                sLibOpenIMU->baudStep = 1;
                sLibOpenIMU->cmdPending = false;
                sLibOpenIMU->stateStartMs = sLibOpenIMU_IO->getTickMs();
                return;
            }
            if (sLibOpenIMU->baudStep == 1)
            {
                /* 模组已切换波特率 → 主机串口同步切换 → 进入验证子步 */
                sLibOpenIMU_IO->setBaudrate(libOpenIMU_BaudValue[sLibOpenIMU->targetBaud]);
                printf("[OpenIMU] host baud switched to %lu\r\n",
                       (unsigned long)libOpenIMU_BaudValue[sLibOpenIMU->targetBaud]);
                sLibOpenIMU->baudStep = 2;
                sLibOpenIMU->cmdPending = false;
                sLibOpenIMU->stateStartMs = sLibOpenIMU_IO->getTickMs();
                return;
            }
            /* baudStep == 2：目标波特率验证通过 → 更新 baud 并进入下一状态 */
            sLibOpenIMU->baud = sLibOpenIMU->targetBaud;
            printf("[OpenIMU] baud change verified: %lu\r\n",
                   (unsigned long)libOpenIMU_BaudValue[sLibOpenIMU->targetBaud]);
            libOpenIMU_SetState(LIBOPENIMU_STATE_SET_LED_OFF);
            return;
        }
        /* 其它行（残留响应）忽略 */
    }

    /* 超时重试 */
    if (sLibOpenIMU_IO->getTickMs() - sLibOpenIMU->stateStartMs >= LIBOPENIMU_RESP_TIMEOUT_MS)
    {
        printf("[OpenIMU] baud cmd timeout, retry.\r\n");
        libOpenIMU_Retry();
    }
}

/** @brief 上传 token 映射（bit → 模组 token 名 → float 个数），按固定输出顺序排列 */
typedef struct
{
    IMU_rawType bit;      /*!< 位掩码 */
    const char *token;    /*!< 模组 token 名 */
    uint8_t     floats;   /*!< 该 token 的 float 个数 */
} IMU_TokenMap_TypeDef;

/** @brief 固定输出顺序：姿态 → accel_raw → accel_cali → gyro_raw → gyro_cali → mag_raw → mag_cali
 * （与 ATCMD.md §6.1 模组输出顺序一致，命令按同序生成）
 */
static const IMU_TokenMap_TypeDef IMU_TokenTable[] =
{
    { IMU_RAW_QUAT_BASE,        "quat_base",        4 },
    { IMU_RAW_QUAT_ADDITIONAL,  "quat_additional",  4 },
    { IMU_RAW_EULER_BASE,       "euler_base",       3 },
    { IMU_RAW_EULER_ADDITIONAL, "euler_additional", 3 },
    { IMU_RAW_ACCEL_RAW,        "accel_raw",        3 },
    { IMU_RAW_ACCEL_CALI,       "accel_cali",       3 },
    { IMU_RAW_GYRO_RAW,         "gyro_raw",         3 },
    { IMU_RAW_GYRO_CALI,        "gyro_cali",        3 },
    { IMU_RAW_MAG_RAW,          "mag_raw",          3 },
    { IMU_RAW_MAG_CALI,         "mag_cali",         3 },
};
#define IMU_TOKEN_TABLE_SIZE ( sizeof( IMU_TokenTable ) / sizeof( IMU_TokenTable[0] ) )

/**
 * @brief Dynamically build the upload-format command, expected string and frame length | 按IMU_rawType位掩码动态生成上传格式命令、校验串与期望帧长
 * @param rawType Selected token bitmask | 选中的 token 位掩码
 * @param fmt Upload format (string or hex) | 上传格式（字符串或十六进制）
 * @return void
 * @note Table-driven token list in fixed order attitude→accel_raw→accel_cali→gyro_raw→gyro_cali→mag_raw→mag_cali,
 *       matching the module output order (ATCMD §6.1). Illegal masks are safely resolved:
 *       empty→IMU_RAW_ALL, multiple attitude→quat_additional, no attitude→prepend quat_additional.
 *       The generic IMU_RAW_QUAT intent is resolved to quat_additional only on the direct-driver path
 *       (the portable layer resolves it per AlgFilterType before calling libOpenIMU_Init). | 表驱动按固定顺序生成token列表；非法位掩码安全回退（空→全选、多姿态→quat_additional、无姿态→补quat_additional）；通用IMU_RAW_QUAT仅在直传驱动路径下按quat_additional解析
 */
static void libOpenIMU_BuildUploadFormat(IMU_rawType rawType, libOpenIMU_UploadFormat fmt)
{
    char list[LIBOPENIMU_UPLOADFORMAT_BUF_SIZE];
    char *p = list;
    size_t rem = sizeof(list);
    uint8_t floatCount = 0;
    uint8_t attitudeCount = 0;
    const char *fmtName;

    /* 空组合安全策略：未选中任何数据组时回退全选（保持既有全量行为） */
    if (rawType == 0)
    {
        printf("[OpenIMU] IMU_rawType=0, fallback to IMU_RAW_ALL.\r\n");
        rawType = IMU_RAW_ALL;
    }

    /* 显式姿态位计数（互斥校验） */
    attitudeCount = (uint8_t)( ((rawType & IMU_RAW_QUAT_BASE)        ? 1 : 0)
                             + ((rawType & IMU_RAW_QUAT_ADDITIONAL)  ? 1 : 0)
                             + ((rawType & IMU_RAW_EULER_BASE)       ? 1 : 0)
                             + ((rawType & IMU_RAW_EULER_ADDITIONAL) ? 1 : 0) );

    if (attitudeCount > 1)
    {
        /* 多个姿态 token：模组只允许一个，回退为 quat_additional */
        printf("[OpenIMU] multiple attitude tokens, fallback to quat_additional.\r\n");
        rawType &= ~( IMU_RAW_ATTITUDE_MASK | IMU_RAW_QUAT );
        rawType |= IMU_RAW_QUAT_ADDITIONAL;
    }
    else if (attitudeCount == 1)
    {
        /* 显式姿态位优先：清除通用 quat 意图标记 */
        rawType &= ~IMU_RAW_QUAT;
    }
    else /* attitudeCount == 0 */
    {
        if (rawType & IMU_RAW_QUAT)
        {
            /* 通用 quat 意图位（直传驱动路径的防御性默认）：按 quat_additional 处理 */
            printf("[OpenIMU] generic IMU_RAW_QUAT -> quat_additional.\r\n");
            rawType &= ~IMU_RAW_QUAT;
            rawType |= IMU_RAW_QUAT_ADDITIONAL;
        }
        else
        {
            /* 有附加传感器但无姿态位：模组要求必须含姿态 token，否则其它数据无法读取，补 quat_additional */
            printf("[OpenIMU] no attitude token, prepend quat_additional.\r\n");
            rawType |= IMU_RAW_QUAT_ADDITIONAL;
        }
    }
    sLibOpenIMU->IMU_rawType = rawType;

    /* 表驱动按固定顺序生成 token 列表（逗号分隔），与模组输出顺序一致 */
    for (size_t i = 0; i < IMU_TOKEN_TABLE_SIZE; i++)
    {
        if (rawType & IMU_TokenTable[i].bit)
        {
            int n = snprintf(p, rem, "%s%s", (p == list) ? "" : ",", IMU_TokenTable[i].token);
            p += n;
            rem -= (size_t)n;
            floatCount += IMU_TokenTable[i].floats;
        }
    }

    /* 生成字符串/二进制两条 AT 命令与校验期望串 */
    fmtName = (fmt == LIBOPENIMU_UPLOAD_FORMAT_STRING) ? "string" : "hex";
    snprintf(sLibOpenIMU->uploadFormatCmdStr, sizeof(sLibOpenIMU->uploadFormatCmdStr),
             "AT+UPLOADFORMAT=string,%s\r\n", list);
    snprintf(sLibOpenIMU->uploadFormatCmdHex, sizeof(sLibOpenIMU->uploadFormatCmdHex),
             "AT+UPLOADFORMAT=hex,%s\r\n", list);
    snprintf(sLibOpenIMU->expectedUploadFormat, sizeof(sLibOpenIMU->expectedUploadFormat),
             "%s,%s", fmtName, list);

    /* 期望 float 个数与二进制帧字节数 */
    sLibOpenIMU->frameFloatCount = floatCount;
    sLibOpenIMU->hexFrameBytes = (uint16_t)((uint32_t)floatCount * (uint32_t)sizeof(float));
}

/**
 * @brief Microsecond-counter elapsed since startUs, wrap-safe | 微秒计数器自startUs起经过的us（含回绕）
 * @details Uses the free-running us counter from libOpenIMU_IO.getUs | 使用libOpenIMU_IO.getUs获取的自由运行微秒计数器
 * @param startUs Start timestamp in us | 起始时间戳（微秒）
 * @return Elapsed microseconds, accounting for wraparound | 经过的微秒数（已处理回绕）
 */
static uint32_t libOpenIMU_TmrElapsedUs(uint32_t startUs)
{
    uint32_t nowUs = sLibOpenIMU_IO->getUs();
    uint32_t period = sLibOpenIMU_IO->fullCycleUs;

    if (nowUs >= startUs)
    {
        return nowUs - startUs;
    }
    return (nowUs + period) - startUs;
}

/* === 字符串（STRING）格式解析 === */
/**
 * @brief Copy n floats from values (starting at *idx) into dst | 从values（*idx起）复制n个float到dst
 * @return void
 */
static void libOpenIMU_DispatchCopy(const float *values, int *idx, float *dst, uint8_t n)
{
    for (uint8_t i = 0; i < n; i++)
    {
        dst[i] = values[(*idx)++];
    }
}

/**
 * @brief Dispatch parsed float values into libOpenIMU_Frame by the fixed token order | 按固定token顺序把解析值分发到libOpenIMU_Frame各字段
 * @details Order matches the module output order (attitude → accel_raw → accel_cali → gyro_raw → gyro_cali → mag_raw → mag_cali).
 *          Only the tokens selected in IMU_rawType are filled; unselected groups stay 0 (memset at init) | 顺序与模组输出一致；仅填充IMU_rawType选中的token，未选中组保持0
 * @param values Parsed float array | 解析后的float数组
 * @param idx Pointer to the read offset (advanced) | 读取偏移（自动推进）
 * @return void
 */
static void libOpenIMU_DispatchTokens(const float *values, int *idx)
{
    IMU_rawType rt = sLibOpenIMU->IMU_rawType;

    /* 姿态 token（互斥，已由 BuildUploadFormat 校验） */
    if (rt & IMU_RAW_QUAT_BASE)
    {
        libOpenIMU_DispatchCopy(values, idx, sLibOpenIMU->frame.quat_wxyz, 4);
    }
    if (rt & IMU_RAW_QUAT_ADDITIONAL)
    {
        libOpenIMU_DispatchCopy(values, idx, sLibOpenIMU->frame.quat_wxyz, 4);
    }
    if (rt & IMU_RAW_EULER_BASE)
    {
        libOpenIMU_DispatchCopy(values, idx, sLibOpenIMU->frame.euler_deg, 3);
    }
    if (rt & IMU_RAW_EULER_ADDITIONAL)
    {
        libOpenIMU_DispatchCopy(values, idx, sLibOpenIMU->frame.euler_deg, 3);
    }
    /* 附加传感器：raw 与 cali */
    if (rt & IMU_RAW_ACCEL_RAW)
    {
        libOpenIMU_DispatchCopy(values, idx, sLibOpenIMU->frame.accel_raw_g, 3);
    }
    if (rt & IMU_RAW_ACCEL_CALI)
    {
        libOpenIMU_DispatchCopy(values, idx, sLibOpenIMU->frame.accel_g, 3);
    }
    if (rt & IMU_RAW_GYRO_RAW)
    {
        libOpenIMU_DispatchCopy(values, idx, sLibOpenIMU->frame.gyro_raw_dps, 3);
    }
    if (rt & IMU_RAW_GYRO_CALI)
    {
        libOpenIMU_DispatchCopy(values, idx, sLibOpenIMU->frame.gyro_dps, 3);
    }
    if (rt & IMU_RAW_MAG_RAW)
    {
        libOpenIMU_DispatchCopy(values, idx, sLibOpenIMU->frame.mag_raw_uT, 3);
    }
    if (rt & IMU_RAW_MAG_CALI)
    {
        libOpenIMU_DispatchCopy(values, idx, sLibOpenIMU->frame.mag_uT, 3);
    }
}

/**
 * @brief Parse one line of a string-format frame into libOpenIMU_Frame | 解析一行字符串帧到libOpenIMU_Frame
 * @details Parses the comma-separated floats (count = frameFloatCount) | 解析逗号分隔浮点数（个数=frameFloatCount）
 * @param line Pointer to the frame line | 帧行字符串指针
 * @return true = parsed OK and the latest valid frame was updated; false = parse failed | true=解析成功并更新最新有效帧；false=解析失败
 * @note Only the tokens selected in IMU_rawType are filled; unselected groups stay 0 | 仅填充IMU_rawType选中的token，未选中组保持0
 */
static bool libOpenIMU_ParseFrame(const char *line)
{
    float values[LIBOPENIMU_FRAME_FLOAT_CNT_MAX];
    int count = 0;
    int idx = 0;
    const char *p = line;

    /* 按动态期望个数（frameFloatCount）解析逗号分隔浮点数 */
    while (*p != 0 && count < sLibOpenIMU->frameFloatCount)
    {
        char *end;
        values[count] = strtof(p, &end);
        if (end == p)
        {
            /* 非数字 */
            return false;
        }
        count++;
        if (*end == ',')
        {
            p = end + 1;
        }
        else if (*end == 0)
        {
            break;
        }
        else
        {
            return false;
        }
    }

    if (count < sLibOpenIMU->frameFloatCount)
    {
        /* 数值不足 */
        return false;
    }

    sLibOpenIMU->frame.timestampMs = sLibOpenIMU_IO->getTickMs();

    /* 按固定 token 顺序分发到被选中组，未选中组保持 0（初始化已 memset） */
    libOpenIMU_DispatchTokens(values, &idx);

    sLibOpenIMU->frameValid = true;
    return true;
}

/**
 * @brief Read complete lines and try to parse a data frame | 读取完整行并尝试解析为数据帧
 * @return true = one valid frame was parsed; false = no valid frame yet | true=已解析到一帧有效数据；false=暂无有效帧
 * @note Invalid lines (e.g. residual OK) are ignored and reading continues | 无效行（如残留OK）被忽略并继续读取
 */
static bool libOpenIMU_TryParseFrame(void)
{
    char line[LIBOPENIMU_LINE_BUF_SIZE];

    while (libOpenIMU_RxReadLine(line, sizeof(line)))
    {
        if (libOpenIMU_ParseFrame(line))
        {
            // libOpenIMU_PrintFrame();
            return true;
        }
        /* 无效行（如残留 OK）忽略，继续读 */
    }
    return false;
}
/* === 二进制（HEX）格式解析 === */

/**
 * @brief Convert 4 bytes to float in little-endian order | 按小端序将4字节转换为float
 * @details Byte-order safe and independent of the host endianness | 跨平台安全，不依赖主机字节序
 * @param p Pointer to 4 bytes of little-endian float data | 4字节小端float数据指针
 * @return The converted float value | 转换后的float值
 */
static float libOpenIMU_BytesToFloatLE(const uint8_t *p)
{
    float f;
    uint8_t *dst = (uint8_t *)&f;

    dst[0] = p[0];
    dst[1] = p[1];
    dst[2] = p[2];
    dst[3] = p[3];
    return f;
}

/**
 * @brief Parse binary-format frame data | 解析二进制格式帧数据
 * @details Contains only little-endian floats of the IMU_rawType-selected tokens, no trailer | 仅含IMU_rawType选中token的小端float，无帧尾
 * @param data Pointer to the raw frame bytes | 原始帧数据指针
 * @return void
 * @note Order matches the fixed token order (attitude → accel_raw → accel_cali → gyro_raw → gyro_cali → mag_raw → mag_cali) | 顺序与固定token顺序一致
 */
static void libOpenIMU_ParseHexFrameData(const uint8_t *data)
{
    float values[LIBOPENIMU_FRAME_FLOAT_CNT_MAX];
    uint32_t off = 0;
    int idx = 0;

    /* 按帧内 float 总数（frameFloatCount）读取 little-endian float */
    for (uint8_t i = 0; i < sLibOpenIMU->frameFloatCount; i++)
    {
        values[i] = libOpenIMU_BytesToFloatLE(data + off);
        off += 4;
    }

    sLibOpenIMU->frame.timestampMs = sLibOpenIMU_IO->getTickMs();

    /* 按固定 token 顺序分发到被选中组，未选中组保持 0（初始化已 memset） */
    libOpenIMU_DispatchTokens(values, &idx);

    sLibOpenIMU->frameValid = true;
}

/**
 * @brief Receive and parse one binary-format frame | 接收并解析一帧二进制格式数据
 * @details Frame length is dynamic per IMU_rawType (hexFrameBytes), no trailer marker | 帧长按IMU_rawType动态（hexFrameBytes），无帧尾标记
 * @return true = one valid frame was parsed; false = frame not yet complete | true=已解析到一帧有效数据；false=暂未收满一帧
 * @note Binary data may contain 0x0A, so line parsing cannot be used; framing must be by dynamic length | 二进制数据可能含0x0A，不能走行解析，必须按动态帧长定帧
 */
static bool libOpenIMU_TryParseHexFrame(void)
{
    /* 帧长按 IMU_rawType 动态生成（hexFrameBytes = 选中 float 个数 × 4，无帧头/帧尾） */
    const uint32_t hexFrameDataBytes = sLibOpenIMU->hexFrameBytes;

    /* 1) 搬入原始字节（不做行解析） */
    libOpenIMU_RxAccumulateBytes();

    /* 2) 按动态帧长定帧（仅含被选中组），收满一帧即解析 */
    if (sLibOpenIMU->rxLen >= hexFrameDataBytes)
    {
        libOpenIMU_ParseHexFrameData(sLibOpenIMU->rxBuf);
        sLibOpenIMU->rxLen -= hexFrameDataBytes;
        memmove(sLibOpenIMU->rxBuf, sLibOpenIMU->rxBuf + hexFrameDataBytes, sLibOpenIMU->rxLen);
        libOpenIMU_PrintFrame();
        return true;
    }
    return false;
}

/**
 * @brief Steady state: send AT+requestFrame and wait ≤3ms for one frame | 稳态：发送AT+requestFrame并在≤3ms内等待并解析一帧
 * @return void
 * @note Wait window is LIBOPENIMU_FRAME_TIMEOUT_MS | 等待窗口为LIBOPENIMU_FRAME_TIMEOUT_MS
 */
static void libOpenIMU_RequestFrame(void)
{
    uint32_t startUs;

    libOpenIMU_SendCmd(LIBOPENIMU_CMD_REQUEST_FRAME);

    startUs = sLibOpenIMU_IO->getUs();
    do
    {
        if (sLibOpenIMU->uploadFormat == LIBOPENIMU_UPLOAD_FORMAT_STRING)
        {
            if (libOpenIMU_TryParseFrame())
            {
                return;
            }
        }
        else
        {
            if (libOpenIMU_TryParseHexFrame())
            {
                return;
            }
        }
    } while (libOpenIMU_TmrElapsedUs(startUs) < (LIBOPENIMU_FRAME_TIMEOUT_MS * 1000));
}

/**
 * @brief Detect the baudrate currently used by the OpenIMU module | 探测OpenIMU模组当前使用的波特率
 * @details Iterates all supported baudrates: switches the host UART, sends AT\r\n, waits for \r\nOK\r\n | 遍历支持的波特率：逐档切换主机串口、发送AT\r\n并等待\r\nOK\r\n响应
 * @return Matching libOpenIMU_BaudRate on hit, or LIBOPENIMU_BAUD_UNKNOWN if none match | 命中返回对应libOpenIMU_BaudRate；全部未命中返回LIBOPENIMU_BAUD_UNKNOWN
 * @note On a hit the host UART stays at the detected baudrate, no further switch needed | 命中后主机串口即保持在探测到的波特率上，无需再次切换
 */
libOpenIMU_BaudRate libOpenIMU_DetectBaudrate(void)
{
    uint32_t i;
    char line[LIBOPENIMU_LINE_BUF_SIZE];

    /* 未提供波特率切换回调：无法探测，按既有波特率继续 */
    if (sLibOpenIMU_IO->setBaudrate == NULL)
    {
        printf("[OpenIMU] baud detect skipped: setBaudrate not provided.\r\n");
        return LIBOPENIMU_BAUD_UNKNOWN;
    }

    for (i = 0; i < LIBOPENIMU_BAUD_COUNT; i++)
    {
        uint32_t startMs;
        bool gotOk = false;

        /* 切换主机串口波特率并排空 RX（丢弃旧波特率残留数据） */
        sLibOpenIMU_IO->setBaudrate(libOpenIMU_BaudValue[i]);
        libOpenIMU_DrainRx();

        /* 发送 AT 在线测试命令 */
        libOpenIMU_SendCmd(LIBOPENIMU_CMD_AT);

        /* 短超时内等待精确的 OK 行（libOpenIMU_RxReadLine 已剥 CRLF） */
        startMs = sLibOpenIMU_IO->getTickMs();
        while (sLibOpenIMU_IO->getTickMs() - startMs < LIBOPENIMU_BAUD_DETECT_TIMEOUT_MS)
        {
            if (libOpenIMU_RxReadLine(line, sizeof(line)))
            {
                if (strcmp(line, "OK") == 0)
                {
                    gotOk = true;
                    break;
                }
                /* 其他行（乱码/非 OK 响应）忽略 */
            }
        }

        if (gotOk)
        {
            printf("[%d][OpenIMU] baud detected: %lu\r\n", sLibOpenIMU_IO->getTickMs(), (unsigned long)libOpenIMU_BaudValue[i]);
            return (libOpenIMU_BaudRate)i;
        }

        /* 该档波特率未响应：打印以定位失败的波特率 */
        printf("[%d][OpenIMU] baud probe failed: no OK response @ %lu\r\n", sLibOpenIMU_IO->getTickMs(), (unsigned long)libOpenIMU_BaudValue[i]);
    }

    printf("[OpenIMU] baud detect failed: no supported baud responded.\r\n");
    return LIBOPENIMU_BAUD_UNKNOWN;
}

/**
 * @brief Initialize the module and reset the state machine to INIT | 初始化模块并复位状态机到INIT
 * @details Detects the baudrate first, then enters the initialization state machine | 先探测波特率，再进入初始化状态机
 * @param pIo Platform IO abstraction (time, UART read/write) | 平台IO抽象（时间与串口读写回调）
 * @param pInst Module state instance to operate on | 模块运行状态实例
 * @param uploadFormat Upload format (string or hex) | 上传格式（字符串或十六进制）
 * @param IMU_rawType Selected data-group bitmask | 上传内容组合位域
 * @param targetBaud Target baudrate; LIBOPENIMU_BAUD_UNKNOWN keeps the detected value | 目标波特率；LIBOPENIMU_BAUD_UNKNOWN表示保持探测值
 * @return void
 * @note Requires the underlying UART6 to be initialized before calling | 调用前需完成底层串口（UART6）初始化
 */
void libOpenIMU_Init(libOpenIMU_IO *pIo, libOpenIMU_TypeDef *pInst, libOpenIMU_UploadFormat uploadFormat, IMU_rawType IMU_rawType, libOpenIMU_BaudRate targetBaud)
{
    sLibOpenIMU_IO = pIo;
    sLibOpenIMU = pInst;
    memset(sLibOpenIMU, 0, sizeof(*sLibOpenIMU));
    sLibOpenIMU->uploadFormat = uploadFormat;

    /* 保存上传内容组合并动态生成上传格式命令/校验串/期望帧长（未选中组经 memset 保持 0）；
     * quat 变体已由调用方（portable 层）按 AlgFilterType 解析为显式姿态位，驱动不再读取 algFilterType */
    sLibOpenIMU->IMU_rawType = IMU_rawType;
    libOpenIMU_BuildUploadFormat(IMU_rawType, uploadFormat);

    /* 保存目标波特率（UNKNOWN=不更改，保持探测值）；SET_BAUDRATE 状态按此配置模组波特率 */
    sLibOpenIMU->targetBaud = targetBaud;

    /* 先探测模组当前波特率（成功后主机串口已保持在该波特率），再进入初始化状态机 */
    sLibOpenIMU->baud = libOpenIMU_DetectBaudrate();

    libOpenIMU_SetState(LIBOPENIMU_STATE_INIT);
}

/**
 * @brief State machine advance, called every loop by IMUSample_task | 状态机推进（由IMUSample_task每轮调用）
 * @return void
 * @note Must be called periodically | 需周期性调用
 */
void libOpenIMU_Poll(void)
{
    switch (sLibOpenIMU->state)
    {
    case LIBOPENIMU_STATE_INIT:
        /* 上电/启动等待已由移植层 libOpenIMU_Portable_Init（bootInitDelayMs）完成，先进入 config 模式
         * （AT+UARTCFG/SETLED 等仅 config 模式有效），再进入 SET_BAUDRATE */
        libOpenIMU_DrainRx();
        libOpenIMU_SetState(LIBOPENIMU_STATE_SET_CONFIG_MODE);
        break;

    case LIBOPENIMU_STATE_SET_BAUDRATE:
        libOpenIMU_InitStepSetBaudrate();
        break;

    case LIBOPENIMU_STATE_SET_CONFIG_MODE:
    case LIBOPENIMU_STATE_SET_LED_OFF:
    case LIBOPENIMU_STATE_SET_UPLOADFORMAT:
    case LIBOPENIMU_STATE_VERIFY_UPLOADFORMAT:
    case LIBOPENIMU_STATE_SET_REQUEST_MEASUREMENT:
        libOpenIMU_InitStep();
        break;

    case LIBOPENIMU_STATE_MEASUREMENT:
        libOpenIMU_RequestFrame();
        break;

    default:
        libOpenIMU_SetState(LIBOPENIMU_STATE_INIT);
        break;
    }
}

/**
 * @brief Get the latest valid frame data | 获取最新有效帧数据
 * @param pFrame Output buffer for the frame; must not be NULL | 输出帧数据缓冲区，不能为NULL
 * @return true = one valid frame was copied; false = no valid frame available or bad pointer | true=已成功获取一帧有效数据；false=暂无有效帧或指针无效
 * @note The returned frame is a copy of the internal latest frame | 返回的帧为内部最新帧的拷贝
 */
bool libOpenIMU_GetFrame(libOpenIMU_Frame *pFrame)
{
    if (pFrame == NULL || !sLibOpenIMU->frameValid)
    {
        return false;
    }
    *pFrame = sLibOpenIMU->frame;
    return true;
}

/** @name Print switches
 * @brief Print toggles for libOpenIMU_PrintFrame data groups (1=print, 0=off) | libOpenIMU_PrintFrame中各数据组打印开关（1=打印，0=不打印）
 * @{
 */
/**
 * @brief Print quaternion (w,x,y,z) | 打印四元数（w,x,y,z）
 */
#define LIBOPENIMU_PRINT_QUAT (0)
/**
 * @brief Print accelerometer (x,y,z in g) | 打印加速度计（x,y,z，单位g）
 */
#define LIBOPENIMU_PRINT_ACCEL (0)
/**
 * @brief Print gyroscope (x,y,z in dps) | 打印陀螺仪（x,y,z，单位°/s）
 */
#define LIBOPENIMU_PRINT_GYRO (0)
/**
 * @brief Print magnetometer (x,y,z in µT) | 打印磁力计（x,y,z，单位µT）
 */
#define LIBOPENIMU_PRINT_MAG (0)

#define LIBOPENIMU_ATLEAST_ONE_PRINT (LIBOPENIMU_PRINT_QUAT || LIBOPENIMU_PRINT_ACCEL || LIBOPENIMU_PRINT_GYRO || LIBOPENIMU_PRINT_MAG)
/** @} */

/**
 * @brief Print the latest valid frame data (debug) | 打印最新有效帧数据（调试用）
 * @return void
 * @note Each data group's printing is independently controlled by the LIBOPENIMU_PRINT_* switches | 各组打印由LIBOPENIMU_PRINT_*开关分别控制
 */
void libOpenIMU_PrintFrame(void)
{
    libOpenIMU_Frame *pFrame = &sLibOpenIMU->frame;

    if (!sLibOpenIMU->frameValid)
    {
        printf("[OpenIMU] no valid frame yet\r\n");
        return;
    }

#if LIBOPENIMU_ATLEAST_ONE_PRINT
    printf("[OpenIMU] t=%lu", (unsigned long)pFrame->timestampMs);
#endif

#if (LIBOPENIMU_PRINT_QUAT == 1)
    printf(" q(wxyz)=%.4f,%.4f,%.4f,%.4f",
           pFrame->quat_wxyz[0], pFrame->quat_wxyz[1], pFrame->quat_wxyz[2], pFrame->quat_wxyz[3]);
#endif
#if (LIBOPENIMU_PRINT_ACCEL == 1)
    printf(" a(g)=%.3f,%.3f,%.3f",
           pFrame->accel_g[0], pFrame->accel_g[1], pFrame->accel_g[2]);
#endif
#if (LIBOPENIMU_PRINT_GYRO == 1)
    printf(" g(dps)=%.3f,%.3f,%.3f",
           pFrame->gyro_dps[0], pFrame->gyro_dps[1], pFrame->gyro_dps[2]);
#endif
#if (LIBOPENIMU_PRINT_MAG == 1)
    printf(" mag(uT)=%.3f,%.3f,%.3f",
           pFrame->mag_uT[0], pFrame->mag_uT[1], pFrame->mag_uT[2]);
#endif

#if LIBOPENIMU_ATLEAST_ONE_PRINT
    printf("\r\n");
#endif
}
