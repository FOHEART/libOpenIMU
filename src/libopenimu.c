/************************************************************
 * Copyright (C), 2015-2020, BEIJING FOHEART Tech. Co., Ltd.
 * FileName:
 * Author:
 * Date:
 * Description:     OpenIMU BNO086 模组（UART6）协议驱动
 * Version:
 * Function List:
 *     1. libOpenIMU_Init
 *     2. libOpenIMU_Poll
 *     3. libOpenIMU_GetFrame
 * History:
 *     <author>  <time>   <version >   <desc>
 *     MaoxiaoHu 16/01/01     1.0   build this moudle
 ***********************************************************/

/* Includes ------------------------------------------------------------------*/
#include "libopenimu.h"

/* AT 指令（均以 \r\n 结尾） */
#define LIBOPENIMU_CMD_SET_CONFIG_MODE "AT+MODE=config\r\n"
#define LIBOPENIMU_CMD_SET_LED_OFF "AT+SETLED=OFF\r\n"
#define LIBOPENIMU_CMD_SET_ALG_FILTER_BASE "AT+CONFIG=algFilterType,XFPK_Base\r\n"
#define LIBOPENIMU_CMD_SET_ALG_FILTER_ADDITIONAL "AT+CONFIG=algFilterType,XFPK_Additional\r\n"
#define LIBOPENIMU_CMD_SET_UPLOADFORMAT_STRING "AT+UPLOADFORMAT=string,quat,accel,gyro,mag\r\n"
#define LIBOPENIMU_CMD_SET_UPLOADFORMAT_HEX "AT+UPLOADFORMAT=hex,quat,accel,gyro,mag\r\n"
#define LIBOPENIMU_CMD_QUERY_UPLOADFORMAT "AT+UPLOADFORMAT=?\r\n"
#define LIBOPENIMU_CMD_SET_REQUEST_MEASUREMENT "AT+MODE=requestMeasurement\r\n"
#define LIBOPENIMU_CMD_REQUEST_FRAME "AT+requestFrame\r\n"

/* 期望的上传格式（查询校验用） */
#define LIBOPENIMU_EXPECTED_UPLOADFORMAT_STRING "string,quat,accel,gyro,mag"
#define LIBOPENIMU_EXPECTED_UPLOADFORMAT_HEX "hex,quat,accel,gyro,mag"

/* 上传格式（字符串/二进制）由 libOpenIMU_Init 参数传入，经 libOpenIMU_TypeDef.uploadFormat 运行时选择 */

/* 初始化阶段每状态等待响应超时 ms */
#define LIBOPENIMU_RESP_TIMEOUT_MS (200)
/* 每状态最大重试次数 */
#define LIBOPENIMU_MAX_RETRY (3)
/* 稳态请求帧后最多等待 ms */
#define LIBOPENIMU_FRAME_TIMEOUT_MS (3)
/* 字符串帧期望的浮点数个数：由 libOpenIMU_Frame 各数组字段计算（quat4 + accel3 + gyro3 + mag3 = 13） */
#define LIBOPENIMU_FRAME_FLOAT_CNT (                                                    \
    ( sizeof( ((libOpenIMU_Frame *)0)->quat_wxyz ) / sizeof( float ) ) +               \
    ( sizeof( ((libOpenIMU_Frame *)0)->accel_g    ) / sizeof( float ) ) +              \
    ( sizeof( ((libOpenIMU_Frame *)0)->gyro_dps   ) / sizeof( float ) ) +              \
    ( sizeof( ((libOpenIMU_Frame *)0)->mag_uT ) / sizeof( float ) ) )
/* 调用方行缓冲大小（AT 响应/数据帧一行） */
#define LIBOPENIMU_LINE_BUF_SIZE (128)

/* 调试开关：打印 UART6 收到的原始数据与响应行（定位初始化失败用） */
#define LIBOPENIMU_DEBUG_PRINT (0)

/* 波特率探测：候选数（不含 LIBOPENIMU_BAUD_UNKNOWN 哨兵）与每档等待响应超时 ms */
#define LIBOPENIMU_BAUD_COUNT ( ( int )( LIBOPENIMU_BAUD_UNKNOWN - LIBOPENIMU_BAUD_115200 ) )
#define LIBOPENIMU_BAUD_DETECT_TIMEOUT_MS (100)
/* 波特率探测用的 AT 在线测试命令 */
#define LIBOPENIMU_CMD_AT "AT\r\n"



/* 运行状态与平台 IO（实例由 libOpenIMU_portable 提供，经 libOpenIMU_Init 传入） */
static libOpenIMU_TypeDef *sLibOpenIMU;
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
static void libOpenIMU_RequestFrame(void);
static uint32_t libOpenIMU_TmrElapsedUs(uint32_t startUs);
/* 字符串与二进制解析函数均编译，运行时按 sLibOpenIMU->uploadFormat 选择 */
static bool libOpenIMU_TryParseFrame(void);
static bool libOpenIMU_ParseFrame(const char *line);
static float libOpenIMU_BytesToFloatLE(const uint8_t *p);
static void libOpenIMU_ParseHexFrameData(const uint8_t *data);
static bool libOpenIMU_TryParseHexFrame(void);

/* 波特率枚举 → 数值映射表（下标与 libOpenIMU_BaudRate 枚举序一致，不含 LIBOPENIMU_BAUD_UNKNOWN） */
static const uint32_t libOpenIMU_BaudValue[LIBOPENIMU_BAUD_COUNT] = {
    115200, 230400, 256000, 460800, 921600,
    1000000, 1500000, 2000000, 3000000
};

/***********************************************************
 * Function:        libOpenIMU_SetState
 * Description:     切换状态并重置该状态相关字段
 * Input:
 * Input:
 * Output:
 * Return:
 * Others:          Other Description.
 ***********************************************************/
static void libOpenIMU_SetState(libOpenIMU_State state)
{
    sLibOpenIMU->state = state;
    sLibOpenIMU->cmdPending = false;
    sLibOpenIMU->retryCount = 0;
    sLibOpenIMU->formatSeen = false;
    sLibOpenIMU->rxLen = 0;
    sLibOpenIMU->stateStartMs = sLibOpenIMU_IO->getTickMs();
}

/***********************************************************
 * Function:        libOpenIMU_Retry
 * Description:     当前状态超时/失败处理：重发命令或重新初始化
 * Input:
 * Input:
 * Output:
 * Return:
 * Others:          Other Description.
 ***********************************************************/
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

/***********************************************************
 * Function:        libOpenIMU_DrainRx
 * Description:     丢弃 UART6 RX 中残留数据（如模组启动消息）
 * Input:
 * Input:
 * Output:
 * Return:
 * Others:          Other Description.
 ***********************************************************/
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

/***********************************************************
 * Function:        libOpenIMU_SendCmd
 * Description:     通过 UART6 发送一条 AT 指令
 * Input:
 * Input:
 * Output:
 * Return:
 * Others:          Other Description.
 ***********************************************************/
static void libOpenIMU_SendCmd(const char *cmd)
{
    sLibOpenIMU_IO->write((const uint8_t *)cmd, strlen(cmd));
}

/***********************************************************
 * Function:        libOpenIMU_DebugDumpHex
 * Description:     调试：以十六进制+可读字符打印收到的原始字节
 * Input:
 * Input:
 * Output:
 * Return:
 * Others:          Other Description.
 ***********************************************************/
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

/***********************************************************
 * Function:        libOpenIMU_RxReadLine
 * Description:     读取 UART6 RX 到行缓冲，取出一行（以 \n 结尾，容忍前导 CRLF 与分片）
 * Input:
 * Input:
 * Output:
 * Return:          true=已得到一行（outLine 不含换行符）
 * Others:          Other Description.
 ***********************************************************/
/***********************************************************
 * Function:        libOpenIMU_RxAccumulateBytes
 * Description:     把 UART6 RX 中所有数据搬入内部累积缓冲（保留未成行/未成帧数据）
 * Input:
 * Input:
 * Output:
 * Return:
 * Others:          Other Description.
 ***********************************************************/
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

/***********************************************************
 * Function:        libOpenIMU_RxReadLine
 * Description:     读取 UART6 RX 到行缓冲，取出一行（以 \n 结尾，容忍前导 CRLF 与分片）
 * Input:
 * Input:
 * Output:
 * Return:          true=已得到一行（outLine 不含换行符）
 * Others:          Other Description.
 ***********************************************************/
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

/***********************************************************
 * Function:        libOpenIMU_CmdForState
 * Description:     返回当前状态应发送的 AT 指令
 * Input:
 * Input:
 * Output:
 * Return:
 * Others:          Other Description.
 ***********************************************************/
static const char *libOpenIMU_CmdForState(libOpenIMU_State state)
{
    switch (state)
    {
    case LIBOPENIMU_STATE_SET_CONFIG_MODE:
        return LIBOPENIMU_CMD_SET_CONFIG_MODE;
    case LIBOPENIMU_STATE_SET_LED_OFF:
        return LIBOPENIMU_CMD_SET_LED_OFF;
    case LIBOPENIMU_STATE_SET_ALG_FILTER:
        return (sLibOpenIMU->algFilterType == XFPK_Base)
                   ? LIBOPENIMU_CMD_SET_ALG_FILTER_BASE
                   : LIBOPENIMU_CMD_SET_ALG_FILTER_ADDITIONAL;
    case LIBOPENIMU_STATE_SET_UPLOADFORMAT:
        return (sLibOpenIMU->uploadFormat == LIBOPENIMU_UPLOAD_FORMAT_STRING)
                   ? LIBOPENIMU_CMD_SET_UPLOADFORMAT_STRING
                   : LIBOPENIMU_CMD_SET_UPLOADFORMAT_HEX;
    case LIBOPENIMU_STATE_VERIFY_UPLOADFORMAT:
        return LIBOPENIMU_CMD_QUERY_UPLOADFORMAT;
    case LIBOPENIMU_STATE_SET_REQUEST_MEASUREMENT:
        return LIBOPENIMU_CMD_SET_REQUEST_MEASUREMENT;
    default:
        return NULL;
    }
}

/***********************************************************
 * Function:        libOpenIMU_InitStep
 * Description:     初始化状态机单步推进：发送命令→等待响应（超时重试）
 * Input:
 * Input:
 * Output:
 * Return:
 * Others:          Other Description.
 ***********************************************************/
static void libOpenIMU_InitStep(void)
{
    const char *cmd;
    char line[LIBOPENIMU_LINE_BUF_SIZE];

    /* 首次进入该状态：发送命令 */
    if (!sLibOpenIMU->cmdPending)
    {
        cmd = libOpenIMU_CmdForState(sLibOpenIMU->state);
        if (cmd == NULL)
        {
            libOpenIMU_SetState(LIBOPENIMU_STATE_INIT);
            return;
        }
        libOpenIMU_SendCmd(cmd);
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
            const char *expectedFormat;
            /* 校验：需先看到期望的 +UploadFormat 行，再看到 OK */
            expectedFormat = (sLibOpenIMU->uploadFormat == LIBOPENIMU_UPLOAD_FORMAT_STRING)
                                 ? LIBOPENIMU_EXPECTED_UPLOADFORMAT_STRING
                                 : LIBOPENIMU_EXPECTED_UPLOADFORMAT_HEX;
            if (strncmp(line, "+UploadFormat:", strlen("+UploadFormat:")) == 0 && strstr(line, expectedFormat) != NULL)
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

/***********************************************************
 * Function:        libOpenIMU_TmrElapsedUs
 * Description:     微秒计数器（1MHz，经 libOpenIMU_IO.getUs 获取）自 startUs 起经过的 us（含回绕）
 * Input:
 * Input:
 * Output:
 * Return:
 * Others:          Other Description.
 ***********************************************************/
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
/***********************************************************
 * Function:        libOpenIMU_ParseFrame
 * Description:     解析一行字符串帧（13 个逗号分隔浮点数）到 libOpenIMU_Frame
 * Input:
 * Input:
 * Output:
 * Return:          true=解析成功并更新最新有效帧
 * Others:          Other Description.
 ***********************************************************/
static bool libOpenIMU_ParseFrame(const char *line)
{
    float values[LIBOPENIMU_FRAME_FLOAT_CNT];
    int count = 0;
    const char *p = line;

    while (*p != 0 && count < LIBOPENIMU_FRAME_FLOAT_CNT)
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

    if (count < LIBOPENIMU_FRAME_FLOAT_CNT)
    {
        /* 数值不足 */
        return false;
    }

    sLibOpenIMU->frame.timestampMs = sLibOpenIMU_IO->getTickMs();
    sLibOpenIMU->frame.quat_wxyz[0] = values[0];
    sLibOpenIMU->frame.quat_wxyz[1] = values[1];
    sLibOpenIMU->frame.quat_wxyz[2] = values[2];
    sLibOpenIMU->frame.quat_wxyz[3] = values[3];
    sLibOpenIMU->frame.accel_g[0] = values[4];
    sLibOpenIMU->frame.accel_g[1] = values[5];
    sLibOpenIMU->frame.accel_g[2] = values[6];
    sLibOpenIMU->frame.gyro_dps[0] = values[7];
    sLibOpenIMU->frame.gyro_dps[1] = values[8];
    sLibOpenIMU->frame.gyro_dps[2] = values[9];
    sLibOpenIMU->frame.mag_uT[0] = values[10];
    sLibOpenIMU->frame.mag_uT[1] = values[11];
    sLibOpenIMU->frame.mag_uT[2] = values[12];

    sLibOpenIMU->frameValid = true;
    return true;
}

/***********************************************************
 * Function:        libOpenIMU_TryParseFrame
 * Description:     读取完整行并尝试解析为数据帧
 * Input:
 * Input:
 * Output:
 * Return:          true=已解析到一帧有效数据
 * Others:          Other Description.
 ***********************************************************/
static bool libOpenIMU_TryParseFrame(void)
{
    char line[LIBOPENIMU_LINE_BUF_SIZE];

    while (libOpenIMU_RxReadLine(line, sizeof(line)))
    {
        if (libOpenIMU_ParseFrame(line))
        {
            //libOpenIMU_PrintFrame();
            return true;
        }
        /* 无效行（如残留 OK）忽略，继续读 */
    }
    return false;
}
/* === 二进制（HEX）格式解析 === */

/***********************************************************
 * Function:        libOpenIMU_BytesToFloatLE
 * Description:     按小端序将 4 字节转换为 float（跨平台安全，不依赖主机字节序）
 * Input:
 * Input:
 * Output:
 * Return:
 * Others:          Other Description.
 ***********************************************************/
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

/***********************************************************
 * Function:        libOpenIMU_ParseHexFrameData
 * Description:     解析二进制格式帧数据（52 字节 = 13 个 little-endian float，无帧尾）
 * Input:
 * Input:
 * Output:
 * Return:
 * Others:          顺序 quat(4) → accel(3) → gyro(3) → mag(3)
 ***********************************************************/
static void libOpenIMU_ParseHexFrameData(const uint8_t *data)
{
    sLibOpenIMU->frame.timestampMs = sLibOpenIMU_IO->getTickMs();
    sLibOpenIMU->frame.quat_wxyz[0] = libOpenIMU_BytesToFloatLE(data + 0);
    sLibOpenIMU->frame.quat_wxyz[1] = libOpenIMU_BytesToFloatLE(data + 4);
    sLibOpenIMU->frame.quat_wxyz[2] = libOpenIMU_BytesToFloatLE(data + 8);
    sLibOpenIMU->frame.quat_wxyz[3] = libOpenIMU_BytesToFloatLE(data + 12);
    sLibOpenIMU->frame.accel_g[0] = libOpenIMU_BytesToFloatLE(data + 16);
    sLibOpenIMU->frame.accel_g[1] = libOpenIMU_BytesToFloatLE(data + 20);
    sLibOpenIMU->frame.accel_g[2] = libOpenIMU_BytesToFloatLE(data + 24);
    sLibOpenIMU->frame.gyro_dps[0] = libOpenIMU_BytesToFloatLE(data + 28);
    sLibOpenIMU->frame.gyro_dps[1] = libOpenIMU_BytesToFloatLE(data + 32);
    sLibOpenIMU->frame.gyro_dps[2] = libOpenIMU_BytesToFloatLE(data + 36);
    sLibOpenIMU->frame.mag_uT[0] = libOpenIMU_BytesToFloatLE(data + 40);
    sLibOpenIMU->frame.mag_uT[1] = libOpenIMU_BytesToFloatLE(data + 44);
    sLibOpenIMU->frame.mag_uT[2] = libOpenIMU_BytesToFloatLE(data + 48);

    sLibOpenIMU->frameValid = true;
}

/***********************************************************
 * Function:        libOpenIMU_TryParseHexFrame
 * Description:     接收并解析一帧二进制格式数据（固定帧长 52 字节，无帧尾标记）
 * Input:
 * Input:
 * Output:
 * Return:          true=已解析到一帧有效数据
 * Others:          二进制数据可能含 0x0A，不能走行解析，必须按固定帧长定帧
 ***********************************************************/
static bool libOpenIMU_TryParseHexFrame(void)
{
    /* 固定帧长字节数：在使用时实时计算（随 libOpenIMU_Frame 结构与 sizeof(float) 自动调整） */
    const uint32_t hexFrameDataBytes = (uint32_t)( LIBOPENIMU_FRAME_FLOAT_CNT * sizeof( float ) );

    /* 1) 搬入原始字节（不做行解析） */
    libOpenIMU_RxAccumulateBytes();

    /* 2) 固定帧长（13 个 little-endian float，无帧尾标记），收满一帧即解析 */
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

/***********************************************************
 * Function:        libOpenIMU_RequestFrame
 * Description:     稳态：发送 AT+requestFrame 并在 ≤3ms 内等待并解析一帧
 * Input:
 * Input:
 * Output:
 * Return:
 * Others:          Other Description.
 ***********************************************************/
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

/***********************************************************
 * Function:        libOpenIMU_DetectBaudrate
 * Description:     探测 OpenIMU 模组当前使用的波特率：遍历支持的波特率，
 *                  逐档切换主机串口、发送 AT\r\n 并等待 \r\nOK\r\n 响应
 * Input:
 * Input:
 * Output:
 * Return:          命中返回对应 libOpenIMU_BaudRate；全部未命中返回 LIBOPENIMU_BAUD_UNKNOWN
 * Others:          命中后主机串口即保持在探测到的波特率上，无需再次切换
 ***********************************************************/
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

/***********************************************************
 * Function:        libOpenIMU_Init
 * Description:     初始化模块并复位状态机到 INIT（先探测波特率）
 * Input:
 * Input:
 * Output:
 * Return:
 * Others:          Other Description.
 ***********************************************************/
void libOpenIMU_Init(libOpenIMU_IO *pIo, libOpenIMU_TypeDef *pInst, libOpenIMU_UploadFormat uploadFormat)
{
    sLibOpenIMU_IO = pIo;
    sLibOpenIMU = pInst;
    memset(sLibOpenIMU, 0, sizeof(*sLibOpenIMU));
    sLibOpenIMU->uploadFormat = uploadFormat;
    sLibOpenIMU->algFilterType = sLibOpenIMU_IO->getXfpkType();

    /* 先探测模组当前波特率（成功后主机串口已保持在该波特率），再进入初始化状态机 */
    sLibOpenIMU->baud = libOpenIMU_DetectBaudrate();

    libOpenIMU_SetState(LIBOPENIMU_STATE_INIT);
}

/***********************************************************
 * Function:        libOpenIMU_Poll
 * Description:     状态机推进（由 IMUSample_task 每轮调用）
 * Input:
 * Input:
 * Output:
 * Return:
 * Others:          Other Description.
 ***********************************************************/
void libOpenIMU_Poll(void)
{
    switch (sLibOpenIMU->state)
    {
    case LIBOPENIMU_STATE_INIT:
        /* 上电/启动等待已由移植层 libOpenIMU_Portable_Init（bootInitDelayMs）完成，直接进入配置阶段 */
        libOpenIMU_DrainRx();
        libOpenIMU_SetState(LIBOPENIMU_STATE_SET_CONFIG_MODE);
        break;

    case LIBOPENIMU_STATE_SET_CONFIG_MODE:
    case LIBOPENIMU_STATE_SET_LED_OFF:
    case LIBOPENIMU_STATE_SET_ALG_FILTER:
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

bool libOpenIMU_GetFrame(libOpenIMU_Frame *pFrame)
{
    if (pFrame == NULL || !sLibOpenIMU->frameValid)
    {
        return false;
    }
    *pFrame = sLibOpenIMU->frame;
    return true;
}

/* 打印开关：libOpenIMU_PrintFrame 中各数据组可单独控制打印（1=打印，0=不打印） */
#define LIBOPENIMU_PRINT_QUAT  (1)  /* 四元数 */
#define LIBOPENIMU_PRINT_ACCEL (0)  /* 加速度计 */
#define LIBOPENIMU_PRINT_GYRO  (0)  /* 陀螺仪 */
#define LIBOPENIMU_PRINT_MAG   (0)  /* 磁力计 */

void libOpenIMU_PrintFrame(void)
{
    libOpenIMU_Frame *pFrame = &sLibOpenIMU->frame;

    if (!sLibOpenIMU->frameValid)
    {
        printf("[OpenIMU] no valid frame yet\r\n");
        return;
    }

    printf("[OpenIMU] t=%lu", (unsigned long)pFrame->timestampMs);
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

    printf("\r\n");
}
