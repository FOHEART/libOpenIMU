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
 *     4. libOpenIMU_PrintFrame
 * History:
 *     <author>  <time>   <version >   <desc>
 *     MaoxiaoHu 16/01/01     1.0   build this moudle
 ***********************************************************/
#ifndef __LIBOPENIMU_H
#define __LIBOPENIMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <limits.h>
#include <stdbool.h>
#include <ctype.h>
#include <math.h>
#include <assert.h>

/* OpenIMU BNO086 一帧已解析数据 */
typedef struct
{
	uint32_t timestampMs;      /*!< 接收到该帧的时间 ms */
	float    quat_wxyz[4];     /*!< 四元数 w,x,y,z */
	float    accel_g[3];       /*!< 加速度计 x,y,z（g） */
	float    gyro_dps[3];      /*!< 陀螺仪 x,y,z（°/s） */
	float    mag_uT[3];       /*!< 磁力计 x,y,z（模组 µT 原始值，不换算） */
}libOpenIMU_Frame;

/* OpenIMU 算法滤波类型（与 AT+CONFIG=algFilterType 取值一致） */
typedef enum {
	XFPK_Base = 0,        /* 游戏旋转矢量（仅陀螺仪+加速度计） */
	XFPK_Additional = 1   /* 标准旋转矢量（陀螺仪+加速度计+磁力计，默认） */
}XFPK_Type;

/* OpenIMU 上传格式（经 libOpenIMU_Init 参数指定，替代编译期宏） */
typedef enum {
	LIBOPENIMU_UPLOAD_FORMAT_STRING = 0,  /* 字符串（13 浮点 CSV） */
	LIBOPENIMU_UPLOAD_FORMAT_HEX = 1      /* 二进制 JustFloat（固定帧长） */
}libOpenIMU_UploadFormat;

/* OpenIMU 开机初始化状态机 */
typedef enum {
	LIBOPENIMU_STATE_INIT                    = 0,  /*!< 上电/复位后初始，等待模组启动完成 */
	LIBOPENIMU_STATE_SET_CONFIG_MODE         = 1,  /*!< 发送 AT+MODE=config，等待 OK */
	LIBOPENIMU_STATE_SET_LED_OFF             = 2,  /*!< 发送 AT+SETLED=OFF，关闭状态 LED */
	LIBOPENIMU_STATE_SET_ALG_FILTER          = 3,  /*!< 发送 AT+CONFIG=algFilterType,<value>，应用算法滤波类型 */
	LIBOPENIMU_STATE_SET_UPLOADFORMAT        = 4,  /*!< 发送 AT+UPLOADFORMAT=string,quat,accel,gyro,mag，等待 OK */
	LIBOPENIMU_STATE_VERIFY_UPLOADFORMAT     = 5,  /*!< 发送 AT+UPLOADFORMAT=?，校验上传格式生效 */
	LIBOPENIMU_STATE_SET_REQUEST_MEASUREMENT = 6,  /*!< 发送 AT+MODE=requestMeasurement，等待 OK */
	LIBOPENIMU_STATE_MEASUREMENT             = 7   /*!< 稳态：AT+requestFrame 逐帧请求并解析 */
}libOpenIMU_State;

/* 平台抽象：时间（ms/us）与串口读写，由 portable 层实现并通过 libOpenIMU_Init 传入 */
typedef struct
{
	uint32_t ( *getTickMs )( void );                     /*!< 获取当前毫秒（支持 32 位回绕） */
	uint32_t ( *getUs )( void );                         /*!< 获取当前微秒（自由运行计数器原始值） */
	uint32_t ( *rxAvailable )( void );                   /*!< 串口可读字节数 */
	uint32_t ( *read )( uint8_t *pBuf, uint32_t len );   /*!< 串口读，返回实际读取字节数 */
	uint32_t ( *write )( const uint8_t *pData, uint32_t len ); /*!< 串口写 */
	uint32_t fullCycleUs;                                /*!< 微秒计数器满周期（us），用于回绕计算 */
	XFPK_Type ( *getXfpkType )( void );                  /*!< 读取节点配置的算法滤波类型 */
}libOpenIMU_IO;

/* UART6 接收累积缓冲大小（缓存未成行的数据） */
#define LIBOPENIMU_RX_BUF_SIZE (128)

/* OpenIMU 模块运行状态（由 portable 层实例化，指针传入 libOpenIMU_Init） */
typedef struct
{
	libOpenIMU_State state;                /*!< 当前状态 */
	uint8_t retryCount;                    /*!< 当前状态重试计数 */
	uint32_t stateStartMs;                 /*!< 进入当前状态的时刻 ms */
	bool cmdPending;                       /*!< 当前状态已发送命令，等待响应 */
	bool formatSeen;                       /*!< 校验状态：已看到期望的 +UploadFormat 行 */
	bool frameValid;                       /*!< 最新帧是否有效 */
	XFPK_Type algFilterType;               /*!< 算法滤波类型（配置阶段应用） */
	libOpenIMU_UploadFormat uploadFormat;  /*!< 上传格式（libOpenIMU_Init 参数传入） */
	libOpenIMU_Frame frame;                /*!< 最新有效帧 */
	uint8_t rxBuf[LIBOPENIMU_RX_BUF_SIZE]; /*!< UART6 接收累积缓冲 */
	uint16_t rxLen;                        /*!< 累积缓冲有效长度 */
}libOpenIMU_TypeDef;

void libOpenIMU_Init( libOpenIMU_IO *pIo, libOpenIMU_TypeDef *pInst, libOpenIMU_UploadFormat uploadFormat );
void libOpenIMU_Poll( void );
bool libOpenIMU_GetFrame( libOpenIMU_Frame *pFrame );
void libOpenIMU_PrintFrame( void );

#ifdef __cplusplus
}
#endif

#endif /* __LIBOPENIMU_H */
