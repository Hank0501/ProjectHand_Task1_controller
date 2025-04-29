#ifndef __CAN__SYS__TABLE_H
#define __CAN__SYS__TABLE_H

#include "stm32f4xx_hal.h"

/* Note ------------------------------*/

// CAN packages are transfered in 8 bytes and the below form :
// | reseved | instruction | send_from_whom | instruction index | 4bytes param

/**************************************
Only six kinds of ID wil be in in this CAN system :
01 : send from raspberry PI,     received by STM and UNO
02 : send from raspberry PI,     received by STM only
03 : send from raspberry PI,     received by UNO only
05 : send from STM,              received by raspberry PI only
07 : send from UNO,              received by STM and UNO
08 : send from UNO,              received by raspberry PI only

Only three

***************************************/

#define TxID_ONLY_PI 5

#define NUM_CAN_DATA_BITS 8

#define CANSYS_INSTR_CMD 1
#define CANSYS_INSTR_DATA 2
#define CANSYS_INSTR_STATE 3

#define CANSYS_PI 1
#define CANSYS_STM 2
#define CANSYS_UNO 3

// two bytes : [[INSTR]_[SEND FROM WHOM]][INDEX]
#define CANSYS_CMD_PI_CONNECTING 0x1101
#define CANSYS_CMD_PI_INITSTATE 0x1103
#define CANSYS_CMD_PI_HOLD 0x1104
#define CANSYS_CMD_PI_REALEASE 0x1105
// #define CANSYS_CMD_PI_HOLDINGFORCE 0x0003

#define CANSYS_STATE_STM_CONNECT_NOTOK 0x3200
#define CANSYS_STATE_STM_CONNECT_OK 0x3201
#define CANSYS_STATE_STM_INIT_OK 0x3202
#define CANSYS_STATE_STM_INIT_NOTOK 0x3203
#define CANSYS_STATE_STM_START_HOLDING 0x3204
#define CANSYS_STATE_STM_START_RELEASING 0x3205
#define CANSYS_STATE_STM_MISS_HOLDING 0x3206
#define CANSYS_STATE_STM_MISS_RELEASING 0x3207

#define CANSYS_STATE_ONLY_FOR_EXPERIMENT_HOLDING 0x1106
#define CANSYS_STATE_ONLY_FOR_EXPERIMENT_RELEASING 0x1107

// uint16_t canIndexProcess(uint8_t *array)
// {
//     return (uint16_t)(((array[1] << 4) | array[2]) << 8) | array[3];
// }

void TxDataAssign(uint8_t *TxData, uint16_t dataType, int param)
{
    TxData[0] = 0;
    TxData[1] = (uint8_t)((dataType & 0xf000) >> 12);
    TxData[2] = (uint8_t)((dataType & 0x0f00) >> 8);
    TxData[3] = (uint8_t)((dataType & 0x00ff));
    TxData[4] = (uint8_t)((param & 0xff000000) >> 24);
    TxData[5] = (uint8_t)((param & 0x00ff0000) >> 16);
    TxData[6] = (uint8_t)((param & 0x0000ff00) >> 8);
    TxData[7] = (uint8_t)((param & 0x000000ff));
}

int RxParamToInt(uint8_t *RxData)
{
    int var = 0;
    var |= (int)RxData[4];
    var |= (int)RxData[5] << 8;
    var |= (int)RxData[6] << 16;
    var |= (int)RxData[7] << 24;
    return var;
}
#endif /* CAN__SYS__TABLE_H */