#ifndef __BOOTLOADER_H
#define __BOOTLOADER_H

#include "main.h"

/* Bootloader command set */
#define GET_CMD_COMMAND        0x00U  /*!< Get CMD command               */
#define GET_VER_COMMAND        0x01U  /*!< Get Version command           */
#define GET_ID_COMMAND         0x02U  /*!< Get ID command                */
#define RMEM_COMMAND           0x11U  /*!< Read Memory command           */
#define GO_COMMAND             0x21U  /*!< Go command                    */
#define WMEM_COMMAND           0x31U  /*!< Write Memory command          */
#define EMEM_COMMAND           0x44U  /*!< Erase Memory command          */
#define WP_COMMAND             0x63U  /*!< Write Protect command         */
#define WU_COMMAND             0x73U  /*!< Write Unprotect command       */
#define RP_COMMAND             0x82U  /*!< Readout Protect command       */
#define RU_COMMAND             0x92U  /*!< Readout Unprotect command     */

#define BL_SPI_SOF 0x5AU
#define BL_ACK 0x79U
#define BL_NAK 0x1FU

/* Extended erase special parameters */
#define ERASE_ALL    0xFFFFU
#define ERASE_BANK1  0xFFFEU
#define ERASE_BANK2  0xFFFDU

void     BL_Init(SPI_HandleTypeDef *hspi);
void     BL_Get_Command(uint8_t *pData);
uint8_t  BL_GetVersion_Command(void);
uint16_t BL_GetID_Command(void);
void     BL_ReadMemory_Command(uint32_t address, uint8_t nob, uint8_t *pData);
void     BL_Go_Command(uint32_t address);
void     BL_WriteMemory_Command(uint32_t address, uint8_t nob, uint8_t *pData);
void     BL_EraseMemory_Command(uint16_t nb, uint8_t code);
void     BL_WriteProtect_Command(uint8_t nb, uint8_t *codes);
void     BL_WriteUnprotect_Command(void);
void     BL_ReadoutProtect_Command(void);
void     BL_ReadoutUnprotect_Command(void);

#endif /* __BOOTLOADER_H */
