/*
 * ZD25WQ80C.h
 *
 *  Created on: 2026-07-11
 *      Author: Jesse Jabez Arendse
 *
 *  Driver for Zetta ZD25WQ80C 8Mbit SPI NOR Flash.
 *  Supports SPI Mode 0 and Mode 3, software CS control.
 */

#ifndef INC_ZD25WQ80C_H_
#define INC_ZD25WQ80C_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"
#include "main.h"  /* FLASH_CS_Pin / FLASH_CS_GPIO_Port */

extern SPI_HandleTypeDef hspi2;
#define ZD25WQ80C_SPI_BUS       (&hspi2)
#define ZD25WQ80C_CS_PIN        (FLASH_CS_Pin)
#define ZD25WQ80C_CS_PORT       (FLASH_CS_GPIO_Port)

#define ZD25WQ80C_SPI_TIMEOUT   1000

/* JEDEC / Device ID */
#define ZD25WQ80C_MANUFACTURER_ID   0xBA
#define ZD25WQ80C_JEDEC_ID_HIGH     0x40   /* ID[15:8] */
#define ZD25WQ80C_JEDEC_ID_LOW      0x14   /* ID[7:0]  */
#define ZD25WQ80C_DEVICE_ID         0x13   /* from REMS */

/* Memory geometry */
#define ZD25WQ80C_PAGE_SIZE         256U
#define ZD25WQ80C_SECTOR_SIZE       (4U * 1024U)
#define ZD25WQ80C_BLOCK32_SIZE      (32U * 1024U)
#define ZD25WQ80C_BLOCK64_SIZE      (64U * 1024U)
#define ZD25WQ80C_FLASH_SIZE        (1024U * 1024U)

/* Instruction set */
#define ZD25WQ80C_CMD_WREN          0x06   /* Write Enable */
#define ZD25WQ80C_CMD_WRDI          0x04   /* Write Disable */
#define ZD25WQ80C_CMD_RDSR1         0x05   /* Read Status Register 1 */
#define ZD25WQ80C_CMD_RDSR2         0x35   /* Read Status Register 2 */
#define ZD25WQ80C_CMD_WRSR          0x01   /* Write Status Register */
#define ZD25WQ80C_CMD_READ          0x03   /* Read Data */
#define ZD25WQ80C_CMD_FAST_READ     0x0B   /* Fast Read */
#define ZD25WQ80C_CMD_PP            0x02   /* Page Program */
#define ZD25WQ80C_CMD_SE            0x20   /* Sector Erase  (4 KB) */
#define ZD25WQ80C_CMD_HBE           0x52   /* Half Block Erase (32 KB) */
#define ZD25WQ80C_CMD_BE            0xD8   /* Block Erase (64 KB) */
#define ZD25WQ80C_CMD_CE            0x60   /* Chip Erase */
#define ZD25WQ80C_CMD_PE            0x81   /* Page Erase */
#define ZD25WQ80C_CMD_DP            0xB9   /* Deep Power Down */
#define ZD25WQ80C_CMD_RDP           0xAB   /* Release from Deep Power Down / Read Device ID */
#define ZD25WQ80C_CMD_REMS          0x90   /* Read Electronic Manufacturer & Device ID */
#define ZD25WQ80C_CMD_RDID          0x9F   /* Read JEDEC ID */
#define ZD25WQ80C_CMD_RUID          0x4B   /* Read Unique ID */
#define ZD25WQ80C_CMD_RSTEN         0x66   /* Reset Enable */
#define ZD25WQ80C_CMD_RST           0x99   /* Reset */

/* Status Register 1 bits */
#define ZD25WQ80C_SR1_WIP           (1U << 0)  /* Write In Progress */
#define ZD25WQ80C_SR1_WEL           (1U << 1)  /* Write Enable Latch */
#define ZD25WQ80C_SR1_BP0           (1U << 2)
#define ZD25WQ80C_SR1_BP1           (1U << 3)
#define ZD25WQ80C_SR1_BP2           (1U << 4)
#define ZD25WQ80C_SR1_BP3           (1U << 5)
#define ZD25WQ80C_SR1_BP4           (1U << 6)
#define ZD25WQ80C_SR1_SRP0          (1U << 7)

/* Status Register 2 bits */
#define ZD25WQ80C_SR2_SRP1          (1U << 0)
#define ZD25WQ80C_SR2_QE            (1U << 1)  /* Quad Enable */

typedef struct
{
    SPI_HandleTypeDef   *spi;
    GPIO_TypeDef        *cs_port;
    uint16_t             cs_pin;
    uint8_t              initialized;

    /* Cached register values updated by refreshZD25WQ80CValues() */
    uint8_t              status_reg1;
    uint8_t              status_reg2;
} ZD25WQ80C_t;

extern ZD25WQ80C_t flash;

/* Lifecycle */
uint8_t initZD25WQ80C(void);
void    refreshZD25WQ80CValues(void);

/* Identity */
uint8_t ZD25WQ80C_ReadJEDECID(uint8_t *mfr, uint8_t *id_high, uint8_t *id_low);

/* Power management */
void ZD25WQ80C_DeepPowerDown(void);
void ZD25WQ80C_WakeUp(void);
void ZD25WQ80C_SoftwareReset(void);

/* Read */
HAL_StatusTypeDef ZD25WQ80C_Read(uint32_t address, uint8_t *buf, uint32_t len);
HAL_StatusTypeDef ZD25WQ80C_FastRead(uint32_t address, uint8_t *buf, uint32_t len);

/* Write */
HAL_StatusTypeDef ZD25WQ80C_PageProgram(uint32_t address, const uint8_t *buf, uint16_t len);

/* Erase */
HAL_StatusTypeDef ZD25WQ80C_SectorErase(uint32_t address);
HAL_StatusTypeDef ZD25WQ80C_HalfBlockErase(uint32_t address);
HAL_StatusTypeDef ZD25WQ80C_BlockErase(uint32_t address);
HAL_StatusTypeDef ZD25WQ80C_ChipErase(void);

#endif /* INC_ZD25WQ80C_H_ */
