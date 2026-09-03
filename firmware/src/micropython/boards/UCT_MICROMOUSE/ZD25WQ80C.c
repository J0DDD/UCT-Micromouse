/*
 * ZD25WQ80C.c
 *
 *  Created on: 2026-07-11
 *      Author: Jesse Jabez Arendse
 *
 *  Driver for Zetta ZD25WQ80C 8Mbit SPI NOR Flash.
 *  Datasheet: MicroMouseTemplate_Code/Core/Src/ZD25WQ80C/ZD25WQ80C_datasheet.pdf
 */

#include "ZD25WQ80C.h"

ZD25WQ80C_t flash = {
    .spi         = ZD25WQ80C_SPI_BUS,
    .cs_port     = ZD25WQ80C_CS_PORT,
    .cs_pin      = ZD25WQ80C_CS_PIN,
    .initialized = 0,
    .status_reg1 = 0,
    .status_reg2 = 0,
};

/* --------------------------------------------------------------------------
 * Private helpers
 * -------------------------------------------------------------------------- */

static inline void cs_assert(void)
{
    HAL_GPIO_WritePin(flash.cs_port, flash.cs_pin, GPIO_PIN_RESET);
}

static inline void cs_deassert(void)
{
    uint32_t timeout = 50000;
    while ((flash.spi->Instance->SR & SPI_SR_BSY) && --timeout);
    HAL_GPIO_WritePin(flash.cs_port, flash.cs_pin, GPIO_PIN_SET);
}

static inline uint8_t spi_transfer_byte(uint8_t tx)
{
    if (!(flash.spi->Instance->CR1 & SPI_CR1_SPE)) {
        __HAL_SPI_ENABLE(flash.spi);
    }
    uint32_t timeout = 50000;
    while (!(flash.spi->Instance->SR & SPI_SR_TXE) && --timeout);
    *(volatile uint8_t *)&flash.spi->Instance->DR = tx;
    timeout = 50000;
    while (!(flash.spi->Instance->SR & SPI_SR_RXNE) && --timeout);
    return *(volatile uint8_t *)&flash.spi->Instance->DR;
}

/* --------------------------------------------------------------------------
 * Internal helpers
 * -------------------------------------------------------------------------- */

static uint8_t read_status_reg1(void)
{
    cs_assert();
    spi_transfer_byte(ZD25WQ80C_CMD_RDSR1);
    uint8_t sr = spi_transfer_byte(0x00);
    cs_deassert();
    return sr;
}

/* Blocks until WIP is cleared. Returns HAL_TIMEOUT if the device never clears. */
static HAL_StatusTypeDef wait_for_ready(uint32_t timeout_ms)
{
    for (volatile uint32_t loop = 0; loop < timeout_ms * 5000; loop++)
    {
        if (!(read_status_reg1() & ZD25WQ80C_SR1_WIP))
            return HAL_OK;
    }
    return HAL_TIMEOUT;
}

static HAL_StatusTypeDef write_enable(void)
{
    cs_assert();
    spi_transfer_byte(ZD25WQ80C_CMD_WREN);
    cs_deassert();
    return HAL_OK;
}

/* --------------------------------------------------------------------------
 * Lifecycle
 * -------------------------------------------------------------------------- */

/**
 * @brief  Initialise the ZD25WQ80C by waking it and verifying the JEDEC ID.
 * @retval 1  Success — device found and confirmed.
 * @retval 0  Failure — SPI error or ID mismatch.
 */
uint8_t initZD25WQ80C(void)
{
    /* Ensure SPI peripheral is enabled */
    __HAL_SPI_ENABLE(flash.spi);

    /* Drive CS high before any transaction */
    cs_deassert();

    /* Release from deep power-down / allow tVSL (power-on setup time) */
    ZD25WQ80C_WakeUp();
    for (volatile int d = 0; d < 50000; d++) { __NOP(); }

    uint8_t mfr = 0, id_h = 0, id_l = 0;
    for (int retry = 0; retry < 15; retry++) {
        if (ZD25WQ80C_ReadJEDECID(&mfr, &id_h, &id_l)) {
            if (mfr == ZD25WQ80C_MANUFACTURER_ID &&
                id_h == ZD25WQ80C_JEDEC_ID_HIGH  &&
                id_l == ZD25WQ80C_JEDEC_ID_LOW) {
                flash.initialized = 1;
                refreshZD25WQ80CValues();
                return 1;
            }
        }
        for (volatile int d = 0; d < 20000; d++) { __NOP(); }
    }

    return 0;
}

/**
 * @brief  Read both status registers into the flash struct.
 */
void refreshZD25WQ80CValues(void)
{
    cs_assert();
    spi_transfer_byte(ZD25WQ80C_CMD_RDSR1);
    flash.status_reg1 = spi_transfer_byte(0x00);
    cs_deassert();

    cs_assert();
    spi_transfer_byte(ZD25WQ80C_CMD_RDSR2);
    flash.status_reg2 = spi_transfer_byte(0x00);
    cs_deassert();
}

/* --------------------------------------------------------------------------
 * Identity
 * -------------------------------------------------------------------------- */

/**
 * @brief  Read the 3-byte JEDEC ID.
 * @param  mfr     Manufacturer ID output (0xBA for Zetta).
 * @param  id_high Memory type byte output (0x40).
 * @param  id_low  Capacity byte output (0x14 = 8Mbit).
 * @retval 1  Success.
 * @retval 0  SPI error.
 */
uint8_t ZD25WQ80C_ReadJEDECID(uint8_t *mfr, uint8_t *id_high, uint8_t *id_low)
{
    cs_assert();
    spi_transfer_byte(ZD25WQ80C_CMD_RDID);
    *mfr     = spi_transfer_byte(0x00);
    *id_high = spi_transfer_byte(0x00);
    *id_low  = spi_transfer_byte(0x00);
    cs_deassert();
    return 1;
}

/* --------------------------------------------------------------------------
 * Power management
 * -------------------------------------------------------------------------- */

/**
 * @brief  Put the device into Deep Power Down.
 */
void ZD25WQ80C_PowerDown(void)
{
    cs_assert();
    spi_transfer_byte(ZD25WQ80C_CMD_DP);
    cs_deassert();
    flash.initialized = 0;
}

/**
 * @brief  Release from Deep Power Down. Allow 3 µs (tRES1) before next access.
 */
void ZD25WQ80C_WakeUp(void)
{
    cs_assert();
    spi_transfer_byte(ZD25WQ80C_CMD_RDP);
    spi_transfer_byte(0xFF);
    spi_transfer_byte(0xFF);
    spi_transfer_byte(0xFF);
    cs_deassert();
}

/**
 * @brief  Issue a software reset (RSTEN then RST). Allows 30 µs before next access.
 */
void ZD25WQ80C_SoftwareReset(void)
{
    cs_assert();
    spi_transfer_byte(ZD25WQ80C_CMD_RSTEN);
    cs_deassert();

    cs_assert();
    spi_transfer_byte(ZD25WQ80C_CMD_RST);
    cs_deassert();

    flash.initialized = 0;
}

/* --------------------------------------------------------------------------
 * Read operations
 * -------------------------------------------------------------------------- */

/**
 * @brief  Read a block of data from any 24-bit address.
 * @param  address  24-bit byte address.
 * @param  buf      Destination buffer.
 * @param  len      Number of bytes to read.
 * @retval HAL_OK on success.
 */
HAL_StatusTypeDef ZD25WQ80C_Read(uint32_t address, uint8_t *buf, uint32_t len)
{
    cs_assert();
    spi_transfer_byte(ZD25WQ80C_CMD_READ);
    spi_transfer_byte((uint8_t)(address >> 16));
    spi_transfer_byte((uint8_t)(address >> 8));
    spi_transfer_byte((uint8_t)(address));
    for (uint32_t i = 0; i < len; i++) {
        buf[i] = spi_transfer_byte(0x00);
    }
    cs_deassert();
    return HAL_OK;
}

/**
 * @brief  Fast Read — supports higher clock speeds via one dummy byte after address.
 * @param  address  24-bit byte address.
 * @param  buf      Destination buffer.
 * @param  len      Number of bytes to read.
 * @retval HAL_OK on success.
 */
HAL_StatusTypeDef ZD25WQ80C_FastRead(uint32_t address, uint8_t *buf, uint32_t len)
{
    cs_assert();
    spi_transfer_byte(ZD25WQ80C_CMD_FAST_READ);
    spi_transfer_byte((uint8_t)(address >> 16));
    spi_transfer_byte((uint8_t)(address >> 8));
    spi_transfer_byte((uint8_t)(address));
    spi_transfer_byte(0xFF); // dummy byte
    for (uint32_t i = 0; i < len; i++) {
        buf[i] = spi_transfer_byte(0x00);
    }
    cs_deassert();
    return HAL_OK;
}

/* --------------------------------------------------------------------------
 * Write operations
 * -------------------------------------------------------------------------- */

/**
 * @brief  Program up to 256 bytes into a single page.
 *         The address + len must not cross a 256-byte page boundary.
 * @param  address  24-bit byte address (must be page-aligned for a full page).
 * @param  buf      Source data buffer.
 * @param  len      Bytes to program (1–256).
 * @retval HAL_OK on success, HAL_TIMEOUT if WIP never cleared, HAL_ERROR on SPI fault.
 */
HAL_StatusTypeDef ZD25WQ80C_PageProgram(uint32_t address, const uint8_t *buf, uint16_t len)
{
    write_enable();

    cs_assert();
    spi_transfer_byte(ZD25WQ80C_CMD_PP);
    spi_transfer_byte((uint8_t)(address >> 16));
    spi_transfer_byte((uint8_t)(address >> 8));
    spi_transfer_byte((uint8_t)(address));
    for (uint16_t i = 0; i < len; i++) {
        spi_transfer_byte(buf[i]);
    }
    cs_deassert();

    /* tPP typical 0.5 ms, max 3 ms */
    return wait_for_ready(50);
}

/* --------------------------------------------------------------------------
 * Erase operations
 * -------------------------------------------------------------------------- */

/**
 * @brief  Erase a 4 KB sector.
 * @param  address  Any address within the target sector.
 * @retval HAL_OK on success.
 */
HAL_StatusTypeDef ZD25WQ80C_SectorErase(uint32_t address)
{
    write_enable();

    cs_assert();
    spi_transfer_byte(ZD25WQ80C_CMD_SE);
    spi_transfer_byte((uint8_t)(address >> 16));
    spi_transfer_byte((uint8_t)(address >> 8));
    spi_transfer_byte((uint8_t)(address));
    cs_deassert();

    /* tSE typical 45 ms, max 300 ms */
    return wait_for_ready(500);
}

/**
 * @brief  Erase a 32 KB block.
 * @param  address  Any address within the target block.
 * @retval HAL_OK on success.
 */
HAL_StatusTypeDef ZD25WQ80C_BlockErase32K(uint32_t address)
{
    write_enable();

    cs_assert();
    spi_transfer_byte(ZD25WQ80C_CMD_HBE);
    spi_transfer_byte((uint8_t)(address >> 16));
    spi_transfer_byte((uint8_t)(address >> 8));
    spi_transfer_byte((uint8_t)(address));
    cs_deassert();

    /* tHBE typical 120 ms, max 800 ms */
    return wait_for_ready(1000);
}

/**
 * @brief  Erase a 64 KB block.
 * @param  address  Any address within the target block.
 * @retval HAL_OK on success.
 */
HAL_StatusTypeDef ZD25WQ80C_BlockErase(uint32_t address)
{
    write_enable();

    cs_assert();
    spi_transfer_byte(ZD25WQ80C_CMD_BE);
    spi_transfer_byte((uint8_t)(address >> 16));
    spi_transfer_byte((uint8_t)(address >> 8));
    spi_transfer_byte((uint8_t)(address));
    cs_deassert();

    /* tBE typical 150 ms, max 1000 ms */
    return wait_for_ready(1200);
}

/**
 * @brief  Erase the entire chip (~10 s typical, 200 s worst case).
 * @retval HAL_OK on success.
 */
HAL_StatusTypeDef ZD25WQ80C_ChipErase(void)
{
    HAL_StatusTypeDef ret = write_enable();
    if (ret != HAL_OK)
        return ret;

    uint8_t cmd = ZD25WQ80C_CMD_CE;
    cs_assert();
    ret = HAL_SPI_Transmit(flash.spi, &cmd, 1, ZD25WQ80C_SPI_TIMEOUT);
    cs_deassert();

    if (ret != HAL_OK)
        return ret;

    /* tCE max 200 s */
    return wait_for_ready(200000);
}
