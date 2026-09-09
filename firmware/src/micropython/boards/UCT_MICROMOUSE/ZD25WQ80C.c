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
    HAL_GPIO_WritePin(flash.cs_port, flash.cs_pin, GPIO_PIN_SET);
}

static uint8_t read_status_reg1(void)
{
    uint8_t cmd = ZD25WQ80C_CMD_RDSR1;
    uint8_t sr  = 0;
    cs_assert();
    HAL_SPI_Transmit(flash.spi, &cmd, 1, ZD25WQ80C_SPI_TIMEOUT);
    HAL_SPI_Receive(flash.spi, &sr, 1, ZD25WQ80C_SPI_TIMEOUT);
    cs_deassert();
    return sr;
}

/* Blocks until WIP is cleared. Returns HAL_TIMEOUT if the device never clears. */
static HAL_StatusTypeDef wait_for_ready(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    volatile uint32_t loop_count = 100000;
    while (read_status_reg1() & ZD25WQ80C_SR1_WIP)
    {
        if (--loop_count == 0 || (HAL_GetTick() - start) >= timeout_ms)
            return HAL_TIMEOUT;
    }
    return HAL_OK;
}

static HAL_StatusTypeDef write_enable(void)
{
    uint8_t cmd = ZD25WQ80C_CMD_WREN;
    cs_assert();
    HAL_StatusTypeDef ret = HAL_SPI_Transmit(flash.spi, &cmd, 1, ZD25WQ80C_SPI_TIMEOUT);
    cs_deassert();
    return ret;
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
    /* Drive CS high before any transaction */
    cs_deassert();

    /* Release from deep power-down in case the device was left powered but
       sleeping from a previous session. The RDP command issues a dummy byte
       sequence; tRES1 = 3 µs so a HAL_Delay(1) is sufficient. */
    ZD25WQ80C_WakeUp();
    HAL_Delay(1);

    uint8_t mfr, id_h, id_l;
    if (!ZD25WQ80C_ReadJEDECID(&mfr, &id_h, &id_l))
        return 0;

    if (mfr != ZD25WQ80C_MANUFACTURER_ID ||
        id_h != ZD25WQ80C_JEDEC_ID_HIGH  ||
        id_l != ZD25WQ80C_JEDEC_ID_LOW)
        return 0;

    flash.initialized = 1;
    refreshZD25WQ80CValues();
    return 1;
}

/**
 * @brief  Read both status registers into the flash struct.
 */
void refreshZD25WQ80CValues(void)
{
    uint8_t cmd;
    uint8_t sr;

    cmd = ZD25WQ80C_CMD_RDSR1;
    cs_assert();
    HAL_SPI_Transmit(flash.spi, &cmd, 1, ZD25WQ80C_SPI_TIMEOUT);
    HAL_SPI_Receive(flash.spi, &sr, 1, ZD25WQ80C_SPI_TIMEOUT);
    cs_deassert();
    flash.status_reg1 = sr;

    cmd = ZD25WQ80C_CMD_RDSR2;
    cs_assert();
    HAL_SPI_Transmit(flash.spi, &cmd, 1, ZD25WQ80C_SPI_TIMEOUT);
    HAL_SPI_Receive(flash.spi, &sr, 1, ZD25WQ80C_SPI_TIMEOUT);
    cs_deassert();
    flash.status_reg2 = sr;
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
    uint8_t cmd  = ZD25WQ80C_CMD_RDID;
    uint8_t resp[3] = {0};

    cs_assert();
    HAL_StatusTypeDef ret = HAL_SPI_Transmit(flash.spi, &cmd, 1, ZD25WQ80C_SPI_TIMEOUT);
    if (ret == HAL_OK)
        ret = HAL_SPI_Receive(flash.spi, resp, 3, ZD25WQ80C_SPI_TIMEOUT);
    cs_deassert();

    if (ret != HAL_OK)
        return 0;

    *mfr     = resp[0];
    *id_high = resp[1];
    *id_low  = resp[2];
    return 1;
}

/* --------------------------------------------------------------------------
 * Power management
 * -------------------------------------------------------------------------- */

/**
 * @brief  Enter Deep Power Down mode (~1 µA typical standby current).
 */
void ZD25WQ80C_DeepPowerDown(void)
{
    uint8_t cmd = ZD25WQ80C_CMD_DP;
    cs_assert();
    HAL_SPI_Transmit(flash.spi, &cmd, 1, ZD25WQ80C_SPI_TIMEOUT);
    cs_deassert();
    flash.initialized = 0;
}

/**
 * @brief  Release from Deep Power Down. Allow 3 µs (tRES1) before next access.
 */
void ZD25WQ80C_WakeUp(void)
{
    uint8_t buf[4] = {ZD25WQ80C_CMD_RDP, 0xFF, 0xFF, 0xFF};
    cs_assert();
    HAL_SPI_Transmit(flash.spi, buf, 4, ZD25WQ80C_SPI_TIMEOUT);
    cs_deassert();
}

/**
 * @brief  Issue a software reset (RSTEN then RST). Allows 30 µs before next access.
 */
void ZD25WQ80C_SoftwareReset(void)
{
    uint8_t cmd;

    cmd = ZD25WQ80C_CMD_RSTEN;
    cs_assert();
    HAL_SPI_Transmit(flash.spi, &cmd, 1, ZD25WQ80C_SPI_TIMEOUT);
    cs_deassert();

    cmd = ZD25WQ80C_CMD_RST;
    cs_assert();
    HAL_SPI_Transmit(flash.spi, &cmd, 1, ZD25WQ80C_SPI_TIMEOUT);
    cs_deassert();

    flash.initialized = 0;
}

/* --------------------------------------------------------------------------
 * Read operations
 * -------------------------------------------------------------------------- */

/**
 * @brief  Read data at up to 33 MHz (standard READ command, no dummy byte).
 * @param  address  24-bit byte address within the 1 MB address space.
 * @param  buf      Destination buffer.
 * @param  len      Number of bytes to read.
 * @retval HAL_OK on success.
 */
HAL_StatusTypeDef ZD25WQ80C_Read(uint32_t address, uint8_t *buf, uint32_t len)
{
    uint8_t cmd[4] = {
        ZD25WQ80C_CMD_READ,
        (uint8_t)(address >> 16),
        (uint8_t)(address >> 8),
        (uint8_t)(address),
    };

    cs_assert();
    HAL_StatusTypeDef ret = HAL_SPI_Transmit(flash.spi, cmd, 4, ZD25WQ80C_SPI_TIMEOUT);
    if (ret == HAL_OK)
        ret = HAL_SPI_Receive(flash.spi, buf, len, ZD25WQ80C_SPI_TIMEOUT);
    cs_deassert();
    return ret;
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
    uint8_t cmd[5] = {
        ZD25WQ80C_CMD_FAST_READ,
        (uint8_t)(address >> 16),
        (uint8_t)(address >> 8),
        (uint8_t)(address),
        0xFF,   /* dummy byte */
    };

    cs_assert();
    HAL_StatusTypeDef ret = HAL_SPI_Transmit(flash.spi, cmd, 5, ZD25WQ80C_SPI_TIMEOUT);
    if (ret == HAL_OK)
        ret = HAL_SPI_Receive(flash.spi, buf, len, ZD25WQ80C_SPI_TIMEOUT);
    cs_deassert();
    return ret;
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
    HAL_StatusTypeDef ret;

    ret = write_enable();
    if (ret != HAL_OK)
        return ret;

    uint8_t cmd[4] = {
        ZD25WQ80C_CMD_PP,
        (uint8_t)(address >> 16),
        (uint8_t)(address >> 8),
        (uint8_t)(address),
    };

    cs_assert();
    ret = HAL_SPI_Transmit(flash.spi, cmd, 4, ZD25WQ80C_SPI_TIMEOUT);
    if (ret == HAL_OK)
        ret = HAL_SPI_Transmit(flash.spi, (uint8_t *)buf, len, ZD25WQ80C_SPI_TIMEOUT);
    cs_deassert();

    if (ret != HAL_OK)
        return ret;

    /* tPP typical 0.5 ms, max 3 ms */
    return wait_for_ready(10);
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
    HAL_StatusTypeDef ret = write_enable();
    if (ret != HAL_OK)
        return ret;

    uint8_t cmd[4] = {
        ZD25WQ80C_CMD_SE,
        (uint8_t)(address >> 16),
        (uint8_t)(address >> 8),
        (uint8_t)(address),
    };

    cs_assert();
    ret = HAL_SPI_Transmit(flash.spi, cmd, 4, ZD25WQ80C_SPI_TIMEOUT);
    cs_deassert();

    if (ret != HAL_OK)
        return ret;

    /* tSE typical 60 ms, max 400 ms */
    return wait_for_ready(500);
}

/**
 * @brief  Erase a 32 KB half-block.
 * @param  address  Any address within the target half-block.
 * @retval HAL_OK on success.
 */
HAL_StatusTypeDef ZD25WQ80C_HalfBlockErase(uint32_t address)
{
    HAL_StatusTypeDef ret = write_enable();
    if (ret != HAL_OK)
        return ret;

    uint8_t cmd[4] = {
        ZD25WQ80C_CMD_HBE,
        (uint8_t)(address >> 16),
        (uint8_t)(address >> 8),
        (uint8_t)(address),
    };

    cs_assert();
    ret = HAL_SPI_Transmit(flash.spi, cmd, 4, ZD25WQ80C_SPI_TIMEOUT);
    cs_deassert();

    if (ret != HAL_OK)
        return ret;

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
    HAL_StatusTypeDef ret = write_enable();
    if (ret != HAL_OK)
        return ret;

    uint8_t cmd[4] = {
        ZD25WQ80C_CMD_BE,
        (uint8_t)(address >> 16),
        (uint8_t)(address >> 8),
        (uint8_t)(address),
    };

    cs_assert();
    ret = HAL_SPI_Transmit(flash.spi, cmd, 4, ZD25WQ80C_SPI_TIMEOUT);
    cs_deassert();

    if (ret != HAL_OK)
        return ret;

    /* tBE typical 150 ms, max 1 s */
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
