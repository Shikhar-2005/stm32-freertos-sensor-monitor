/**
  ******************************************************************************
  * @file           : l3gd20.c
  * @brief          : L3GD20 3-axis gyroscope SPI driver implementation
  *
  * Hardware connection on STM32F429I-DISC1:
  *   SPI5: PF7=SCK, PF8=MISO, PF9=MOSI
  *   CS:   PC1 (active low, software-controlled)
  *
  * SPI Protocol:
  *   - Mode 0 (CPOL=0, CPHA=0)
  *   - 8-bit data, MSB first
  *   - Bit 7 of address byte: 0=write, 1=read
  *   - Bit 6 of address byte: 0=single, 1=auto-increment
  *
  * Datasheet: ST DocID022116 Rev 3
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "l3gd20.h"
#include "main.h"

/* Private defines -----------------------------------------------------------*/

/**
 * @brief SPI timeout in milliseconds.
 *
 * The L3GD20 maximum SPI clock is 10 MHz. At our clock rate, a 7-byte
 * transaction takes well under 1ms. 100ms timeout is extremely generous
 * and only exists to prevent infinite hangs if the SPI bus is broken.
 */
#define L3GD20_SPI_TIMEOUT  100U

/* Private function prototypes -----------------------------------------------*/
static void L3GD20_CS_Low(void);
static void L3GD20_CS_High(void);
static uint8_t L3GD20_ReadReg(uint8_t reg);
static void L3GD20_WriteReg(uint8_t reg, uint8_t value);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Assert chip select (active low).
 *
 * CS must be pulled low BEFORE any SPI transaction begins.
 * The L3GD20 latches the address byte on the first falling edge of SCK
 * after CS goes low.
 */
static void L3GD20_CS_Low(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);
}

/**
 * @brief  Deassert chip select (return high).
 *
 * CS must be pulled high AFTER the SPI transaction completes.
 * This also resets the L3GD20's internal address pointer.
 */
static void L3GD20_CS_High(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief  Read a single register from the L3GD20.
 *
 * Protocol: Send [address | 0x80], receive [data] on next byte.
 * We use HAL_SPI_TransmitReceive because SPI is full-duplex —
 * we must clock out dummy bytes to receive data.
 *
 * @param  reg  Register address (0x00-0x3F)
 * @retval Register value
 */
static uint8_t L3GD20_ReadReg(uint8_t reg)
{
    uint8_t tx[2];
    uint8_t rx[2];

    /* Set read bit (bit 7) on the address byte */
    tx[0] = reg | L3GD20_SPI_READ;
    tx[1] = 0x00;  /* Dummy byte — clocks out data from slave */

    L3GD20_CS_Low();
    HAL_SPI_TransmitReceive(&hspi5, tx, rx, 2, L3GD20_SPI_TIMEOUT);
    L3GD20_CS_High();

    /* rx[0] is garbage (received while sending address), rx[1] is the data */
    return rx[1];
}

/**
 * @brief  Write a single register on the L3GD20.
 *
 * Protocol: Send [address | 0x00], send [data].
 * Write flag is 0, so we just send the raw address.
 *
 * @param  reg    Register address (0x00-0x3F)
 * @param  value  Value to write
 */
static void L3GD20_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2];
    uint8_t rx[2];  /* HAL requires rx buffer even though we don't use it */

    tx[0] = reg | L3GD20_SPI_WRITE;  /* Write flag = 0, so this is just reg */
    tx[1] = value;

    L3GD20_CS_Low();
    HAL_SPI_TransmitReceive(&hspi5, tx, rx, 2, L3GD20_SPI_TIMEOUT);
    L3GD20_CS_High();
}

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Initialize the L3GD20 gyroscope.
 *
 * Configuration:
 *   CTRL_REG1 (0x20) = 0x0F:
 *     - Bit 3 (PD) = 1: Power-on (exit power-down mode)
 *     - Bit 2 (Zen) = 1: Enable Z axis
 *     - Bit 1 (Yen) = 1: Enable Y axis
 *     - Bit 0 (Xen) = 1: Enable X axis
 *     - Bits 7:4 = 0000: ODR = 95 Hz, BW cutoff = 12.5 Hz
 *
 * @note  Must be called before FreeRTOS scheduler starts (uses HAL_Delay).
 */
void L3GD20_Init(void)
{
    /* Ensure CS starts high (deselected) */
    L3GD20_CS_High();

    /*
     * Write CTRL_REG1: power on + enable all axes.
     * We use the lowest ODR (95 Hz) since our task only reads at 10 Hz.
     * Higher ODR would just waste power and fill the FIFO unnecessarily.
     */
    L3GD20_WriteReg(L3GD20_REG_CTRL_REG1, 0x0F);

    /*
     * 1ms delay for sensor stabilization.
     * The L3GD20 datasheet specifies a turn-on time of 250ms for full
     * accuracy, but basic readings are available much sooner. We'll get
     * stable data after a few task cycles anyway.
     */
    HAL_Delay(1);
}

/**
 * @brief  Read the WHO_AM_I register.
 *
 * This is the standard "is the sensor alive?" check. The WHO_AM_I register
 * is read-only and hardcoded in silicon — if we get the wrong value, either
 * the SPI bus is misconfigured or the chip is damaged.
 *
 * @retval Device ID. Expected: 0xD4 (L3GD20) or 0xD7 (L3GD20H)
 */
uint8_t L3GD20_ReadID(void)
{
    return L3GD20_ReadReg(L3GD20_REG_WHO_AM_I);
}

/**
 * @brief  Read all three gyroscope axes in one SPI transaction.
 *
 * Uses the auto-increment feature (bit 6 of address) to read 6 consecutive
 * bytes (OUT_X_L through OUT_Z_H) in a single CS assertion. This is both
 * faster and more consistent than 3 separate 2-byte reads, because all
 * axes are sampled from the same measurement cycle.
 *
 * Byte order: X_L, X_H, Y_L, Y_H, Z_L, Z_H
 * Assembly:   value = (int16_t)((high_byte << 8) | low_byte)
 *
 * @param[out] x  Raw X-axis angular rate
 * @param[out] y  Raw Y-axis angular rate
 * @param[out] z  Raw Z-axis angular rate
 * @retval HAL status (HAL_OK on success)
 */
HAL_StatusTypeDef L3GD20_ReadXYZ(int16_t *x, int16_t *y, int16_t *z)
{
    HAL_StatusTypeDef status;

    /*
     * tx[0] = address byte with READ + AUTO_INCREMENT flags
     * tx[1..6] = dummy bytes to clock out the 6 data bytes
     *
     * Total transaction: 1 address byte + 6 data bytes = 7 bytes
     */
    uint8_t tx[7] = {0};
    uint8_t rx[7] = {0};

    /* 0x28 = OUT_X_L, 0x80 = read, 0x40 = auto-increment */
    tx[0] = L3GD20_REG_OUT_X_L | L3GD20_SPI_READ | L3GD20_SPI_AUTO_INC;

    L3GD20_CS_Low();
    status = HAL_SPI_TransmitReceive(&hspi5, tx, rx, 7, L3GD20_SPI_TIMEOUT);
    L3GD20_CS_High();

    if (status == HAL_OK)
    {
        /*
         * Combine low and high bytes into signed 16-bit values.
         * The cast to int16_t correctly handles two's complement for
         * negative angular rates (e.g., rotation in the opposite direction).
         *
         * rx[0] = garbage (received while sending address byte)
         * rx[1] = OUT_X_L, rx[2] = OUT_X_H
         * rx[3] = OUT_Y_L, rx[4] = OUT_Y_H
         * rx[5] = OUT_Z_L, rx[6] = OUT_Z_H
         */
        *x = (int16_t)((uint16_t)rx[2] << 8 | rx[1]);
        *y = (int16_t)((uint16_t)rx[4] << 8 | rx[3]);
        *z = (int16_t)((uint16_t)rx[6] << 8 | rx[5]);
    }

    return status;
}
