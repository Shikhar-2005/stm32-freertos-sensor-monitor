/**
  ******************************************************************************
  * @file           : l3gd20.h
  * @brief          : L3GD20 3-axis gyroscope SPI driver header
  *
  * The L3GD20 is an onboard gyroscope on the STM32F429I-DISC1.
  * Connected via SPI5 (PF7=SCK, PF8=MISO, PF9=MOSI) with CS on PC1.
  *
  * This driver uses polling SPI (HAL_SPI_TransmitReceive) which is
  * acceptable because:
  *   1. Transfers are short (2-7 bytes)
  *   2. SPI5 runs at low prescaler — transfer completes in ~microseconds
  *   3. DMA overhead would exceed the actual transfer time
  ******************************************************************************
  */

#ifndef __L3GD20_H
#define __L3GD20_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdint.h>

/* L3GD20 Register Addresses -------------------------------------------------*/

/** @defgroup L3GD20_Registers L3GD20 Register Map
  * @brief Only the registers we actually use are defined.
  *        Full register map is in the L3GD20 datasheet (DocID022116).
  * @{
  */
#define L3GD20_REG_WHO_AM_I     0x0F  /**< Device identification register */
#define L3GD20_REG_CTRL_REG1    0x20  /**< Output data rate, power mode, axis enable */
#define L3GD20_REG_CTRL_REG4    0x23  /**< Full-scale selection, SPI mode */
#define L3GD20_REG_OUT_X_L      0x28  /**< X-axis angular rate data (low byte) */
/** @} */

/* L3GD20 SPI Protocol Flags ------------------------------------------------*/

/**
 * @brief SPI read/write and auto-increment flags.
 *
 * The L3GD20 uses bit 7 of the address byte to indicate read (1) or write (0).
 * Bit 6 enables auto-increment of the register address for multi-byte reads,
 * which lets us read all 6 output bytes (X_L, X_H, Y_L, Y_H, Z_L, Z_H) in
 * a single SPI transaction — much more efficient than 6 separate reads.
 */
#define L3GD20_SPI_READ         0x80  /**< Bit 7: 1 = read operation */
#define L3GD20_SPI_WRITE        0x00  /**< Bit 7: 0 = write operation */
#define L3GD20_SPI_AUTO_INC     0x40  /**< Bit 6: 1 = auto-increment address */

/* Expected WHO_AM_I values --------------------------------------------------*/

/**
 * @brief Expected values from the WHO_AM_I register.
 *
 * Different revisions of the L3GD20 may return different IDs:
 *   - 0xD4: L3GD20 (original)
 *   - 0xD7: L3GD20H (newer variant, also found on some DISC1 boards)
 */
#define L3GD20_WHO_AM_I_VALUE   0xD4
#define L3GD20H_WHO_AM_I_VALUE  0xD7

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Initialize the L3GD20 gyroscope.
 *
 * Powers on the sensor and enables all three axes by writing to CTRL_REG1.
 * Includes a 1ms stabilization delay after configuration.
 *
 * @note   Must be called BEFORE the FreeRTOS scheduler starts (from main),
 *         because it uses HAL_Delay which relies on SysTick — once the
 *         scheduler starts, SysTick is owned by FreeRTOS.
 */
void L3GD20_Init(void);

/**
 * @brief  Read the WHO_AM_I register to verify device presence.
 * @retval Device ID (expected: 0xD4 or 0xD7)
 */
uint8_t L3GD20_ReadID(void);

/**
 * @brief  Read all three gyroscope axes in a single SPI transaction.
 *
 * Uses auto-increment mode to read 6 consecutive bytes starting from
 * OUT_X_L (0x28). This is more efficient than individual register reads
 * and ensures all axes are sampled at the same time instant.
 *
 * @param[out] x  Pointer to store raw X-axis angular rate (signed 16-bit)
 * @param[out] y  Pointer to store raw Y-axis angular rate (signed 16-bit)
 * @param[out] z  Pointer to store raw Z-axis angular rate (signed 16-bit)
 * @retval HAL_OK on success, HAL_ERROR/HAL_TIMEOUT on failure
 */
HAL_StatusTypeDef L3GD20_ReadXYZ(int16_t *x, int16_t *y, int16_t *z);

#ifdef __cplusplus
}
#endif

#endif /* __L3GD20_H */
