/***************************************************************************//**
 * @file main.c
 * @brief This project demonstrates the leader configuration of the EFx32xG2x
 * I2C peripheral. Two EFx32xG2x devices are connected together, one running the
 * leader project, the other running the follower project. The leader starts up
 * and enters a while loop waiting for a button press on push button 0. When
 * push button 0 is pressed, the program performs an I2C test. This routine
 * reads the follower device's current buffer values, increments each value by
 * 1, and writes the new values back to the follower device. The leader then
 * reads back the follower values again and verifies the new values match what
 * was previously written. Upon a successful write, LED0 is toggled and the
 * device re-enters the while loop, waiting again for user input through push
 * button 0. This project runs in a continuous loop, re-running the I2C test
 * with every PB0 button press and toggling LED0 with each successful iteration.
 * If there is an I2C transmission error, or if the verification step of the I2C
 * test fails, LED1 is turned on and the leader sits and remains in an infinite
 * while loop. Connecting to the device via debugger while in the infinite loop,
 * the I2C error code can be retrieved.
 *******************************************************************************
 * # License
 * <b>Copyright 2022 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *******************************************************************************
 * # Evaluation Quality
 * This code has been minimally tested to ensure that it builds and is suitable 
 * as a demonstration for evaluation purposes only. This code will be maintained
 * at the sole discretion of Silicon Labs.
 ******************************************************************************/
 
#include <stdio.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_i2c.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "bsp.h"

/*
// Defines
#define I2C_FOLLOWER_ADDRESS              0xE2
#define I2C_TXBUFFER_SIZE                 10
#define I2C_RXBUFFER_SIZE                 10
// Buffers
uint8_t i2c_txBuffer[I2C_TXBUFFER_SIZE];
uint8_t i2c_rxBuffer[I2C_RXBUFFER_SIZE];
// Transmission flags
volatile bool i2c_startTx;
*/

/***************************************************************************//**
 * @brief Enable clocks
 ******************************************************************************/
void initCMU(void){}

/***************************************************************************//**
 * @brief GPIO initialization
 ******************************************************************************/
void initGPIO(void){}

/***************************************************************************//**
 * @brief Setup I2C
 ******************************************************************************/
void initI2C(void)
{
      // Using default settings
      I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
      // Using PA5 (SDA) and PA6 (SCL)
      GPIO_PinModeSet (gpioPortA, 5, gpioModeWiredAndPullUpFilter, 1);
      GPIO_PinModeSet (gpioPortA, 6, gpioModeWiredAndPullUpFilter, 1);


      // Route I2C pins to GPIO
      GPIO->I2CROUTE[0].SDAROUTE = (GPIO->I2CROUTE[0].SDAROUTE
          & ~_GPIO_I2C_SDAROUTE_MASK)
          | (gpioPortA << _GPIO_I2C_SDAROUTE_PORT_SHIFT
              | (5 << _GPIO_I2C_SDAROUTE_PIN_SHIFT));
      GPIO->I2CROUTE[0].SCLROUTE = (GPIO->I2CROUTE[0].SCLROUTE
          & ~_GPIO_I2C_SCLROUTE_MASK)
          | (gpioPortA << _GPIO_I2C_SCLROUTE_PORT_SHIFT
              | (6 << _GPIO_I2C_SCLROUTE_PIN_SHIFT));
      GPIO->I2CROUTE[0].ROUTEEN = GPIO_I2C_ROUTEEN_SDAPEN | GPIO_I2C_ROUTEEN_SCLPEN;


      /* Initialising the I2C */
      I2C_Init(I2C0, &i2cInit);
      I2C0->CTRL = 1; //Enabled and master mode
}

// All defines

// device address (write A0, read A1)
// 1010000 (0x50) + 0 (w) or 1 (r)
#define SENSOR_ADDRESS              0x50



/***************************************************************************//**
 * @brief I2C read numBytes from follower device starting at target address
 ******************************************************************************/
void I2C_LeaderRead(uint16_t followerAddress, uint8_t targetAddress, uint8_t *rxBuff, uint8_t numBytes){}

/***************************************************************************//**
 * @brief I2C write numBytes to follower device starting at target address
 ******************************************************************************/
void I2C_LeaderWrite(uint16_t followerAddress, uint8_t targetAddress, uint8_t *txBuff, uint8_t numBytes){}

/***************************************************************************//**
 * @brief I2C Read/Increment/Write/Verify
 ******************************************************************************/
bool testI2C(void){}

/***************************************************************************//**
 * @brief GPIO Interrupt handler
 ******************************************************************************/
void GPIO_EVEN_IRQHandler(void){}

/***************************************************************************//**
 * @brief Main function
 ******************************************************************************/
int main(void)
{


}
