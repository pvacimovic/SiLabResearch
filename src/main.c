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
#define ADDRESS_READ                0xA1
#define ADDRESS_WRITE               0xA0

#define INIT_REG                    0x14
#define DATA_REG                    0x08

/***************************************************************************//**
 * @brief I2C read numBytes from follower device starting at target address
 ******************************************************************************/
uint8_t I2C_Read(uint16_t followerAddress, uint8_t targetAddress, uint8_t *rxBuff, uint8_t numBytes)
{
    // Transfer structure
    I2C_TransferSeq_TypeDef i2cTransfer;
    I2C_TransferReturn_TypeDef result;

    uint8_t TxBuffer[1]; //Just a buffer to
    TxBuffer[0] = targetAddress;

    // Initialize I2C transfer
    i2cTransfer.addr = followerAddress;
    i2cTransfer.flags = I2C_FLAG_WRITE_READ; // must write target address before reading
    i2cTransfer.buf[0].data = TxBuffer; //The buffer to be used by the write part
    i2cTransfer.buf[0].len = 1; //Length of the write
    i2cTransfer.buf[1].data = rxBuff; //Buffer to be used by the read part
    i2cTransfer.buf[1].len = numBytes; //Number of bytes to read

    result = I2C_TransferInit (I2C0, &i2cTransfer);

    // Send data
    while (result == i2cTransferInProgress)
    {
        result = I2C_Transfer (I2C0);
    }

    // if there is an issue figure it out
    if (result != i2cTransferDone)
      return 1;

    return 0;
}

/***************************************************************************//**
 * @brief I2C write numBytes to follower device starting at target address
 ******************************************************************************/
uint8_t I2C_Write(uint16_t followerAddress, uint8_t targetAddress, uint8_t *txBuff)
{
     // Transfer structure
     I2C_TransferSeq_TypeDef i2cTransfer;
     I2C_TransferReturn_TypeDef result;

     uint8_t TxBuffer[2];
     TxBuffer[0] = targetAddress; // target register to write on
     TxBuffer[1] = *txBuff; // 1 byte to write

     // Initialize I2C transfer
     i2cTransfer.addr = followerAddress;
     i2cTransfer.flags = I2C_FLAG_WRITE;
     i2cTransfer.buf[0].data = TxBuffer;
     i2cTransfer.buf[0].len = 2;
     i2cTransfer.buf[1].data = NULL;
     i2cTransfer.buf[1].len = 0;

     result = I2C_TransferInit (I2C0, &i2cTransfer);

     // Send data
     while (result == i2cTransferInProgress)
     {
         result = I2C_Transfer (I2C0);
     }

     // if there is an issue figure it out
     if (result != i2cTransferDone)
       return 1;

     return 0;
}

/***************************************************************************//**
 * @brief I2C Read/Increment/Write/Verify
 ******************************************************************************/
uint8_t getTemp(double *temp)
{
    uint8_t initWord = 0xC1;
    uint8_t temperature[2];

    if(I2C_Write(SENSOR_ADDRESS, INIT_REG, &initWord))
    {
      return 1;
    }

    // sleep 100ms
//    for (int i = 0; i < 10000000; i++) {
//              // Do nothing, just waste time
//    }

    if(I2C_Read(SENSOR_ADDRESS, DATA_REG, temperature, 2))
      return 2;

    (*temp) = (temperature[0] << 8 | temperature[1]) * 0.005;

    return 0;
}

/***************************************************************************//**
 * @brief GPIO Interrupt handler
 ******************************************************************************/
void GPIO_EVEN_IRQHandler(void){}

/***************************************************************************//**
 * @brief Main function
 ******************************************************************************/
int main(void)
{
  // Chip errata
  CHIP_Init ();

  // Initialize the I2C
  initCMU ();
  initGPIO ();
  initI2C ();

  double temp;

  uint8_t OK = getTemp(&temp);

  // use temp
  OK = 1;
}
