/**
 * @file bme68x_common.c
 * @author Conor Barry (conor@horatiopistachio.com)
 * @brief  This file is the application specific version of examples/common/common.c. It provides the interface functions for the BME68x sensor.
 * @version 0.1
 * @date 2024-12-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bme68x.h"

#include "bme68x_common.h"

#include <stdio.h>
#include <err.h>
#include <errno.h>
#include <stdlib.h>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
// #include <linux/unistd.h>
#include <unistd.h>
#include <time.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>


/******************************************************************************/
/*!                 Macro definitions                                         */
/*! BME68X shuttle board ID */
#define BME68X_SHUTTLE_ID  0x93

#define I2C_PORT "/dev/i2c-1"

/******************************************************************************/
/*!                Static variable definition                                 */
static uint8_t dev_addr;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function map to Linux platform
 */
BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    (void)intf_ptr;

    int file = open(I2C_PORT, O_RDWR);
    if (file < 0)
    {
        printf("Failed to open the i2c bus\n");
        // exit(1);
        return -1;

    }

    if (ioctl(file, I2C_SLAVE, device_addr) < 0)
    {
        printf("Failed to set the i2c address\n");
        // exit(1);
        return -2;
    }


    // Write register to device.
    if (write(file, &reg_addr, 1) != 1)
    {
        printf("Failed to write to the i2c bus\n");
        // exit(1);
        return -3;
    }

    char *buf = malloc(len);

    if ( read(file, buf, len) != len)
    {
        printf("Failed to read from the i2c bus\n");
        // exit(1);
        free(buf);
        return -4;
    }
    else {
        // printf("Read: 0x%x from reg: 0x%x on dev:%x\n", buf[0], reg_addr, device_addr);
        memcpy(reg_data, buf, len);
    }

    free(buf);

    return 0;
}

/*!
 * I2C write function map to Linux platform
 */
BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    (void)intf_ptr;

    int file = open(I2C_PORT, O_RDWR);
    if (file < 0)
    {
        printf("Failed to open the i2c bus\n");
        // exit(1);
        return -1;
    }
    if (ioctl(file, I2C_SLAVE, device_addr) < 0)
    {
        printf("Failed to set the i2c address\n");
        // exit(1);
        return -2;
    }
    char *buf = malloc(len + 1);
    memcpy(buf, &reg_addr, 1);
    // buf[0] = reg_addr;
    memcpy(buf + 1, reg_data, len);

    if (write(file, buf, len + 1) != len + 1)
    {
        printf("Failed to write to the i2c bus\n");
        // exit(1);
        free(buf);
        return -3;
    }

    // printf("Wrote: 0x%x to 0x%x\n", reg_data[0], reg_addr);
    free(buf);
    return 0;
}


/*!
 * Delay function map to Linux platform
 */
void bme68x_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    usleep(period);
}

void bme68x_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BME68X_OK:

            /* Do nothing */
            break;
        case BME68X_E_NULL_PTR:
            printf("API name [%s]  Error [%d] : Null pointer\r\n", api_name, rslt);
            break;
        case BME68X_E_COM_FAIL:
            printf("API name [%s]  Error [%d] : Communication failure\r\n", api_name, rslt);
            break;
        case BME68X_E_INVALID_LENGTH:
            printf("API name [%s]  Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
            break;
        case BME68X_E_DEV_NOT_FOUND:
            printf("API name [%s]  Error [%d] : Device not found\r\n", api_name, rslt);
            break;
        case BME68X_E_SELF_TEST:
            printf("API name [%s]  Error [%d] : Self test error\r\n", api_name, rslt);
            break;
        case BME68X_W_NO_NEW_DATA:
            printf("API name [%s]  Warning [%d] : No new data found\r\n", api_name, rslt);
            break;
        default:
            printf("API name [%s]  Error [%d] : Unknown error code\r\n", api_name, rslt);
            break;
    }
}

int8_t bme68x_interface_init(struct bme68x_dev *bme, uint8_t intf)
{
    int8_t rslt = BME68X_OK;

    if (bme != NULL)
    {

        /* Bus configuration : I2C */
        if (intf == BME68X_I2C_INTF)
        {
            // printf("I2C Interface\n");
            dev_addr = BME68X_I2C_ADDR_LOW;
            bme->read = bme68x_i2c_read;
            bme->write = bme68x_i2c_write;
            bme->intf = BME68X_I2C_INTF;

        }
        /* Bus configuration : SPI */
        else if (intf == BME68X_SPI_INTF)
        {
            printf("SPI Interface Not Supported\n");
            rslt = BME68X_E_NULL_PTR;
            return rslt;
        }


        bme->delay_us = bme68x_delay_us;
        bme->intf_ptr = &dev_addr;
        bme->amb_temp = 25; /* The ambient temperature in deg C is used for defining the heater temperature */
        rslt = BME68X_OK;
    }
    else
    {
        rslt = BME68X_E_NULL_PTR;
    }

    return rslt;
}

