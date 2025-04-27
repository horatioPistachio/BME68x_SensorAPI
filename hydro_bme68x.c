/**
 * @file hydro_bme68x.c
 * @author Conor Barry (conor@horatiopistachio.com)
 * @brief This file provides a wrapper on top of the BME68x sensor API. This wrapper takes care of the low level interfaces, timeings, etc. to simi
 * @brief to simplify the python program.
 * @version 0.1
 * @date 2024-12-27
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "bme68x.h"
#include "bme68x_defs.h"
#include "bme68x_common.h"

#include <stdio.h>
#include <err.h>
#include <errno.h>
#include <stdlib.h>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>



// #include <i2c/smbus.h>


#define BME68X_I2C_ADDR_PRIMARY 0x44
#define I2C_PORT "/dev/i2c-1"

#define SAMPLE_COUNT  UINT16_C(10)

struct bme68x_dev bme;


static int8_t i2c_read(uint8_t dev_id, uint8_t reg_addr, union i2c_smbus_data *data, size_t *len);
static int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, union i2c_smbus_data *data, uint16_t len);

/**
 * @brief This function initialises the BME68x sensor. It sets up the I2C interface.
 * 
 */
void init_bme68x()
{

}

// static int8_t i2c_read(uint8_t dev_id, uint8_t reg_addr, union i2c_smbus_data *data, size_t *len)
// {
//     int file = open(I2C_PORT, O_RDWR);
//     if (file < 0)
//     {
//         printf("Failed to open the i2c bus\n");
//         exit(1);
//     }
//     int rc = ioctl(file, I2C_SLAVE, dev_id);
//     if (rc < 0)
//     {
//         printf("Failed to set the i2c address\n");
//         exit(1);
//     }
    
//     // struct i2c_smbus_ioctl_data args;
//     // args.read_write = I2C_SMBUS_READ;
//     // args.command = NULL;
//     // args.size = len;
//     // args.data =  data;

//     // rc = ioctl(file, I2C_SMBUS, &args);
//     // if (rc < 0)
//     // {
//     //     printf("Failed to read from the i2c bus err:%d\n", rc);
//     //     exit(1);
//     // }

//     char buf[2];
//     buf[0] = reg_addr;
//     buf[1] = 0x00;

//     if (read(file, buf, 1) != 1)
//     {
//         printf("Failed to read from the i2c bus\n");
//         exit(1);
//     }
//     else {
//         printf("Read: %d\n", buf[0]);
//         data->word = buf[0];
//         *len = 1;
//     }

    
//     return 0;
// }

// static int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, union i2c_smbus_data *data, uint16_t len)
// {
//    int file = open(I2C_PORT, O_RDWR);
//     if (file < 0)
//     {
//         printf("Failed to open the i2c bus\n");
//         exit(1);
//     }
//     int rc = ioctl(file, I2C_SLAVE, dev_id);
//     if (rc < 0)
//     {
//         printf("Failed to set the i2c address\n");
//         exit(1);
//     }
    
//     char buf[2];
//     buf[0] = reg_addr;
//     buf[1] = data->word;
//     if (write(file, buf, 2) != 2)
//     {
//         printf("Failed to write to the i2c bus\n");
//         exit(1);
//     }
//     return 0;
// }

const char * get_bme_reading()
{
    struct bme68x_dev bme;
    int8_t rslt;
    struct bme68x_conf conf;
    struct bme68x_heatr_conf heatr_conf;
    struct bme68x_data data[3];
    uint32_t del_period;
    uint32_t time_ms = 0;
    uint8_t n_fields;
    uint16_t sample_count = 1;
    char *c = malloc(100);

    /* Heater temperature in degree Celsius */
    uint16_t temp_prof[10] = { 200, 240, 280, 320, 360, 360, 320, 280, 240, 200 };

    /* Heating duration in milliseconds */
    uint16_t dur_prof[10] = { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 };

    /* Interface preference is updated as a parameter
     * For I2C : BME68X_I2C_INTF
     * For SPI : BME68X_SPI_INTF
     */
    rslt = bme68x_interface_init(&bme, BME68X_I2C_INTF);
    bme68x_check_rslt("bme68x_interface_init", rslt);
    if (rslt != BME68X_OK)
    {
        snprintf(c, 100, "Error: %d\n", rslt);
        return c;
    }

    rslt = bme68x_init(&bme);
    bme68x_check_rslt("bme68x_init", rslt);
    if (rslt != BME68X_OK)
    {
        snprintf(c, 100, "Error: %d\n", rslt);
        return c;
    }

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    rslt = bme68x_get_conf(&conf, &bme);
    bme68x_check_rslt("bme68x_get_conf", rslt);
    if (rslt != BME68X_OK)
    {
        snprintf(c, 100, "Error: %d\n", rslt);
        return c;
    }

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    conf.filter = BME68X_FILTER_OFF;
    conf.odr = BME68X_ODR_NONE; /* This parameter defines the sleep duration after each profile */
    conf.os_hum = BME68X_OS_16X;
    conf.os_pres = BME68X_OS_1X;
    conf.os_temp = BME68X_OS_2X;
    rslt = bme68x_set_conf(&conf, &bme);
    bme68x_check_rslt("bme68x_set_conf", rslt);
    if (rslt != BME68X_OK)
    {
        snprintf(c, 100, "Error: %d\n", rslt);
        return c;
    }

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    heatr_conf.enable = BME68X_ENABLE;
    heatr_conf.heatr_temp_prof = temp_prof;
    heatr_conf.heatr_dur_prof = dur_prof;
    heatr_conf.profile_len = 10;
    rslt = bme68x_set_heatr_conf(BME68X_SEQUENTIAL_MODE, &heatr_conf, &bme);
    bme68x_check_rslt("bme68x_set_heatr_conf", rslt);
    if (rslt != BME68X_OK)
    {
        snprintf(c, 100, "Error: %d\n", rslt);
        return c;
    }

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    rslt = bme68x_set_op_mode(BME68X_SEQUENTIAL_MODE, &bme);
    bme68x_check_rslt("bme68x_set_op_mode", rslt);
    if (rslt != BME68X_OK)
    {
        snprintf(c, 100, "Error: %d\n", rslt);
        return c;
    }

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    // printf(
    //     "Sample, TimeStamp(ms), Temperature(deg C), Pressure(Pa), Humidity(%%), Gas resistance(ohm), Status, Profile index, Measurement index\n");
    while (sample_count <= SAMPLE_COUNT)
    {
        /* Calculate delay period in microseconds */
        del_period = bme68x_get_meas_dur(BME68X_SEQUENTIAL_MODE, &conf, &bme) + (heatr_conf.heatr_dur_prof[0] * 1000);
        bme.delay_us(del_period, bme.intf_ptr);

        time_ms = clock();

        rslt = bme68x_get_data(BME68X_SEQUENTIAL_MODE, data, &n_fields, &bme);
        bme68x_check_rslt("bme68x_get_data", rslt);

        /* Check if rslt == BME68X_OK, report or handle if otherwise */
        for (uint8_t i = 0; i < n_fields; i++)
        {
#ifdef BME68X_USE_FPU
            // printf("%u, %lu, %.2f, %.2f, %.2f, %.2f, 0x%x, %d, %d\n",
            //        sample_count,
            //        (long unsigned int)time_ms + (i * (del_period / 2000)),
            //        data[i].temperature,
            //        data[i].pressure,
            //        data[i].humidity,
            //        data[i].gas_resistance,
            //        data[i].status,
            //        data[i].gas_index,
            //        data[i].meas_index);
#else
            printf("%u, %lu, %d, %lu, %lu, %lu, 0x%x, %d, %d\n",
                   sample_count,
                   (long unsigned int)time_ms + (i * (del_period / 2000)),
                   (data[i].temperature / 100),
                   (long unsigned int)data[i].pressure,
                   (long unsigned int)(data[i].humidity / 1000),
                   (long unsigned int)data[i].gas_resistance,
                   data[i].status,
                   data[i].gas_index,
                   data[i].meas_index);
#endif
            sample_count++;
        }
    }

    
    snprintf(c, 100, "{\"temperature\": %.2f, \"pressure\": %.2f, \"humidity\": %.2f, \"gas_resistance\": %.2f}", data[0].temperature, data[0].pressure, data[0].humidity, data[0].gas_resistance);
    return c;

    return 0;
}