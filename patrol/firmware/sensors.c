#include "sensors.h"
#include "calibration.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "clib_port.h"
#include <stdlib.h>

static volatile uint32_t edges;
static void encoder_edge(void) {
    static uint32_t previous;
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_5);
    uint32_t now = clib_micros();
    if (now - previous >= 1000) {
        ++edges;
        previous = now;
    }
}

static void bus_init(void) {
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C1)) {}
    I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), false);
}

void sensors_init(void) {
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_5);
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_RISING_EDGE);
    GPIOIntRegister(GPIO_PORTA_BASE, encoder_edge);
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_5);
    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_5);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C1)) {}
    GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    GPIOPinConfigure(GPIO_PA7_I2C1SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);
    bus_init();
}

static bool transfer(uint32_t command) {
    I2CMasterIntClear(I2C1_BASE);
    I2CMasterControl(I2C1_BASE, command);
    uint32_t start = clib_micros();
    while (!I2CMasterIntStatus(I2C1_BASE, false))
        if (clib_micros() - start >= 1800)
            return false;
    return I2CMasterErr(I2C1_BASE) == I2C_MASTER_ERR_NONE;
}

static bool select_register(uint8_t reg) {
    I2CMasterSlaveAddrSet(I2C1_BASE, 0x68, false);
    I2CMasterDataPut(I2C1_BASE, reg);
    return transfer(I2C_MASTER_CMD_BURST_SEND_START);
}

static bool write_register(uint8_t reg, uint8_t value) {
    if (!select_register(reg))
        return false;
    I2CMasterDataPut(I2C1_BASE, value);
    return transfer(I2C_MASTER_CMD_BURST_SEND_FINISH);
}

static bool read_registers(uint8_t reg, uint8_t *bytes, unsigned length) {
    if (!select_register(reg))
        return false;
    I2CMasterSlaveAddrSet(I2C1_BASE, 0x68, true);
    for (unsigned i = 0; i < length; ++i) {
        uint32_t command = I2C_MASTER_CMD_BURST_RECEIVE_CONT;
        if (length == 1) command = I2C_MASTER_CMD_SINGLE_RECEIVE;
        else if (!i) command = I2C_MASTER_CMD_BURST_RECEIVE_START;
        else if (i + 1 == length) command = I2C_MASTER_CMD_BURST_RECEIVE_FINISH;
        if (!transfer(command))
            return false;
        bytes[i] = (uint8_t)I2CMasterDataGet(I2C1_BASE);
    }
    return true;
}

void sensors_sample(Car *car, uint32_t now) {
    static bool configured;
    static uint32_t next, last, calibration_edges;
    static unsigned samples;
    static int32_t sum;
    static double bias;
    uint8_t bytes[14];
    double yaw = 0;
    if (!configured && (int32_t)(now - next) >= 0) {
        bus_init();
        configured = read_registers(0x75, bytes, 1) && bytes[0] == 0x68 &&
            write_register(0x6b, 1) && write_register(0x1a, 3) &&
            write_register(0x1b, 0) && write_register(0x1c, 0);
        next = now + (configured ? 100 : 1000);
    } else if (configured && (int32_t)(now - next) >= 0) {
        configured = read_registers(0x3b, bytes, sizeof bytes);
        if (configured) {
            for (int i = 0; i < 6; i++) {
                int j = 2 * (i + (i >= 3)); /* skip temperature */
                car->imu[i] = (int16_t)((bytes[j] << 8) | bytes[j + 1]);
            }
            if (!car->imu_ready) {
                bool still = !car->left && !car->right && edges == calibration_edges &&
                    abs(car->imu[3]) < 655 && abs(car->imu[4]) < 655 && abs(car->imu[5]) < 655;
                if (still) {
                    sum += car->imu[5];
                    if (++samples == 100) {
                        bias = sum / 100.0;
                        car->imu_ready = true;
                    }
                } else {
                    sum = samples = 0;
                }
                calibration_edges = edges;
            } else if (now - last <= 100) {
                yaw = GYRO_Z_SIGN * (car->imu[5] - bias) / 131.0 * (now - last) / 1000;
            } else {
                car->imu_ready = false;
                sum = samples = 0;
            }
            last = now;
        } else {
            car->imu_ready = false;
            sum = samples = 0;
        }
        next = now + (configured ? 10 : 1000);
    }
    car_feedback(car, edges, yaw, now);
}
