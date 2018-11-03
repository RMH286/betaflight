/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#undef USE_DSHOT_DMAR           // OMNIBUS (F3) does not benefit from burst Dshot

// Removed to make the firmware fit into flash (in descending order of priority):
#undef USE_GYRO_OVERFLOW_CHECK
#undef USE_GYRO_LPF2

#undef USE_ITERM_RELAX
#undef USE_RC_SMOOTHING_FILTER

#undef USE_HUFFMAN
#undef USE_PINIO
#undef USE_PINIOBOX

#undef USE_TELEMETRY_HOTT
#undef USE_TELEMETRY_MAVLINK
#undef USE_TELEMETRY_LTM
#undef USE_SERIALRX_XBUS
#undef USE_SERIALRX_SUMH
#undef USE_PWM

#undef USE_BOARD_INFO
#undef USE_EXTENDED_CMS_MENUS
#undef USE_RTC_TIME
#undef USE_RX_MSP
#undef USE_ESC_SENSOR_INFO

#define TARGET_BOARD_IDENTIFIER "HOVERBOT"

#define LED0_PIN                PC14
#define LED1_PIN                PA13

// to be added to circuit
//#define USE_BEEPER
//#define BEEPER_PIN              PC15
//#define BEEPER_INVERTED

#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PC13
#define USE_MPU_DATA_READY_SIGNAL

#define GYRO_1_SPI_INSTANCE     SPI1
#define GYRO_1_CS_PIN           PA4

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
// double check (layout is not done yet)
#define GYRO_1_ALIGN            CW90_DEG

#define USE_ACC
#define USE_ACC_SPI_MPU6000
// double check (layout is not done yet)
#define ACC_1_ALIGN             CW90_DEG

#define USE_RANGEFINDER
#define USE_RANGEFINDER_VL53L0X

#define USE_VCP
#define USBD_PRODUCT_STRING "hoverbot"

#define USE_UART3

#define SERIAL_PORT_COUNT       2

#define UART3_TX_PIN            PB10
#define UART3_RX_PIN            PB11

// #define USE_I2C
// #define USE_I2C_DEVICE_1
// #define I2C1_SCL                PB6
// #define I2C1_SDA                PB7
// #define I2C_DEVICE              (I2CDEV_1)

#define USE_SPI
#define USE_SPI_DEVICE_1

#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_2

#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      PB12
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#define USE_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define VBAT_ADC_PIN                PB1
#define CURRENT_METER_ADC_PIN       PB0
#define ADC_INSTANCE                ADC3
#define VBAT_SCALE_DEFAULT          100

#define CURRENT_TARGET_CPU_VOLTAGE 3.0

// board uses an ina139, RL=0.005, Rs=30000
// V/A = (0.005 * 0.001 * 30000) * I
// rescale to 1/10th mV / A -> * 1000 * 10
// use 3.0V as cpu and adc voltage -> rescale by 3.0/3.3
#define CURRENT_METER_SCALE_DEFAULT    (0.005 * 0.001 * 30000) * 1000 * 10 * (CURRENT_TARGET_CPU_VOLTAGE / 3.3)
#define CURRENT_METER_OFFSET_DEFAULT   0

#define DEFAULT_RX_FEATURE      FEATURE_RX_PPM
#define DEFAULT_FEATURES        (FEATURE_OSD)
//#define USE_TARGET_CONFIG

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(4))

#define USABLE_TIMER_CHANNEL_COUNT 5
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(4) | TIM_N(8))
