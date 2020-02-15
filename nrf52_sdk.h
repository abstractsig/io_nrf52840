#ifndef Nordic_SDK_H__
#define Nordic_SDK_H__
#include <io_cpu.h>

/*

Copyright (c) 2010 - 2018, Nordic Semiconductor ASA

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form, except as embedded into a Nordic
   Semiconductor ASA integrated circuit in a product or a software update for
   such product, must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other
   materials provided with the distribution.

3. Neither the name of Nordic Semiconductor ASA nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

4. This software, with or without modification, must only be used with a
   Nordic Semiconductor ASA integrated circuit.

5. Any software provided in binary form under this license must not be reverse
   engineered, decompiled, modified and/or disassembled.

THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef NRF52_TO_NRF52840_H
#define NRF52_TO_NRF52840_H

/*lint ++flb "Enter library region */

/* This file is given to prevent your SW from not compiling with the name changes between nRF51 or nRF52832 and nRF52840 devices.
 * It redefines the old nRF51 or nRF52832 names into the new ones as long as the functionality is still supported. If the
 * functionality is gone, there old names are not defined, so compilation will fail. Note that also includes macros
 * from the nrf52_namechange.h file. */
 
/* Differences between latest nRF52 headers and nRF52840 headers. */

/* UART */
/* The registers PSELRTS, PSELTXD, PSELCTS, PSELRXD were restructured into a struct. */
#define PSELRTS       PSEL.RTS
#define PSELTXD       PSEL.TXD
#define PSELCTS       PSEL.CTS
#define PSELRXD       PSEL.RXD

/* TWI */
/* The registers PSELSCL, PSELSDA were restructured into a struct. */
#define PSELSCL       PSEL.SCL
#define PSELSDA       PSEL.SDA


/* LPCOMP */
/* The hysteresis control enumerated values has changed name for nRF52840 devices. */
#define LPCOMP_HYST_HYST_NoHyst     LPCOMP_HYST_HYST_Disabled
#define LPCOMP_HYST_HYST_Hyst50mV   LPCOMP_HYST_HYST_Enabled


/* From nrf52_name_change.h. Several macros changed in different versions of nRF52 headers. By defining the following, any code written for any version of nRF52 headers will still compile. */

/* I2S */
/* Several enumerations changed case. Adding old macros to keep compilation compatibility. */
#define I2S_ENABLE_ENABLE_DISABLE           I2S_ENABLE_ENABLE_Disabled
#define I2S_ENABLE_ENABLE_ENABLE            I2S_ENABLE_ENABLE_Enabled
#define I2S_CONFIG_MODE_MODE_MASTER         I2S_CONFIG_MODE_MODE_Master
#define I2S_CONFIG_MODE_MODE_SLAVE          I2S_CONFIG_MODE_MODE_Slave
#define I2S_CONFIG_RXEN_RXEN_DISABLE        I2S_CONFIG_RXEN_RXEN_Disabled
#define I2S_CONFIG_RXEN_RXEN_ENABLE         I2S_CONFIG_RXEN_RXEN_Enabled
#define I2S_CONFIG_TXEN_TXEN_DISABLE        I2S_CONFIG_TXEN_TXEN_Disabled
#define I2S_CONFIG_TXEN_TXEN_ENABLE         I2S_CONFIG_TXEN_TXEN_Enabled
#define I2S_CONFIG_MCKEN_MCKEN_DISABLE      I2S_CONFIG_MCKEN_MCKEN_Disabled
#define I2S_CONFIG_MCKEN_MCKEN_ENABLE       I2S_CONFIG_MCKEN_MCKEN_Enabled
#define I2S_CONFIG_SWIDTH_SWIDTH_8BIT       I2S_CONFIG_SWIDTH_SWIDTH_8Bit
#define I2S_CONFIG_SWIDTH_SWIDTH_16BIT      I2S_CONFIG_SWIDTH_SWIDTH_16Bit
#define I2S_CONFIG_SWIDTH_SWIDTH_24BIT      I2S_CONFIG_SWIDTH_SWIDTH_24Bit
#define I2S_CONFIG_ALIGN_ALIGN_LEFT         I2S_CONFIG_ALIGN_ALIGN_Left
#define I2S_CONFIG_ALIGN_ALIGN_RIGHT        I2S_CONFIG_ALIGN_ALIGN_Right
#define I2S_CONFIG_FORMAT_FORMAT_ALIGNED    I2S_CONFIG_FORMAT_FORMAT_Aligned
#define I2S_CONFIG_CHANNELS_CHANNELS_STEREO I2S_CONFIG_CHANNELS_CHANNELS_Stereo
#define I2S_CONFIG_CHANNELS_CHANNELS_LEFT   I2S_CONFIG_CHANNELS_CHANNELS_Left
#define I2S_CONFIG_CHANNELS_CHANNELS_RIGHT  I2S_CONFIG_CHANNELS_CHANNELS_Right

/* LPCOMP */
/* Corrected typo in RESULT register. */
#define LPCOMP_RESULT_RESULT_Bellow         LPCOMP_RESULT_RESULT_Below


/*lint --flb "Leave library region" */

#endif /* NRF51_TO_NRF52840_H */

/*

Copyright (c) 2010 - 2018, Nordic Semiconductor ASA

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form, except as embedded into a Nordic
   Semiconductor ASA integrated circuit in a product or a software update for
   such product, must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other
   materials provided with the distribution.

3. Neither the name of Nordic Semiconductor ASA nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

4. This software, with or without modification, must only be used with a
   Nordic Semiconductor ASA integrated circuit.

5. Any software provided in binary form under this license must not be reverse
   engineered, decompiled, modified and/or disassembled.

THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef _NRF52840_PERIPHERALS_H
#define _NRF52840_PERIPHERALS_H


/* Power Peripheral */
#define POWER_PRESENT
#define POWER_COUNT 1

#define POWER_FEATURE_RAM_REGISTERS_PRESENT
#define POWER_FEATURE_RAM_REGISTERS_COUNT       9

#define POWER_FEATURE_VDDH_PRESENT

/* Floating Point Unit */
#define FPU_PRESENT
#define FPU_COUNT 1

/* Systick timer */
#define SYSTICK_PRESENT
#define SYSTICK_COUNT 1

/* Software Interrupts */
#define SWI_PRESENT
#define SWI_COUNT 6

/* Memory Watch Unit */
#define MWU_PRESENT
#define MWU_COUNT 1

/* GPIO */
#define GPIO_PRESENT
#define GPIO_COUNT 2

#define P0_PIN_NUM 32
#define P1_PIN_NUM 16

/* ACL */
#define ACL_PRESENT

#define ACL_REGIONS_COUNT 8

/* Radio */
#define RADIO_PRESENT
#define RADIO_COUNT 1

#define RADIO_EASYDMA_MAXCNT_SIZE 8

/* Accelerated Address Resolver */
#define AAR_PRESENT
#define AAR_COUNT 1

#define AAR_MAX_IRK_NUM 16

/* AES Electronic CodeBook mode encryption */
#define ECB_PRESENT
#define ECB_COUNT 1

/* AES CCM mode encryption */
#define CCM_PRESENT
#define CCM_COUNT 1

/* NFC Tag */
#define NFCT_PRESENT
#define NFCT_COUNT 1

#define NFCT_EASYDMA_MAXCNT_SIZE 9

/* Peripheral to Peripheral Interconnect */
#define PPI_PRESENT
#define PPI_COUNT 1

#define PPI_CH_NUM 20
#define PPI_FIXED_CH_NUM 12
#define PPI_GROUP_NUM 6
#define PPI_FEATURE_FORKS_PRESENT

/* Event Generator Unit */
#define EGU_PRESENT
#define EGU_COUNT 6

#define EGU0_CH_NUM 16
#define EGU1_CH_NUM 16
#define EGU2_CH_NUM 16
#define EGU3_CH_NUM 16
#define EGU4_CH_NUM 16
#define EGU5_CH_NUM 16

/* Timer/Counter */
#define TIMER_PRESENT
#define TIMER_COUNT 5

#define TIMER0_MAX_SIZE 32
#define TIMER1_MAX_SIZE 32
#define TIMER2_MAX_SIZE 32
#define TIMER3_MAX_SIZE 32
#define TIMER4_MAX_SIZE 32

#define TIMER0_CC_NUM 4
#define TIMER1_CC_NUM 4
#define TIMER2_CC_NUM 4
#define TIMER3_CC_NUM 6
#define TIMER4_CC_NUM 6

/* Real Time Counter */
#define RTC_PRESENT
#define RTC_COUNT 3

#define RTC0_CC_NUM 3
#define RTC1_CC_NUM 4
#define RTC2_CC_NUM 4

/* RNG */
#define RNG_PRESENT
#define RNG_COUNT 1

/* Watchdog Timer */
#define WDT_PRESENT
#define WDT_COUNT 1

/* Temperature Sensor */
#define TEMP_PRESENT
#define TEMP_COUNT 1

/* Serial Peripheral Interface Master */
#define SPI_PRESENT
#define SPI_COUNT 3

/* Serial Peripheral Interface Master with DMA */
#define SPIM_PRESENT
#define SPIM_COUNT 4

#define SPIM0_MAX_DATARATE  8
#define SPIM1_MAX_DATARATE  8
#define SPIM2_MAX_DATARATE  8
#define SPIM3_MAX_DATARATE  32

#define SPIM0_FEATURE_HARDWARE_CSN_PRESENT  0
#define SPIM1_FEATURE_HARDWARE_CSN_PRESENT  0
#define SPIM2_FEATURE_HARDWARE_CSN_PRESENT  0
#define SPIM3_FEATURE_HARDWARE_CSN_PRESENT  1

#define SPIM0_FEATURE_DCX_PRESENT  0
#define SPIM1_FEATURE_DCX_PRESENT  0
#define SPIM2_FEATURE_DCX_PRESENT  0
#define SPIM3_FEATURE_DCX_PRESENT  1

#define SPIM0_FEATURE_RXDELAY_PRESENT  0
#define SPIM1_FEATURE_RXDELAY_PRESENT  0
#define SPIM2_FEATURE_RXDELAY_PRESENT  0
#define SPIM3_FEATURE_RXDELAY_PRESENT  1

#define SPIM0_EASYDMA_MAXCNT_SIZE 16
#define SPIM1_EASYDMA_MAXCNT_SIZE 16
#define SPIM2_EASYDMA_MAXCNT_SIZE 16
#define SPIM3_EASYDMA_MAXCNT_SIZE 16

/* Serial Peripheral Interface Slave with DMA*/
#define SPIS_PRESENT
#define SPIS_COUNT 3

#define SPIS0_EASYDMA_MAXCNT_SIZE 16
#define SPIS1_EASYDMA_MAXCNT_SIZE 16
#define SPIS2_EASYDMA_MAXCNT_SIZE 16

/* Two Wire Interface Master */
#define TWI_PRESENT
#define TWI_COUNT 2

/* Two Wire Interface Master with DMA */
#define TWIM_PRESENT
#define TWIM_COUNT 2

#define TWIM0_EASYDMA_MAXCNT_SIZE 16
#define TWIM1_EASYDMA_MAXCNT_SIZE 16

/* Two Wire Interface Slave with DMA */
#define TWIS_PRESENT
#define TWIS_COUNT 2

#define TWIS0_EASYDMA_MAXCNT_SIZE 16
#define TWIS1_EASYDMA_MAXCNT_SIZE 16

/* Universal Asynchronous Receiver-Transmitter */
#define UART_PRESENT
#define UART_COUNT 1

/* Universal Asynchronous Receiver-Transmitter with DMA */
#define UARTE_PRESENT
#define UARTE_COUNT 2

#define UARTE0_EASYDMA_MAXCNT_SIZE 16
#define UARTE1_EASYDMA_MAXCNT_SIZE 16

/* Quadrature Decoder */
#define QDEC_PRESENT
#define QDEC_COUNT 1

/* Successive Approximation Analog to Digital Converter */
#define SAADC_PRESENT
#define SAADC_COUNT 1

#define SAADC_EASYDMA_MAXCNT_SIZE 15

#define SAADC_CH_NUM 8

/* GPIO Tasks and Events */
#define GPIOTE_PRESENT
#define GPIOTE_COUNT 1

#define GPIOTE_CH_NUM 8

#define GPIOTE_FEATURE_SET_PRESENT
#define GPIOTE_FEATURE_CLR_PRESENT

/* Low Power Comparator */
#define LPCOMP_PRESENT
#define LPCOMP_COUNT 1

#define LPCOMP_REFSEL_RESOLUTION 16

#define LPCOMP_FEATURE_HYST_PRESENT

/* Comparator */
#define COMP_PRESENT
#define COMP_COUNT 1

/* Pulse Width Modulator */
#define PWM_PRESENT
#define PWM_COUNT 4

#define PWM0_CH_NUM 4
#define PWM1_CH_NUM 4
#define PWM2_CH_NUM 4
#define PWM3_CH_NUM 4

#define PWM0_EASYDMA_MAXCNT_SIZE 15
#define PWM1_EASYDMA_MAXCNT_SIZE 15
#define PWM2_EASYDMA_MAXCNT_SIZE 15
#define PWM3_EASYDMA_MAXCNT_SIZE 15

/* Pulse Density Modulator */
#define PDM_PRESENT
#define PDM_COUNT 1

#define PDM_EASYDMA_MAXCNT_SIZE 15

/* Inter-IC Sound Interface */
#define I2S_PRESENT
#define I2S_COUNT 1

#define I2S_EASYDMA_MAXCNT_SIZE 14

/* Universal Serial Bus Device */
#define USBD_PRESENT
#define USBD_COUNT 1

#define USBD_EASYDMA_MAXCNT_SIZE 7

/* ARM TrustZone Cryptocell 310 */
#define CRYPTOCELL_PRESENT
#define CRYPTOCELL_COUNT 1

/* Quad SPI */
#define QSPI_PRESENT
#define QSPI_COUNT 1

#define QSPI_EASYDMA_MAXCNT_SIZE 20

#endif      // _NRF52840_PERIPHERALS_H

#ifndef Nordic_NRF_GPIO_H__
#define Nordic_NRF_GPIO_H__

#define ASSERT(expr)
#define NRFX_ASSERT(m)


/**
 * @defgroup nrf_gpio_hal GPIO HAL
 * @{
 * @ingroup nrf_gpio
 * @brief   Hardware access layer for managing the GPIO peripheral.
 */

#if (GPIO_COUNT == 1)
#define NUMBER_OF_PINS (P0_PIN_NUM)
#define GPIO_REG_LIST  {NRF_PERIPHERAL(NRF_P0)}
#elif (GPIO_COUNT == 2)
#define NUMBER_OF_PINS (P0_PIN_NUM + P1_PIN_NUM)
#define GPIO_REG_LIST  {NRF_P0, NRF_P1}
#else
#error "Not supported."
#endif


/**
 * @brief Macro for mapping port and pin numbers to values understandable for nrf_gpio functions.
 */
#define NRF_GPIO_PIN_MAP(port, pin) (((port) << 5) | ((pin) & 0x1F))

/**
 * @brief Pin direction definitions.
 */
typedef enum
{
    NRF_GPIO_PIN_DIR_INPUT  = GPIO_PIN_CNF_DIR_Input, ///< Input.
    NRF_GPIO_PIN_DIR_OUTPUT = GPIO_PIN_CNF_DIR_Output ///< Output.
} nrf_gpio_pin_dir_t;

/**
 * @brief Connection of input buffer.
 */
typedef enum
{
    NRF_GPIO_PIN_INPUT_CONNECT    = GPIO_PIN_CNF_INPUT_Connect,   ///< Connect input buffer.
    NRF_GPIO_PIN_INPUT_DISCONNECT = GPIO_PIN_CNF_INPUT_Disconnect ///< Disconnect input buffer.
} nrf_gpio_pin_input_t;

/**
 * @brief Enumerator used for selecting the pin to be pulled down or up at the time of pin configuration.
 */
typedef enum
{
    NRF_GPIO_PIN_NOPULL   = GPIO_PIN_CNF_PULL_Disabled, ///<  Pin pull-up resistor disabled.
    NRF_GPIO_PIN_PULLDOWN = GPIO_PIN_CNF_PULL_Pulldown, ///<  Pin pull-down resistor enabled.
    NRF_GPIO_PIN_PULLUP   = GPIO_PIN_CNF_PULL_Pullup,   ///<  Pin pull-up resistor enabled.
} nrf_gpio_pin_pull_t;

/**
 * @brief Enumerator used for selecting output drive mode.
 */
typedef enum
{
    NRF_GPIO_PIN_S0S1 = GPIO_PIN_CNF_DRIVE_S0S1, ///< !< Standard '0', standard '1'.
    NRF_GPIO_PIN_H0S1 = GPIO_PIN_CNF_DRIVE_H0S1, ///< !< High-drive '0', standard '1'.
    NRF_GPIO_PIN_S0H1 = GPIO_PIN_CNF_DRIVE_S0H1, ///< !< Standard '0', high-drive '1'.
    NRF_GPIO_PIN_H0H1 = GPIO_PIN_CNF_DRIVE_H0H1, ///< !< High drive '0', high-drive '1'.
    NRF_GPIO_PIN_D0S1 = GPIO_PIN_CNF_DRIVE_D0S1, ///< !< Disconnect '0' standard '1'.
    NRF_GPIO_PIN_D0H1 = GPIO_PIN_CNF_DRIVE_D0H1, ///< !< Disconnect '0', high-drive '1'.
    NRF_GPIO_PIN_S0D1 = GPIO_PIN_CNF_DRIVE_S0D1, ///< !< Standard '0', disconnect '1'.
    NRF_GPIO_PIN_H0D1 = GPIO_PIN_CNF_DRIVE_H0D1, ///< !< High-drive '0', disconnect '1'.
} nrf_gpio_pin_drive_t;

/**
 * @brief Enumerator used for selecting the pin to sense high or low level on the pin input.
 */
typedef enum
{
    NRF_GPIO_PIN_NOSENSE    = GPIO_PIN_CNF_SENSE_Disabled, ///<  Pin sense level disabled.
    NRF_GPIO_PIN_SENSE_LOW  = GPIO_PIN_CNF_SENSE_Low,      ///<  Pin sense low level.
    NRF_GPIO_PIN_SENSE_HIGH = GPIO_PIN_CNF_SENSE_High,     ///<  Pin sense high level.
} nrf_gpio_pin_sense_t;

/**
 * @brief Function for configuring the GPIO pin range as output pins with normal drive strength.
 *        This function can be used to configure pin range as simple output with gate driving GPIO_PIN_CNF_DRIVE_S0S1 (normal cases).
 *
 * @param pin_range_start Specifies the start number (inclusive) in the range of pin numbers to be configured (allowed values 0-30).
 *
 * @param pin_range_end Specifies the end number (inclusive) in the range of pin numbers to be configured (allowed values 0-30).
 *
 * @note For configuring only one pin as output, use @ref nrf_gpio_cfg_output.
 *       Sense capability on the pin is disabled and input is disconnected from the buffer as the pins are configured as output.
 */
__STATIC_INLINE void nrf_gpio_range_cfg_output(uint32_t pin_range_start, uint32_t pin_range_end);

/**
 * @brief Function for configuring the GPIO pin range as input pins with given initial value set, hiding inner details.
 *        This function can be used to configure pin range as simple input.
 *
 * @param pin_range_start Specifies the start number (inclusive) in the range of pin numbers to be configured (allowed values 0-30).
 *
 * @param pin_range_end Specifies the end number (inclusive) in the range of pin numbers to be configured (allowed values 0-30).
 *
 * @param pull_config State of the pin range pull resistor (no pull, pulled down, or pulled high).
 *
 * @note  For configuring only one pin as input, use @ref nrf_gpio_cfg_input.
 *        Sense capability on the pin is disabled and input is connected to buffer so that the GPIO->IN register is readable.
 */
__STATIC_INLINE void nrf_gpio_range_cfg_input(uint32_t            pin_range_start,
                                              uint32_t            pin_range_end,
                                              nrf_gpio_pin_pull_t pull_config);

/**
 * @brief Pin configuration function.
 *
 * The main pin configuration function.
 * This function allows to set any aspect in PIN_CNF register.
 * @param pin_number Specifies the pin number.
 * @param dir        Pin direction.
 * @param input      Connect or disconnect the input buffer.
 * @param pull       Pull configuration.
 * @param drive      Drive configuration.
 * @param sense      Pin sensing mechanism.
 */
__STATIC_INLINE void nrf_gpio_cfg(
    uint32_t             pin_number,
    nrf_gpio_pin_dir_t   dir,
    nrf_gpio_pin_input_t input,
    nrf_gpio_pin_pull_t  pull,
    nrf_gpio_pin_drive_t drive,
    nrf_gpio_pin_sense_t sense);

/**
 * @brief Function for configuring the given GPIO pin number as output, hiding inner details.
 *        This function can be used to configure a pin as simple output with gate driving GPIO_PIN_CNF_DRIVE_S0S1 (normal cases).
 *
 * @param pin_number Specifies the pin number.
 *
 * @note  Sense capability on the pin is disabled and input is disconnected from the buffer as the pins are configured as output.
 */
__STATIC_INLINE void nrf_gpio_cfg_output(uint32_t pin_number);

/**
 * @brief Function for configuring the given GPIO pin number as input, hiding inner details.
 *        This function can be used to configure a pin as simple input.
 *
 * @param pin_number Specifies the pin number.
 * @param pull_config State of the pin range pull resistor (no pull, pulled down, or pulled high).
 *
 * @note  Sense capability on the pin is disabled and input is connected to buffer so that the GPIO->IN register is readable.
 */
__STATIC_INLINE void nrf_gpio_cfg_input(uint32_t pin_number, nrf_gpio_pin_pull_t pull_config);

/**
 * @brief Function for resetting pin configuration to its default state.
 *
 * @param pin_number Specifies the pin number.
 */
__STATIC_INLINE void nrf_gpio_cfg_default(uint32_t pin_number);

/**
 * @brief Function for configuring the given GPIO pin number as a watcher. Only input is connected.
 *
 * @param pin_number Specifies the pin number.
 *
 */
__STATIC_INLINE void nrf_gpio_cfg_watcher(uint32_t pin_number);

/**
 * @brief Function for disconnecting input for the given GPIO.
 *
 * @param pin_number Specifies the pin number.
 *
 */
__STATIC_INLINE void nrf_gpio_input_disconnect(uint32_t pin_number);

/**
 * @brief Function for configuring the given GPIO pin number as input, hiding inner details.
 *        This function can be used to configure pin range as simple input.
 *        Sense capability on the pin is configurable and input is connected to buffer so that the GPIO->IN register is readable.
 *
 * @param pin_number   Specifies the pin number.
 * @param pull_config  State of the pin pull resistor (no pull, pulled down, or pulled high).
 * @param sense_config Sense level of the pin (no sense, sense low, or sense high).
 */
__STATIC_INLINE void nrf_gpio_cfg_sense_input(uint32_t             pin_number,
                                              nrf_gpio_pin_pull_t  pull_config,
                                              nrf_gpio_pin_sense_t sense_config);

/**
 * @brief Function for configuring sense level for the given GPIO.
 *
 * @param pin_number   Specifies the pin number.
 * @param sense_config Sense configuration.
 *
 */
__STATIC_INLINE void nrf_gpio_cfg_sense_set(uint32_t pin_number, nrf_gpio_pin_sense_t sense_config);

/**
 * @brief Function for setting the direction for a GPIO pin.
 *
 * @param pin_number Specifies the pin number for which to set the direction.
 *
 * @param direction Specifies the direction.
 */
__STATIC_INLINE void nrf_gpio_pin_dir_set(uint32_t pin_number, nrf_gpio_pin_dir_t direction);

/**
 * @brief Function for setting a GPIO pin.
 *
 * Note that the pin must be configured as an output for this function to have any effect.
 *
 * @param pin_number Specifies the pin number to set.
 */
__STATIC_INLINE void nrf_gpio_pin_set(uint32_t pin_number);

/**
 * @brief Function for clearing a GPIO pin.
 *
 * Note that the pin must be configured as an output for this
 * function to have any effect.
 *
 * @param pin_number Specifies the pin number to clear.
 */
__STATIC_INLINE void nrf_gpio_pin_clear(uint32_t pin_number);

/**
 * @brief Function for toggling a GPIO pin.
 *
 * Note that the pin must be configured as an output for this
 * function to have any effect.
 *
 * @param pin_number Specifies the pin number to toggle.
 */
__STATIC_INLINE void nrf_gpio_pin_toggle(uint32_t pin_number);

/**
 * @brief Function for writing a value to a GPIO pin.
 *
 * Note that the pin must be configured as an output for this
 * function to have any effect.
 *
 * @param pin_number Specifies the pin number to write.
 *
 * @param value Specifies the value to be written to the pin.
 * @arg 0 Clears the pin.
 * @arg >=1 Sets the pin.
 */
__STATIC_INLINE void nrf_gpio_pin_write(uint32_t pin_number, uint32_t value);

/**
 * @brief Function for reading the input level of a GPIO pin.
 *
 * Note that the pin must have input connected for the value
 * returned from this function to be valid.
 *
 * @param pin_number Specifies the pin number to read.
 *
 * @return 0 if the pin input level is low. Positive value if the pin is high.
 */
__STATIC_INLINE uint32_t nrf_gpio_pin_read(uint32_t pin_number);

/**
 * @brief Function for reading the output level of a GPIO pin.
 *
 * @param pin_number Specifies the pin number to read.
 *
 * @return 0 if the pin output level is low. Positive value if pin output is high.
 */
__STATIC_INLINE uint32_t nrf_gpio_pin_out_read(uint32_t pin_number);

/**
 * @brief Function for reading the sense configuration of a GPIO pin.
 *
 * @param pin_number Specifies the pin number to read.
 *
 * @retval Sense configuration.
 */
__STATIC_INLINE nrf_gpio_pin_sense_t nrf_gpio_pin_sense_get(uint32_t pin_number);

/**
 * @brief Function for setting output direction on selected pins on a given port.
 *
 * @param p_reg    Pointer to the peripheral registers structure.
 * @param out_mask Mask specifying the pins to set as output.
 *
 */
__STATIC_INLINE void nrf_gpio_port_dir_output_set(NRF_GPIO_Type * p_reg, uint32_t out_mask);

/**
 * @brief Function for setting input direction on selected pins on a given port.
 *
 * @param p_reg    Pointer to the peripheral registers structure.
 * @param in_mask  Mask specifying the pins to set as input.
 *
 */
__STATIC_INLINE void nrf_gpio_port_dir_input_set(NRF_GPIO_Type * p_reg, uint32_t in_mask);

/**
 * @brief Function for writing the direction configuration of GPIO pins in a given port.
 *
 * @param p_reg    Pointer to the peripheral registers structure.
 * @param dir_mask Mask specifying the direction of pins. Bit set means that the given pin is configured as output.
 *
 */
__STATIC_INLINE void nrf_gpio_port_dir_write(NRF_GPIO_Type * p_reg, uint32_t dir_mask);

/**
 * @brief Function for reading the direction configuration of a GPIO port.
 *
 * @param p_reg    Pointer to the peripheral registers structure.
 *
 * @retval Pin configuration of the current direction settings. Bit set means that the given pin is configured as output.
 */
__STATIC_INLINE uint32_t nrf_gpio_port_dir_read(NRF_GPIO_Type const * p_reg);

/**
 * @brief Function for reading the input signals of GPIO pins on a given port.
 *
 * @param p_reg Pointer to the peripheral registers structure.
 *
 * @retval Port input values.
 */
__STATIC_INLINE uint32_t nrf_gpio_port_in_read(NRF_GPIO_Type const * p_reg);

/**
 * @brief Function for reading the output signals of GPIO pins of a given port.
 *
 * @param p_reg Pointer to the peripheral registers structure.
 *
 * @retval Port output values.
 */
__STATIC_INLINE uint32_t nrf_gpio_port_out_read(NRF_GPIO_Type const * p_reg);

/**
 * @brief Function for writing the GPIO pins output on a given port.
 *
 * @param p_reg Pointer to the peripheral registers structure.
 * @param value Output port mask.
 *
 */
__STATIC_INLINE void nrf_gpio_port_out_write(NRF_GPIO_Type * p_reg, uint32_t value);

/**
 * @brief Function for setting high level on selected GPIO pins of a given port.
 *
 * @param p_reg    Pointer to the peripheral registers structure.
 * @param set_mask Mask with pins to set as logical high level.
 *
 */
__STATIC_INLINE void nrf_gpio_port_out_set(NRF_GPIO_Type * p_reg, uint32_t set_mask);

/**
 * @brief Function for setting low level on selected GPIO pins of a given port.
 *
 * @param p_reg    Pointer to the peripheral registers structure.
 * @param clr_mask Mask with pins to set as logical low level.
 *
 */
__STATIC_INLINE void nrf_gpio_port_out_clear(NRF_GPIO_Type * p_reg, uint32_t clr_mask);

/**
 * @brief Function for reading pins state of multiple consecutive ports.
 *
 * @param start_port Index of the first port to read.
 * @param length     Number of ports to read.
 * @param p_masks    Pointer to output array where port states will be stored.
 */
__STATIC_INLINE void nrf_gpio_ports_read(uint32_t start_port, uint32_t length, uint32_t * p_masks);

#if defined(GPIO_DETECTMODE_DETECTMODE_LDETECT) || defined(__NRF_DOXYGEN__)
/**
 * @brief Function for reading latch state of multiple consecutive ports.
 *
 * @param start_port Index of the first port to read.
 * @param length     Number of ports to read.
 * @param p_masks    Pointer to output array where latch states will be stored.
 */
__STATIC_INLINE void nrf_gpio_latches_read(uint32_t start_port, uint32_t length,
                                           uint32_t * p_masks);

/**
 * @brief Function for reading latch state of single pin.
 *
 * @param pin_number Pin number.
 * @return 0 if latch is not set. Positive value otherwise.
 *
 */
__STATIC_INLINE uint32_t nrf_gpio_pin_latch_get(uint32_t pin_number);

/**
 * @brief Function for clearing latch state of a single pin.
 *
 * @param pin_number Pin number.
 *
 */
__STATIC_INLINE void nrf_gpio_pin_latch_clear(uint32_t pin_number);
#endif

#ifndef SUPPRESS_INLINE_IMPLEMENTATION

/**
 * @brief Function for extracting port and relative pin number from absolute pin number.
 *
 * @param[inout] Pointer to absolute pin number which is overriden by relative to port pin number.
 *
 * @return Pointer to port register set.
 *
 */
__STATIC_INLINE NRF_GPIO_Type * nrf_gpio_pin_port_decode(uint32_t * p_pin)
{
    NRFX_ASSERT(*p_pin < NUMBER_OF_PINS);
#if (GPIO_COUNT == 1)
    // The oldest definition case
    return NRF_PERIPHERAL(NRF_P0);
#else
    if (*p_pin < P0_PIN_NUM)
    {
        return NRF_PERIPHERAL(NRF_P0);
    }
    else
    {
        *p_pin = *p_pin & (P0_PIN_NUM - 1);
        return NRF_PERIPHERAL(NRF_P1);
    }
#endif
}


__STATIC_INLINE void nrf_gpio_range_cfg_output(uint32_t pin_range_start, uint32_t pin_range_end)
{
    /*lint -e{845} // A zero has been given as right argument to operator '|'" */
    for (; pin_range_start <= pin_range_end; pin_range_start++)
    {
        nrf_gpio_cfg_output(pin_range_start);
    }
}


__STATIC_INLINE void nrf_gpio_range_cfg_input(uint32_t            pin_range_start,
                                              uint32_t            pin_range_end,
                                              nrf_gpio_pin_pull_t pull_config)
{
    /*lint -e{845} // A zero has been given as right argument to operator '|'" */
    for (; pin_range_start <= pin_range_end; pin_range_start++)
    {
        nrf_gpio_cfg_input(pin_range_start, pull_config);
    }
}


__STATIC_INLINE void nrf_gpio_cfg(
    uint32_t             pin_number,
    nrf_gpio_pin_dir_t   dir,
    nrf_gpio_pin_input_t input,
    nrf_gpio_pin_pull_t  pull,
    nrf_gpio_pin_drive_t drive,
    nrf_gpio_pin_sense_t sense)
{
    NRF_GPIO_Type * reg = nrf_gpio_pin_port_decode(&pin_number);

    reg->PIN_CNF[pin_number] = ((uint32_t)dir << GPIO_PIN_CNF_DIR_Pos)
                               | ((uint32_t)input << GPIO_PIN_CNF_INPUT_Pos)
                               | ((uint32_t)pull << GPIO_PIN_CNF_PULL_Pos)
                               | ((uint32_t)drive << GPIO_PIN_CNF_DRIVE_Pos)
                               | ((uint32_t)sense << GPIO_PIN_CNF_SENSE_Pos);
}


__STATIC_INLINE void nrf_gpio_cfg_output(uint32_t pin_number)
{
    nrf_gpio_cfg(
        pin_number,
        NRF_GPIO_PIN_DIR_OUTPUT,
        NRF_GPIO_PIN_INPUT_DISCONNECT,
        NRF_GPIO_PIN_NOPULL,
        NRF_GPIO_PIN_S0S1,
        NRF_GPIO_PIN_NOSENSE);
}


__STATIC_INLINE void nrf_gpio_cfg_input(uint32_t pin_number, nrf_gpio_pin_pull_t pull_config)
{
    nrf_gpio_cfg(
        pin_number,
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_CONNECT,
        pull_config,
        NRF_GPIO_PIN_S0S1,
        NRF_GPIO_PIN_NOSENSE);
}


__STATIC_INLINE void nrf_gpio_cfg_default(uint32_t pin_number)
{
    nrf_gpio_cfg(
        pin_number,
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_DISCONNECT,
        NRF_GPIO_PIN_NOPULL,
        NRF_GPIO_PIN_S0S1,
        NRF_GPIO_PIN_NOSENSE);
}


__STATIC_INLINE void nrf_gpio_cfg_watcher(uint32_t pin_number)
{
    NRF_GPIO_Type * reg = nrf_gpio_pin_port_decode(&pin_number);
    /*lint -e{845} // A zero has been given as right argument to operator '|'" */
    uint32_t cnf = reg->PIN_CNF[pin_number] & ~GPIO_PIN_CNF_INPUT_Msk;

    reg->PIN_CNF[pin_number] = cnf | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos);
}


__STATIC_INLINE void nrf_gpio_input_disconnect(uint32_t pin_number)
{
    NRF_GPIO_Type * reg = nrf_gpio_pin_port_decode(&pin_number);
    /*lint -e{845} // A zero has been given as right argument to operator '|'" */
    uint32_t cnf = reg->PIN_CNF[pin_number] & ~GPIO_PIN_CNF_INPUT_Msk;

    reg->PIN_CNF[pin_number] = cnf | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);
}


__STATIC_INLINE void nrf_gpio_cfg_sense_input(uint32_t             pin_number,
                                              nrf_gpio_pin_pull_t  pull_config,
                                              nrf_gpio_pin_sense_t sense_config)
{
    nrf_gpio_cfg(
        pin_number,
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_CONNECT,
        pull_config,
        NRF_GPIO_PIN_S0S1,
        sense_config);
}


__STATIC_INLINE void nrf_gpio_cfg_sense_set(uint32_t pin_number, nrf_gpio_pin_sense_t sense_config)
{
    NRF_GPIO_Type * reg = nrf_gpio_pin_port_decode(&pin_number);

    /*lint -e{845} // A zero has been given as right argument to operator '|'" */
    reg->PIN_CNF[pin_number] &= ~GPIO_PIN_CNF_SENSE_Msk;
    reg->PIN_CNF[pin_number] |= (sense_config << GPIO_PIN_CNF_SENSE_Pos);
}


__STATIC_INLINE void nrf_gpio_pin_dir_set(uint32_t pin_number, nrf_gpio_pin_dir_t direction)
{
    if (direction == NRF_GPIO_PIN_DIR_INPUT)
    {
        nrf_gpio_cfg(
            pin_number,
            NRF_GPIO_PIN_DIR_INPUT,
            NRF_GPIO_PIN_INPUT_CONNECT,
            NRF_GPIO_PIN_NOPULL,
            NRF_GPIO_PIN_S0S1,
            NRF_GPIO_PIN_NOSENSE);
    }
    else
    {
        NRF_GPIO_Type * reg = nrf_gpio_pin_port_decode(&pin_number);
        reg->DIRSET = (1UL << pin_number);
    }
}


__STATIC_INLINE void nrf_gpio_pin_set(uint32_t pin_number)
{
    NRF_GPIO_Type * reg = nrf_gpio_pin_port_decode(&pin_number);

    nrf_gpio_port_out_set(reg, 1UL << pin_number);
}


__STATIC_INLINE void nrf_gpio_pin_clear(uint32_t pin_number)
{
    NRF_GPIO_Type * reg = nrf_gpio_pin_port_decode(&pin_number);

    nrf_gpio_port_out_clear(reg, 1UL << pin_number);
}


__STATIC_INLINE void nrf_gpio_pin_toggle(uint32_t pin_number)
{
    NRF_GPIO_Type * reg        = nrf_gpio_pin_port_decode(&pin_number);
    uint32_t        pins_state = reg->OUT;

    reg->OUTSET = (~pins_state & (1UL << pin_number));
    reg->OUTCLR = (pins_state & (1UL << pin_number));
}


__STATIC_INLINE void nrf_gpio_pin_write(uint32_t pin_number, uint32_t value)
{
    if (value == 0)
    {
        nrf_gpio_pin_clear(pin_number);
    }
    else
    {
        nrf_gpio_pin_set(pin_number);
    }
}


__STATIC_INLINE uint32_t nrf_gpio_pin_read(uint32_t pin_number)
{
    NRF_GPIO_Type * reg = nrf_gpio_pin_port_decode(&pin_number);

    return ((nrf_gpio_port_in_read(reg) >> pin_number) & 1UL);
}


__STATIC_INLINE uint32_t nrf_gpio_pin_out_read(uint32_t pin_number)
{
    NRF_GPIO_Type * reg = nrf_gpio_pin_port_decode(&pin_number);

    return ((nrf_gpio_port_out_read(reg) >> pin_number) & 1UL);
}


__STATIC_INLINE nrf_gpio_pin_sense_t nrf_gpio_pin_sense_get(uint32_t pin_number)
{
    NRF_GPIO_Type * reg = nrf_gpio_pin_port_decode(&pin_number);

    return (nrf_gpio_pin_sense_t)((reg->PIN_CNF[pin_number] &
                                   GPIO_PIN_CNF_SENSE_Msk) >> GPIO_PIN_CNF_SENSE_Pos);
}


__STATIC_INLINE void nrf_gpio_port_dir_output_set(NRF_GPIO_Type * p_reg, uint32_t out_mask)
{
    p_reg->DIRSET = out_mask;
}


__STATIC_INLINE void nrf_gpio_port_dir_input_set(NRF_GPIO_Type * p_reg, uint32_t in_mask)
{
    p_reg->DIRCLR = in_mask;
}


__STATIC_INLINE void nrf_gpio_port_dir_write(NRF_GPIO_Type * p_reg, uint32_t value)
{
    p_reg->DIR = value;
}


__STATIC_INLINE uint32_t nrf_gpio_port_dir_read(NRF_GPIO_Type const * p_reg)
{
    return p_reg->DIR;
}


__STATIC_INLINE uint32_t nrf_gpio_port_in_read(NRF_GPIO_Type const * p_reg)
{
    return p_reg->IN;
}


__STATIC_INLINE uint32_t nrf_gpio_port_out_read(NRF_GPIO_Type const * p_reg)
{
    return p_reg->OUT;
}


__STATIC_INLINE void nrf_gpio_port_out_write(NRF_GPIO_Type * p_reg, uint32_t value)
{
    p_reg->OUT = value;
}


__STATIC_INLINE void nrf_gpio_port_out_set(NRF_GPIO_Type * p_reg, uint32_t set_mask)
{
    p_reg->OUTSET = set_mask;
}


__STATIC_INLINE void nrf_gpio_port_out_clear(NRF_GPIO_Type * p_reg, uint32_t clr_mask)
{
    p_reg->OUTCLR = clr_mask;
}


__STATIC_INLINE void nrf_gpio_ports_read(uint32_t start_port, uint32_t length, uint32_t * p_masks)
{
    NRF_GPIO_Type * gpio_regs[GPIO_COUNT] = GPIO_REG_LIST;

    NRFX_ASSERT(start_port + length <= GPIO_COUNT);
    uint32_t i;

    for (i = start_port; i < (start_port + length); i++)
    {
        *p_masks = nrf_gpio_port_in_read(gpio_regs[i]);
        p_masks++;
    }
}


#ifdef GPIO_DETECTMODE_DETECTMODE_LDETECT
__STATIC_INLINE void nrf_gpio_latches_read(uint32_t start_port, uint32_t length, uint32_t * p_masks)
{
    NRF_GPIO_Type * gpio_regs[GPIO_COUNT] = GPIO_REG_LIST;
    uint32_t        i;

    for (i = start_port; i < (start_port + length); i++)
    {
        *p_masks = gpio_regs[i]->LATCH;
        p_masks++;
    }
}


__STATIC_INLINE uint32_t nrf_gpio_pin_latch_get(uint32_t pin_number)
{
    NRF_GPIO_Type * reg = nrf_gpio_pin_port_decode(&pin_number);

    return (reg->LATCH & (1 << pin_number)) ? 1 : 0;
}


__STATIC_INLINE void nrf_gpio_pin_latch_clear(uint32_t pin_number)
{
    NRF_GPIO_Type * reg = nrf_gpio_pin_port_decode(&pin_number);

    reg->LATCH = (1 << pin_number);
}


#endif
#endif // SUPPRESS_INLINE_IMPLEMENTATION

/**
 * Copyright (c) 2015 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
#endif // NRF_GPIO_H__

#endif /* Nordic_SDK_H__ */
