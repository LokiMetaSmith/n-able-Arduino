 

#ifndef _VARIANT_OP2_52833_
#define _VARIANT_OP2_52833_

/** Master clock frequency */
#define VARIANT_MCK       (64000000ul)

#define USE_LFXO      // Board uses 32khz crystal for LF
// define USE_LFRC    // Board uses RC for LF

#define NRFX_POWER_ENABLED              1
#define NRFX_POWER_CONFIG_DEFAULT_DCDCEN
//#define NRFX_POWER_CONFIG_DEFAULT_DCDCENHV

#define NRFX_POWER_DEFAULT_CONFIG_IRQ_PRIORITY  7

#define NRFX_CLOCK_ENABLED 1

// <0=> RC
// <1=> XTAL
// <2=> Synth
#if defined(USE_LFRC)
#define NRFX_CLOCK_CONFIG_LF_SRC 0
#define NRFX_CLOCK_CONFIG_LF_CAL_ENABLED 1
#elif defined(USE_LFXO)
#define NRFX_CLOCK_CONFIG_LF_SRC 1
#elif defined(USE_LFSYNT)
#define NRFX_CLOCK_CONFIG_LF_SRC 2
#endif

#define NRFX_SPIM_ENABLED            1
#define NRFX_SPIM_MISO_PULL_CFG      1 // pulldown
#define NRFX_SPIM_EXTENDED_ENABLED   0

#define NRFX_SPIM0_ENABLED           0 // used as I2C
#define NRFX_SPIM1_ENABLED           0 // used as I2C
#define NRFX_SPIM2_ENABLED           1

#define NRFX_QSPI_ENABLED   0
#define NRFX_SPIM3_ENABLED  0
#define NRFX_NVMC_ENABLED   1

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

// Number of pins defined in PinDescription array
#define PINS_COUNT           (48u)
#define NUM_DIGITAL_PINS     (48u)
#define NUM_ANALOG_INPUTS    (6u)
#define NUM_ANALOG_OUTPUTS   (0u)

// LEDs
#define PIN_LED1             (10)
#define PIN_LED2             (38)
#define PIN_LED3             (9)


#define LED_BUILTIN          PIN_LED1
#define LED_CONN             PIN_LED2

#define LED_RED              PIN_LED1
#define LED_BLUE             PIN_LED2
#define LED_GREEN            PIN_LED3

#define LED_STATE_ON         0         // State when LED is litted

/*
 * Buttons
 */
#define PIN_BUTTON1          11
#define PIN_BUTTON2          8


/*
 * Analog pins
 */
#define PIN_AREF            (2)
#define PIN_A0               (2)//2
#define PIN_A1               (4)//3
#define PIN_A2               (28)//4
#define PIN_A3               (29)//5
#define PIN_A4               (30)//28
#define PIN_A5               (31)
#define PIN_A6               (0xff)//30
#define PIN_A7               (0xff)//31

static const uint8_t A0  = PIN_A0 ;
static const uint8_t A1  = PIN_A1 ;
static const uint8_t A2  = PIN_A2 ;
static const uint8_t A3  = PIN_A3 ;
static const uint8_t A4  = PIN_A4 ;
static const uint8_t A5  = PIN_A5 ;
static const uint8_t A6  = PIN_A6 ;
static const uint8_t A7  = PIN_A7 ;
#define ADC_RESOLUTION    14



/*
 * Serial interfaces
 */

// Arduino Header D0, D1
#define PIN_SERIAL1_RX      (33) // P1.01
#define PIN_SERIAL1_TX      (34) // P1.02

// Connected to Jlink CDC
#define PIN_SERIAL2_RX      (8)
#define PIN_SERIAL2_TX      (6)

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 0

#define PIN_SPI_MISO         (46)
#define PIN_SPI_MOSI         (45)
#define PIN_SPI_SCK          (47)

static const uint8_t SS   = 44 ;
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;

/*
 * Wire Interfaces (external and internal)
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (47)
#define PIN_WIRE_SCL         (44)

// Other
#define PIN_NFC1            (9)
#define PIN_NFC2            (10)
#define RESET_PIN           (18)

#ifdef __cplusplus
}
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#endif
