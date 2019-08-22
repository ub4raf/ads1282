
#define BTN_GPIO_PERIPH (SYSCTL_PERIPH_GPIOF)
#define BTN_GPIO_BASE   (GPIO_PORTF_BASE)
#define BTN_LEFT        (GPIO_PIN_4)
#define BTN_RIGHT       (GPIO_PIN_0)

#define LED_GPIO_PERIPH (SYSCTL_PERIPH_GPIOF)
#define LED_GPIO_BASE   (GPIO_PORTF_BASE)
#define LED_RED         (GPIO_PIN_1)
#define LED_BLUE        (GPIO_PIN_2)
#define LED_GREEN       (GPIO_PIN_3)


#define UPLOAD_START    (0x6000)//(0x8000)
#define FLASH_MAX       (0x40000)
#define FIRMWARE_START  (UPLOAD_START   +0x2000)    //  8000
#define DATA_START      (FIRMWARE_START +0x2000)
#define PAGE_SIZE       1024

#define SPI_BITRATE     10000
#define CHAN_NUM        1


// define the peripheral/port/pins for the LEDs
#define LED_PERIF   SYSCTL_PERIPH_GPIOF
#define LED_PORT    GPIO_PORTF_BASE
#define LEDS        (LED_RED|LED_GREEN|LED_BLUE)
#define LED_SET(X)  ROM_GPIOPinWrite(LED_PORT, LEDS, (X));

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"
#include "utils/uartstdio.h"

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"

#include "utils/uartstdio.h"

#include "inc/hw_flash.h"
#include "inc/hw_gpio.h"
#include "jsmn.h"
#include <string.h>


uint8_t   data_load[PAGE_SIZE];

uint8_t read1282Register (uint8_t register_adr);
uint8_t write1282Register (uint8_t register_adr, uint8_t data);

void
ConfigureUART(void)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 115200, /*ROM_SysCtlClockGet());*/16000000);
}

int main(void)
{

    uint32_t i=0;
    uint32_t    byte[4];

    for (i = 0; i < PAGE_SIZE; i++) {
         data_load[i] = 0;
     }

    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);

    ConfigureUART();

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    ROM_SysCtlPeripheralEnable(LED_PERIF);
        ROM_GPIOPinTypeGPIOOutput(LED_PORT, LEDS);


         // Setup GPIO for buttons
         ROM_SysCtlPeripheralEnable(BTN_GPIO_PERIPH);

     #if BTN_GPIO_PERIPH == SYSCTL_PERIPH_GPIOF && (BTN_LEFT == GPIO_PIN_0)
         HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; // Unlocks the GPIO_CR register
         HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= GPIO_PIN_0; // Allow changes to PF0
         HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0; // Lock register again
     #endif

         ROM_GPIODirModeSet(BTN_GPIO_BASE, BTN_LEFT,  GPIO_DIR_MODE_IN);
         ROM_GPIOPadConfigSet(BTN_GPIO_BASE, BTN_LEFT,  GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
//while(1){}



    // Setup GPIO for buttons
    ROM_SysCtlPeripheralEnable(LED_GPIO_PERIPH);
    ROM_SysCtlPeripheralEnable(BTN_GPIO_PERIPH);


    while(!(ROM_SysCtlPeripheralReady(LED_GPIO_PERIPH)));
    //
    // Check if the peripheral access is enabled.
    //
    //while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))

        ROM_GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, BTN_LEFT | BTN_RIGHT);
        //HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; // Unlocks the GPIO_CR register
        //HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= GPIO_PIN_0; // Allow changes to PF0
        //HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0; // Lock register again
#define BTN_GPIO_PERIPH (SYSCTL_PERIPH_GPIOF)

#if BTN_GPIO_PERIPH == SYSCTL_PERIPH_GPIOF && (BTN_LEFT == GPIO_PIN_0 || BTN_RIGHT == GPIO_PIN_0)
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; // Unlocks the GPIO_CR register
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= GPIO_PIN_0; // Allow changes to PF0
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0; // Lock register again
#endif

        ROM_GPIODirModeSet(BTN_GPIO_BASE, BTN_LEFT | BTN_RIGHT,  GPIO_DIR_MODE_IN);
        ROM_GPIOPadConfigSet(BTN_GPIO_BASE, BTN_LEFT | BTN_RIGHT,  GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);


        ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
        ROM_GPIOPinTypeGPIOOutput(LED_GPIO_BASE, LED_GREEN | LED_BLUE| LED_RED);
        UARTprintf("Hello world");
        while(1)
            {
            // If one of the buttons is not pressed, jump to the user program
            if (!(ROM_GPIOPinRead(BTN_GPIO_BASE, BTN_LEFT)&BTN_LEFT))
            {
            ROM_SysCtlDelay(ROM_SysCtlClockGet() / 5);
            LED_SET(LED_RED);
            ROM_SysCtlDelay(ROM_SysCtlClockGet() / 5);
            LED_SET(LED_BLUE);
            }
            }

         SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

         SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

         GPIOPinConfigure(GPIO_PA2_SSI0CLK);
         GPIOPinConfigure(GPIO_PA3_SSI0FSS);
         GPIOPinConfigure(GPIO_PA4_SSI0RX);
         GPIOPinConfigure(GPIO_PA5_SSI0TX);
         //      PA5 - SSI0Tx
         //      PA4 - SSI0Rx
         //      PA3 - SSI0Fss
         //      PA2 - SSI0CLK
         GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 |
                        GPIO_PIN_2);

      #if defined(TARGET_IS_TM4C129_RA0) ||                                         \
         defined(TARGET_IS_TM4C129_RA1) ||                                         \
         defined(TARGET_IS_TM4C129_RA2)
         SSIConfigSetExpClk(SSI0_BASE, ui32SysClock, SSI_FRF_TI,
                            SSI_MODE_MASTER, SPI_BITRATE, 8);
     #else
         SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_TI,
                            SSI_MODE_MASTER, SPI_BITRATE, 8);
     #endif

         SSIEnable(SSI0_BASE);
         while(SSIDataGetNonBlocking(SSI0_BASE, &byte[0]))
         {
         }



    }

