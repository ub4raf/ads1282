//*****************************************************************************
//
// hello.c - Simple hello world example.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "driverlib/ssi.h"

#include "inc/hw_flash.h"
#include "jsmn.h"

#define BTN_CTL        (SYSCTL_PERIPH_GPIOF)
#define BTN_PORT        (GPIO_PORTF_BASE)
#define BTN_LEFT        (GPIO_PIN_4)
#define BTN_RIGHT       (GPIO_PIN_0)
#define BTNS            (BTN_LEFT|BTN_RIGHT)

#define LED_RED         (GPIO_PIN_1)
#define LED_BLUE        (GPIO_PIN_2)
#define LED_GREEN       (GPIO_PIN_3)

#define LED_CTL        (SYSCTL_PERIPH_GPIOF)
#define LED_PORT        (GPIO_PORTF_BASE)
#define LEDS            (LED_RED|LED_GREEN|LED_BLUE)
#define LED_SET(X)      ROM_GPIOPinWrite(LED_PORT, LEDS, (X))

#define UPLOAD_START    (0x6000)//(0x8000)
#define FLASH_MAX       (0x40000)
#define FIRMWARE_START  (UPLOAD_START   +0x2000)    //  8000
#define DATA_START      (FIRMWARE_START +0x2000)
#define PAGE_SIZE       1024

#define SPI_BITRATE     250000

typedef enum    {
        WAKEUP    = 0x00,
        STANDBY   = 0x02,
        SYNC      = 0x04,
        RESET     = 0x06,
        RDATAC    = 0x10, // Read data continuous
        SDATAC    = 0x11, // Stop read data continuous
        RDATA     = 0x12, // Read data on command (in SDATAC mode)
        OFSCAL    = 0x60,
        GANCAL    = 0x61
                } _1282command;

uint8_t     data[PAGE_SIZE];
uint8_t     freq=1;
uint8_t     channels=1;

void SysTickInt (void);

void
ConfigureUART(void)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 115200, 16000000);
}

void
ConfigureSPI(void)
{
    uint32_t byte;
     ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
     ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
     ROM_GPIOPinConfigure(GPIO_PA2_SSI0CLK);
     ROM_GPIOPinConfigure(GPIO_PA3_SSI0FSS);
     ROM_GPIOPinConfigure(GPIO_PA4_SSI0RX);
     ROM_GPIOPinConfigure(GPIO_PA5_SSI0TX);
     //      PA5 - SSI0Tx
     //      PA4 - SSI0Rx
     //      PA3 - SSI0Fss
     //      PA2 - SSI0CLK
     ROM_GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 |
                    GPIO_PIN_2);
  #if defined(TARGET_IS_TM4C129_RA0) ||                                         \
     defined(TARGET_IS_TM4C129_RA1) ||                                         \
     defined(TARGET_IS_TM4C129_RA2)
     SSIConfigSetExpClk(SSI0_BASE, ui32SysClock, SSI_FRF_TI,
                        SSI_MODE_MASTER, SPI_BITRATE, 8);
 #else
     ROM_SSIConfigSetExpClk(SSI0_BASE, ROM_SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,//SSI_FRF_TI,
                        SSI_MODE_MASTER, SPI_BITRATE, 8);
 #endif
     ROM_SSIEnable(SSI0_BASE);
     //while(SSIDataGetNonBlocking(SSI0_BASE, &byte)){}
     while(ROM_SSIDataGetNonBlocking(SSI0_BASE, &byte))
                 UARTprintf("cfg thrash 0x%X ",byte&0xFF);
}

void
ConfigureGPIO(void)
{
    ROM_SysCtlPeripheralEnable(BTN_CTL);
    ROM_SysCtlPeripheralEnable(LED_CTL);
    while(!(ROM_SysCtlPeripheralReady(LED_CTL)));
    ROM_GPIOPinTypeGPIOOutput(LED_PORT, LEDS);
         #if BTN_GPIO_PERIPH == SYSCTL_PERIPH_GPIOF && (BTN_LEFT == GPIO_PIN_0)
             HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; // Unlocks the GPIO_CR register
             HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= GPIO_PIN_0; // Allow changes to PF0
             HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0; // Lock register again
         #endif
    ROM_GPIODirModeSet(BTN_PORT, BTNS,  GPIO_DIR_MODE_IN);
    ROM_GPIOPadConfigSet(BTN_PORT, BTNS,  GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}

void ConfigureSysTick(void)
    {
        ROM_SysTickPeriodSet(ROM_SysCtlClockGet()/freq);
        ROM_SysCtlDelay(ROM_SysCtlClockGet()/100);
        //ROM_
        SysTickIntRegister(SysTickInt);
        ROM_SysTickIntEnable();
        ROM_SysTickEnable();
    }

void    send_command1282    (_1282command cmd)
    {
        uint32_t byte=0;
        ROM_SSIDataPut(SSI0_BASE,cmd);
        while(ROM_SSIBusy(SSI0_BASE)){}
        ROM_SysCtlDelay(3*ROM_SysCtlClockGet()/SPI_BITRATE);
        while(SSIDataGetNonBlocking(SSI0_BASE, &byte));
            //UARTprintf("CMD 0x%x thrash 0x%X ",cmd,byte&0xFF);
    }
uint8_t read_registers1282(uint8_t uiadress, uint32_t* data, uint8_t number)
    {
        uint32_t num,byte;
        //if  (number==0||number>0x1F)    return
        num = (number-1)&0x1F;
        send_command1282(SDATAC);   //stop
        ROM_SSIDataPut(SSI0_BASE,0x20+uiadress);
        while(ROM_SSIBusy(SSI0_BASE)){}
        ROM_SysCtlDelay(3*ROM_SysCtlClockGet()/SPI_BITRATE);
        while(SSIDataGetNonBlocking(SSI0_BASE, &byte))
            UARTprintf("rd %x thrash 0x%X ",0x20+uiadress,byte&0xFF);
        ROM_SSIDataPut(SSI0_BASE,num);
        while(ROM_SSIBusy(SSI0_BASE)){}
        ROM_SysCtlDelay(3*ROM_SysCtlClockGet()/SPI_BITRATE);
        while(SSIDataGetNonBlocking(SSI0_BASE, &byte))
            UARTprintf("nm %x thrash 0x%X\n",num, byte&0xFF);
        for(num=0;num<number;num++)
            {
                ROM_SSIDataPut(SSI0_BASE,0x00);
                while(ROM_SSIBusy(SSI0_BASE)){}
                ROM_SysCtlDelay(3*ROM_SysCtlClockGet()/SPI_BITRATE);
                //while(
                SSIDataGetNonBlocking(SSI0_BASE, &byte);
                data[num]=byte&0xFF;
                    UARTprintf("reg[%d]:0x%X  ",num,data[num]);
            }
        UARTprintf("\nread %d bytes\n",num);

        return number;
    }

uint8_t write_registers1282 (uint8_t uiadress, uint32_t* data, uint8_t number)
    {
        uint32_t num,byte;
        if  (number==0||number>0x1F)    return 1;
        num = (number-1)&0x1F;
        send_command1282(SDATAC);   //stop
        ROM_SSIDataPut(SSI0_BASE,0x40+uiadress);
        while(ROM_SSIBusy(SSI0_BASE)){}
        ROM_SysCtlDelay(3*ROM_SysCtlClockGet()/SPI_BITRATE);
        while(SSIDataGetNonBlocking(SSI0_BASE, &byte))
            UARTprintf("wr %x thrash 0x%X ",0x40+uiadress,byte&0xFF);
        ROM_SSIDataPut(SSI0_BASE,num);
        while(ROM_SSIBusy(SSI0_BASE)){}
        ROM_SysCtlDelay(3*ROM_SysCtlClockGet()/SPI_BITRATE);
        while(SSIDataGetNonBlocking(SSI0_BASE, &byte))
            UARTprintf("nm %x thrash 0x%X\n",num, byte&0xFF);
        for(num=0;num<number;num++)
            {
                ROM_SSIDataPut(SSI0_BASE,data[num]);
                while(ROM_SSIBusy(SSI0_BASE)){}
                ROM_SysCtlDelay(3*ROM_SysCtlClockGet()/SPI_BITRATE);
                //while(
                SSIDataGetNonBlocking(SSI0_BASE, &byte);
                    UARTprintf("thrash [%d]: 0x%X ",num,byte&0xFF);
            }
        UARTprintf("\nwrote %d bytes\n",num);
        uint32_t    data_buff[16];
        read_registers1282(uiadress,data_buff,number);
        for(num=0;num<number;num++)
             {
                if(data_buff[num]!=data[num]) return 2;
             };
        UARTprintf("\nall %d registers successful compared\n",num);
        return 0;
    }

uint8_t configure_1282(void)
    {
        uint32_t    byte[16];
        UARTprintf("ads1282 configuring:\n");
        ROM_SysCtlDelay(ROM_SysCtlClockGet()/10);
        send_command1282(RESET);
        ROM_SysCtlDelay(ROM_SysCtlClockGet()/10);
        send_command1282(SDATAC);
        UARTprintf("\nall reg read:\n");
        read_registers1282(0x00,byte,10);
        byte[0]=0x42;
        //byte[1]=0x08;//ain1
        byte[1]=0x18;//ain2
        //byte[1]=0x28;//400 ohm short
        //byte[1]=0x38;//ain1=ain2
        //byte[1]=0x48;//short to AINN2
        UARTprintf("\n2 reg write:\n");
        byte[15]=write_registers1282(0x01,byte,2);
        UARTprintf("\n2 reg compare: ");
        if(byte[15])
            {
                UARTprintf("registers are not equal\n");
                return 1;
            }
        UARTprintf("succes!\n");

        return 0;
    }

void    blink(uint32_t off_ms, uint32_t on_ms)
    {
        LED_SET(LED_RED);
        ROM_SysCtlDelay(on_ms*ROM_SysCtlClockGet()/1000);
        LED_SET(0);
        ROM_SysCtlDelay(off_ms*ROM_SysCtlClockGet()/1000);
        LED_SET(LED_BLUE);
        ROM_SysCtlDelay(on_ms*ROM_SysCtlClockGet()/1000);
        LED_SET(0);
        ROM_SysCtlDelay(off_ms*ROM_SysCtlClockGet()/1000);
        LED_SET(LED_GREEN);
        ROM_SysCtlDelay(on_ms*ROM_SysCtlClockGet()/1000);
        LED_SET(0);
        ROM_SysCtlDelay(off_ms*ROM_SysCtlClockGet()/1000);
    }

int
main(void)
{
    uint32_t i=0;
    uint32_t    byte[16];
    uint32_t page_index=0, data_index=0;
    for (i = 0; i < PAGE_SIZE; i++) data[i] = 0;
    ROM_FPULazyStackingEnable();
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);
    ConfigureUART();
    ConfigureGPIO();
    ConfigureSPI();
    UARTprintf("Hello ads1282\n");

    //for(i=DATA_START;i<FLASH_MAX;i+=PAGE_SIZE)   ROM_FlashErase(i);
    UARTprintf("Flash erased from 0x%x to 0x%x\n",DATA_START,FLASH_MAX);
    byte[0]=configure_1282();
    if(byte[0])
        {
            UARTprintf("ERROR during initialization: %d\n",byte[0]);
            while(1)    blink(1000,1000);
        }
    ConfigureSysTick();
    //send_command1282(RDATA);// manual adc start
    //send_command1282(RDATAC);//start auto adc
    while(page_index*PAGE_SIZE<FLASH_MAX-DATA_START)
    {
        while (ROM_GPIOPinRead(BTN_PORT, BTN_LEFT))
            ROM_SysCtlDelay(ROM_SysCtlClockGet() / 1000000);

        ROM_SSIDataPut(SSI0_BASE,0x00);
        ROM_SSIDataPut(SSI0_BASE,0x00);
        ROM_SSIDataPut(SSI0_BASE,0x00);
        ROM_SSIDataPut(SSI0_BASE,0x00);
        while(ROM_SSIBusy(SSI0_BASE)){}
        ROM_SysCtlDelay(3*ROM_SysCtlClockGet()/SPI_BITRATE);
        for(i=0;i<4;i++)
            {
                while(SSIDataGetNonBlocking(SSI0_BASE, &byte[i]))
                    UARTprintf(" data[%d]:0x%X ",i,byte[i]&0xFF);
            }

        blink(1,1);
        data_index+=5;
        if(data_index>PAGE_SIZE)
            {
                //ROM_FlashWrite();
                UARTprintf("Page wrote at %X with %d bytes\n",page_index*PAGE_SIZE+DATA_START,PAGE_SIZE);
                page_index++;
                data_index-=PAGE_SIZE;
            }
    }
    //1282_aq_stop();
    UARTprintf("Flash is full, aq stopped\n");
    while(1)
        {
            blink(0,300);
        }
}

void SysTickInt (void)
    {
        send_command1282(RDATA);//start adc shot
        UARTprintf("\n");
    }
