/*
 * File:   main.c
 * Author: andym
 *
 * Created on 16 May 2021, 20:51
 * 
 * Version 3, 17th July 2021:  Added supply voltages and fan speed sensing.
 * Version 4, 18th Sept 2021:  Changed to use fixed 50 byte universal packet size.
 * 
 * LoRa Temperature and Relative Humidity Sensor Transmitter.
 * 
 * Temperature sensed using MAX31865 PT1000 interface connected to SPI.
 * RH sensed using HIH5031 connected to ADS1110 on I2C interface.
 * 
 * RA2 controls sensor & RS232 supply & divider & reference (turn off when not in use - high is off, low is on)
 * driven by PNP transistor from port pin to get enough voltage drive.
 * 
 * RFM95W LoRa Module

 * AN0 reads battery voltage through a resistor divider
 * AN1 reads local temperature through 10k NTC and 10k resistor as a divider from 3.3V
 * AN2 reads VIN
 * AN3 reads Fan Voltage
 * RE2 has red LED
 * RE1 has green LED
 * RB5 has fan tacho input.
 * 
 * Internal oscillator used at 64MHz (16MHz with 4x PLL)
 * 
 * Reference Sensor Types
 * 0    BASE RECEIVER
 * 1    Rain
 * 2    UV/Visible Light
 * 3    Temp/RH* (This sensor)
 * 4    Extra temp(s)
 * 5    Soil moisture
 * 6    Lightning
 * 7    Radiation
 * 8    Wind
 * 9    Leaf wetness
 * 10   Electric Field
 */


#include <xc.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "config.h"
#include "defines.h"
#include "usart2.h"
#include "LoRa.h"
#include "i2c1.h"
#include "MAX31865.h"
#include "ADS1110.h"
#include "CRC16.h"

#define DEBUG 1
#define TX_FREQ 866.5
#define SYNC_WORD 0x55
#define BATT_UVLO 2100 //2V UVLO below which transmitter operation is prevented
#define BATT_UVLO_ATOD BATT_UVLO/4
#define DATA_PACKET_LENGTH 50
#define ID0 0x00
#define ID1 0x03
#define SOFTWARE_VERSION 0x04

void configureIO();
uint16_t readBattery();
uint16_t readTemperature();
uint16_t readVSupply();
uint16_t readVFan();
void setupAtoD();
uint16_t readRH();
void transmitData(void);
void disablePeripherals(void);
void setupInterruptPin(void); //Sets up interrupt on fan tacho input

uint8_t txData[DATA_PACKET_LENGTH]; //Transmit buffer
uint8_t address[8] = {0x6C,0x27,0xDA,0x88,0x2F,0xE4,0x1A,0xF0}; //This should be unique
uint16_t batt=0; //Battery (microcontroller supply) voltage reading from internal A to D
uint16_t temp=0; //Temperature reading from internal A to D
uint16_t vsupply=0; //Supply voltage reading from internal A to D
uint16_t vfan=0;  //Fan voltage reading from internal A to D
uint32_t messageCount=0;
uint16_t extTemp=0; //Temperature reading from MAX31865
uint16_t rh=0;      //RH reading from ADS1110
uint32_t fanTachoCount=0; //Counts fan tacho pulses, resets everytime a transmit occurs.

void main(void) {
    setupInterruptPin();
    while(1){
        configureIO();
        ClrWdt(); //Reset watchdog timer
        GREEN_LED=1;
        configureSPI();
        __delay_ms(10); //Wait for things to power up
        MAX31865writeConfigByte(0xC1); //Configure the device
        __delay_ms(100); //Wait for temperature reading to stabilise
        batt = readBattery();
        temp = readTemperature();
        vsupply = readVSupply();
        vfan = readVFan();
        extTemp = MAX31865readData();
        rh = readRH();
        

        if(batt>BATT_UVLO_ATOD){
            INTCONbits.GIE=0; //Global interrupt disable
            transmitData();
            INTCONbits.GIE=1; //Global interrupt enable
        }
        else{
            //Flash the red LED 3 times
            RED_LED=1; //Red LED on
            __delay_ms(300);
            RED_LED=0;
            __delay_ms(300);
            RED_LED=1; //Red LED on
            __delay_ms(300);
            RED_LED=0;
            __delay_ms(300);
            RED_LED=1; //Red LED on
            __delay_ms(300);
            RED_LED=0;
            __delay_ms(300);
        }
        



        if(DEBUG){
            printf("BATT %d\r\n", batt);
            printf("TEMP %d\r\n", temp);
            printf("EXT TEMP %d\r\n", extTemp);
            printf("EXT RH %d\r\n", rh);
            printf("VSUPP %d\r\n", vsupply);
            printf("VFAN %d\r\n", vfan);
            printf("TACHO %d\r\n", fanTachoCount);
        }
        //disablePeripherals(); //Turn everything off
        //No sleeping, unit has permanent power
        GREEN_LED=0; //Off when waiting
        fanTachoCount=0;
        for(uint16_t i=0;i<60;i++){
            __delay_ms(1000);
            ClrWdt(); //Reset watchdog timer
        }

    }
}

void transmitData(){
    if(DEBUG){
        printf("Transmitting...\r\n");
    }

    
    
    txData[0] = DATA_PACKET_LENGTH;
    txData[1] = ID0; //Copy in the ID
    txData[2] = ID1; //Copy in the ID
    for(uint8_t i=0;i<8;i++){
        txData[i+3] = address[i]; //Copy in the address
    }
    txData[11] = SOFTWARE_VERSION;
    
    //Message count
    txData[12]=(uint8_t)((messageCount>>24)&0xFF); //MSB
    txData[13]=(uint8_t)((messageCount>>16)&0xFF); //Upper middle
    txData[14]=(uint8_t)((messageCount>>8)&0xFF); //Lower middle
    txData[15]=(uint8_t)((messageCount & 0xFF)); //LSB
    
    //MCU supply voltage value (10-bit in 2 bytes)
    txData[16]=(uint8_t)((batt>>8)&0xFF); //MSB
    txData[17]=(uint8_t)(batt & 0xFF); //LSB
    
    //Sensor local temperature value (16-bit)
    txData[18]=(uint8_t)((temp>>8)&0xFF); //MSB
    txData[19]=(uint8_t)(temp & 0xFF); //LSB
    
    txData[20] = (uint8_t)((vsupply>>8)&0xFF); //MSB
    txData[21] = (uint8_t)((vsupply & 0xFF)); //LSB
    
    txData[22] = (uint8_t)((vfan>>8)&0xFF); //MSB
    txData[23] = (uint8_t)((vfan & 0xFF)); //LSB
    
    //External temperature
    txData[24]=(uint8_t)((extTemp>>8)&0xFF); //MSB
    txData[25]=(uint8_t)((extTemp)&0xFF); //LSB
    
    //External relative humidity
    txData[26]=(uint8_t)((rh>>8)&0xFF); //MSB
    txData[27]=(uint8_t)((rh & 0xFF)); //LSB
    
    //Fan tacho count
    txData[28] = (uint8_t)((fanTachoCount>>24)&0xFF); //MSB
    txData[29] = (uint8_t)((fanTachoCount>>16)&0xFF); //Byte 2
    txData[30] = (uint8_t)((fanTachoCount>>8)&0xFF); //Byte 3
    txData[31] = (uint8_t)((fanTachoCount & 0xFF)); //LSB
    
    //Fill the rest of the data area with 0
    for(uint8_t i=32;i<48;i++){
        txData[i] = 0;
    }
    
    //Calculate CRC16 and add to end of message
    unsigned short int calcCRC = CRC16(txData, DATA_PACKET_LENGTH-2);
    txData[49] = (calcCRC&0xFF00u)>>8u; //MSB
    txData[48] = (calcCRC&0xFF); //LSB
    
    //Set the transmitter up and send the data
    LoRaStart(TX_FREQ, SYNC_WORD); //Configure module
    if(DEBUG){
        printf("TXF: %f\r\n", LoRaGetFrequency());
    }
    LoRaClearIRQFlags();
    RED_LED=1; //Red LED on
    LoRaTXData(txData, DATA_PACKET_LENGTH); //Send data
    if(DEBUG){
        printf("Wait for end of transmission...\r\n");
    }
    uint8_t j=0;
    for(j=0;j<50;j++){
        uint8_t flags = LoRaGetIRQFlags();
        //printf("IRQ %d %d \r\n",j, flags);
        if(flags>0){
            break;
        }
        __delay_ms(10); //We are done with transmission
    }
    if(DEBUG){
        if(j>48){
            printf("TX Fail\r\n");
        }
        else{
            printf("Done.\r\n");
        }
    }
    LoRaSleepMode(); //Put module to sleep
    __delay_ms(10);
    messageCount++;
    RED_LED=0; //Red LED off
}

uint16_t readRH(){
    uint16_t rhTemp = readADS1110(ADS1110_ADDRESS);
    return rhTemp;
}

void configureIO(){
    //Internal oscillator
    OSCCONbits.IRCF=0b111; //Set internal clock to 16MHz
    OSCCONbits.OSTS=0; //Device is running from internal oscillator
    OSCCON2bits.PLLRDY=1; //System clock comes from 4xPLL
    OSCTUNEbits.PLLEN=1;//Enable PLL
    
    //SPI2 pins
    TRISDbits.RD1=1; //SDI2 set as input (MISO2)
    TRISDbits.RD4=0; //SDO2 set as output (MOSI2)
    TRISDbits.RD0=0; //SCK2 set as output (SCK2)
    TRISDbits.RD3=0; //#SS2 set as output (CS2)
    TRISCbits.RC0=0; //#SS for MAX31865
    ANSELDbits.ANSD0=0; //Turn off analogue function of port pin
    ANSELDbits.ANSD1=0;
    ANSELDbits.ANSD2=0;
    ANSELDbits.ANSD4=0;
    
    PMD2bits.ADCMD=0; //Turn ADC on
    if(DEBUG){
        PMD0bits.UART2MD=0; //Turn UART2 on
    }
    PMD1bits.MSSP1MD=0; //Turn I2C on
    PMD1bits.MSSP2MD=0; //Turn SPI2 on
    ANSELAbits.ANSA2=0; //Analogue off
    TRISAbits.RA2=0; //Output
    LATAbits.LATA2=0; //External circuitry on
    ANSELEbits.ANSE1=0; //Turn off analogue on RE1
    ANSELEbits.ANSE2=0; //Turn off analogue on RE2
    ANSELBbits.ANSB4=0; //Turn off analogue on RB4
    TRISEbits.RE1=0; //Green LED for status
    TRISEbits.RE2=0; //Red LED for status
    TRISBbits.RB5=1; //Input (fan tacho)
    ANSELBbits.ANSB5=0; //Turn off analogue on RB5
    RED_LED=0; //Red LED off
    if(DEBUG){
        USART2_Start(BAUD_57600); //Start USART2
    }
    setupAtoD(); //Setup to read AN0 (reads supply voltage [battery])
    I2C1_Initialize(100000); //Starts I2C module 1 (100kHz fixed)
    I2C1_Check_Data_Stuck(); //Check if bus is stuck and attempt to unstick it.
    configADS1110(); //Configure the external A to D converter.
}

void setupAtoD(){
    //Setup AN0 for a to d converter (RA0)
    
    //Set ANSELbit to disable digital input buffer
    ANSELAbits.ANSA0=1;
    ANSELAbits.ANSA1=1;
    ANSELAbits.ANSA2 = 1;
    ANSELAbits.ANSA3 = 1;
    
    //Set TRISXbit to disable digital output driver
    TRISAbits.RA0=1;
    TRISAbits.RA1=1;
    TRISAbits.RA2 = 1;
    TRISAbits.RA3 = 1;
    
    //Set voltage references
    ADCON1bits.PVCFG=0; //A/D Vref+ connected to Vdd
    ADCON1bits.NVCFG=0; //A/D Vref- connected to internal signal AVss
    VREFCON0bits.FVRS=0b01; //Fixed voltage reference is 1.024V
    VREFCON0bits.FVREN=1; //Enable internal reference
    
    //Select channel 0 for A to D
    ADCON0bits.CHS=0;
    
    //Set A to D acquisition time
    ADCON2bits.ACQT=0b010; //Tacq = 4 Tad (4탎)
    
    //Set A to D clock period
    ADCON2bits.ADCS=0b110; //Clock period set to Fosc/64 = 1탎 (64MHz clock)
    
    //Set result format
    ADCON2bits.ADFM = 1; //Data is mostly in the ADRESL register with 2 bits in the ADRESH register
    
    //Turn on the A to D module
    ADCON0bits.ADON=1;
}

/**
 * Reads the microcontroller supply voltage A to D
 */
uint16_t readBattery(){
    //Select channel 0 for A to D
    ADCON0bits.CHS=0;
    ADCON1bits.PVCFG=0b10; //A/D Vref+ connected to internal reference FVR BUF2
    ADCON0bits.GO_NOT_DONE=1; //Start the A to D process
    while(ADCON0bits.GO_NOT_DONE){
        //Wait for conversion to complete (about 15탎)
    }
    uint16_t result = ADRESH * 256 + ADRESL; //Read A to D result
    return result;
}

uint16_t readTemperature(){
    ADCON1bits.PVCFG=0; //A/D Vref+ connected to Vdd
    //Select channel 1 for A to D
    ADCON0bits.CHS=1;
    ADCON0bits.GO_NOT_DONE=1; //Start the A to D process
    while(ADCON0bits.GO_NOT_DONE){
        //Wait for conversion to complete (about 15탎)
    }
    uint16_t result = ADRESH * 256 + ADRESL; //Read A to D result
    return result;
}

/**
 * Reads the incoming supply voltage A to D
 */
uint16_t readVSupply(){
    //Select channel 2 for A to D
    ADCON0bits.CHS=0b10;
    ADCON1bits.PVCFG=0b10; //A/D Vref+ connected to internal reference FVR BUF2
    ADCON0bits.GO_NOT_DONE=1; //Start the A to D process
    while(ADCON0bits.GO_NOT_DONE){
        //Wait for conversion to complete (about 15탎)
    }
    uint16_t result = ADRESH * 256 + ADRESL; //Read A to D result
    return result;
}

/**
 * Reads the fan voltage A to D
 */
uint16_t readVFan(){
    //Select channel 3 for A to D
    ADCON0bits.CHS=0b11;
    ADCON1bits.PVCFG=0b10; //A/D Vref+ connected to internal reference FVR BUF2
    ADCON0bits.GO_NOT_DONE=1; //Start the A to D process
    while(ADCON0bits.GO_NOT_DONE){
        //Wait for conversion to complete (about 15탎)
    }
    uint16_t result = ADRESH * 256 + ADRESL; //Read A to D result
    return result;
}

void disablePeripherals(){
    ADCON0bits.ADON=0; //Turn off A to D module
    //Set all pins as outputs
    TRISA=0;
    TRISB=0x01; //Set all outputs except RB0
    TRISC=0;
    TRISD=0;
    TRISE=0;
    LATA=0;
    LATB=0;
    LATC=0;
    LATD=0;
    LATE=0;
    LATAbits.LA2=1; //Turn off external peripherals
    //Deal with SDI in case LoRa module is driving it
    TRISDbits.RD1=1; //SDIx must have corresponding TRIS bit set (input)
    ANSELDbits.ANSD1=0; //Input buffer enabled
    LATDbits.LATD3=1; //Set SS high so LoRa chip is not selected
    //LATCbits.LATC0=1; //Set SS high for MAX31865 so it is not selected
    PMD0bits.UART2MD=1; //Turn off UART2
    PMD0bits.UART1MD=1; //Turn off UART1
    PMD0bits.TMR6MD=1; //Turn off timer 6
    PMD0bits.TMR5MD=1; //Turn off timer 5
    PMD0bits.TMR4MD=1; //Turn off timer 4
    PMD0bits.TMR3MD=1; //Turn off timer 3
    PMD0bits.TMR2MD=1; //Turn off timer 2
    PMD0bits.TMR1MD=1; //Turn off timer 1
    PMD0bits.SPI2MD=1; //Turn off SPI2
    PMD0bits.SPI1MD=1; //Turn off SPI1
    PMD1=0xFF; //Turn off all peripherals in PMD1 (including MSSP2 for SPI2)
    PMD2=0xFF; //Turn off all peripherals in PMD2 (ADC, comparators, CTMU)
}

void setupInterruptPin(void){
    INTCONbits.RBIE=1; //Enables the IOCx port change interrupt
    IOCBbits.IOCB5=1; //Enable interrupt on RB4 change
    INTCONbits.PEIE=1; //Enables unmasked peripheral interrupts
    INTCONbits.GIE=1; //Global interrupt enable
    INTCONbits.RBIF=0; //Clear interrupt flag for port B bits
}

void __interrupt() Isr(void){
    uint8_t bValue = PORTBbits.RB5; //Read port b
    if(bValue>0){
        //port bit has changed to a 1
        fanTachoCount++;
    }
    INTCONbits.RBIF=0; //Clear interrupt flag for port B
    //GREEN_LED=!GREEN_LED;
}
