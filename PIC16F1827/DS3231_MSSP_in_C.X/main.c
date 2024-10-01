//CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection->INTOSC oscillator: I/O function on CLKIN pin
#pragma config WDTE = OFF    // Watchdog Timer Enable->WDT disabled
#pragma config PWRTE = OFF    // Power-up Timer Enable->PWRT disabled
#pragma config MCLRE = OFF    // MCLR Pin Function Select->MCLR/VPP pin function is digital input
#pragma config CP = OFF    // Flash Program Memory Code Protection->Program memory code protection is disabled
#pragma config CPD = OFF    // Data Memory Code Protection->Data memory code protection is disabled
#pragma config BOREN = OFF    // Brown-out Reset Enable->Brown-out Reset disabled
#pragma config CLKOUTEN = OFF    // Clock Out Enable->CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin
#pragma config IESO = ON    // Internal/External Switchover->Internal/External Switchover mode is enabled
#pragma config FCMEN = ON    // Fail-Safe Clock Monitor Enable->Fail-Safe Clock Monitor is enabled

//CONFIG2
#pragma config WRT = OFF    // Flash Memory Self-Write Protection->Write protection off
#pragma config PLLEN = ON    // PLL Enable->4x PLL enabled
#pragma config STVREN = OFF    // Stack Overflow/Underflow Reset Enable->Stack Overflow or Underflow will not cause a Reset
#pragma config BORV = LO    // Brown-out Reset Voltage Selection->Brown-out Reset Voltage (Vbor), low trip point selected.
#pragma config LVP = ON    // Low-Voltage Programming Enable->Low-voltage programming enabled

#include <xc.h>
#include <pic16f1827.h>

#define _XTAL_FREQ 16000000

#define DS3231_ADDRESS 0x68  // Dirección I2C del DS3231
/*
#define SDA PORTBbits.RB1;
#define SCL PORTBbits.RB4;
#define SEN SSP1CON2bits.SEN;
#define PEN SSP1CON2bits.PEN;
#define SSP1IF PIR1bits.SSP1IF;
#define BF SSP1STATbits.BF;
#define RW SSP1STATbits.R_nW;
*/

void __interrupt() tcInt(void) {
    __delay_us(1);
}

void main(void) {
    OSCCON = 0b11111011; // 16Mhz
    
    LATA = 0x00;
    LATB = 0x12;
    
    TRISA = 0x20;
    TRISB = 0x12;
    
    ANSELA = 0x00;
    ANSELB = 0x00;
    
    WPUA = 0x20;
    WPUB = 0x12;
    
    OPTION_REGbits.nWPUEN = 0x0;
    
    IOCBP = 0x0;
    IOCBN = 0x0;
    IOCBF = 0x0;
    
    SSP1STAT = 0x80;
    SSP1CON1 = 0x8;
    SSP1CON2 = 0x0;
    SSP1CON3 = 0x0;
    SSP1ADD = 0x27;
    
    PIE1bits.SSP1IE = 1;
    PIR1bits.SSP1IF = 0;
    PIE2bits.BCL1IE = 1;
    
    SSP1CON1bits.SSPEN = 1;
    
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1; // Enable global interrupt
    
    
    while (1) {
        __delay_us(100);
    
        SSP1CON2bits.SEN = 1;  // Generar condición de Start
        //while (SSP1CON2bits.SEN);  // Esperar a que se complete
        
        while(!PIR1bits.SSP1IF); /* Wait for interrupt generation by MSSP. */
        PIR1bits.SSP1IF = 0; /* Clear interrupt bit */
        
        __delay_us(200);
        
        PORTAbits.RA0 = !PORTAbits.RA0;
    }
    
    return;
    
}