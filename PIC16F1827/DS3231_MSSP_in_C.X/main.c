 /*
 * MAIN Generated Driver File
 * 
 * @file main.c
 * 
 * @defgroup main MAIN
 * 
 * @brief This is the generated driver implementation file for the MAIN driver.
 *
 * @version MAIN Driver Version 1.0.2
 *
 * @version Package Version: 3.1.2
*/

/*
� [2024] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/
#include "mcc_generated_files/system/system.h"

#define DS3231_ADDRESS 0x68

void delay(unsigned long time_ms);
void convert(uint8_t *low, uint8_t *hight, uint8_t *data);

uint8_t low_seconds;
uint8_t hig_seconds;

uint8_t low_minuts;
uint8_t hig_minuts;


/*
    Main application
*/

int main(void)
{
    SYSTEM_Initialize();
    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts 
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global and Peripheral Interrupts 
    // Use the following macros to: 

    // Enable the Global Interrupts 
    INTERRUPT_GlobalInterruptEnable(); 

    // Disable the Global Interrupts 
    //INTERRUPT_GlobalInterruptDisable(); 

    // Enable the Peripheral Interrupts 
    INTERRUPT_PeripheralInterruptEnable(); 

    // Disable the Peripheral Interrupts 
    //INTERRUPT_PeripheralInterruptDisable(); 

    uint8_t raw_data[3];
    uint8_t raw_data_write[1];
    
    raw_data_write[0] = 0x00;
    raw_data[0] = 0x01;
    raw_data[1] = 0x05;
    raw_data[2] = 0x09;
    
    
    unsigned char mask = 0b00001111; // M�scara para seleccionar RB0-RB3
    
    I2C1_Write(DS3231_ADDRESS, raw_data, 3);
    
    while(1)
    {
        //PORTAbits.RA4 = 1;
        //delay(200);
        
        
        
        bool ok_wr = I2C1_WriteRead(DS3231_ADDRESS, raw_data_write, 1, raw_data, 3);
        if (ok_wr) {
            //PORTAbits.RA7 = 0;
        } else {
            //PORTAbits.RA7 = 1;
        }
        
        while(i2c1Status.state != 0){}
        
        //lower_seconds = raw_data[0] & 0x0F; // M�scara para los 4 bits menos significativos
        //upper_seconds = (raw_data[0] >> 4) & 0x0F; // Desplazar y aplicar m�scara
        
        convert(&low_seconds, &hig_seconds, &(raw_data[0]));
        convert(&low_minuts, &hig_minuts, &(raw_data[1]));
        
        delay(200);
        //PORTAbits.RA4 = 0;
    }    
}

void delay(unsigned long time_ms){
    for (unsigned long i=0; i<time_ms; i++ ){
        __delay_ms(1);
        if(PORTAbits.RA7 == 1){
            PORTA = low_minuts;
            PORTAbits.RA6 = 1;
        } else {
            PORTA = hig_minuts;
            PORTAbits.RA7 = 1;
        }
    }
}

void convert(uint8_t *low, uint8_t *hight, uint8_t *data){
    *low = *data & 0x0F; // M�scara para los 4 bits menos significativos
    *hight = (*data >> 4) & 0x0F; // Desplazar y aplicar m�scara
}
