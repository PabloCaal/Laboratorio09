/* 
 * File:   main.c
 * Author: Pablo Caal
 *
 * Created on 26 de abril de 2022, 11:59 AM
 */

// PIC16F887 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT        // Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF                   // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF                  // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF                  // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF                     // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF                    // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF                  // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF                   // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF                  // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF                    // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V               // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF                    // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

/*------------------------------------------------------------------------------
 * CONSTANTES 
 ------------------------------------------------------------------------------*/
#define _XTAL_FREQ 4000000
#define IN_MIN 0                // Valor minimo de entrada del potenciometro
#define IN_MAX 255              // Valor máximo de entrada del potenciometro
#define OUT_MIN 250             // Valor minimo de ancho de pulso de señal PWM   (18 para servo MG996R)
#define OUT_MAX 500             // Valor máximo de ancho de pulso de señal PWM   (79 para servo MG996R)

/* Cálculo de valor de OUT para cada ancho de pulso
 * OUT = (Ancho de pulso)/((1/Fosc)*PrecalerTMR2)
 *      OUT_MIN = (1ms)/((1/4MHz)*16) = 250
 *      OUT_MAX = (2ms)/((1/4MHz)*16) = 500
*/

/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/
unsigned short CCPR = 0, CCPR_2 = 0;    // Variable para almacenar ancho de pulso al hacer la interpolación lineal
uint8_t contador = 0;                   // Contador del timer0
int POT3 = 0;                           // Variable de tercer potenciómetro

/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
void setup(void);
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, 
            unsigned short out_min, unsigned short out_max);

/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){     
    if(PIR1bits.ADIF){                          // Verificación de interrupción del módulo ADC
        
        if(ADCON0bits.CHS == 0){                // Verificación de interrupción por canal AN0
            CCPR = map(ADRESH, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);   // Asignación de valor de ancho de pulso a CCPR
            CCPR1L = (uint8_t)(CCPR>>2);        // Almacenamiento de los 8 bits mas significativos en CCPR1L
            CCP1CONbits.DC1B = CCPR & 0b11;     // Almacenamiento de los 2 bits menos significativos en DC1B
        }
        
        else if(ADCON0bits.CHS == 1){           // Verificación de interrupción por canal AN1
            CCPR_2 = map(ADRESH, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Asignación de valor de ancho de pulso a CCPR_2
            CCPR2L = (uint8_t)(CCPR_2>>2);      // Guardamos los 8 bits mas significativos en CPR2L
            CCP2CONbits.DC2B0 = CCPR_2 & 0b01;    
            CCP2CONbits.DC2B1 = (CCPR_2 & 0b10)>>1;  // Almacenamiento de los 2 bits menos significativos en DC2B0 y DC2B1
        }
        
        else if(ADCON0bits.CHS == 2){           // Verificación de interrupción por canal AN2
            POT3 = ADRESH;                      // Asignación de valor de ancho de pulso a POT3
        }        
        PIR1bits.ADIF = 0;                      // Limpieza de bandera de interrupción del módulo ADC
    }
    
    else if(INTCONbits.T0IF){                   // Verificación de interrupción del TIMER0 
            contador++;                         // Aumento en el contador
            if(contador < POT3){                // Rango de contador: de 0 a 40
                PORTCbits.RC3 = 1;              // Activar el bit RC3 del PORTC
            } 
            else{
                PORTCbits.RC3 = 0;              // Activar el bit RC3 del PORTC
            }
            INTCONbits.T0IF = 0;                // Limpieza de bandera de interrupción del TIMER0
            TMR0 = 249;                         // Ingreso de número correspondiente para retardo del TIMER0            
    }        
    return;
}

/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    while(1){
        if (ADCON0bits.GO == 0){
            if(ADCON0bits.CHS == 0){        // Canal activo -> AN0
                ADCON0bits.CHS = 1;         // Cambio de canal
            }
            else if (ADCON0bits.CHS == 1){  // Canal activo -> AN1
                ADCON0bits.CHS = 2;         // Cambio de canal
            }
            else if (ADCON0bits.CHS == 2){  // Canal activo -> AN1
                ADCON0bits.CHS = 0;         // Cambio de canal
            }
             __delay_us(40);                // Sample time
            ADCON0bits.GO = 1;              // On
        }
    }
    return;
}

/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void setup(void){
    ANSEL = 0b111;              // AN0, AN1 y AN2 como entradas analógicas
    ANSELH = 0x00;              // I/O digitales
    TRISA = 0b111;              // AN0, AN1 y AN2 como entradas
    TRISC = 0b100;              // RC3 como salida
    PORTA = 0x00;               // Limpieza de PORTA
    
    // Configuración reloj interno
    OSCCONbits.IRCF = 0b110;    // 4 MHz
    OSCCONbits.SCS = 1;         // Oscilador interno
    
    // Configuración ADC
    ADCON0bits.ADCS = 0b01;     // Fosc/8
    ADCON1bits.VCFG0 = 0;       // VDD
    ADCON1bits.VCFG1 = 0;       // VSS
    ADCON0bits.CHS = 0b0000;    // Seleccionamos el AN0
    ADCON1bits.ADFM = 0;        // Justificado a la izquierda
    ADCON0bits.ADON = 1;        // Habilitamos modulo ADC
    __delay_us(40);             // Sample time  
    
    // Configuración PWM
    TRISCbits.TRISC2 = 1;       // Deshabilitamos salida de CCP1
    TRISCbits.TRISC1 = 1;       // Deshabilitamos salida de CCP2
    PR2 = 249;                  // periodo de 4 ms
    
    // PR2 = (PWM period)/(4(1/Fosc)(PrescalerTMR2))-1
    // PR2 = (4 ms)/(4(1/4MHz)(16))-1
    // PR2 = 250-1 = 249
    
    // Configuración CCP
    // CCP1
    CCP1CON = 0;                // Apagamos CCP1
    CCP1CONbits.P1M = 0;        // Modo single output
    CCP1CONbits.CCP1M = 0b1100; // PWM       
    CCPR1L = 250>>2;
    CCP1CONbits.DC1B = 250 & 0b11;    // 
    
    //CCP2
    CCP2CON = 0;                // Apagamos CCP2
    CCP2CONbits.CCP2M = 0b1100; // PWM       
    CCPR2L = 250>>2;                  
    CCP2CONbits.DC2B0 = 250 & 0b01;         // 
    CCP2CONbits.DC2B1 = (250 & 0b10)>>1;    // 
    
    PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2
    T2CONbits.T2CKPS = 0b11;    // prescaler 1:16
    T2CONbits.TMR2ON = 1;       // Encendemos TMR2
    while(!PIR1bits.TMR2IF);    // Esperar un cliclo del TMR2
    PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2 nuevamente
    
    TRISCbits.TRISC2 = 0;       // Habilitamos salida de PWM (CCP1)
    TRISCbits.TRISC1 = 0;       // Habilitamos salida de PWM (CCP2)
    
    // Configuracion interrupciones
    PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
    PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
    INTCONbits.GIE = 1;         // Habilitamos int. globales
    
    // Configuraciones para TIMER0
    INTCONbits.T0IE = 1;            // Habilitación de interrupciones del TIMER0
    INTCONbits.T0IF = 0;            // Limpieza bandera de interrupción del TIMER0
    OPTION_REGbits.T0CS = 0;        // Configuración del TIMER0 como temporizador
    OPTION_REGbits.PSA = 0;         // Configuración del Prescaler para el TIMER0 y no para el Watchdog timer
    OPTION_REGbits.PS = 0b000;      // Prescaler de 1:2 PS<2:0> = 000
       
    /* Cálculo del valor a ingresar al TIMER0 para que tenga retardo de 0.015 ms
	; N = 256 - (Temp/(4 x Tosc x Presc))
	; N = 256 - (0.015 ms/(4 x (1/4 MHz) x 2))
	; N = 249
     * Para el post laboratorio
    */
    TMR0 = 249;
}

/*------------------------------------------------------------------------------
 * FUNCIONES 
 ------------------------------------------------------------------------------*/
// Función para hacer la interpolación lineal del valor de la entrada analógica 
unsigned short map(uint8_t x, uint8_t x0, uint8_t x1, 
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}