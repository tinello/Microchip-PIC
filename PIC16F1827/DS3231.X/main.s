PROCESSOR 16F1827

; PIC16F1827 Configuration Bit Settings

; Assembly source line config statements

; CONFIG1
  CONFIG  FOSC = INTOSC         ; Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
  CONFIG  WDTE = OFF            ; Watchdog Timer Enable (WDT disabled)
  CONFIG  PWRTE = ON            ; Power-up Timer Enable (PWRT enabled)
  CONFIG  MCLRE = OFF           ; MCLR Pin Function Select (MCLR/VPP pin function is digital input)
  CONFIG  CP = OFF              ; Flash Program Memory Code Protection (Program memory code protection is disabled)
  CONFIG  CPD = OFF             ; Data Memory Code Protection (Data memory code protection is disabled)
  CONFIG  BOREN = OFF           ; Brown-out Reset Enable (Brown-out Reset disabled)
  CONFIG  CLKOUTEN = OFF        ; Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
  CONFIG  IESO = OFF            ; Internal/External Switchover (Internal/External Switchover mode is disabled)
  CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

; CONFIG2
  CONFIG  WRT = OFF             ; Flash Memory Self-Write Protection (Write protection off)
  CONFIG  PLLEN = OFF           ; PLL Enable (4x PLL disabled)
  CONFIG  STVREN = OFF          ; Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will not cause a Reset)
  CONFIG  BORV = LO             ; Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
  CONFIG  DEBUG = OFF           ; In-Circuit Debugger Mode (In-Circuit Debugger disabled, ICSPCLK and ICSPDAT are general purpose I/O pins)
  CONFIG  LVP = OFF             ; Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

// config statements should precede project file includes.
#include <xc.inc>


PSECT udata_shr
	    
;Constantes
;PORTA Bits
PORTA_RA0		EQU 0x00
PORTA_RA1_START		EQU 0x01
PORTA_RA2		EQU 0x02
PORTA_RA3		EQU 0x03
PORTA_RA4		EQU 0x04
PORTA_RA5		EQU 0x05
PORTA_RA6		EQU 0x06
PORTA_RA7		EQU 0x07

;PORTB Bits
PORTB_RB0_SOLENOIDE	EQU 0x00
PORTB_RB1_PUMP		EQU 0x01
PORTB_RB2		EQU 0x02
PORTB_RB3		EQU 0x03
PORTB_RB4		EQU 0x04
PORTB_RB5		EQU 0x05
PORTB_RB6		EQU 0x06
PORTB_RB7		EQU 0x07

;OPTION_REG Bits
OPTION_NOT_RBPU		EQU 0x07
		
STATUS_C		EQU 0x00

;Variables
retardo1		EQU 0x20
retardo2		EQU 0x21
retardo3		EQU 0x22

	
	
	
PSECT	code, delta=2, abs
ORG	0x0000
resetVector:
	goto	setup

	
PSECT	code, delta=2, abs
ORG	0x000A
setup:
conf_internal_clock_8mhz:
	banksel	OSCCON
	movlw	01101010B ; Clock interno, 4Mhz, PLL disable.
        movwf	OSCCON
	
	
	
	bcf	OPTION_REG, 7 ; Weak pull-ups are enabled by individual WPUx latch values
	
	banksel	PORTA
	clrf	PORTA
	banksel	LATA
	clrf	LATA
	banksel	ANSELA
	clrf	ANSELA
	banksel	TRISA
	movlw	00100000B ; 1 = inputs, 0 = outpus
	movwf	TRISA
	
	banksel	PORTB
	clrf	PORTB
	banksel	LATB
	clrf	LATB
	banksel	ANSELB
	clrf	ANSELB
	banksel	WPUB
	movlw	00000011B ; 0 = Disabled pull-up, 1 = Enabled pull-up
	movwf	WPUB
	banksel	TRISB
	movlw	00000011B ; 1 = inputs, 0 = outpus
	movwf	TRISB
	
	
	
main:
    movlb   0		    ; Bank 0
    ;call    sleep_1_second
    call    Retardo_4micros
    call    Retardo_4micros
    call    Retardo_4micros
    call    Retardo_4micros
    call    Retardo_4micros
    call    Retardo_4micros
    call    Retardo_4micros
    call    DS3231_Read
    movf    Segundo, 0
    movwf   PORTA	    ; Muestro los segundos en el Puerto A
    
    ;prueba alta impedancia
    ;call    sleep_1_second
    ;bcf	    PORTB, 0
    ;call    sleep_1_second
    ;bsf	    PORTB, 0
    
    goto    main

    
    
    
    
;**************** Librería "RETARDOS.INC" Para MPLAB X PIC-AS ******************
;
; Estas subrutinas permiten realizar las tareas de retardos y esperas.
;
;*******************************************************************************
Retardo_4micros:    ; La llamada "call" aporta 2 ciclos máquina.
    return	    ; El salto del retorno aporta 2 ciclos máquina.



; SLEEP 1 Second
sleep_1_second:
	movlb   0		    ; Bank 0
	movlw	0x0C
	movwf	retardo3

sleep_1_second_three:
	movlw	0xFC
	movwf	retardo2
sleep_1_second_two:
	movlw	0xFA
	movwf	retardo1
sleep_1_second_one:
	decfsz	retardo1
	goto	sleep_1_second_one
	
	decfsz	retardo2
	goto	sleep_1_second_two
	
	decfsz	retardo3
	goto	sleep_1_second_three
	
	retlw	0x00
    
    
    
    

;**************** Librería "I2C.INC" Para MPLAB X PIC-AS *******************
;
; Estas subrutinas permiten realizar las tareas básicas de control del bus serie
; I2C, por parte de un solo microcontrolador maestro.
;
; ZONA DE DATOS ****************************************************************
;
PSECT udata_shr
I2C_ContadorBits    EQU 0x30	; Cuenta los bits a transmitir o a recibir.
I2C_Dato	    EQU	0x31	; Dato a transmitir o recibido.
I2C_Flags	    EQU 0x32	; Guarda la información del estado del bus I2C.

scl_pin	    EQU 0x00
sda_pin	    EQU 0x01
i2c_port    EQU 0x0D	; Port B
i2c_tris    EQU 0x8D	; Tris B


; ZONA DE PROGRAMA *************************************************************
PSECT	code, delta=2
;
; Subrutina "SDA_Bajo" ---------------------------------------------------------
;
SDA_Bajo:
	banksel	i2c_tris	    ; Configura la línea SDA como salida.
	bcf	i2c_port, sda_pin
	banksel	i2c_port
	bcf	i2c_port, sda_pin   ; SDA en bajo.
	return
;
; Subrutina "SDA_AltaImpedancia" --------------------------------------------------------
;
SDA_AltaImpedancia:
	banksel	i2c_tris		; Configura la línea SDA entrada.
	bsf	i2c_port, sda_pin				; Lo pone en alta impedancia y, gracias a la
	banksel	i2c_port		; Rp de esta línea, se mantiene a nivel alto.
	return
;
; Subrutina "SCL_Bajo" ------------------------------------------------------------------
;
SCL_Bajo:
	banksel	i2c_tris
	bcf	i2c_port, scl_pin	; La línea de reloj SCL en bajo. Escribe en el bus.
	banksel	i2c_port
	bcf	i2c_port, scl_pin
	return
;
; Subrutina "SCL_AltaImpedancia" --------------------------------------------------------
;
SCL_AltaImpedancia:
	banksel	i2c_tris
	bsf	i2c_port, scl_pin	; Lo pone en alta impedancia, mantiene un nivel alto y se puede leer el bus.
	banksel	i2c_port
SCL_EsperaNivelAlto:
	btfss	i2c_port, scl_pin	; Si algún esclavo mantiene esta línea en bajo
	goto	SCL_EsperaNivelAlto	; hay que esperar.
	return
;
; Subrutina "I2C_EnviaStart" ------------------------------------------------------------
;
; Esta subrutina envía una condición de Start o inicio.
;
I2C_EnviaStart:
	call	SDA_AltaImpedancia		; Línea SDA en alto.
	call	SCL_AltaImpedancia		; Línea SCL en alto.
	call	Retardo_4micros			; Tiempo tBUF del protocolo.
	call	SDA_Bajo				; Flanco de bajada de SDA mientras SCL está alto.
	call	Retardo_4micros			; Tiempo tHD;STA del protocolo.
	call	SCL_Bajo				; Flanco de bajada del reloj SCL.
	call	Retardo_4micros	
	return
;
; Subrutina "I2C_EnviaStop" -------------------------------------------------------------
;
; Esta subrutina envía un condición de Stop o parada.
;
I2C_EnviaStop:
	call	SDA_Bajo
	call	SCL_AltaImpedancia		; Flanco de subida de SCL.
	call	Retardo_4micros			; Tiempo tSU;STO del protocolo.
	call	SDA_AltaImpedancia		; Flanco de subida de SDA.
	call	Retardo_4micros			; Tiempo tBUF del protocolo.
	return
;
; Subrutina "I2C_EnviaByte" -------------------------------------------------------------
;
; El microcontrolador maestro transmite un byte por el bus I2C, comenzando por el bit
; MSB. El byte a transmitir debe estar cargado previamente en el registro de trabajo W.
; De la subrutina ejecutada anteriormente I2C_EnviaStart o esta misma I2C_EnviaByte, 
; la línea SCL se debe encontrar a nivel bajo al menos durante 5 µs.
;
I2C_EnviaByte:
	movwf	I2C_Dato				; Almacena el byte a transmitir.
	movlw	0x08					; A transmitir 8 bits.
	movwf	I2C_ContadorBits
I2C_EnviaBit:
	rlf	I2C_Dato,1				; Chequea el bit, llevándolo previamente al Carry.
	btfsc	STATUS, STATUS_C
	goto	I2C_EnviaUno
I2C_EnviaCero:
	call	SDA_Bajo				; Si es "0" envía un nivel bajo.
	goto	I2C_FlancoSCL
I2C_EnviaUno:
	call	SDA_AltaImpedancia		; Si es "1" lo activará a alto.
I2C_FlancoSCL:
	call	SCL_AltaImpedancia		; Flanco de subida del SCL.
	call	Retardo_4micros			; Tiempo tHIGH del protocolo.
	call	SCL_Bajo				; Termina el semiperiodo positivo del reloj.
	call	Retardo_4micros			; Tiempo tHD;DAT del protocolo.
	decfsz	I2C_ContadorBits		; Lazo para los ocho bits.
	goto	I2C_EnviaBit
	call	SDA_AltaImpedancia		; Libera la línea de datos.
	call	SCL_AltaImpedancia		; Pulso en alto de reloj para que el esclavo
	call	Retardo_4micros			; pueda enviar el bit ACK.
	call	SCL_Bajo
	call	Retardo_4micros
	return
;
; Subrutina "I2C_LeeByte" ---------------------------------------------------------------
;
; El microcontrolador maestro lee un byte desde el esclavo conectado al bus I2C. El dato
; recibido se carga en el registro I2C_Dato y lo envía a la subrutina superior a través
; del registro W. Se empieza a leer por el bit de mayor peso MSB.
; De alguna de las subrutinas ejecutadas anteriormente I2C_EnviaStart, I2C_EnviaByte
; o esta misma I2C_LeeByte, la línea SCL lleva en bajo al menos 5 µs.

I2C_LeeByte:
	movlw	0x08					; A recibir 8 bits.
	movwf	I2C_ContadorBits
	call	SDA_AltaImpedancia		; Deja libre la línea de datos.
I2C_LeeBit:
	call	SCL_AltaImpedancia		; Flanco de subida del reloj.
	bcf	STATUS, STATUS_C				; En principio supone que es "0".
	btfsc	i2c_port, sda_pin						; Lee el bit
	bsf	STATUS, STATUS_C				; Si es "1" carga 1 en el Carry.
	rlf	I2C_Dato,F				; Lo introduce en el registro.
	call	SCL_Bajo				; Termina el semiperiodo positivo del reloj.
	call	Retardo_4micros			; Tiempo tHD;DAT del protocolo.
	decfsz	I2C_ContadorBits		; Lazo para los 8 bits.
	goto	I2C_LeeBit
;
; Chequea si este es el último byte a leer para enviar o no el bit de reconocimiento
; ACK en consecuencia.
;
	btfss 	I2C_Flags,0		; Si es el último, no debe enviar
									; el bit de reconocimiento ACK.
	call	SDA_Bajo				; Envía el bit de reconocimiento ACK
									; porque todavía no es el último byte a leer.
	call	SCL_AltaImpedancia		; Pulso en alto del SCL para transmitir el
	call	Retardo_4micros			; bit ACK de reconocimiento. Este es tHIGH.
	call	SCL_Bajo				; Pulso de bajada del SCL.
	call	Retardo_4micros
	movf	I2C_Dato,0				; El resultado se manda en el registro de
	return							; de trabajo W.
	
	
	
	
	
	
	

;******************************** Librería DS3231.INC **************************
;
; Estas subrutinas permiten realizar las tareas de manejo del reloj-calendario DS3231
; Este dispositivo transmite la información vía serie a través de un bus I2C.
;
; ZONA DE DATOS ****************************************************************
PSECT udata_shr
Anho	    EQU 0x33	    ; Guarda el año.
Mes	    EQU 0x34	    ; Guarda el mes.
Dia	    EQU 0x35	    ; Guarda el día.
DiaSemana   EQU 0x36	    ; Guarda el día de la semana: lunes, etc.
Hora	    EQU 0x37	    ; Guarda las horas.
Minuto	    EQU 0x38	    ; Guarda los minutos.
Segundo	    EQU 0x39	    ; Guarda los segundos.

DS3231_DIR_ESCRITURA	EQU	0xD0	; Dirección del DS3231 para escribir.
DS3231_DIR_LECTURA	EQU	0xD1	; Dirección del DS3231 para leer.
 
; Subrutina "DS3231_Inicializa" ---------------------------------------------------------
;
; Configura la señal cuadrada que genera el DS3231 en su pin SQW/OUT a 1 Hz.
PSECT	code, delta=2
DS3231_Inicializa: ; TODO Migrar al DS3231
	call	I2C_EnviaStart				; Envía condición de Start.
	movlw	DS3231_DIR_ESCRITURA		; Indica al DS3231 que el byte a escribir,
	call	I2C_EnviaByte				; está en la posición 07h, que corresponde
	movlw	0x07						; al control de la señal cuadrada.
	call	I2C_EnviaByte
	movlw	00010000B					; Escribe en el registro de control para
	call	I2C_EnviaByte				; configurar onda cuadrada del DS3231 a 1 Hz.
	call	I2C_EnviaStop				; Termina de enviar datos.
	return


; Subrutina "DS3231_CargaInicial" -------------------------------------------------------
;
; Realiza una carga inicial en los registros internos del reloj-calendario DS3231 a fecha
; Lunes, 1 de Enero de 2004 a las 0:00:00.
;
DS3231_CargaInicial:
	movlw	1							; Inicializa todos los datos del reloj: Año, mes,
	movwf	Dia							; día, día de la semana, hora, minuto y segundo.
	movwf	Mes
	movwf	DiaSemana
	movlw	4
	movwf	Anho						; Inicializa en el año 2004.
	clrf	Hora
	clrf	Minuto
	clrf	Segundo						; Después lo graba en el DS3231 para
;	call	DS3231_Escribe				; ponerlo en marcha.
;	return
;
; Subrutina "DS3231_Escribe" ----------------------------------------------------------------
;
; Carga los datos de los registros Anho, Mes, etc., dentro del DS3231.

DS3231_Escribe:
	call	I2C_EnviaStart				; Envía condición de Start.
	movlw	DS3231_DIR_ESCRITURA		; Indica al DS3231 que el primer byte a escribir
	call	I2C_EnviaByte				; está en la posición 00h que corresponde
	movlw	0x00						; a los segundos.
	call	I2C_EnviaByte
	movf	Segundo,0					; Pasa los segundos de la memoria del PIC16F84A
	call	I2C_EnviaByte				; al DS3231.
	movf	Minuto,0					; Y se repite el proceso para el resto.
	call	I2C_EnviaByte
	movf	Hora,0
	call	I2C_EnviaByte
	movf	DiaSemana,0
	call	I2C_EnviaByte
	movf	Dia,0
	call	I2C_EnviaByte
	movf	Mes,0
	call	I2C_EnviaByte
	movf	Anho,0
	call	I2C_EnviaByte
	call	I2C_EnviaStop				; Termina de enviar datos.
	return


; Subrutina "DS3231_Read" ----------------------------------------------------------------
;
; Se leen las variables de tiempo del DS3231 y se guardan en los registros correspondientes.
;
DS3231_Read:
    call    I2C_EnviaStart		; Envía condición de Start.
    movlw   DS3231_DIR_ESCRITURA	; Indica al DS3231 que el primer byte
    call    I2C_EnviaByte				; a leer está en la posición 00H, que corresponde
    movlw   0x00						; a los segundos.
    call    I2C_EnviaByte
    call    I2C_EnviaStop				; Envía condición de Stop.
	
	
    bcf	    I2C_Flags, 0	    ; Inicio
    call    I2C_EnviaStart	    ; Envía condición de Start.
    movlw   DS3231_DIR_LECTURA
    call    I2C_EnviaByte
    call    I2C_LeeByte		    ; Lee los segundos.
    movwf   Segundo		    ; Lo carga en el registro correspondiente.
    call    I2C_LeeByte		    ; Lee el resto de los registros utilizando
    movwf   Minuto		    ; el mismo procedimiento.
    call    I2C_LeeByte
    movwf   Hora
    call    I2C_LeeByte
    movwf   DiaSemana
    call    I2C_LeeByte
    movwf   Dia
    call    I2C_LeeByte
    movwf   Mes
    bsf	    I2C_Flags,0		    ; Para terminar.
    call    I2C_LeeByte
    movwf   Anho
    call    I2C_EnviaStop	    ; Acaba de leer.
    return

    END


