    ; Archivo: ds3231_i2c_pic_as_corrected_v2.asm
    ; PIC: PIC16F1827
    ; Comunicación con el DS3231 para leer/escribir fecha y hora

    PROCESSOR 16F1827
    
    ; Assembly source line config statements

; CONFIG1
  CONFIG  FOSC = INTOSC         ; Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
  CONFIG  WDTE = OFF            ; Watchdog Timer Enable (WDT disabled)
  CONFIG  PWRTE = ON           ; Power-up Timer Enable (PWRT disabled)
  CONFIG  MCLRE = OFF           ; MCLR Pin Function Select (MCLR/VPP pin function is digital input)
  CONFIG  CP = OFF              ; Flash Program Memory Code Protection (Program memory code protection is disabled)
  CONFIG  CPD = OFF             ; Data Memory Code Protection (Data memory code protection is disabled)
  CONFIG  BOREN = OFF           ; Brown-out Reset Enable (Brown-out Reset disabled)
  CONFIG  CLKOUTEN = OFF        ; Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
  CONFIG  IESO = OFF            ; Internal/External Switchover (Internal/External Switchover mode is disabled)
  CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

; CONFIG2
  CONFIG  WRT = OFF             ; Flash Memory Self-Write Protection (Write protection off)
  CONFIG  PLLEN = OFF            ; PLL Enable (4x PLL enabled)
  CONFIG  STVREN = OFF          ; Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will not cause a Reset)
  CONFIG  BORV = LO             ; Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
  CONFIG  DEBUG = OFF           ; In-Circuit Debugger Mode (In-Circuit Debugger disabled, ICSPCLK and ICSPDAT are general purpose I/O pins)
  CONFIG  LVP = OFF             ; Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

    #include <xc.inc>

    ; Definir frecuencia de trabajo
    #define _XTAL_FREQ 4000000

    ; Definiciones para la comunicación I2C
    #define DS3231_ADDRESS 0x68
    #define WRITE 0
    #define READ 1
    
    #define SMP 0x07
    #define CKE 0x06
    
    #define SEN 0x00
    #define PEN 0x02
    #define ACKSTAT 0x06
    #define RCEN 0x03
    
    #define SSPEN 0x05

    #define SSP1IE 0x03
    #define BCL1IE 0x03
    #define GIE 0x07
    #define PEIE 0x06

    #define BCL1IF  0x02
    #define SSP1IF  0x03
    
    ; ------------------------------
    ; Sección de datos: Usando PSECT para variables
    ; ------------------------------
    PSECT udata_shr,global,space=1
    second:   DS 1    ; Variable para segundos
    minute:   DS 1    ; Variable para minutos
    hour:     DS 1    ; Variable para horas
    day:      DS 1    ; Variable para día
    date:     DS 1    ; Variable para fecha
    month:    DS 1    ; Variable para mes
    year:     DS 1    ; Variable para año
    sleep1:   DS 1    ; Variable para sleep1
    sleep2:   DS 1    ; Variable para sleep2
    sleep3:   DS 1    ; Variable para sleep3
    counts:   DS 1    ; 

    ; ------------------------------
    ; Sección de código: Inicio de programa
    ; ------------------------------
    PSECT reset_vec,global,class=CODE,delta=2
    ORG 0x00
    GOTO Main           ; Ir a la rutina principal después del reset

    ; ------------------------------
    ; Sección de código: Vector de interrupciones
    ; ------------------------------
    ;PSECT int_vec,global,class=CODE,delta=2
    ;ORG 0x04
    ;GOTO ISR            ; Ir a la rutina de interrupción
    PSECT	code, delta=2, abs
    ORG	    0x04
    
i2c1_error_isr:
    BANKSEL PIR2
    BTFSS   PIR2, BCL1IF
    goto    i2c1_isr    
    bcf	    PIR2, BCL1IF
    ;BANKSEL PORTA
    ;BSF	    PORTA, 3
    
i2c1_isr:
    BANKSEL PIR1
    BTFSS   PIR1, SSP1IF
    goto    continuar

    bcf	    PIR1, SSP1IF
    INCF    counts, 1
    movf    counts, 0
    andlw   00000111B
    BANKSEL PORTA
    iorwf   PORTA, 1
    

    
continuar:
    retfie

    ; ------------------------------
    ; Sección de código: Programa principal
    ; ------------------------------
    PSECT main_code,global,class=CODE,delta=2
Main:
    ; Configurar el reloj interno
    BANKSEL OSCCON
    MOVLW   0x68           ; Configurar frecuencia a 4MHz
    ;MOVLW   0x60           ; Configurar frecuencia a 4MHz
    MOVWF   OSCCON

    BANKSEL OPTION_REG
    bcf	    OPTION_REG, 7 ; Weak pull-ups are enabled by individual WPUx latch values
    
    
    
    ; Configuración de los pines I2C (RB1 y RB4)
    ;BANKSEL TRISB
    ;BSF	    TRISB, 1         ; RB1 (SDA) como entrada
    ;BSF	    TRISB, 4         ; RB4 (SCL) como entrada
    banksel	LATB
    clrf	LATB
    banksel	ANSELB
    clrf	ANSELB
    banksel	WPUB
    movlw	00010010B ; 0 = Disabled pull-up, 1 = Enabled pull-up
    movwf	WPUB
    banksel	TRISB
    movlw	00010010B ; 1 = inputs, 0 = outpus
    movwf	TRISB
    
    
    banksel	LATA
    clrf	LATA
    banksel	ANSELA
    clrf	ANSELA
    
    BANKSEL TRISA
    movlw   00100000B ; 1 = inputs, 0 = outpus
    movwf   TRISA

    BANKSEL PORTA
    clrf    PORTA
    ;bsf	    PORTA, 3
    
    ; Inicializar I2C
    CALL I2C_Init

    BANKSEL INTCON
    bsf	    INTCON, PEIE
    bsf	    INTCON, GIE
    

MainLoop:
    BANKSEL PORTA
    BTFSC   PORTA, 7
    goto    poner_0
    goto    poner_1
    
poner_0:
    BCF	    PORTA, 7
    goto    leer_rtc
    
poner_1:
    BSF	    PORTA, 7
    goto    leer_rtc
    
leer_rtc:
    CALL    sleep_1_second
    BANKSEL PORTA
    bcf	    PORTA, 6
    bcf	    PORTA, 4
    bcf	    PORTA, 3
    bcf	    PORTA, 2
    bcf	    PORTA, 1
    bcf	    PORTA, 0
    movlw   0x00
    movwf   counts
    CALL    sleep_1_second
    
    
    ;BANKSEL PORTA
    //movlw   0b01000000
    //xorwf   PORTA, 1
    ;bsf	    PORTA, 6
    
    ; Leer la hora actual desde el DS3231
    CALL Read_Time
    
    ;BANKSEL PORTA
    //movlw   0b00100000
    //xorwf   PORTA, 1
    ;bsf	    PORTA, 4
    

    ; Aquí puedes agregar más código para manipular los datos leídos
    ;movf    second, 0
    ;andlw   0x03
    ;BANKSEL PORTA
    ;iorwf   PORTA, 1	    ; Muestro los segundos en el Puerto A
    
    ;CALL    sleep_1_second
    
    GOTO MainLoop        ; Bucle infinito

    
; SLEEP 1 Second
sleep_1_second:
    movlb	0	; Bank 0
    movlw	0x0C
    movwf	sleep3

sleep_1_second_three:
    movlw	0xFC
    movwf	sleep2
sleep_1_second_two:
    movlw	0xFA
    movwf	sleep1
sleep_1_second_one:
    decfsz	sleep1
    goto	sleep_1_second_one

    decfsz	sleep2
    goto	sleep_1_second_two

    decfsz	sleep3
    goto	sleep_1_second_three

    retlw	0x00
    
    
; ---------------------------
; Inicializar I2C
; ---------------------------
I2C_Init:
    BANKSEL SSP1STAT
    ;BSF SSP1STAT, SMP     ; Configurar muestreo
    ;BCF	SSP1STAT, CKE
    ;CKE disabled; SMP Standard Speed;
    movlw   0x80
    movwf   SSP1STAT

    BANKSEL SSP1CON1
    ;MOVLW 0x28           ; Configurar I2C en modo maestro
    ;SSPM FOSC/4_SSPxADD_I2C; CKP disabled; SSPEN disabled; SSPOV no_overflow; WCOL no_collision;
    movlw 0x08
    MOVWF SSP1CON1
    
    BANKSEL SSP1CON2
    ;SEN disabled; RSEN disabled; PEN disabled; RCEN disabled; ACKEN disabled; ACKDT acknowledge; GCEN disabled;
    MOVLW 0x00
    MOVWF SSP1CON2
    
    BANKSEL SSP1CON3
    ;MOVLW 0x38
    ;DHEN disabled; AHEN disabled; SBCDE disabled; SDAHT 100ns; BOEN disabled; SCIE disabled; PCIE disabled;
    MOVLW 0x00
    MOVWF SSP1CON3
    
    BANKSEL SSP1ADD
    MOVLW 0x09              ; Configurar velocidad del reloj para I2C (100kHz con Fosc = 4MHz)
    MOVWF SSP1ADD
    
    BANKSEL PIE1
    bsf	    PIE1, SSP1IE
    BANKSEL PIE2
    bsf	    PIE2, BCL1IE
    
    
    BANKSEL SSP1CON1
    BSF SSP1CON1, SSPEN
    
    RETURN

; ---------------------------
; Iniciar la comunicación I2C
; ---------------------------
I2C_Start:
    BANKSEL SSP1CON2
    BSF SSP1CON2, SEN     ; Iniciar condición de START
    BTFSC SSP1CON2, SEN   ; Esperar a que se complete el START
    GOTO $-1
    RETURN

; ---------------------------
; Detener la comunicación I2C
; ---------------------------
I2C_Stop:
    BANKSEL SSP1CON2
    BSF SSP1CON2, PEN     ; Iniciar condición de STOP
    BTFSC SSP1CON2, PEN   ; Esperar a que se complete el STOP
    GOTO $-1
    RETURN

; ---------------------------
; Enviar un byte a través de I2C
; ---------------------------
I2C_Write:
    BANKSEL SSP1BUF
    MOVWF SSP1BUF         ; Enviar el byte almacenado en W
    BANKSEL SSP1CON2
    BTFSC SSP1CON2, ACKSTAT ; Esperar a que el esclavo responda con ACK
    GOTO $-1
    RETURN

; ---------------------------
; Leer un byte desde el I2C
; ---------------------------
I2C_Read:
    BANKSEL SSP1CON2
    BSF SSP1CON2, RCEN    ; Habilitar la recepción
    BTFSC SSP1CON2, RCEN  ; Esperar a que se complete la recepción
    GOTO $-1
    BANKSEL SSP1BUF
    MOVF SSP1BUF, W       ; Mover el byte recibido a W
    RETURN

; ---------------------------
; Leer la fecha y hora del DS3231
; ---------------------------
Read_Time:
    CALL I2C_Start
    MOVLW DS3231_ADDRESS << 1 | WRITE
    CALL I2C_Write
    MOVLW 0x00           ; Dirección del registro de segundos
    CALL I2C_Write
    ;CALL I2C_Stop
    CALL I2C_Start
    MOVLW DS3231_ADDRESS << 1 | READ
    CALL I2C_Write
    ; Leer segundos
    CALL I2C_Read
    MOVWF second
    ; Leer minutos
    CALL I2C_Read
    MOVWF minute
    ; Leer horas
    ;CALL I2C_Read
    ;MOVWF hour
    ; Leer día
    ;CALL I2C_Read
    ;MOVWF day
    ; Leer fecha
    ;CALL I2C_Read
    ;MOVWF date
    ; Leer mes
    ;CALL I2C_Read
    ;MOVWF month
    ; Leer año
    ;CALL I2C_Read
    ;MOVWF year
    
    BANKSEL PORTA
    bsf	    PORTA, 6
    CALL I2C_Stop
    BANKSEL PORTA
    bsf	    PORTA, 4
    RETURN

; ---------------------------
; Rutina de interrupciones
; ---------------------------
;ISR:
;    BANKSEL PORTA
;    BSF	    PORTA, 2
;    RETFIE

    END
