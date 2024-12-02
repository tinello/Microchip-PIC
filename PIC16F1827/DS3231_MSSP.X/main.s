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
    #define IOCIE 0x03
    #define INTF 0x01
    #define INTEDG 0x06
    

    #define BCL1IF  0x02
    #define SSP1IF  0x03
    
    #define HFIOFR 0x04
    #define HFIOFS 0x00
    
    #define WCOL 0x07
    #define SSP1OV 0x06
    
    #define BF 0x00
    #define R_W 0x02
    #define Z 0x02
    
    #define ACKDT 0x05
    #define ACKEN 0x04
    
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
    flags:    DS 1    ; 

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
    banksel PORTB
    bsf	    PORTB, 0
    
i2c1_isr:
    BANKSEL PIR1
    BTFSS   PIR1, SSP1IF
    goto    continuar
    bcf	    PIR1, SSP1IF
    banksel OPTION_REG
    bcf	    OPTION_REG, 1 ;INTF
    bcf	    flags, 0
    banksel PORTB
    bsf	    PORTB, 2
    

    
continuar:
    retfie

    ; ------------------------------
    ; Sección de código: Programa principal
    ; ------------------------------
    PSECT main_code,global,class=CODE,delta=2
Main:
    ; Configurar el reloj interno
    BANKSEL OSCCON
    MOVLW   0x6A           ; Configurar frecuencia a 4MHz MCC
    ;MOVLW   0xEA           ; Configurar frecuencia a 4MHz IA
    MOVWF   OSCCON

    banksel OSCSTAT
    clrf    OSCSTAT
    ;btfss OSCSTAT, HFIOFR ; Int. osc. running?
    ;goto $-1 ; No, loop back
    ;btfss OSCSTAT, HFIOFS ; Osc. stable?
    ;goto $-1 ; No, loop back.
    
    banksel OSCTUNE
    clrf    OSCTUNE
    
    banksel BORCON
    clrf    BORCON
    
    
    
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
    
    BANKSEL IOCBP
    clrf    IOCBP
    
    BANKSEL IOCBN
    clrf    IOCBN
    
    BANKSEL IOCBF
    clrf    IOCBF
    
    
    ; Inicializar I2C
    CALL I2C_Init

    
    banksel SSP1MSK
    movlw   0xFE
    movwf   SSP1MSK
    
    BANKSEL INTCON
    bsf	    INTCON, IOCIE
    bcf	    INTCON, INTF ;Clears the Interrupt flag for the external interrupt, INT.
    bsf	    INTCON, GIE
    bsf	    INTCON, PEIE
    
    BANKSEL OPTION_REG
    bsf	    OPTION_REG, INTEDG ;Sets the edge detect of the external interrupt to positive edge. This way, the Interrupt flag will be set when the external interrupt pin level transitions from low to high.
    
    BANKSEL PIR1
    bcf	    PIR1, SSP1IF
    
    bcf	    flags, 0

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
    
    ; Leer la hora actual desde el DS3231
    CALL Read_Time
    
    ; Aquí puedes agregar más código para manipular los datos leídos
    movf    second, 0
    andlw   0x0F
    BANKSEL PORTA
    iorwf   PORTA, 1	    ; Muestro los segundos en el Puerto A
    BSF	    PORTA, 6
    
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
    ;MOVLW 0x28           ; Configurar I2C en modo maestro IA
    ;SSPM FOSC/4_SSPxADD_I2C; CKP disabled; SSPEN disabled; SSPOV no_overflow; WCOL no_collision;
    movlw 0x08	; MCC
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
    BANKSEL SSP1CON1
    bcf	    SSP1CON1, WCOL
    bcf	    SSP1CON1, SSP1OV
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
    banksel SSP1STAT
    BTFSS SSP1STAT, BF
    GOTO $-1
    BANKSEL SSP1BUF
    MOVF SSP1BUF, 0       ; Mover el byte recibido a W
    
    ; Manejo del ACK/NACK
    btfss   STATUS, Z               ; Verificar ACK (W = 0)
    bcf     SSP1CON2, ACKDT         ; ACK (ACKDT = 0)
    btfsc   STATUS, Z
    bsf     SSP1CON2, ACKDT         ; NACK (ACKDT = 1)
    bsf     SSP1CON2, ACKEN         ; Iniciar secuencia ACK/NACK
    btfsc   SSP1CON2, ACKEN         ; Esperar a que se complete
    goto    $-1
    
    RETURN

;////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

i2c_idle:
   banksel  SSP1STAT
   btfsc    SSP1STAT,R_W
   goto	    $-1

i2c_idle_loop:
   banksel  SSP1CON2
   movf	    SSP1CON2, 0
   andlw    0x1F
   
   banksel  STATUS
   btfss    STATUS, Z
   goto	    i2c_idle_loop
   return
 
; ---------------------------
; Leer la fecha y hora del DS3231
; ---------------------------
Read_Time:
    bsf	    flags, 0
    CALL I2C_Start
    BTFSC   flags, 0
    GOTO $-1
    
    bsf	    flags, 0
    MOVLW DS3231_ADDRESS << 1 | WRITE
    CALL I2C_Write
    BTFSC   flags, 0
    GOTO $-1
    
    bsf	    flags, 0
    MOVLW 0x00           ; Dirección del registro de segundos
    CALL I2C_Write
    BTFSC   flags, 0
    GOTO $-1
    
    
    bsf	    flags, 0
    CALL I2C_Start
    BTFSC   flags, 0
    GOTO $-1
    
    bsf	    flags, 0
    MOVLW DS3231_ADDRESS << 1 | READ
    CALL I2C_Write
    BTFSC   flags, 0
    GOTO $-1
    
    ; Leer segundos
    bsf	    flags, 0
    CALL I2C_Read
    MOVWF second
    BTFSC   flags, 0
    GOTO $-1
    ; Leer minutos
    ;CALL I2C_Read
    ;MOVWF minute
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
    
    CALL I2C_Stop
    
    
    RETURN

; ---------------------------
; Rutina de interrupciones
; ---------------------------
;ISR:
;    BANKSEL PORTA
;    BSF	    PORTA, 2
;    RETFIE

    END
