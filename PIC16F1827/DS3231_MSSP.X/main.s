    ; Archivo: ds3231_i2c_pic_as_corrected_v2.asm
    ; PIC: PIC16F1827
    ; Comunicaci�n con el DS3231 para leer/escribir fecha y hora

    PROCESSOR 16F1827
    #include <xc.inc>

    ; Configuraci�n de los bits de configuraci�n
    CONFIG FOSC=INTOSC, WDTE=OFF, PWRTE=OFF, MCLRE=ON, CP=OFF, BOREN=ON, CLKOUTEN=OFF
    CONFIG WRT=OFF, PLLEN=OFF, STVREN=ON, BORV=LO, LVP=OFF

    ; Definir frecuencia de trabajo
    #define _XTAL_FREQ 4000000

    ; Definiciones para la comunicaci�n I2C
    #define DS3231_ADDRESS 0x68
    #define WRITE 0
    #define READ 1
    
    #define SMP 0x07

    ; ------------------------------
    ; Secci�n de datos: Usando PSECT para variables
    ; ------------------------------
    PSECT udata_shr,global,space=1
    second:   DS 1    ; Variable para segundos
    minute:   DS 1    ; Variable para minutos
    hour:     DS 1    ; Variable para horas
    day:      DS 1    ; Variable para d�a
    date:     DS 1    ; Variable para fecha
    month:    DS 1    ; Variable para mes
    year:     DS 1    ; Variable para a�o

    ; ------------------------------
    ; Secci�n de c�digo: Inicio de programa
    ; ------------------------------
    PSECT reset_vec,global,class=CODE,delta=2
    ORG 0x00
    GOTO Main           ; Ir a la rutina principal despu�s del reset

    ; ------------------------------
    ; Secci�n de c�digo: Vector de interrupciones
    ; ------------------------------
    PSECT int_vec,global,class=CODE,delta=2
    ORG 0x04
    GOTO ISR            ; Ir a la rutina de interrupci�n

    ; ------------------------------
    ; Secci�n de c�digo: Programa principal
    ; ------------------------------
    PSECT main_code,global,class=CODE,delta=2
Main:
    ; Configurar el reloj interno
    BANKSEL OSCCON
    MOVLW 0x60           ; Configurar frecuencia a 4MHz
    MOVWF OSCCON

    ; Configuraci�n de los pines I2C (RB1 y RB4)
    BANKSEL TRISB
    BSF TRISB, 1         ; RB1 (SDA) como entrada
    BSF TRISB, 4         ; RB4 (SCL) como entrada

    ; Inicializar I2C
    CALL I2C_Init

    ; Leer la hora actual desde el DS3231
    CALL Read_Time

    ; Aqu� puedes agregar m�s c�digo para manipular los datos le�dos

MainLoop:
    GOTO MainLoop        ; Bucle infinito

; ---------------------------
; Inicializar I2C
; ---------------------------
I2C_Init:
    BANKSEL SSP1CON1
    MOVLW 0x28           ; Configurar I2C en modo maestro
    MOVWF SSP1CON1
    BANKSEL SSP1ADD
    MOVLW 9              ; Configurar velocidad del reloj para I2C (100kHz con Fosc = 4MHz)
    MOVWF SSP1ADD
    BANKSEL SSP1STAT
    BCF SSP1STAT, SMP     ; Configurar muestreo
    RETURN

; ---------------------------
; Iniciar la comunicaci�n I2C
; ---------------------------
I2C_Start:
    BANKSEL SSP1CON1
    BSF SSP1CON1, SEN     ; Iniciar condici�n de START
    BTFSC SSP1CON1, SEN   ; Esperar a que se complete el START
    GOTO $-1
    RETURN

; ---------------------------
; Detener la comunicaci�n I2C
; ---------------------------
I2C_Stop:
    BANKSEL SSP1CON1
    BSF SSP1CON1, PEN     ; Iniciar condici�n de STOP
    BTFSC SSP1CON1, PEN   ; Esperar a que se complete el STOP
    GOTO $-1
    RETURN

; ---------------------------
; Enviar un byte a trav�s de I2C
; ---------------------------
I2C_Write:
    BANKSEL SSP1BUF
    MOVWF SSP1BUF         ; Enviar el byte almacenado en W
    BTFSC SSP1CON1, ACKSTAT ; Esperar a que el esclavo responda con ACK
    GOTO $-1
    RETURN

; ---------------------------
; Leer un byte desde el I2C
; ---------------------------
I2C_Read:
    BANKSEL SSP1CON1
    BSF SSP1CON1, RCEN    ; Habilitar la recepci�n
    BTFSC SSP1CON1, RCEN  ; Esperar a que se complete la recepci�n
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
    MOVLW 0x00           ; Direcci�n del registro de segundos
    CALL I2C_Write
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
    CALL I2C_Read
    MOVWF hour
    ; Leer d�a
    CALL I2C_Read
    MOVWF day
    ; Leer fecha
    CALL I2C_Read
    MOVWF date
    ; Leer mes
    CALL I2C_Read
    MOVWF month
    ; Leer a�o
    CALL I2C_Read
    MOVWF year

    CALL I2C_Stop
    RETURN

; ---------------------------
; Rutina de interrupciones
; ---------------------------
ISR:
    RETFIE

    END
