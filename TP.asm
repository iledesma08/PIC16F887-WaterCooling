;**********************************************************************
;   This file is a basic code template for assembly code generation   *
;   on the PIC16F887. This file contains the basic code               *
;   building blocks to build upon.                                    *
;                                                                     *
;   Refer to the MPASM User's Guide for additional information on     *
;   features of the assembler (Document DS33014).                     *
;                                                                     *
;   Refer to the respective PIC data sheet for additional             *
;   information on the instruction set.                               *
;                                                                     *
;**********************************************************************
;                                                                     *
;    Filename:	    xxx.asm                                           *
;    Date:                                                            *
;    File Version:                                                    *
;                                                                     *
;    Author:                                                          *
;    Company:                                                         *
;                                                                     *
;                                                                     *
;**********************************************************************
;                                                                     *
;    Files Required: P16F887.INC                                      *
;                                                                     *
;**********************************************************************
;                                                                     *
;    Notes:                                                           *
;                                                                     *
;**********************************************************************


	list		p=16f887	; list directive to define processor
	#include	<p16f887.inc>	; processor specific variable definitions


; '__CONFIG' directive is used to embed configuration data within .asm file.
; The labels following the directive are located in the respective .inc file.
; See respective data sheet for additional information on configuration word.

	__CONFIG    _CONFIG1, _LVP_OFF & _FCMEN_ON & _IESO_OFF & _BOR_OFF & _CPD_OFF & _CP_OFF & _MCLRE_ON & _PWRTE_ON & _WDT_OFF & _INTRC_OSC_NOCLKOUT
	__CONFIG    _CONFIG2, _WRT_OFF & _BOR21V



;***** VARIABLE DEFINITIONS
w_temp		EQU	0x7D		; variable used for context saving
status_temp	EQU	0x7E		; variable used for context saving
pclath_temp	EQU	0x7F		; variable used for context saving
AUX		EQU	0x20		; variable usada para multiplexar los displays y el teclado
DIG1		EQU	0x21		; variable usada para almacenar el digito 1 del display
DIG2		EQU	0x22		; variable usada para almacenar el digito 2 del display
CONTSAMPLE	EQU	0x23		; variable usada para esperar el tiempo de conversion necesario antes de activar ADC
ADCRESULT	EQU	0x24		; variable usada para almacenar el resultado del ADC
ADCDIGIT	EQU	0x25		; variable usada para almacenar el resultado del ADC y definir el DIG2 del Display
TEMPREF		EQU	0x26		; variable usada para almacenar la temperatura de referencia


;**********************************************************************
	ORG     0x00		    ; processor reset vector
  	GOTO    MAIN		    ; go to beginning of program

	ORG     0x04		    ; interrupt vector location
	GOTO	ISR		    ; go to interrupt rutine

MAIN
	BANKSEL	    ADCON1
	MOVLW	    b'00000001'
	MOVWF	    TRISA	    ; Configura RA0 como Entrada, el resto como salida
	MOVLW	    b'00011110'	    ; Configura RB1 a RB4 como Entradas y RB5 a RB7 como Salidas
	MOVWF	    TRISB
	CLRF	    TRISC	    ; Configura RC6 como salida (para la transmision)
	CLRF	    TRISD	    ; Configura Puerto D como salidas (segmentos del display)
	MOVLW	    b'01100000'	    ; Habilita interrupcion por perifericos (p/Timer1) y por overflow de Timer0. Limpia bandera Timer0
	MOVWF	    INTCON
	MOVLW	    b'00000001'	    ; Habilita interrupcion por Timer1 (va a ser el que activa el ADC y cambia el contenido de los Displays)
	MOVWF	    PIE1            
	MOVLW	    b'00000101'	    ; Habilita Rpu y configura el prescaler de Timer0 en 64
	MOVWF	    OPTION_REG	    ; Habilita Rpu
	MOVLW	    b'00011110'	    ; Rpu de RB1 a RB4
	MOVWF	    WPUB
	MOVLW	    b'10000000'	    ; Justificado a la derecha (priorizo ADRESL), Vref- = GND, Vref+ = Vcc
	MOVWF	    ADCON1	    
	MOVLW	    b'00100100'	    ; Habilita la Transmision y configura Transmision de 8 bits habilitada a alta velocidad (BRGH=1)
	MOVWF	    TXSTA
	MOVLW	    .25		    ; Configuro un Baud rate de 9600 teniendo en cuenta que Fosc = 4MHz (tabla 12-5 del datasheet)
	MOVWF	    SPBRG
	BANKSEL	    ANSEL
	BSF	    ANSEL,0	    ; Configura RA0 como Analogica, el resto es Digital
	CLRF	    ANSELH
	BCF	    BAUDCTL,BRG16   ; Generador de Baudios de 8-bits
	BANKSEL	    ADCON0
	MOVLW	    b'11000001'	    ; El ADC usa FRC, selecciona Canal AN0, ADC habilitado
	MOVWF	    ADCON0
	MOVLW	    .100	    
	MOVWF	    TMR0	    ; Cargo Timer0 para que dure 10ms (para los displays y el teclado)
	CLRF	    TMR1L
	CLRF	    TMR1H
	MOVLW	    b'00110001'	    ; Habilita Timer1 y configura el prescaler
	MOVWF	    T1CON
	CLRF	    PIR1	    ; Limpia flag de Timer1
	BSF	    RCSTA,SPEN	    ; habilita RX y TX como puertos seriales
	BCF	    STATUS,RP0
	BCF	    STATUS,RP1
	MOVLW	    b'11111111'	    ; inicia el sistema con todo apagado
	MOVWF	    PORTA
	MOVLW	    b'11011011'	    ; carga un binario en el PORTB para multiplexar y tambien en la variable Aux
	MOVWF	    PORTB
	MOVWF	    AUX
	MOVLW	    .26		    ; carga 26 como temperatura de referencia por default
	MOVWF	    TEMPREF
	CLRF	    PORTD
	CLRW
	BCF	    STATUS,C
	BCF	    STATUS,Z
	BSF	    INTCON,GIE
	GOTO	    $
	
ISR
    MOVWF       w_temp		    ; Guarda contexto
    SWAPF       STATUS,w          
    MOVWF       status_temp       

    BTFSS       PIR1,TMR1IF	    ; Si es interrupción por TMR1F, se saltea una línea
    GOTO        MUX
    GOTO        SAMPLE
    
MUX
    BCF         STATUS,C
    BTFSS       AUX,0		    ; Salta si el primer bit de Aux es 1
    BSF         STATUS,C
    BTFSS       AUX,1		    ; Salta si el segundo bit de Aux es 1
    BSF         STATUS,C         
    RRF         AUX,F		    ; Rota Aux a la derecha
    MOVF	AUX,W
    MOVWF       PORTB		    ; Hace el multiplexado
    
    BTFSC       PORTB,6
    MOVF        DIG1,W		    ; Si el sexto bit de PORTB es 0, muestra el DIG2 en el display
    BTFSC       PORTB,5
    MOVF        DIG2,W		    ; Si el quinto bit de PORTB es 0, muestra el DIG1 en el display
    CALL        TABLA
    MOVWF       PORTD		    ; Muestra el número en el display correspondiente
    
    MOVLW       .100
    MOVWF       TMR0		    ; Carga Timer0 para que dure 10ms (para los displays y el teclado)
    BCF         INTCON,T0IF
    GOTO        EXIT_ISR

TABLA				    ; La tabla se hace teniendo en cuenta Display de Anodo Comun
    ADDWF       PCL,F
    RETLW       b'00111111'	    ; 0
    RETLW       b'00000110'	    ; 1
    RETLW       b'01011011'	    ; 2
    RETLW       b'01001111'	    ; 3
    RETLW       b'01100110'	    ; 4
    RETLW       b'01101101'	    ; 5
    RETLW       b'01111101'	    ; 6
    RETLW       b'00000111'	    ; 7
    RETLW       b'01111111'	    ; 8
    RETLW       b'01101111'	    ; 9    
    
SAMPLE
    CALL	SAMPLETIME	    ; Acquisition delay
    BSF		ADCON0,GO	    ; Inicia la conversion
    BTFSC	ADCON0,GO	    ; Testea si se hizo la conversion
    GOTO	$-1		    ; Si no se termino la conversion, vuelve a testear
    BANKSEL	ADRESL
    BCF		STATUS,C	    ; Setea carry en 0 para no tener problemas con al rotar
    RRF		ADRESL,W	    ; Rota a la derecha (porque el ADC tiene una resolucion de 5mV y el sensor un paso de 10mV entonces hay un bit de mas)
    BANKSEL	TMR1L
    MOVWF	ADCRESULT	    ; Guarda el resultado del ADC
    MOVWF	ADCDIGIT	    ; Guarda el resultado del ADC en una variable auxiliar
    CALL	COMSERIE	    ; Envia los datos por el puerto de comunicacion en serie RC6 (TX)
    CALL	DIGITOS		    ; Setea los digitos del display
    CALL	CHECK		    ; Verifica que hacer con perifericos
    CLRF        TMR1L		    ; Limpia el Timer1
    CLRF        TMR1H
    BCF         PIR1,TMR1IF
    GOTO        EXIT_ISR

SAMPLETIME			    ;DELAY DE 12 uS
    MOVLW	.4
    MOVWF	CONTSAMPLE
    DECFSZ	CONTSAMPLE,F
    GOTO $-1
    RETURN
    
DIGITOS
    CLRF	DIG1		    ; Limpio DIG1 para no tener problemas con iteraciones anteriores
    MOVLW	.10
    SUBWF	ADCDIGIT,F	    ; Se le resta 10 al resultado del ADC
    INCF	DIG1,F		    ; Se cuenta cuantas veces se le puede restar 10 al resultado
    BTFSC	STATUS,C	    ; Cuando la resta de un numero negativo (C=0), deja de restar. Cuando esto suceda, hay una resta de mas
    GOTO	$-3
    DECF	DIG1,F		    ; Reduzco en 1 la cantidad de restas por la resta adicional
    ADDWF	ADCDIGIT,W	    ; Agrego 10 por la resta adicional y al resultado lo paso a W porque es el digito 2
    MOVWF	DIG2		    ; El resto de la division pasa a ser el digito 2
    RETURN

CHECK
    MOVF	ADCRESULT,W	    ; Copia ADCRESULT en W para la resta
    SUBWF	TEMPREF,W	    ; TEMPREF-ADCRESULT
    BTFSC	STATUS,Z	    ; Salta si el resultado de la resta es distinto de 0
    GOTO	OFFALL
    BTFSS	STATUS,C	    ; Salta si el resultado de la resta es positivo (prender resistencia)
    GOTO	REFRIGERATE
    GOTO	HEAT		    ; Salta si el resultado de la resta es negativo (prende fan y bomba de agua)
    
OFFALL
    MOVLW	b'11111111'	    ; Apaga todo, resistencia, bomba y fan
    MOVWF	PORTA
    RETURN			    ; Vuelve a SAMPLE
    
REFRIGERATE
    MOVLW	b'00001000'	    ; Desactiva resistencia
    MOVWF	PORTA
    RETURN			    ; Vuelve a SAMPLE
    
HEAT
    MOVLW	b'00000110'	    ; Apaga bomba y fan y prende resistencia
    MOVWF	PORTA
    RETURN			    ; Vuelve a SAMPLE
    
COMSERIE
    MOVF	ADCRESULT,W	    ; Copia ADCRESULT en W
    MOVWF	TXREG		    ; Se escribe el resultado del ADC en TXREG
    NOP				    ; "Polling TXIF immediately following the TXREG write will return invalid results"
    BTFSS	PIR1,TXIF	    ; Sigue cuando TXREG se haya vaciado (no es necesario habilitar TXIE)
    GOTO	$-1
    RETURN
    
EXIT_ISR
    SWAPF       status_temp,w	    ; Recupera contexto
    MOVWF       STATUS            
    SWAPF       w_temp,f
    SWAPF       w_temp,w          
    RETFIE			    ; Vuelve de la interrupción

	END