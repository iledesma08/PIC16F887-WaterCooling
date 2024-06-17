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
w_temp		EQU	0x7D		; Para guardar contexto
status_temp	EQU	0x7E		
	
AUX		EQU	0x20		; Para multiplexar los displays y el teclado
DIG1		EQU	0x21		; Para almacenar el digito 1 del display
DIG2		EQU	0x22		; Para almacenar el digito 2 del display
CONT_TEMP	EQU	0x23		; Para esperar el tiempo de conversion necesario antes de activar ADC
ADCRESULT	EQU	0x24		; Para almacenar el resultado del ADC
ADCDIGIT	EQU	0x25		; Para almacenar el resultado del ADC y definir el DIG2 del Display
TEMPREF		EQU	0x26		; Para almacenar la temperatura de referencia
TEMPTECLADO	EQU	0x27		; Para almacenar lo que se va ingresando por teclado
KEY_CHECK	EQU	0x28		; Para chequear si sigue presionada la misma tecla
PAD_MUL		EQU	0x29		; Para obtener el MSD del numero ingresado por teclado
PAD_DIG1	EQU	0x30		; Se utiliza auxiliarmente para obtener el MSD del numero ingresado por teclado
AUX_ADD		EQU	0x31		; Se utiliza para guardar temporalmente al digito ingresado por teclado
AUX_SWAP	EQU	0x32		; Para facilitar el swap que se hace para establecer TEMPREF
AUX_COMSERIE	EQU	0x33		; Para hacer una comunicacion inicial 

;**********************************************************************
	ORG     0x00		    ; processor reset vector
  	GOTO    MAIN		    ; go to beginning of program

	ORG     0x04		    ; interrupt vector location
	GOTO	ISR		    ; go to interrupt rutine

MAIN
	BANKSEL	    TRISA
	MOVLW	    b'00000001'
	MOVWF	    TRISA	    ; Configura RA0 como Entrada (sensor), el resto como salida (perifericos)
	MOVLW	    b'00011111'	    ; Configura RB0 a RB4 como Entradas (teclado) y RB5 a RB7 como Salidas (multiplexado)
	MOVWF	    TRISB
	BCF	    TRISC,6	    ; Configura RC6 como salida (para la transmision)
	CLRF	    TRISD	    ; Configura Puerto D como salidas (segmentos del display)
	MOVLW	    b'01101000'	    ; Habilita interrupcion por perifericos (p/Timer1), por overflow de Timer0 y por cambios en PORTB. Limpia T0IF y RBIF
	MOVWF	    INTCON
	MOVLW	    b'00000001'	    ; Habilita interrupcion por Timer1 (va a ser el que activa el ADC y cambia el contenido de los Displays)
	MOVWF	    PIE1            
	MOVLW	    b'10000101'	    ; Habilita Rpu y configura el prescaler de Timer0 en 64
	MOVWF	    OPTION_REG	    ; Habilita Rpu
	MOVLW	    b'00011011'	    
	MOVWF	    IOCB	    ; Interrupcion por cambios en RB0 a RB4 (sin RB2 porque causa interrupcion por alguna razon)
	; Se ponen manualmente resistencias de pull-down en RB0 a RB4 sin RB2
	MOVLW	    b'10000000'	    ; Justificado a la derecha (priorizo ADRESL), Vref- = GND, Vref+ = Vcc
	MOVWF	    ADCON1	    
	MOVLW	    b'00100100'	    ; Habilita la Transmision y configura Transmision de 8 bits habilitada a alta velocidad (BRGH=1)
	MOVWF	    TXSTA
	MOVLW	    .25		    ; Configuro un Baud rate de 9600 teniendo en cuenta que Fosc = 4MHz (tabla 12-5 del datasheet)
	MOVWF	    SPBRG
	BANKSEL	    ANSEL
	BSF	    ANSEL,0	    ; Configura RA0 como Analogica (sensor), el resto es Digital
	CLRF	    ANSELH
	BCF	    BAUDCTL,BRG16   ; Generador de Baudios de 8-bits
	BANKSEL	    ADCON0
	MOVLW	    b'11000001'	    ; El ADC usa FRC, selecciona Canal AN0, ADC habilitado
	MOVWF	    ADCON0
	MOVLW	    .152	    
	MOVWF	    TMR0	    ; Cargo Timer0 para que dure 6,67ms (para los displays y el teclado)
	CLRF	    TMR1L	    ; Limpio Timer1 para que dure el mayor tiempo posible
	CLRF	    TMR1H
	MOVLW	    b'00110001'	    ; Habilita Timer1 y configura el prescaler
	MOVWF	    T1CON
	CLRF	    PIR1	    ; Limpia flag de Timer1
	BSF	    RCSTA,SPEN	    ; habilita RX y TX como puertos seriales
	BCF	    STATUS,RP0
	BCF	    STATUS,RP1
	MOVLW	    b'11111111'	    ; inicia el sistema con todo apagado
	MOVWF	    PORTA
	MOVLW	    b'00100100'	    ; carga un binario en el PORTB para multiplexar y tambien en la variable Aux
	MOVWF	    PORTB
	MOVWF	    AUX
	MOVLW	    .26		    ; carga 26 como temperatura de referencia por default
	MOVWF	    TEMPREF
	; Muesta HI al iniciar el sistema
	MOVLW	    .10		    ; H
	MOVWF	    DIG1
	MOVLW	    .1		    ; 1
	MOVWF	    DIG2
	CLRW
	BCF	    STATUS,C
	BCF	    STATUS,Z
	BSF	    INTCON,GIE
	GOTO	    $
	
ISR
    MOVWF       w_temp		    ; Guarda contexto
    SWAPF       STATUS,w          
    MOVWF       status_temp       

    BTFSC       INTCON,T0IF	    ; Si no es interrupcion por Timer0, salta
    GOTO        MUX
    BTFSC       PIR1,TMR1IF	    ; Si no es interrupcion por Timer1, salta
    GOTO	SAMPLE
    BTFSC       INTCON,RBIE	    ; Si no es interrupcion por cambios en PortB, salta
    GOTO        KEYPAD		    
    GOTO	EXIT_ISR	    ; Si es cualquier otro tipo de interrupcion por alguna razon, pasa directamente al final
    
MUX
    BSF         STATUS,C
    BTFSC       AUX,0		    ; Si el primer o segundo bit de AUX son 1, pone 0 en STATUS,C
    BCF         STATUS,C
    BTFSC       AUX,1		    
    BCF         STATUS,C         
    RRF         AUX,F		    ; Rota Aux a la derecha
    MOVF	AUX,W		    ; Copia Aux en W
    MOVWF       PORTB		    ; Hace el multiplexado
    
    CLRW
    BTFSC       PORTB,7
    MOVF        DIG1,W		    ; Si el septimo bit de PORTB es 1, muestra el DIG1 en el display (MSD)
    BTFSC       PORTB,6
    MOVF        DIG2,W		    ; Si el sexto bit de PORTB es 1, muestra el DIG2 en el display (LSD)
    CALL        TABLADISPLAY
    MOVWF       PORTD		    ; Activa los segmentos del display para mostrar el numero de digito que corresponda
    
    BTFSC	KEY_CHECK,2
    DECF	KEY_CHECK,F	    ; Si KEY_CHECK es mayor a 011, le resta 1 (solamente le puede restar 1 tres veces)
    
    MOVLW       .152
    MOVWF       TMR0		    ; Carga Timer0 para que dure 6,67ms (para los displays y el teclado)
    BCF         INTCON,T0IF
    GOTO        EXIT_ISR
TABLADISPLAY			    ; La tabla se hace teniendo en cuenta Display de CATODO Comun
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
    RETLW	b'01110110'	    ; H
    RETLW	b'01111001'	    ; E
    RETLW	b'01010000'	    ; r
    RETLW	b'00000000'	    ; Nada
    
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
    BTFSS	AUX_COMSERIE,0	    ; Hace un envio inicial en serie de TEMPREF y despues no lo hace mas, solamente al cambiar el valor
    CALL	COMSERIE	    ; Envia los datos por el puerto de comunicacion en serie RC6 (TX)
    CALL	DIGITOS		    ; Setea los digitos del display
    CALL	CHECK		    ; Verifica que hacer con perifericos
    CLRF        TMR1L		    ; Limpia el Timer1
    CLRF        TMR1H
    BCF         PIR1,TMR1IF
    GOTO        EXIT_ISR
SAMPLETIME			    ;DELAY DE 12 uS
    MOVLW	.4
    MOVWF	CONT_TEMP
    DECFSZ	CONT_TEMP,F
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
    BTFSC	STATUS,Z	    
    GOTO	OFFALL		    ; Si el resultado de la resta es 0, apaga todo
    BTFSS	STATUS,C	    
    GOTO	REFRIGERATE	    ; Si el resultado de la resta es positivo (prender calentador)
    GOTO	HEAT		    ; Si el resultado de la resta es negativo (prende fan y bomba de agua)
OFFALL
    MOVLW	b'11111111'	    ; Apaga todo, calentador, bomba y fan
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
    
KEYPAD
    BTFSS	KEY_CHECK,2	    ; Solamente se procesa una interrupcion cuando hayan pasado mas de 3 MUX (un recorrido entero del teclado) sin tener un boton apretado
    GOTO	PRESIONAR
END_KEYPAD
    MOVLW	b'00000111'	    ; 111-011(3D)=100 (en este caso no se procesa la interrupcion) 
    MOVWF	KEY_CHECK
    CALL	REBOTE
    BCF         INTCON,RBIF
    GOTO	EXIT_ISR
REBOTE				    ;DELAY DE 1,5 mS
    MOVLW	b'11111111'
    MOVWF	CONT_TEMP
    DECFSZ	CONT_TEMP,F
    GOTO $-1
    MOVLW	b'11111111'
    MOVWF	CONT_TEMP
    DECFSZ	CONT_TEMP,F
    GOTO $-1
    RETURN
PRESIONAR
    BTFSC	PORTB,RB4	    ; Identifica la tecla apretada
    GOTO	TABLA_A
    BTFSC	PORTB,RB3
    GOTO	TABLA_B
    BTFSC	PORTB,RB1
    GOTO	TABLA_C
    BTFSC	PORTB,RB0
    GOTO	TABLA_D		    ; Si llega hasta aca, se esta apretando algo en la fila de abajo
TABLA_A
    BTFSC	AUX,5
    MOVLW	.1
    BTFSC	AUX,6
    MOVLW	.2
    BTFSC	AUX,7
    MOVLW	.3
    GOTO	DIGITAR		    
TABLA_B
    BTFSC	AUX,5
    MOVLW	.4
    BTFSC	AUX,6
    MOVLW	.5
    BTFSC	AUX,7
    MOVLW	.6
    GOTO	DIGITAR		    
TABLA_C
    BTFSC	AUX,5
    MOVLW	.7
    BTFSC	AUX,6
    MOVLW	.8
    BTFSC	AUX,7
    MOVLW	.9		    ; b'1001'
    GOTO	DIGITAR		    
TABLA_D
    BTFSC	AUX,5
    GOTO	LIMPIAR		    ; Si se presiono *, limpia el valor introducido por teclado
    BTFSC	AUX,6
    MOVLW	.0		    ; Si llega hasta aca, es que hay que agregar un digito a TEMPTECLADO
    BTFSC	AUX,7
    GOTO	ESTABLECER	    ; Si se presiono #, configura el valor introducido por teclado
    GOTO	DIGITAR		    
LIMPIAR
    CLRF	TEMPTECLADO
    MOVLW	.11		    ; Muestra "Er" de Erased en el display
    MOVWF	DIG1
    MOVLW	.12
    MOVWF	DIG2
    GOTO	END_KEYPAD
ESTABLECER
    MOVF	TEMPTECLADO,W	    ; Copia el binario que representa lo ingresado por teclado en W
    ANDLW	b'11110000'	    ; Separo el MSD
    BTFSC	STATUS,Z
    GOTO	ESTABLECER_CONT
    MOVWF	AUX_SWAP
    SWAPF	AUX_SWAP,W	    ; Hago swap de los nibbles para multiplicar x10 al MSD
    MOVWF	DIG1
    MOVWF	PAD_MUL		    ; Copio el MSD en PAD_MUL
    CALL	MULTIPLICAR
ESTABLECER_CONT
    MOVLW	b'00001111'
    ANDWF	TEMPTECLADO,F	    ; Borro al MSD (dejo el LSD)
    MOVF	TEMPTECLADO,W
    MOVWF	DIG2
    MOVF	PAD_DIG1,W	    ; Copio el MSD obtenido en W
    ADDWF	TEMPTECLADO,W	    ; PAD_DIG1(MSD)+TEMPTECLADO(LSD)
    MOVWF	TEMPREF		    ; Copia el resultado en TEMPREF
    CLRF	TEMPTECLADO	    ; Borra lo que se ingreso por teclado y espera por un nuevo valor
    CLRF	PAD_DIG1	    ; Borra el valor del MSD obtenido por multiplicacion
    CALL	COMSERIE	    ; Envia el nuevo valor de TEMPREF por comunicacion en serie
    GOTO	END_KEYPAD
COMSERIE
    BTFSS	AUX_COMSERIE,0	    ; Se setea la flag del envio inicial
    INCF	AUX_COMSERIE,F
    MOVF	TEMPREF,W	    ; Copia TEMPREF en W
    MOVWF	TXREG		    ; Se escribe la TEMPREF en TXREG
    NOP				    ; "Polling TXIF immediately following the TXREG write will return invalid results"
    BTFSS	PIR1,TXIF	    ; Sigue cuando TXREG se haya vaciado (no es necesario habilitar TXIE)
    GOTO	$-1
    RETURN
MULTIPLICAR
    MOVLW	.10
    ADDWF	PAD_DIG1,F	    ; Agrega 10 PAD_MUL cantidades de veces
    DECFSZ	PAD_MUL,F
    GOTO	$-2
    RETURN
DIGITAR
    MOVWF	AUX_ADD		    ; Almacena temporalmente el numero presionado (que sera el nuevo LSD)
    MOVWF	DIG2		    ; Muestra el numero presionado en el display
    MOVF	TEMPTECLADO,W
    ANDLW	b'00001111'
    MOVWF	DIG1
    SWAPF	TEMPTECLADO,F	    ; El LSD pasa a ser el MSD
    MOVLW	b'11110000'	    
    ANDWF	TEMPTECLADO,W	    ; Limpia lo que ocupaba el lugar del LSD y al resultado lo guarda en W
    ADDWF	AUX_ADD,W	    ; Calcula el nuevo TEMPTECLADO
    MOVWF	TEMPTECLADO	    ; Reemplazo por el nuevo TEMPTECLADO
    GOTO	END_KEYPAD	

EXIT_ISR
    SWAPF       status_temp,w	    ; Recupera contexto
    MOVWF       STATUS            
    SWAPF       w_temp,f
    SWAPF       w_temp,w          
    RETFIE			    ; Vuelve de la interrupción

	END