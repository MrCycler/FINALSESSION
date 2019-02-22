#include"tm4c123gh6pm.h"
#include"stdint.h"

void config_uart0 (void){
    SYSCTL_RCGCUART_R |= 0x04;
    while((SYSCTL_PRUART_R & 0x04)==0);

    SYSCTL_RCGCGPIO_R |= 0x08;
        // esperamos a que realmente se active
    while((SYSCTL_PRGPIO_R & 0x08)==0);

    UART2_CTL_R &= ~0x01;
    // Velocidad 9600bps (clock = 16MHz)
    UART2_IBRD_R = (UART2_IBRD_R & 0xFFFF0000) | 104; // Parte entera
    UART2_FBRD_R = (UART2_FBRD_R & 0xFFFFFFC0) | 10; // Parte decimal
    // 8, N, 1, FIFOshabilitados
    UART2_LCRH_R = (UART2_LCRH_R & 0xFFFFFF00) | 0x70;
    // Habilitamos ek UART3
    UART2_CTL_R |= (UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN); // enable TX, RX and module

    GPIO_PORTD_AMSEL_R &= ~((1<<6)|(1<<7));
    GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R & 0x00FFFFFF)|0x11000000;; //PD7 AND PD6 ALTERNATIVE FUNCTION
    GPIO_PORTD_AFSEL_R |= (1<<6)|(1<<7);
    GPIO_PORTD_DEN_R |= (1<<6)|(1<<7);
}

/*Función de configuración de salida PWM
* Salida: PC4
* */
void config_PWM(void){
	uint32_t temp;
	SYSCTL_RCGC0_R |= 0X100000; //Se habilita el reloj para el PWM0
	SYSCTL_RCGC2_R |=SYSCTL_RCGC2_GPIOC; //Habilitamos el puerto C para seleccionar PC4 como PWM
	temp=SYSCTL_RCGC2_R;
	GPIO_PORTC_DIR_R |= (1<<4);
	GPIO_PORTC_DR8R_R |=(1<<4);
	GPIO_PORTC_DEN_R |= (1<<4);
	GPIO_PORTC_AFSEL_R |= (1<<4); // Funcion Alterna para PC4, no GPIO
	GPIO_PORTC_PCTL_R=((GPIO_PORTC_PCTL_R & 0xFFF0FFFF)|0x00040000); // Seleccionamos
	//funcion 4 (PWM) de PC4
	SYSCTL_RCC_R = (SYSCTL_RCC_R & ~0x1E0000)| (9<<17); // Pre: 64
	PWM0_3_CTL_R= ((PWM0_3_CTL_R &~ ((3<<6)|(1<<4)|(1<<3)|(1<<1)|(1<<0)))|(1<<2)); //cuenta
	//descendente y sincronizacion local
	PWM0_3_GENA_R &= ~ ((3<<10) | (3<<0));
	PWM0_3_GENA_R |= (2<<2)|(3<<6);//PC4 es 1
}

/*Función para generar una señal PWM*/
void PWM_OUT( uint32_t Freq, uint32_t DC)
{ 	uint32_t valor_freq;
	uint32_t valor_DC;
	PWM0_ENABLE_R &= ~0x40; // Deshabilita PWM0
	PWM0_3_CTL_R &= ~1; //Desactiva generador 3 Modulo 0
	valor_freq=4000000/(Freq);
	valor_DC= (DC*(valor_freq +1)/10000)-1; //Porcentaje de la carga DC%
	PWM0_3_LOAD_R &= ~0xFFFF;
	PWM0_3_LOAD_R |= valor_freq;//Carga el valor para la freq en PWM_LOAD
	PWM0_3_CMPA_R &= ~0xFFFF;
	PWM0_3_CMPA_R |= valor_DC; // carga el valor para el DC en CMPA
	PWM0_3_CTL_R |=1; //Activa generador 3 Modulo 0
	PWM0_ENABLE_R |= (1<<6); //habilita PWM0
}

void config_leds(void){
    // activamos la señal de reloj del puerto F
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    // esperamos a que realmente se active
    while( (SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5)==0) { }
    GPIO_PORTF_DIR_R |= 0x06;
    GPIO_PORTF_DR8R_R |=0x06;
    GPIO_PORTF_DEN_R |=0x06;
    GPIO_PORTF_DATA_R &= ~(0X06);
}

void txcar_uart0(uint8_t car){
    while ((UART2_FR_R & UART_FR_TXFF)!=0); //Espera que esté disponible para transmitir
    UART2_DR_R = car;
}

void txmens_uart0(uint8_t mens[]){
    uint8_t letra;
    uint8_t i=0;
    letra= mens[i++];
    while (letra != '\0'){ //Se envían todos los caracteres hasta el fin de cadena
    txcar_uart0(letra);
    letra= mens[i++];
    }
}

uint8_t rxcar_uart0(void){
    uint8_t dato;
    while ((UART2_FR_R & UART_FR_RXFE)!=0); // Se espera que llegue un dato
    dato = UART2_DR_R&0xFF; // Se toman solo 8 bits
    return dato;
}

void main(void){

    uint8_t letra;//8 bits donde se almacenara lo recibido
    uint8_t Entrada[] = "Introduzca el caracter:\n\r";
    uint8_t Freq=50;//Frecuencia 50 Hz - 20 ms
    config_PWM();
    config_uart0(); // Se configura el UART
    config_leds();
    PWM_OUT (Freq,0);

    while(1){ // Se esperan los aciertos de la longitud de la palabra
        txmens_uart0(Entrada);
        letra = rxcar_uart0();
        txmens_uart0("\n\r");
        switch(letra)
		{
			case 'A':
			PWM_OUT(Freq,250);
			txmens_uart0("angulo 1 \n\r");
			break;
			case 'B':
			PWM_OUT(Freq,500);
			txmens_uart0("angulo 2 \n\r");;
			break;
			case 'C':
			PWM_OUT(Freq,750);
			txmens_uart0("angulo 3 \n\r");;
			break;
			case 'D':
			PWM_OUT(Freq,1000);
			txmens_uart0("angulo 4 \n\r");
			break;
			case 'E':
			PWM_OUT(Freq,1250);
			txmens_uart0("angulo 5 \n\r");
			break;
			default:
			txmens_uart0("Comando inválido \n\r");
		}

    }

}