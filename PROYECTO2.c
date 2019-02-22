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

uint8_t rxcar_uart0(void){
    uint8_t dato;
    while ((UART2_FR_R & UART_FR_RXFE)!=0); // Se espera que llegue un dato
    dato = UART2_DR_R&0xFF; // Se toman solo 8 bits
    return dato;
}

void main(void){

    uint8_t letra;//8 bits donde se almacenara lo recibido
    config_uart0(); // Se configura el UART
    config_leds();

    while(1){ // Se esperan los aciertos de la longitud de la palabra
        letra = rxcar_uart0();
        if (letra=='R')
        {   GPIO_PORTF_DATA_R &= ~(0X06);
            GPIO_PORTF_DATA_R |= 0x02;
        }
        else{
            if(letra=='B'){
                GPIO_PORTF_DATA_R &= ~(0X06);
                GPIO_PORTF_DATA_R |= 0x04;
            }


        }
    }

}