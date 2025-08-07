#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// LCD Connections (4-bit mode)
#define LCD_RS  PA5
#define LCD_E   PA4
#define LCD_D4  PA0
#define LCD_D5  PA1
#define LCD_D6  PA2
#define LCD_D7  PA3

// LED Connections
#define GREEN_LED  PC0
#define RED_LED    PC1
#define YELLOW_LED PC2
#define BLUE_LED   PC3

// Motor Connections
#define MOTOR_DOOR  PB5
#define MOTOR_SPEED PB7  // PWM output (OC0)

// LM35 Temperature Sensor
#define LM35_PIN   PA6  // ADC6

// UART Configuration
#define BAUD_RATE 9600
#define MYUBRR F_CPU/16/BAUD_RATE-1

// Maximum command length
#define MAX_CMD_LEN 20

// Global variables
volatile char buffer[20];
volatile char command[MAX_CMD_LEN];
volatile uint8_t commandReady = 0;
volatile uint8_t cmdIndex = 0;

// Function prototypes
void LCD_Init(void);
void LCD_Cmd(uint8_t cmd);
void LCD_Data(uint8_t data);
void LCD_String(const char *str);
void LCD_Clear(void);
void LCD_GoToXY(uint8_t row, uint8_t col);
void UART_Init(uint16_t ubrr);
void UART_Transmit(uint8_t data);
uint8_t UART_Receive(void);
void UART_SendString(const char *str);
void ADC_Init(void);
uint16_t ADC_Read(uint8_t channel);
float Read_Temperature(void);
void PWM_Init(void);
void PWM_SetDuty(uint8_t duty);
void ProcessCommand(void);

int main(void) {
    // Initialize I/O
    DDRC |= (1<<GREEN_LED)|(1<<RED_LED)|(1<<YELLOW_LED)|(1<<BLUE_LED);
    DDRB |= (1<<MOTOR_DOOR)|(1<<MOTOR_SPEED);
    DDRA |= (1<<LCD_RS)|(1<<LCD_E)|(1<<LCD_D4)|(1<<LCD_D5)|(1<<LCD_D6)|(1<<LCD_D7);

    // Initialize peripherals
    LCD_Init();
    UART_Init(MYUBRR);
    ADC_Init();
    PWM_Init();

    // Test sequence
    PORTC |= (1<<GREEN_LED)|(1<<RED_LED)|(1<<YELLOW_LED)|(1<<BLUE_LED);
    LCD_Clear();
    LCD_String("SYSTEM TEST");
    UART_SendString("System test in progress...\r\n");
    _delay_ms(500);
    PORTC &= ~((1<<GREEN_LED)|(1<<RED_LED)|(1<<YELLOW_LED)|(1<<BLUE_LED));
    LCD_Clear();

    // Enable interrupts
    sei();

    // System ready message
    LCD_String("SYSTEM READY");
    UART_SendString("\r\nSystem initialized\r\n");
    UART_SendString("Available commands:\r\n");
    UART_SendString("*LED_ON# *LED_OFF#\r\n");
    UART_SendString("*MS_50# *MS_100#\r\n");
    UART_SendString("*D_OP# *D_CL#\r\n");
    UART_SendString("*SC#\r\n> ");

    while(1) {
        if(commandReady) {
            ProcessCommand();
            commandReady = 0;
            cmdIndex = 0;
            memset((void*)command, 0, sizeof(command));
            UART_SendString("> ");
        }
        _delay_ms(10);
    }
}

// LCD Functions
void LCD_Init() {
    _delay_ms(50);

    // Initialization sequence for 4-bit mode
    LCD_Cmd(0x33);
    LCD_Cmd(0x32);
    LCD_Cmd(0x28);  // 4-bit mode, 2 lines, 5x8 font
    LCD_Cmd(0x0C);  // Display on, cursor off
    LCD_Cmd(0x06);  // Increment cursor
    LCD_Cmd(0x01);  // Clear display
    _delay_ms(2);
}

void LCD_Cmd(uint8_t cmd) {
    PORTA = (PORTA & 0x0F) | (cmd & 0xF0);
    PORTA &= ~(1<<LCD_RS);
    PORTA |= (1<<LCD_E);
    _delay_us(50);
    PORTA &= ~(1<<LCD_E);
    _delay_us(100);

    PORTA = (PORTA & 0x0F) | ((cmd << 4) & 0xF0);
    PORTA |= (1<<LCD_E);
    _delay_us(50);
    PORTA &= ~(1<<LCD_E);
    _delay_ms(2);
}

void LCD_Data(uint8_t data) {
    PORTA = (PORTA & 0x0F) | (data & 0xF0);
    PORTA |= (1<<LCD_RS);
    PORTA |= (1<<LCD_E);
    _delay_us(50);
    PORTA &= ~(1<<LCD_E);
    _delay_us(100);

    PORTA = (PORTA & 0x0F) | ((data << 4) & 0xF0);
    PORTA |= (1<<LCD_E);
    _delay_us(50);
    PORTA &= ~(1<<LCD_E);
    _delay_ms(2);
}

void LCD_String(const char *str) {
    while(*str) {
        LCD_Data(*str++);
    }
}

void LCD_Clear() {
    LCD_Cmd(0x01);
    _delay_ms(2);
}

void LCD_GoToXY(uint8_t row, uint8_t col) {
    uint8_t address;
    switch(row) {
        case 0: address = 0x80 + col; break;
        case 1: address = 0xC0 + col; break;
        default: address = 0x80;
    }
    LCD_Cmd(address);
}

// UART Functions
void UART_Init(uint16_t ubrr) {
    UBRRH = (uint8_t)(ubrr>>8);
    UBRRL = (uint8_t)ubrr;
    UCSRB = (1<<RXEN)|(1<<TXEN)|(1<<RXCIE);
    UCSRC = (1<<URSEL)|(3<<UCSZ0);
}

void UART_Transmit(uint8_t data) {
    while(!(UCSRA & (1<<UDRE)));
    UDR = data;
}

uint8_t UART_Receive(void) {
    while(!(UCSRA & (1<<RXC)));
    return UDR;
}

void UART_SendString(const char *str) {
    while(*str) {
        UART_Transmit(*str++);
    }
}

// ADC Functions
void ADC_Init() {
    ADMUX = (1<<REFS0);  // AVcc reference, right-adjusted (10-bit)
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);  // Enable ADC, prescaler 128
    DIDR0 = (1<<ADC6D);  // Disable digital input on ADC6 (PA6)
}

uint16_t ADC_Read(uint8_t channel) {
    if (channel > 7) return 0;  // Safety check
    ADMUX = (ADMUX & 0xF8) | (channel & 0x07);
    ADCSRA |= (1<<ADSC);
    while (ADCSRA & (1<<ADSC));
    return ADC;  // 10-bit result
}

float Read_Temperature(void) {
    uint16_t adc_value = ADC_Read(6);  // Channel 6 (PA6)
    return (adc_value * 500.0) / 1024.0;  // LM35 calculation (10-bit)
}

// PWM Functions
void PWM_Init() {
    DDRB |= (1<<MOTOR_SPEED);  // Set PB7 as output (OC0)
    TCCR0 = (1<<WGM00)|(1<<WGM01)|(1<<COM01)|(1<<CS01);  // Fast PWM, non-inverting, prescaler 8
    OCR0 = 0;  // Start with 0% duty
}

void PWM_SetDuty(uint8_t duty) {
    OCR0 = (uint16_t)duty * 255 / 100;
}

// Command Processing
void ProcessCommand(void) {
    LCD_Clear();
    char uartMsg[40];

    if(strcmp(command, "LED_ON") == 0) {
        PORTC |= (1<<BLUE_LED);
        LCD_String("BLUE:ON ");
        sprintf(uartMsg, "Blue LED activated\r\n");
    }
    else if(strcmp(command, "LED_OFF") == 0) {
        PORTC &= ~(1<<BLUE_LED);
        LCD_String("BLUE:OFF");
        sprintf(uartMsg, "Blue LED deactivated\r\n");
    }
    else if(strcmp(command, "MS_50") == 0) {
        PWM_SetDuty(50);
        LCD_String("MOTOR:50%");
        sprintf(uartMsg, "Motor speed set to 50%%\r\n");
    }
    else if(strcmp(command, "MS_100") == 0) {
        PWM_SetDuty(100);
        LCD_String("MOTOR:100%");
        sprintf(uartMsg, "Motor speed set to 100%%\r\n");
    }
    else if(strcmp(command, "D_OP") == 0) {
        PORTB |= (1<<MOTOR_DOOR);
        LCD_String("DOOR:OPEN");
        sprintf(uartMsg, "Door opened\r\n");
    }
    else if(strcmp(command, "D_CL") == 0) {
        PORTB &= ~(1<<MOTOR_DOOR);
        LCD_String("DOOR:CLOSED");
        sprintf(uartMsg, "Door closed\r\n");
    }
    else if(strcmp(command, "SC") == 0) {
        float temp = Read_Temperature();

        LCD_GoToXY(0,0);
        sprintf(buffer, "TEMP:%.1fC", temp);
        LCD_String(buffer);

        LCD_GoToXY(1,0);
        PORTC &= ~((1<<GREEN_LED)|(1<<YELLOW_LED)|(1<<RED_LED));

        if(temp < 26) {
            PORTC |= (1<<GREEN_LED);
            LCD_String("COOL:GREEN");
            sprintf(uartMsg, "Temperature: %.1f°C (Cool)\r\n", temp);
        }
        else if(temp < 36) {
            PORTC |= (1<<YELLOW_LED);
            LCD_String("WARM:YELLOW");
            sprintf(uartMsg, "Temperature: %.1f°C (Warm)\r\n", temp);
        }
        else {
            PORTC |= (1<<RED_LED);
            LCD_String("HOT:RED");
            sprintf(uartMsg, "Temperature: %.1f°C (Hot)\r\n", temp);
        }
    }
    else {
        LCD_String("INVALID CMD");
        sprintf(uartMsg, "Error: Unknown command '%s'\r\n", command);
    }

    UART_SendString(uartMsg);
}

// UART Interrupt Handler
ISR(USART_RXC_vect) {
    char received = UDR;
    UART_Transmit(received);  // Echo character back

    if(received == '#') {
        command[cmdIndex] = '\0';
        commandReady = 1;
    }
    else if(received == '*') {
        cmdIndex = 0;
    }
    else if(cmdIndex < MAX_CMD_LEN - 1) {
        command[cmdIndex++] = received;
    }
    else {
        // Buffer overflow - reset
        cmdIndex = 0;
        UART_SendString("\r\nERROR: Command too long\r\n> ");
    }
}
