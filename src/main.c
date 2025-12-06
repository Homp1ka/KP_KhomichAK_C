#include <xc.h>        // PIC compiler header
#include <stdint.h>    // Standard integer types

#define _XTAL_FREQ 4000000UL  // MCU frequency 4 MHz (for compiler delays)


#pragma config OSC = INTIO67     // Internal oscillator, RA6/RA7 as I/O
#pragma config FCMEN = OFF       // Fail-Safe Clock Monitor disabled
#pragma config IESO = OFF        // Internal/External Oscillator Switchover disabled
#pragma config PWRT = OFF        // Power-up Timer disabled
#pragma config BOREN = OFF       // Brown-out Reset disabled
#pragma config BORV = 3          // Brown-out Reset Voltage (2.8V)
#pragma config WDT = OFF         // Watchdog Timer disabled
#pragma config WDTPS = 32768     // Watchdog Timer Postscaler (1:32768)
#pragma config CCP2MX = PORTC    // CCP2 MUX bit configuration
#pragma config PBADEN = OFF      // PORTB<4:0> as digital I/O on Reset
#pragma config LPT1OSC = OFF     // Low-Power Timer1 Oscillator disabled
#pragma config MCLRE = ON        // MCLR pin enabled (reset button)
#pragma config STVREN = ON       // Stack full/underflow will cause Reset
#pragma config LVP = OFF         // Single-Supply ICSP disabled
#pragma config XINST = OFF       // Extended Instruction Set disabled
#pragma config DEBUG = OFF       // Background Debugger disabled


volatile unsigned char adc_update_flag = 0; // Flag: "time to read ADC" (set by Timer0 ISR)
volatile uint32_t system_tick = 0;          // System milliseconds counter (incremented in ISR)

uint16_t adc_raw = 0;      // Raw ADC value (0..1023 corresponds to 0..5V)
uint16_t filtered = 0;     // Filtered (smoothed) ADC value



//Calculation for 1 ms at 4 MHz:
  //Fosc/4 = 1 MHz = 1 µs period
  //With prescaler 1:8: 8 µs per count
  //Required: 1000 µs / 8 µs = 125 counts
 //nitial value: 65536 - 125 = 65411 = 0xFF83
 //
void setupSimpleTimer(void) {
    // Turn off Timer0 for configuration
    T0CONbits.TMR0ON = 0;
    
    // Configure Timer0:
    T0CONbits.T08BIT = 0;   // 16-bit mode (counts to 65535)
    T0CONbits.T0CS   = 0;   // Internal clock (Fosc/4)
    T0CONbits.PSA    = 0;   // Use prescaler (not bypass)
    
    // Prescaler 1:8 (faster for milliseconds counting)
    T0CONbits.T0PS2  = 0;
    T0CONbits.T0PS1  = 1;
    T0CONbits.T0PS0  = 1;

    // Load initial value for 1 ms interval
    TMR0H = 0xFF;  // High byte of 0xFF83
    TMR0L = 0x83;  // Low byte of 0xFF83

    // Enable Timer0 interrupt
    INTCONbits.TMR0IE = 1;  // Enable Timer0 overflow interrupt
    INTCONbits.TMR0IF = 0;  // Clear interrupt flag
    INTCONbits.GIE    = 1;  // Global interrupt enable

    // Start Timer0
    T0CONbits.TMR0ON = 1;
}

/**
 * @brief Returns current system time in milliseconds
 * @return uint32_t Milliseconds since system start
 * 
 * This function provides a non-blocking way to track time.
 * The counter will overflow after approximately 49 days.
 */
uint32_t getTickCount(void) {
    return system_tick;  // Simple return of volatile variable
}


uint8_t checkTimeout(uint32_t start_time, uint32_t interval_ms) {
    uint32_t current = getTickCount();
    
    // Handle normal case (no overflow)
    if (current >= start_time) {
        return (current - start_time >= interval_ms);
    }
    
    // Handle overflow case (counter wrapped around)
    return 0;
}



  //SPBRG = (Fosc / (16 * Baud)) - 1 ≈ (4000000/(16*9600))-1 ≈ 25
 
void setupUsart(void) {
    // Configure RC6 (TX) as output
    TRISCbits.TRISC6 = 0;

    // UART configuration:
    TXSTAbits.BRGH = 1;   // High speed baud rate generator
    TXSTAbits.SYNC = 0;   // Asynchronous mode (UART)
    RCSTAbits.SPEN = 1;   // Enable serial port
    TXSTAbits.TXEN = 1;   // Enable transmitter

    // Set baud rate to 9600 at 4 MHz
    SPBRG = 25;
}


void UsartWriteChar(char c) {
 
    while (!TXSTAbits.TRMT);  // TRMT=1 when shift register is empty
    TXREG = c;                // Load character into transmit register
}


void UsartWriteString(const char *str) {
    while (*str) {
        UsartWriteChar(*str++);
    }
}


void UsartWriteNumber(uint16_t value) {
    char buf[6];   // Buffer for 5 digits (max 65535) + null terminator
    int i = 0;

    // Special case: number is 0
    if (value == 0) {
        UsartWriteChar('0');
        return;
    }

    // Extract digits from right to left
    while (value > 0) {
        buf[i++] = '0' + (value % 10);  // Get last digit
        value /= 10;                    // Remove last digit
    }
    
    // Send digits in correct order (left to right)
    for (int j = i - 1; j >= 0; j--) {
        UsartWriteChar(buf[j]);
    }
}


void setupAdc(void) {
    // Configure RA0/AN0 as analog input (1=input, 0=output)
    TRISAbits.TRISA0 = 1;

    // ADC port configuration:
    // ADCON1 = 0x0E = 0b00001110
    // Bits 3-0: PCFG3:PCFG0 = 1110 (AN0 analog, others digital)
    // Bits 5-4: VCFG1:VCFG0 = 00 (VDD/VSS as voltage references)
    ADCON1 = 0x0E;

    // Select channel AN0 and enable ADC
    ADCON0bits.CHS = 0b0000;  // Channel AN0 (binary 0000)
    ADCON0bits.ADON = 1;      // Turn on ADC module

    // Configure conversion parameters:
    ADCON2bits.ADCS = 0b010;  // Conversion clock = Fosc/32
    ADCON2bits.ACQT = 0b010;  // Acquisition time = 4 TAD
    ADCON2bits.ADFM = 1;      // Result right justified (0-1023)

    // Simple stabilization delay (non-blocking loop)
    for(uint16_t i = 0; i < 1000; i++);
}


uint16_t adcReadAn0(void) {
    // Ensure AN0 is selected (redundant but safe)
    ADCON0bits.CHS = 0b0000;
    
    // Short delay for channel switching (non-blocking)
    for(uint8_t i = 0; i < 10; i++);
    
    // Start conversion
    ADCON0bits.GO = 1;
    
    // Wait for conversion to complete
    while (ADCON0bits.GO);

    // Combine 10-bit result from two registers:
    // ADRESH contains bits 9-8 (2 MSB)
    // ADRESL contains bits 7-0 (8 LSB)
    return (((uint16_t)ADRESH << 8) | ADRESL);
}


void setupPwm(void) {
    // Configure RC2/CCP1 as output
    TRISCbits.TRISC2 = 0;

    // Configure Timer2 for PWM:
    T2CONbits.T2CKPS = 0b11;   // Prescaler 1:16
    PR2 = 62;                  // PWM period register
    
    /*
     * PWM frequency calculation:
     * Fpwm = Fosc / (4 * prescaler * (PR2 + 1))
     *      = 4000000 / (4 * 16 * 63)
     *      ≈ 992 Hz ≈ 1 kHz
     */

    // Configure CCP1 for PWM mode
    CCP1CONbits.CCP1M = 0b1100; // PWM mode (bits 3-2: 11, bits 1-0: 00)

    // Initial duty cycle = 0% (LED off)
    CCPR1L = 0;                // High 8 bits of duty cycle
    CCP1CONbits.DC1B = 0;      // Low 2 bits of duty cycle

    // Start Timer2
    T2CONbits.TMR2ON = 1;
}


void pwmSetDuty(uint16_t duty) {
    // Limit duty cycle to maximum 1023 (10 bits)
    if (duty > 1023) duty = 1023;

    // Split 10-bit value into registers
    CCPR1L = (uint8_t)(duty >> 2);   // Bits 9-2 (shift right by 2)
    CCP1CONbits.DC1B = duty & 0x03;  // Bits 1-0 (mask with 0b00000011)
}


void __interrupt() ISR(void) {
    // Check if interrupt is from Timer0 overflow
    if (INTCONbits.TMR0IF) {
        // Reload Timer0 for next 1 ms interval
        TMR0H = 0xFF;
        TMR0L = 0x83;
        
        // Increment system milliseconds counter
        system_tick++;
        
        // Set ADC update flag every 500 ms
        // Using modulo operator: (tick % 500) == 0
        if ((system_tick % 500) == 0) {
            adc_update_flag = 1;
        }
        
        // Clear Timer0 interrupt flag (MANDATORY!)
        INTCONbits.TMR0IF = 0;
    }
    
    // Other interrupt sources could be checked here
    // Example: if (PIR1bits.TMR1IF) { ... }
}


void main(void) {
    // 1. CONFIGURE SYSTEM CLOCK
    // Set internal oscillator to 4 MHz
    OSCCONbits.IRCF = 0b110;  // Internal RC Frequency = 4 MHz
    OSCCONbits.SCS  = 0b10;   // System Clock Select = internal oscillator
    
    // Wait for oscillator to stabilize
    while (!OSCCONbits.IOFS);  // Wait for HFIOFS flag (oscillator stable)

    // 2. INITIALIZE ALL MODULES (order is important!)
    setupSimpleTimer();  // First: system timer (needed for timing)
    setupUsart();        // Second: UART for debugging
    setupAdc();          // Third: ADC for analog readings
    setupPwm();          // Fourth: PWM for LED control

    // 3. INITIAL READINGS AND VARIABLES
    adc_raw = adcReadAn0();   // First ADC reading
    filtered = adc_raw;       // Initialize filter with first value

    // 4. STARTUP MESSAGE
    UsartWriteString("=== SYSTEM TIMER WORKING ===\r\n");
    UsartWriteString("Timer: 1ms ticks, ADC: 500ms\r\n");
    UsartWriteString("============================\r\n");

    // 5. TASK CONTROL VARIABLES
    uint32_t last_uart_time = getTickCount();  // Last UART transmission time
    uint32_t last_blink_time = getTickCount(); // Last blink time
    uint8_t blink_state = 0;                   // Virtual LED state

    // 6. MAIN SUPERVISORY LOOP (runs forever)
    while (1) {
        // TASK 1: ADC READING AND PWM CONTROL
        // This task is triggered by interrupt flag (every 500 ms)
        if (adc_update_flag) {
            adc_update_flag = 0;  // Clear flag
            
            // Read new ADC value
            adc_raw = adcReadAn0();
            
            // Apply exponential smoothing filter (alpha = 1/8)
            // filtered = filtered + (new - filtered) / 8
            int16_t diff = (int16_t)adc_raw - (int16_t)filtered;
            filtered = filtered + (diff >> 3);  // >>3 = divide by 8
            
            // Update LED brightness via PWM
            pwmSetDuty(filtered);
        }
        
        // TASK 2: UART DATA TRANSMISSION (every 300 ms)
        // Non-blocking: checks time without delaying
        if (checkTimeout(last_uart_time, 300)) {
            last_uart_time = getTickCount();  // Update timestamp
            
            // Convert ADC value to millivolts (0-1023 to 0-5000 mV)
            uint32_t mv = ((uint32_t)filtered * 5000UL) / 1023UL;
            
            // Send formatted data via UART
            UsartWriteString("T=");
            UsartWriteNumber((uint16_t)getTickCount());
            UsartWriteString("ms ADC=");
            UsartWriteNumber(filtered);
            UsartWriteString(" U=");
            UsartWriteNumber((uint16_t)mv);
            UsartWriteString("mV\r\n");
        }
        
        // TASK 3: VIRTUAL LED BLINKING (every 1000 ms)
        // Demonstrates parallel task execution
        if (checkTimeout(last_blink_time, 1000)) {
            last_blink_time = getTickCount();
            blink_state = !blink_state;  // Toggle state
            
            // In real hardware, this would control an actual LED
            // Here we just send message to demonstrate timing
            if (blink_state) {
                UsartWriteString("[LED ON]\r\n");
            } else {
                UsartWriteString("[LED OFF]\r\n");
            }
        }
        
      
    }
}

