// ============================================
//               INCLUDES / DEFINES
// ============================================
#include "stm32f0xx.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

// LED pin definitions (PWM remapped)
#define LED1_RED_PIN 6   // PC6 - TIM3_CH1
#define LED1_GREEN_PIN 7 // PC7 - TIM3_CH2
#define LED1_BLUE_PIN 8  // PC8 - TIM3_CH3

// GPIO pin definitions
#define GPIO_PIN_0 (1U << 0)
#define GPIO_PIN_1 (1U << 1)
#define GPIO_PIN_2 (1U << 2)
#define GPIO_PIN_3 (1U << 3)
#define GPIO_PIN_RESET 0
#define BUTTON_GREEN_PIN GPIO_PIN_0
#define BUTTON_ORANGE_PIN GPIO_PIN_1
#define BUTTON_RED_PIN GPIO_PIN_2
#define BUTTON_BLUE_PIN GPIO_PIN_3
#define BUTTON_PORT GPIOB

// Game constants
#define MAX_LEVEL 10
#define LED_ON_TIME 500   // milliseconds
#define BETWEEN_TIME 200  // milliseconds
#define DEBOUNCE_DELAY 50 // milliseconds

// Wavetable constants
#define N 1000
#define RATE 20000

// Global game state variables
uint8_t sequence[MAX_LEVEL];
uint8_t current_level = 0;
bool game_over = false;
bool input_mode = false;
uint8_t current_input = 0;
short int wavetable[N];
int step0 = 0, offset0 = 0;
int step1 = 0, offset1 = 0;
volatile uint32_t volume = 2400;
bool game_started = false;

// ============================================
//              FUNCTION PROTOTYPES
// ============================================
// System setup
void internal_clock(void);
void GPIO_Configure(void);
void PWM_Configure(void);
void SysTick_Configure(void);
void init_adc(void);
void init_dac(void);
void init_wavetable(void);
void init_tim6(void);
void init_usart5(void);

// Timer initializations
void init_tim2(void);
void init_tim3(void);
void init_tim15(void);

// DAC IRQ Handler
void TIM6_DAC_IRQHandler(void);

// Game logic
void reset_game(void);
void generate_sequence(void);
void play_sequence(void);
void handle_user_input(void);
void success_feedback(void);
void game_over_sequence(void);
void win_sequence(void);
void wait_for_restart(void);

// Helpers
void light_led(uint8_t led);
void light_led_color(uint8_t led);
void set_led_brightness(uint8_t led, char color, uint16_t brightness);
bool check_button(uint8_t button);
uint8_t get_button_press(void);
void delay_ms(uint32_t ms);

// ====== OLED Functions (Mackenzie's Code) ======
void nano_wait(unsigned int n);
void spi_cmd(unsigned int data);
void spi_data(unsigned int data);
void spi1_init_oled(void);
void spi1_display1(const char *string);
void spi1_display2(const char *string);
void init_spi1(void);
void oled_clear(void);

// Sound helpers
void error_beep(void);
void set_freq(int chan, float f);
void set_volume(uint16_t adc_value);
uint16_t read_adc(void);
int __io_putchar(int c);

// Diagnostic
void test_led_button_mapping(void);

// ============================================
//          FULL IMPLEMENTATION BELOW
// ============================================

// System setup
void GPIO_Configure(void)
{
    // Enable GPIO clocks for A, B, C
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;

    // ------------------------------
    // Configure PB8–PB11 (Game Logic LEDs)
    // PB8  = White
    // PB9  = Red
    // PB10 = Green
    // PB11 = Blue
    // ------------------------------
    // Set as General Purpose Output Mode (01)
    // Set PB8–PB11 as OUTPUT
    GPIOB->MODER &= ~((3U << (8 * 2)) | (3U << (9 * 2)) | (3U << (10 * 2)) | (3U << (11 * 2)));
    GPIOB->MODER |= ((1U << (8 * 2)) | (1U << (9 * 2)) | (1U << (10 * 2)) | (1U << (11 * 2)));

    // Push-pull output
    GPIOB->OTYPER &= ~((1U << 8) | (1U << 9) | (1U << 10) | (1U << 11));

    // Output speed: High speed (11)
    GPIOB->OSPEEDR &= ~((3U << (8 * 2)) | (3U << (9 * 2)) |
                        (3U << (10 * 2)) | (3U << (11 * 2)));
    GPIOB->OSPEEDR |= ((3U << (8 * 2)) | (3U << (9 * 2)) |
                       (3U << (10 * 2)) | (3U << (11 * 2)));

    // No pull-up/pull-down for LEDs
    GPIOB->PUPDR &= ~((3U << (8 * 2)) | (3U << (9 * 2)) |
                      (3U << (10 * 2)) | (3U << (11 * 2)));

    // ------------------------------
    // Configure PB0–PB3 (buttons) as input with pull-up
    // ------------------------------
    // PB0–PB3 (buttons) as input with pull-up
    GPIOB->MODER &= ~((3U << (0 * 2)) | (3U << (1 * 2)) |
                      (3U << (2 * 2)) | (3U << (3 * 2))); // Input mode (00)

    GPIOB->PUPDR &= ~((3U << (0 * 2)) | (3U << (1 * 2)) |
                      (3U << (2 * 2)) | (3U << (3 * 2))); // Clear previous
    GPIOB->PUPDR |= ((1U << (0 * 2)) | (1U << (1 * 2)) |
                     (1U << (2 * 2)) | (1U << (3 * 2))); // Pull-up (01)
}
void SysTick_Configure(void)
{
    SysTick->LOAD = (48000 - 1);
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}

// PWM Configuration and Timers
void PWM_Configure(void)
{
    // This function delegates PWM initialization to specific timer setups
    init_tim3();
    init_tim2();
    init_tim15();
}
void init_tim2(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 47;
    TIM2->ARR = 999;
    TIM2->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE_Msk |
                  (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC1PE_Msk;
    TIM2->CCMR2 = (6U << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE_Msk;
    TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM2->CR1 = TIM_CR1_CEN;
}
void init_tim3(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->PSC = 47;
    TIM3->ARR = 999;
    TIM3->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE_Msk |
                  (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE_Msk;
    TIM3->CCMR2 = (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE_Msk |
                  (6U << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE_Msk;
    TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM3->CR1 = TIM_CR1_CEN;
}
void init_tim15(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
    TIM15->PSC = 47;
    TIM15->ARR = 999;
    TIM15->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE_Msk |
                   (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE_Msk;
    TIM15->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM15->CR1 = TIM_CR1_CEN;
}

// ADC and DAC functions
void init_adc(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // enable PA1
    GPIOA->MODER |= (3 << (1 * 2));    // PA1 to analog mode
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN; // PA1 as adc in

    ADC1->CFGR1 &= ~ADC_CFGR1_CONT; // single conversion mode
    ADC1->CR |= ADC_CR_ADEN;        // enable adc
    while (!(ADC1->ISR & ADC_ISR_ADRDY))
        ; // wait until ready, then leave function
}
uint16_t read_adc(void)
{
    ADC1->CHSELR = ADC_CHSELR_CHSEL1; // select PA1
    ADC1->CR |= ADC_CR_ADSTART;
    while (!(ADC1->ISR & ADC_ISR_EOC))
        ; // wait for conversion
    return ADC1->DR;
}
void set_volume(uint16_t adc_value)
{
    volume = 100 + ((adc_value * 3000) / 4095);
}
void init_dac(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // enable GPIOA RCC clock
    GPIOA->MODER |= (0x3 << (2 * 4));  // set PA4 to DAC_OUT1: (0x3<<(2*4))= 11 00 00 00 00
    RCC->APB1ENR |= RCC_APB1ENR_DACEN; // enable DAC RCC clock

    DAC->CR &= ~DAC_CR_TSEL1; // select TRGO trigger fro TSEL field in CR register(bit 000(~0x7) shifted to the left 3 bits)

    DAC->CR |= DAC_CR_EN1; // enable DAC
}
void init_wavetable(void)
{
    for (int i = 0; i < N; i++)
        wavetable[i] = (i < N / 2) ? 32767 : -32767; // create wave for buzzer sound
}

// USART Debug Setup
void init_usart5()
{
    RCC->AHBENR |= (RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN); // enable port c and d clocks

    GPIOC->MODER &= ~(GPIO_MODER_MODER12); // reset PC12
    GPIOC->MODER |= (0x2 << (2 * 12));     // set PC12 function to alt func(2)

    GPIOC->AFR[1] |= (0x2 << (16)); // set AFR[1], bit shifted 16 bits, which represents AFR12(which means this is the set bit for AltFunc for PC12) to 0x2(representing AF2, which is USART5_TX)

    GPIOD->MODER &= ~(GPIO_MODER_MODER2); // reset PD2
    GPIOD->MODER |= (0x2 << (2 * 2));     // set PD2 function to alt func(2)

    GPIOD->AFR[0] |= (0x2 << (8)); // set AFR[0], bit shifted 8 bits, which represents AFR2(which means this is the set bit for AltFunc for PD2) to 0x2(representing AF2, which is USART5_RX)

    RCC->APB1ENR |= RCC_APB1ENR_USART5EN;          // enable RCC clock to USART5
    USART5->CR1 &= ~(USART_CR1_UE);                // disable usart bit
    USART5->CR1 &= ~(USART_CR1_M0 | USART_CR1_M1); // set size to 8bits(M0 and M1 must be set to 00)
    USART5->CR2 &= ~(USART_CR2_STOP);              // set stop to be 00(representing 1 stop bit) ///this may be wrong???
    USART5->CR1 &= ~(USART_CR1_PCE);               // set parity control enable to 0 for off
    USART5->CR1 &= ~(USART_CR1_OVER8);             // set OVER8 to 0, representing 16x oversampling
    USART5->BRR = 0x1A1;                           // gotten from family reference for oversampling 16, desired baud rate of 115.2KBps

    USART5->CR1 |= USART_CR1_TE; // enable transmitter bit
    USART5->CR1 |= USART_CR1_RE; // enable reciever bit

    USART5->CR1 |= USART_CR1_UE; // enable usart bit
    while (!((USART5->ISR & USART_ISR_TEACK) && (USART5->ISR & USART_ISR_REACK)))
    {
        // wait until TE adn RE bits are acknowledged(TEACK and REACK and these flags), then the function can continue on to the end
    }
    // return; //end function
}
int __io_putchar(int c)
{
    while (!(USART5->ISR & USART_ISR_TXE))
        ;
    if (c == '\n')
    {                       // check if char is new line
        USART5->TDR = '\r'; // if char is new line, put carraige return first
        while (!(USART5->ISR & USART_ISR_TXE))
        {
            // wait for TXE flag, meaning it has been passed
        }
    }
    USART5->TDR = c; // put char to output
    return c;
}

// Interupt Handler
void init_tim6(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;         // enable clock for TIM6
    TIM6->PSC = ((48000000 / (RATE * 10))) - 1; // set PSC to 48000000/(RATE*10)
    TIM6->ARR = 10 - 1;                         // set ARR to 9

    TIM6->DIER |= TIM_DIER_UIE; // enable UIE bit in DIER to allow an interrupt to occur each time the counter reaches the ARR value, restarting it back at 0

    NVIC_EnableIRQ(TIM6_DAC_IRQn); // enable interrupt for TIM6

    TIM6->CR1 |= TIM_CR1_CEN; // enable TIM6 by setting CEN bit
    return;
}
void TIM6_DAC_IRQHandler()
{                            // TIM7 ISR
    TIM6->SR &= ~TIM_SR_UIF; // clear UIF bit in SR register(acknowledge interrupt)

    offset0 += step0; // increment offset0 by step0
    offset1 += step1; // increment offset0 by step1

    if (offset0 >= (N << 16))
    {
        offset0 -= (N << 16); // decrement offset0 by (N<<16) if offset0 is greater than (N<<16)
    }
    if (offset1 >= (N << 16))
    {
        offset1 -= (N << 16); // decrement offset1 by (N<<16) if offset1 is greater than (N<<16)
    }

    int samp = wavetable[offset0 >> 16] + wavetable[offset1 >> 16]; // samp is the sum of the wavetable of offset0 and offset1

    samp = (samp * volume) >> 15; // multiply samp by volume and shift right 17 bits

    samp += 1200; // increment samp by 2048

    DAC->DHR12R1 = (samp & 0xFFF);

    return;
}

// Sound Helpers
void set_freq(int chan, float f)
{
    if (chan == 0)
    {
        if (f == 0.0)
        {
            step0 = 0;
            offset0 = 0;
        }
        else
            step0 = (f * N / RATE) * (1 << 16);
    }
    if (chan == 1)
    {
        if (f == 0.0)
        {
            step1 = 0;
            offset1 = 0;
        }
        else
            step1 = (f * N / RATE) * (1 << 16);
    }
}
void error_beep(void)
{
    set_freq(0, 250); // set standard freq, tried different frequencies, like this best
    volatile int x;   // included because i kept getting overwritten when checking for set_volume
    for (volatile int i = 0; i < 500000; i++)
    {                 // delay for a set time
        x = i % 1000; // mod i by 1000
        if (x == 0)
        {                           // if i mod 1000 is 0(basically, every 1000 ticks of i), then run set_volume(read_adc) to set the volume incrementally throughout the sound being played
            set_volume(read_adc()); // set volume based on potentiometer value
        }
    }
    set_freq(0, 0); // stop sound
}

// LED Helpers
void set_led_brightness(uint8_t led, char color, uint16_t brightness)
{
    switch (led)
    {
    case 1:
        switch (color)
        {
        case 'R':
            TIM3->CCR1 = brightness;
            break;
        case 'G':
            TIM3->CCR2 = brightness;
            break;
        case 'B':
            TIM3->CCR3 = brightness;
            break;
        }
        break;
    case 2:
        switch (color)
        {
        case 'R':
            TIM3->CCR4 = brightness;
            break;
        case 'G':
            TIM15->CCR1 = brightness;
            break;
        case 'B':
            TIM15->CCR2 = brightness;
            break;
        }
        break;
    case 3:
        switch (color)
        {
        case 'R':
            TIM2->CCR1 = brightness;
            break;
        case 'G':
            TIM2->CCR3 = brightness;
            break;
        case 'B':
            TIM2->CCR4 = brightness;
            break;
        }
        break;
    default:
        break;
    }
}
void light_led_color(uint8_t led)
{
    light_led(led); // Forward call to light_led()
}
void light_led(uint8_t led)
{
    // Turn off all LEDs first
    // GPIOB->ODR &= ~((1U << 8) | (1U << 9) | (1U << 10) | (1U << 11));

    GPIOB->ODR |= ((1U << 8) | (1U << 9) | (1U << 10) | (1U << 11));

    // Light the matching LED
    switch (led)
    {
    case 1:
        GPIOB->ODR |= (1U << 8); // White LED (PB8)
        break;
    case 2:
        GPIOB->ODR |= (1U << 9); // Red LED (PB9)
        break;
    case 3:
        GPIOB->ODR |= (1U << 10); // Green LED (PB10)
        break;
    case 4:
        GPIOB->ODR |= (1U << 11); // Blue LED (PB11)
        break;
    default:
        // If led == 0 or invalid, all LEDs stay off
        break;
    }
}

// Input Helpers
bool check_button(uint8_t button)
{
    switch (button)
    {
    case 1:
        return !(BUTTON_PORT->IDR & GPIO_PIN_0); // NOT '!' because active-low
    case 2:
        return !(BUTTON_PORT->IDR & GPIO_PIN_1);
    case 3:
        return !(BUTTON_PORT->IDR & GPIO_PIN_2);
    case 4:
        return !(BUTTON_PORT->IDR & GPIO_PIN_3);
    default:
        return false;
    }
}

uint8_t get_button_press(void)
{
    uint8_t detected_button = 0;

    while (1)
    {
        for (uint8_t i = 1; i <= 4; i++)
        {
            if (check_button(i))
            {
                // THIS IS THE FIX: Wait until the button is released BEFORE returning.
                while (check_button(i))
                {
                    // Wait for button release
                }
                delay_ms(DEBOUNCE_DELAY); // Optional debounce after release
                return i;
            }
        }
        delay_ms(10); // Prevent tight loop
    }
    return detected_button;
}

// Delay Helper
void delay_ms(uint32_t ms)
{
    volatile uint32_t delay_counter = ms;
    while (delay_counter--)
    {
        for (volatile uint32_t i = 0; i < 4800; i++)
            ; // ~1ms delay at 48MHz
    }
}

// LCD Functions
// Mackenzie's OLED SPI functions:
int score(int current)
{
    return (10 * current);
}
void spi_cmd(unsigned int data)
{
    while (!(SPI1->SR & SPI_SR_TXE))
    {
    }
    SPI1->DR = data;
}
void spi_data(unsigned int data)
{
    spi_cmd(data | 0x200);
}
void nano_wait(unsigned int n)
{
    asm("        mov r0,%0\n"
        "repeat: sub r0,#83\n"
        "        bgt repeat\n" : : "r"(n) : "r0", "cc");
}
void spi1_init_oled(void)
{
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    nano_wait(1000000);
    spi_cmd(0x38); // Function set
    spi_cmd(0x08); // Display OFF
    spi_cmd(0x01); // Clear display
    nano_wait(2000000);
    spi_cmd(0x06); // Entry mode set
    spi_cmd(0x02); // Cursor home
    spi_cmd(0x0C); // Display ON
}
void spi1_display1(const char *string)
{
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    spi_cmd(0x02); // Cursor home
    while (*string != '\0')
    {
        spi_data(*string);
        string++;
    }
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
}
void spi1_display2(const char *string)
{
    spi_cmd(0xC0); // Cursor second row
    while (*string != '\0')
    {
        spi_data(*string);
        string++;
    }
}
void init_spi1(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER &= ~((3 << (15 * 2)) | (3 << (5 * 2)) | (3 << (7 * 2)));
    GPIOA->MODER |= ((2 << (15 * 2)) | (2 << (5 * 2)) | (2 << (7 * 2)));
    GPIOA->AFR[0] &= ~((0xF << (5 * 4)) | (0xF << (7 * 4)));
    GPIOA->AFR[0] |= ((0x0 << (5 * 4)) | (0x0 << (7 * 4)));
    GPIOA->AFR[1] &= ~(0xF << ((15 - 8) * 4));
    GPIOA->AFR[1] |= (0x0 << ((15 - 8) * 4));

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 |= SPI_CR1_BR;
    SPI1->CR2 = SPI_CR2_DS_3 | SPI_CR2_DS_0;
    SPI1->CR1 |= SPI_CR1_MSTR;
    SPI1->CR2 |= SPI_CR2_SSOE;
    SPI1->CR2 |= SPI_CR2_NSSP;
    SPI1->CR2 |= SPI_CR2_TXDMAEN;
    SPI1->CR1 |= SPI_CR1_SPE;
}
void oled_clear(void)
{
    spi_cmd(0x01);      // Clear display
    nano_wait(2000000); // Wait for clear to complete
}

// Game Logic
void reset_game(void)
{
    current_level = 1;
    game_over = false;
    input_mode = false;
    generate_sequence(); // Only once at reset!
}

void generate_sequence(void)
{
    // Hardcoded first 4 steps for level 1:
    sequence[0] = 4; // White
    sequence[1] = 3; // Red
    sequence[2] = 2; // Green
    sequence[3] = 1; // Blue

    // Generate the rest randomly up to MAX_LEVEL
    for (int i = 4; i < MAX_LEVEL; i++)
    {
        sequence[i] = (rand() % 4) + 1;
    }
}

void play_sequence(void)
{
    char buffer[20];

    // Show Level Info
    oled_clear();
    sprintf(buffer, "Level %d:", current_level);
    spi1_display1(buffer);

    // Build sequence string like "W R G B"
    char seq_str[20] = "";
    for (int i = 0; i < current_level; i++)
    {
        switch (sequence[i])
        {
        case 1:
            strcat(seq_str, "B "); // Blue
            break;
        case 2:
            strcat(seq_str, "G "); // Green
            break;
        case 3:
            strcat(seq_str, "R "); // Red
            break;
        case 4:
            strcat(seq_str, "W "); // White
            break;
        }
    }

    spi1_display2(seq_str); // Display sequence on LCD
    delay_ms(1500);         // Pause to read sequence

    // Play LED sequence:
    for (int i = 0; i < current_level; i++)
    {
        light_led(sequence[i]); // LED ON
        delay_ms(500);          // Half-second ON

        light_led(0);  // LEDs OFF
        delay_ms(500); // Half-second OFF between steps
    }

    // Prompt player for their turn:
    oled_clear();
    spi1_display1("Your turn!");
    spi1_display2("Repeat the pattern");

    current_input = 0;
    input_mode = true;
}

void handle_user_input(void)
{
    char buffer[20];

    while (input_mode && !game_over)
    {
        uint8_t button = get_button_press();
        light_led(button); // Feedback

        // Show on OLED what the player just pressed:
        oled_clear();
        spi1_display1("You pressed:");
        switch (button)
        {
        case 1:
            spi1_display2("B");
            break;
        case 2:
            spi1_display2("G");
            break;
        case 3:
            spi1_display2("R");
            break;
        case 4:
            spi1_display2("W");
            break;
        default:
            spi1_display2("?");
            break;
        }
        delay_ms(500);

        if (button != sequence[current_input])
        {
            // Incorrect input:
            game_over = true;
            input_mode = false;
        }
        else
        {
            current_input++;
            if (current_input >= current_level)
            {
                input_mode = false; // Successfully completed this level!
                success_feedback();
                current_level++;

                // Show success message:
                oled_clear();
                spi1_display1("Good job!");
                sprintf(buffer, "Next: Level %d", current_level);
                spi1_display2(buffer);
                delay_ms(1000);
            }
        }
    }

    if (current_level > MAX_LEVEL)
    {
        win_sequence();
        game_over = true;
    }
    else if (game_over)
    {
        game_over_sequence();
    }
}
void success_feedback(void)
{
    for (int i = 0; i < 3; i++)
    {
        light_led(1);
        light_led(2);
        light_led(3);
        light_led(4);
        delay_ms(100);
        light_led(0);
        delay_ms(100);
    }
}
void game_over_sequence(void)
{
    input_mode = false;

    error_beep();
    oled_clear();
    spi1_display1("Game Over!");

    for (int i = 0; i < 5; i++)
    {
        light_led(1);
        light_led(2);
        light_led(3);
        light_led(4);
        delay_ms(100);
        light_led(0);
        delay_ms(100);
    }

    wait_for_restart();
}
void win_sequence(void)
{
    input_mode = false;

    oled_clear();
    spi1_display1("Congratulations!");
    spi1_display2("All levels passed!");

    for (int i = 0; i < 10; i++)
    {
        light_led(1);
        light_led(2);
        light_led(3);
        light_led(4);
        delay_ms(100);
        light_led(0);
        delay_ms(100);
    }

    game_over = true;
    wait_for_restart();
}
void wait_for_restart(void)
{
    while (get_button_press() == 0)
    {
        // Wait here until any button is pressed
    }
    delay_ms(300); // Debounce
}

// Diagnostic Testing
void test_gpio_pullup(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER &= ~(3U << (0 * 2)); // Input mode for PB0
    GPIOB->PUPDR &= ~(3U << (0 * 2)); // Clear PUPDR
    GPIOB->PUPDR |= (1U << (0 * 2));  // Enable pull-up

    while (1)
    {
        char buf[20];
        uint32_t val = GPIOB->IDR & 0x1; // Read PB0 only
        sprintf(buf, "PB0: %lu", val);
        oled_clear();
        spi1_display1(buf);
        delay_ms(300);
    }
}
void test_led_button_mapping(void)
{
    // LEDs OFF at startup (HIGH = OFF for active-low)
    GPIOB->ODR |= ((1U << 8) | (1U << 9) | (1U << 10) | (1U << 11));

    while (1)
    {
        if (!(BUTTON_PORT->IDR & GPIO_PIN_0)) // Button 1
            GPIOB->ODR &= ~(1U << 11);        // Blue ON
        else
            GPIOB->ODR |= (1U << 11); // Blue OFF

        if (!(BUTTON_PORT->IDR & GPIO_PIN_1)) // Button 2
            GPIOB->ODR &= ~(1U << 10);        // Green ON
        else
            GPIOB->ODR |= (1U << 10); // Green OFF

        if (!(BUTTON_PORT->IDR & GPIO_PIN_2)) // Button 3
            GPIOB->ODR &= ~(1U << 9);         // Red ON
        else
            GPIOB->ODR |= (1U << 9); // Red OFF

        if (!(BUTTON_PORT->IDR & GPIO_PIN_3)) // Button 4
            GPIOB->ODR &= ~(1U << 8);         // White ON
        else
            GPIOB->ODR |= (1U << 8); // White OFF

        delay_ms(10);
    }
}

// MAIN Function
int main(void)
{
    internal_clock();
    GPIO_Configure();
    init_spi1();
    spi1_init_oled();

    reset_game(); // 👈 Generate sequence ONCE here, at the start!

    while (1) // 🔥 FULL GAME LOOP
    {
        // Only show this "start" message if it's a fresh game:
        oled_clear();
        spi1_display1("Press button 1");
        spi1_display2("(blue) to start!");

        uint8_t button_pressed = 0;

        // Wait until BUTTON 1 is pressed:
        while (button_pressed == 0)
        {
            for (uint8_t i = 1; i <= 4; i++)
            {
                if (check_button(i))
                {
                    button_pressed = i;
                    while (check_button(i))
                        ; // Wait for release
                    delay_ms(50);
                    break;
                }
            }
        }

        if (button_pressed == 1)
        {
            char buf[20];
            oled_clear();
            sprintf(buf, "Button %d Pressed", button_pressed);
            spi1_display1(buf);
            delay_ms(500);

            // ✅ Game STARTS, now looping through levels:
            game_over = false;
            current_level = 1; // Set level 1 here!

            while (!game_over && current_level <= MAX_LEVEL)
            {
                // Show level and countdown:
                oled_clear();
                char level_buf[20];
                sprintf(level_buf, "Level %d", current_level);
                spi1_display1(level_buf);
                spi1_display2("Get Ready!");
                delay_ms(1000);

                for (int i = 3; i >= 1; i--)
                {
                    char count_buf[20];
                    sprintf(count_buf, "Starting in: %d", i);
                    oled_clear();
                    spi1_display1(count_buf);
                    delay_ms(1000);
                }

                oled_clear();
                spi1_display1("GO!");
                delay_ms(500);

                // 🟢 Play sequence and handle input:
                play_sequence();
                handle_user_input();

                // Check after input if player won all levels:
                if (current_level > MAX_LEVEL)
                {
                    win_sequence();
                    game_over = true;
                }
                else if (game_over)
                {
                    game_over_sequence();
                }
            }
        }
        else
        {
            oled_clear();
            spi1_display1("Wrong button!");
            delay_ms(1000);
        }
        // No reset_game() here — it should only happen at the top after the full game finishes!
    }
}
