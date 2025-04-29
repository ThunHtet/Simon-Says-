void init_adc(void);
void init_dac();
void init_usart5();                                                                                                                    
void init_wavetable(void);
void init_tim6();
void TIM6_DAC_IRQHandler();

//func headers
void internal_clock();
void error_beep(); //output a beep sound for a couple seconds, then turn it off
uint16_t read_adc(void); //read adc value(pot for vol)
void set_volume(uint16_t adc_value); //set volume based on potentiometer