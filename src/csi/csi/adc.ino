#define FILTER 3

// The most recent filtered ADC values
volatile unsigned long adc_val[6] = {0,0,0,0,0,0};
volatile byte adc_pin;

unsigned long get_adc(byte i) { return adc_val[i]; }

void adc_init() {
    // Start the first ADC conversion
    adc_pin = 0;
    ADMUX |= B01000000; ADMUX &= B11010000; ADCSRA |= B11101111;
}

// Called when an analog-digital conversion completes
ISR(ADC_vect)
{
    // Record and amplify the value.
    unsigned long analogVal = (ADCL | (ADCH << 8)) << 4;
    
    // Add the new value to the rolling exponential buffer for that pin.
    adc_val[adc_pin] += (analogVal >> FILTER) - (adc_val[adc_pin] >> FILTER);
    
    // Set the next ADC conversion to happen.
    adc_pin = (adc_pin + 1) % 2;
    
    // Start the next analog-digital conversion
    ADMUX &= B11110000; ADMUX |= adc_pin; ADCSRA |= B01000000;
}
