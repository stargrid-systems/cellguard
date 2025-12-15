pub struct CellagentCore {
    _private: (),
}

impl CellagentCore {}

// Required drivers:
// - ADC (AC)
// - GPIO
// - PWM (TCA)
// - UART (USART)

// TODO:
// - read temperature
// - control pwm for balancing
// - monitor 3v3 power supply
// - output alive signal
// - active balancer on signal??
// - out tiny all off??
// - monitor MCU alive signal??
