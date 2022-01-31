#include <stdint.h>
#include "LPC407x_8x_177x_8x.h"
#define PRESCALE ((PeripheralClock/1000000) - 1) // 1 MHz frequency -> tick every microsecond
#define LED_PERIOD 250000
#define GPIO_POLL_PERIOD 200000
#define US_POLL_PERIOD 150000
#define PERIOD 5000
#define PERIOD_FLOAT 5000.
#define ADC_MAX 0xFFF
#define VALMASK (4095 << 4)
#define MOTOR1_POWER 3500
#define MOTOR2_POWER 5000
#define ADC_COEFFICIENT_MOTOR1 8 * PERIOD_FLOAT / ADC_MAX
#define ADC_COEFFICIENT_MOTOR2 18 * PERIOD_FLOAT / ADC_MAX

const unsigned int RPM = 150; // must be a value between 0 and 120!
static float duty_cycle;
static unsigned int MOTOR1_ONTIME; 
static unsigned int MOTOR2_ONTIME;
// INTERRUPT FLAGS:
	// 1: take GPIO reading
	// 2: timer 2-related interrupt: LED or motors
static unsigned int interrupt_flag = 0;
static unsigned int scenario = 0;
// DIRECTION FLAGS:
	// 0: forward
	// 1: right
	// 2: backward
	// 3: left
	// 4: stopped
static unsigned int direction = 1;
volatile uint32_t rising_edge, pulse_width = 0;
static unsigned int ic_pin = 0;
static unsigned int external_stop = 0;

void get_pulse_width(void)
{
	switch(ic_pin)
	{
		case 0:
			ic_pin = 1;
			rising_edge = LPC_TIM3->CR0;
			break;
		case 1:
			ic_pin = 0;
			pulse_width = LPC_TIM3->CR0 - rising_edge;
			break;
	}
}

void stop(void) 
{
	LPC_GPIO1->PIN = 0;
	direction = 4;
}

void go_forward(void) 
{
	LPC_GPIO1->PIN = (1 << 3 | 1 << 12 | 1 << 23 | 1 << 30);
	LPC_TIM2->MR0 = 0;
	direction = 0;
}

void reverse(void) 
{
	LPC_GPIO1->PIN = (1 << 6 | 1 << 7 | 1 << 20 | 1 << 24);
	LPC_TIM2->MR0 = 0;
	direction = 2;
}

void turn_right(void) 
{
	LPC_GPIO1->PIN = (1 << 12 | 1 << 6 | 1 << 20 | 1 << 23);
	LPC_TIM2->MR0 = LED_PERIOD;
	direction = 1;
}

void turn_left(void) 
{
	LPC_GPIO1->PIN = (1 << 3 | 1 << 7 | 1 << 24 | 1 << 30);
	LPC_TIM2->MR0 = LED_PERIOD;
	direction = 3;
}

void TIMER0_IRQHandler(void) 
{
	// set interrupt flag 1 to indicate taking a GPIO reading
	interrupt_flag = 1;
	LPC_TIM0->IR |= 1;	
}

void TIMER1_IRQHandler(void) 
{
	// set interrupt flag 2 to indicate taking a US reading
	interrupt_flag = 2; 
	LPC_TIM1->IR |= 1;
}

void TIMER3_IRQHandler(void) 
{
	// set interrupt flag 3 to update pulsewidth value and take LDR reading in autonomous mode
	get_pulse_width();
	LPC_TIM3->IR |= 4;
}

void GPIO_Init(void) 
{
	// give power to GPIO
	LPC_SC->PCONP |= (1 << 15);
	// configure GPIO pins to GPIO functionality
		// motor control 
	LPC_IOCON->P1_24 &= ~7;
	LPC_IOCON->P1_23 &= ~7;
	LPC_IOCON->P1_20 &= ~7;
	LPC_IOCON->P1_30 &= ~7;
		// push button
	LPC_IOCON->P2_10 &= ~7;
		// LEDs
	LPC_IOCON->P1_7 &= ~7;
	LPC_IOCON->P1_6 &= ~7;
	LPC_IOCON->P1_12 &= ~7;
	LPC_IOCON->P1_3 &= ~7;
		// joystick switches
	LPC_IOCON->P5_3 &= ~7;
	LPC_IOCON->P5_2 &= ~7;
	LPC_IOCON->P5_4 &= ~7;
	LPC_IOCON->P5_0 &= ~7;
	LPC_IOCON->P5_1 &= ~7;
		// invert joystick pins' and push-button's polarities for ease of reading
	LPC_IOCON->P2_10 |= (1 << 6);
	LPC_IOCON->P5_3 |= (1 << 6);
	LPC_IOCON->P5_2 |= (1 << 6);
	LPC_IOCON->P5_4 |= (1 << 6);
	LPC_IOCON->P5_0 |= (1 << 6);
	LPC_IOCON->P5_1 |= (1 << 6);
	
	// set directions for GPIO pins
		// set motor control and LED pins as output
	LPC_GPIO1->DIR |= (1 << 3 | 3 << 6 | 1 << 20 | 1 << 12 | 3 << 23 | 1 << 30);
		// set joystick and push button as input
	LPC_GPIO2->DIR &= ~(1 << 10);
	LPC_GPIO5->DIR &= ~31;
}

void Timer_Init(void) 
{
	// turn on timers 0, 1, 2, 3
	LPC_SC->PCONP |= (1 << 1 | 1 << 2 | 1 << 22 | 1 << 23);

	// set the timer match pins
	LPC_IOCON->P0_9 |= 3;
	LPC_IOCON->P0_6 |= 3;
	
	// set the timer cap pin
	LPC_IOCON->P0_23 |= 3;
	
	// disable and reset all timers
	LPC_TIM0->TCR = 2;
	LPC_TIM2->TCR = 2;
	LPC_TIM1->TCR = 2;
	LPC_TIM3->TCR = 2;
	
	// set all timers to tick every microsecond for ease of calculation
	LPC_TIM0->PR = PRESCALE;
	LPC_TIM2->PR = PRESCALE;
	LPC_TIM1->PR = PRESCALE;
	LPC_TIM3->PR = PRESCALE;
	
	// take GPIO reading every 250 ms, raise interrupt to take reading
	LPC_TIM0->MR0 = GPIO_POLL_PERIOD;
	LPC_TIM0->MCR |= 3; 
	
	// take US reading every 150 ms, raise interrupt to take reading
	LPC_TIM1->MR0 = US_POLL_PERIOD;
	LPC_TIM1->MCR |= 3; 
	
	// capture raises interrupt on rising and falling edges
	LPC_TIM3->CCR |= 7;
	
	// reset on match, initial value high, toggle
	LPC_TIM2->MR0 = 0;
	LPC_TIM2->MCR |= (1 << 1);
	LPC_TIM2->EMR |= (1 | 3 << 4);
	
	// enable timer 0, 1, 3 interrupts from NVIC
	__NVIC_ClearPendingIRQ(TIMER0_IRQn);
	__NVIC_EnableIRQ(TIMER0_IRQn);
	__NVIC_ClearPendingIRQ(TIMER1_IRQn);
	__NVIC_EnableIRQ(TIMER1_IRQn);
	__NVIC_ClearPendingIRQ(TIMER3_IRQn);
	__NVIC_EnableIRQ(TIMER3_IRQn);
	
	// remove reset and enable timers 0, 1, 2, 3
	LPC_TIM0->TCR = LPC_TIM2->TCR = LPC_TIM1->TCR = LPC_TIM3->TCR = 1;
	
}

void ADC_Init(void)
{
	// calculate duty cycle and set motor on-times
	duty_cycle = RPM / 171.;
	MOTOR1_ONTIME = MOTOR1_POWER * duty_cycle;
	MOTOR2_ONTIME = MOTOR2_POWER * duty_cycle;
	
	// give power to ADC
	LPC_SC->PCONP |= (1 << 12);
	
	// set pins 16 and 17 to ADC, set mode to inactive and enable analog mode
	LPC_IOCON->P0_24 |= 1;
	LPC_IOCON->P0_24 &= ~(3 << 3 | 1 << 7);
	LPC_IOCON->P0_25 |= 1;
	LPC_IOCON->P0_25 &= ~(3 << 3 | 1 << 7);
	
	// disable ADC interrupts
	LPC_ADC->INTEN &= ~(1 << 8);
	
	// set ADC clock to 1 MHz
	LPC_ADC->CR = (PRESCALE << 8); // set ADC clock freq to 1 MHz
	
	LPC_ADC->CR |= (1 << 16);
	
	// enable sampling on pins 1 and 2
	LPC_ADC->CR |= (3 << 1);
	
	// make ADC operational
	LPC_ADC->CR |= (1 << 21);
}

void PWM_Init(void)
{
	int local_motor1_ontime = MOTOR1_ONTIME;
	int local_motor2_ontime = MOTOR2_ONTIME;
	
	// give power to PWM0
	LPC_SC->PCONP |= (1 << 5);
	
	// enable PWM channels
	LPC_PWM0->PCR |= (1 << 9 | 1 << 11 | 1 << 14);
	
	// reset and disable PWM0
	LPC_PWM0->TCR = 2;
	
	// set pins 25, 28, 30 to PWM
	LPC_IOCON->P1_2 |= 3;
	LPC_IOCON->P1_11 |= 3;
	LPC_IOCON->P1_5 |= 3;
	
	// set to tick every microsecond
	LPC_PWM0->PR = PRESCALE;
	
	LPC_PWM0->MR0 = PERIOD;
	LPC_PWM0->MR1 = local_motor1_ontime;
	LPC_PWM0->MR3 = 10; // send square pulse of 10 microsecond
	LPC_PWM0->MR6 = local_motor2_ontime;
	
	// reset TC when MR0 matches
	LPC_PWM0->MCR |= (1 << 1);
	
	// make sure MR registers get updated
	LPC_PWM0->LER |= (1 | 1 << 1 | 1 << 3 | 1 << 6);
	
	// remove reset, start timer and enable PWM
	LPC_PWM0->TCR = 9;
	
}

void change_orientation(int reading)
{
	switch(reading)
	{
		case 1: // JOYSTICK-LEFT
			turn_left();
			break;
		case 2: // JOYSTICK-DOWN
			reverse();
			break;
		case 4: // JOYSTICK-UP
			go_forward();
			break;
		case 8: // JOYSTICK-CENTER
			stop();
			break;
		case 16: // JOYSTICK-RIGHT
			turn_right();
			break;
		case 1024: // PUSH-BUTTON
			// get current scenario and switch it
			scenario = !scenario;
			stop();
			break;
	}
}

void change_orientation_auto(int reading)
{
	switch(reading)
	{
		case 4: // JOYSTICK-UP
			go_forward();
			external_stop = 0;
			break;
		case 8: // JOYSTICK-CENTER
			stop();
			external_stop = 1;
			break;
		case 1024: // PUSH-BUTTON
			// get current scenario and switch it
			stop();
			scenario = !scenario;
			if (!scenario) // if switched to manual, set the motor speeds back up
			{
				LPC_PWM0->MR6 = MOTOR2_ONTIME;
				LPC_PWM0->MR1 = MOTOR1_ONTIME;
				LPC_PWM0->LER |= (1 << 1 | 1 << 6);
			}
			break;
	}
}

void update(void) 
{
	int local_interrupt_flag = interrupt_flag;
	int reading;
	float us_reading;
	switch(local_interrupt_flag)
	{
		case 2: // must take a US reading
			us_reading = pulse_width / 116;
			if(!direction && us_reading < 12)
			{
				stop();
			}
			break;
		case 1: // must take a GPIO reading!
			reading = (LPC_GPIO5->PIN & 31) | (LPC_GPIO2->PIN & 1024);
			if (reading != 0)
			{	
				change_orientation(reading);
			}
			if (pulse_width > 0x6500)
			{
				pulse_width = rising_edge = 0;
			}
			break;
	}
}

void update_autonomous(void) 
{
	int left_flag = 0;
	int local_interrupt_flag = interrupt_flag;
	int reading;
	int ldr_left, ldr_right;
	float us_reading;
	switch(local_interrupt_flag)
	{
		case 2: // must take a US reading
			us_reading = pulse_width / 116;
			while (us_reading < 13)
			{
				stop();
				us_reading = pulse_width / 116;
				left_flag = 1;
			}
			if (left_flag && !external_stop)
			{
				left_flag = 0;
				go_forward();
			}
			// update motor speeds
			if (direction != 4)
			{
				ldr_left = ((LPC_ADC->DR[1] & VALMASK) >> 4);
				ldr_right = ((LPC_ADC->DR[2] & VALMASK) >> 4);
				if (ldr_left > ldr_right)
				{
					LPC_PWM0->MR6 = ADC_COEFFICIENT_MOTOR2 * ldr_left;
					LPC_PWM0->MR1 = .8 * ADC_COEFFICIENT_MOTOR1 * ldr_right;
					LPC_PWM0->LER |= (1 << 1 | 1 << 6);
					LPC_GPIO1->PIN = (1 << 12 | 1 << 6 | 1 << 23);
					LPC_TIM2->MR0 = LED_PERIOD;
				}
				else
				{
					LPC_PWM0->MR6 = .8 * ADC_COEFFICIENT_MOTOR2 * ldr_left;
					LPC_PWM0->MR1 = ADC_COEFFICIENT_MOTOR1 * ldr_right;
					LPC_PWM0->LER |= (1 << 1 | 1 << 6);
					LPC_GPIO1->PIN = (1 << 3 | 1 << 7 | 1 << 30);
					LPC_TIM2->MR0 = LED_PERIOD;
				}
			}
			break;
		case 1: // must take a GPIO reading!
			reading = (LPC_GPIO5->PIN & 31) | (LPC_GPIO2->PIN & 1024);
			if (reading != 0)
			{	
				change_orientation_auto(reading);
			}
			if (pulse_width > 0x6500)
			{
				pulse_width = rising_edge = 0;
			}
			break;
	}
}

int main() 
{
	ADC_Init();
	GPIO_Init();
	Timer_Init();
	PWM_Init();
	__enable_irq();
	while(1) 
	{
		int local_scenario = scenario;
		switch(local_scenario) 
		{
			case 0: // manual
				update();
				break;
			case 1: // autonomous
				update_autonomous();
				break;
		}
		__WFI();
	}
}
