/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : Brandon
 * Version            : V1.0.0
 * Date               : 2023/06/28
 * Description        : Main program body.
*********************************************************************************
* Brandon was here.
*******************************************************************************/

#include "ch32v00x_conf.h"

/* Global define */
#define C_FR (7u) // led flash rate, set at 7.  don't change for this clock spd
#define C_BM (4u) // led brightness modifier, valid between 0 and like 5
#define C_LED_OFFSET (2u)
// led heartbeat calculation
#define C_LED_HB(var) (((var) & (1u << (C_FR + 2u))) && ((var) & (1u << C_FR)))
// led heartbeat duty cycle calculation
#define C_LED_DIM_MASK ((1u << (2u + C_BM)) - 1u)
#define C_LED_DIM(var) (((var) >> (5u - C_BM)) & C_LED_DIM_MASK)
#define C_LED_DIM_ON(var) (C_LED_DIM(var) == 1u)
#define C_LED_DIM_OFF(var) (C_LED_DIM(var) == 2u)
#define C_LED_BR(var) ((C_LED_HB(var) && C_LED_DIM_ON(var)) << C_LED_OFFSET)
#define C_LED_BS(var) ((C_LED_HB(var) && C_LED_DIM_OFF(var)) << C_LED_OFFSET)
#define C_LED_BSHR(var) (C_LED_BR(var) << 16u | C_LED_BS(var))


/* Global Variable */
volatile uint32_t main_counter = 0;
volatile uint8_t main_flag = 0;

/* file-local variables */
static uint8_t t2count = 0;

/*********************************************************************
 * @fn      TIM2_IRQHandler
 *
 * @brief   This function handles timer 2
 *
 * @return  none
 */
__attribute__((interrupt, __used__)) void TIM2_IRQHandler(void)
{
    // should only receive the update flag.  ignore other flags.
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        t2count++;
        if (t2count >= 20)
        {
            t2count = 0;
            main_counter++;
            main_flag = 1;
        }
    }
    return;
}

/*********************************************************************
 * @fn      USER_GPIO_SETUP
 *
 * @brief   Set up GPIO/AFIO
 *
 * @return  none
 */
void USER_GPIO_SETUP(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(
            RCC_APB2Periph_GPIOA |
            RCC_APB2Periph_GPIOC |
            RCC_APB2Periph_GPIOD |
            RCC_APB2Periph_AFIO,
            ENABLE);

    // pin 1 on the package
    // this pin is used as the simplex USART
    // set pin high ahead of time
    GPIOD->BSHR = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    // this pin is connected to the same external pin.
    // we use it as a pull-up before the hard resistor is placed.
    // maybe we can remove this in hardware revision b
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinRemapConfig(GPIO_PartialRemap2_USART1, ENABLE);

    // pin 2 is ground power

    // pin 3 on the package, modbus direction pin
    GPIOA->BCR = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // pin 4 is five volts power

    // pin 5 is pwm output
    GPIOC->BSHR = GPIO_Pin_1; // turn it on for fan startup
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);

    // pin 6 is the LED indicator output
    GPIOC->BCR = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // pin 7 is the analog input
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = 0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // pin 8 is SWIO.  Don't touch it.
    return;
}

/*********************************************************************
 * @fn      USER_TIMER_SETUP
 *
 * @brief   Set up the timer
 *
 * @return  none
 */
void USER_TIMER_SETUP(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};
    TIM_OCInitTypeDef TIM_OCInitStructure = {0};

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM2, DISABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // assume that the GPIO setup has already remapped us

    // assume that we are using the 24MHz HSI oscillator
    // assume that HPRE is set to no division
    // target: 240 cycles * 10kHz (timer top rate)
    // this means that the prescaler division rate should be set to 10
    // which means that the register should be set to 9
    // the timer period should be 240 counts, so the period
    // is set to 239
    TIM_TimeBaseInitStructure.TIM_Period = 119;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 9;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 60;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);

    TIM_CtrlPWMOutputs(TIM2, ENABLE);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_InternalClockConfig(TIM2);
    TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);

    TIM_Cmd(TIM2, ENABLE);
    TIM_GenerateEvent(TIM2, TIM_EventSource_Update);
    return;
}

/*********************************************************************
 * @fn      USER_ADC_SETUP
 *
 * @brief   Set up the adc
 *
 * @return  none
 */
void USER_ADC_SETUP(void)
{
    return;
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    uint8_t ie_flag = 0;
    // SystemInit is called by the startup assembly, so we do not need to call
    NVIC_InitTypeDef NVIC_InitStructure = {0};
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    USER_GPIO_SETUP();
    USER_TIMER_SETUP();
    USER_ADC_SETUP();
    while(1)
    {
        if (main_flag != 0)
        {
            main_flag = 0;
            // rather obtuse code to toggle the LED
            // the branchless code here is fun to write + utterly unnecessary
            // led_hb - a "heartbeat pattern" generator
            // led_dimmer - generates a shorter period for the LED to be lit
            GPIOC->BSHR = C_LED_BSHR(main_counter);
            // sleep until the next interrupt I guess
        }
        if (ie_flag == 0)
        {
            ie_flag = 1;
            GPIOC->BSHR = GPIO_Pin_2;
        }
        __WFI();
        // for some reason, WFI here causes the system to hang
    }
}
