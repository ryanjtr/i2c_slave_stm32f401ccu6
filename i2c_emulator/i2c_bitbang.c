#include "i2c_bitbang.h"

/*Important: When using I2C bitbanging, you have to configure I2C GPIO in ioc
 * to input pull-up resistor
 */

/**
 * The function `DWT_Clock_Enable` enables the DWT cycle counter if it is not already enabled.
 */
void DWT_Clock_Enable(void)
{
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk))
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Bật Trace
        DWT->CYCCNT = 0;                                // Reset bộ đếm
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // Bật bộ đếm chu kỳ
    }
}

void I2C_Bitbang_Init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

    /*Configure SDA pin to input interrupt falling edge
     * after detect start condition, then disable interrupt.
     * When receive 8 bit address, change it to output open-drain float
     */

    /*Configure SCL pin as input first
     * after detecting start condition, then change it to interrupt rising edge (input)
     */

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE7);
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE6);

    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_7);
    LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_7);

    /* Configure GPIO*/

    LL_GPIO_SetPinMode(I2C_GPIO_PORT, I2C_SDA_PIN, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(I2C_GPIO_PORT, I2C_SDA_PIN, LL_GPIO_PULL_NO);

    /*Configure GPIO*/
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_6);
    LL_GPIO_SetPinMode(I2C_GPIO_PORT, I2C_SCL_PIN, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(I2C_GPIO_PORT, I2C_SCL_PIN, LL_GPIO_PULL_NO);

    NVIC_SetPriority(EXTI9_5_IRQn, 0);
    NVIC_EnableIRQ(EXTI9_5_IRQn);

    //    // Cấu hình chân SCL và SDA ở chế độ Open-Drain
    //    LL_GPIO_SetPinMode(I2C_GPIO_PORT, I2C_SCL_PIN, LL_GPIO_MODE_ALTERNATE);
    //    LL_GPIO_SetAFPin_0_7(I2C_GPIO_PORT, I2C_SCL_PIN, LL_GPIO_AF_4); // AF4 cho I2C1
    //    LL_GPIO_SetPinOutputType(I2C_GPIO_PORT, I2C_SCL_PIN, LL_GPIO_OUTPUT_OPENDRAIN);
    //    LL_GPIO_SetPinSpeed(I2C_GPIO_PORT, I2C_SCL_PIN, LL_GPIO_SPEED_FREQ_HIGH);
    //    LL_GPIO_SetPinPull(I2C_GPIO_PORT, I2C_SCL_PIN, LL_GPIO_PULL_UP);

    //    LL_GPIO_SetPinMode(I2C_GPIO_PORT, I2C_SDA_PIN, LL_GPIO_MODE_INPUT);
    //    LL_GPIO_SetPinPull(I2C_GPIO_PORT, I2C_SDA_PIN, LL_GPIO_PULL_NO);
    //    LL_GPIO_SetPinSpeed(I2C_GPIO_PORT, I2C_SDA_PIN, LL_GPIO_SPEED_FREQ_HIGH);
    //
    //    // Cấu hình ngắt ngoại vi cho chân SCL
    //    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE6);
    //    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_6);         // Bật ngắt cho line 6 (SCL)
    //    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_6); // Bắt đầu kích hoạt cạnh lên của SCL
    //                                                   //    LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_6); // Bắt đầu kích hoạt cạnh xuống của SCL

    //    // Bật ngắt EXTI tại NVIC cho line 6
    //    NVIC_SetPriority(EXTI9_5_IRQn, 0);
    //    NVIC_EnableIRQ(EXTI9_5_IRQn);
}
/**
 * @brief  This function provides a delay (in microseconds)
 * @param  microseconds: delay in microseconds
 */
void DWT_Delay_us(volatile uint32_t microseconds)
{
    uint32_t clk_cycle_start = DWT->CYCCNT;
    microseconds *= (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - clk_cycle_start) < microseconds)
        ;
}

/**
 * The function sets the I2C SCL pin to a high logic level.
 */
__STATIC_INLINE void I2C_SCL_High(void)
{
    LL_GPIO_SetOutputPin(I2C_GPIO_PORT, I2C_SCL_PIN);
}

/**
 * The function `I2C_SCL_Low` sets the I2C serial clock (SCL) pin to a low state.
 */
__STATIC_INLINE void I2C_SCL_Low(void)
{
    LL_GPIO_ResetOutputPin(I2C_GPIO_PORT, I2C_SCL_PIN);
}

/**
 * The function sets the I2C SDA pin to a high logic level.
 */
__STATIC_INLINE void I2C_SDA_High(void)
{
    LL_GPIO_SetOutputPin(I2C_GPIO_PORT, I2C_SDA_PIN);
}

/**
 * @brief
 *
 */
__STATIC_INLINE void I2C_SDA_Low(void)
{
    LL_GPIO_ResetOutputPin(I2C_GPIO_PORT, I2C_SDA_PIN);
}

/**
 * The function `I2C_Read_SDA` reads the state of the SDA pin in an I2C communication.
 *
 * @return The function `I2C_Read_SDA` is returning the state of the SDA pin, which is read using the
 * `LL_GPIO_IsInputPinSet` function. The state is a `uint32_t` value indicating whether the SDA pin is
 * high or low.
 */
__STATIC_INLINE unsigned char I2C_Read_SDA(void)
{
    unsigned char state = LL_GPIO_IsInputPinSet(I2C_GPIO_PORT, I2C_SDA_PIN);
    return state;
}

/**
 * The function `I2C_Read_SCL` reads the state of the SCL pin in an I2C communication.
 *
 * @return The function `I2C_Read_SCL` is returning the current state of the I2C SCL (clock) pin. It
 * sets the pin mode to input, reads the state of the pin, and then returns that state.
 */
__STATIC_INLINE unsigned char I2C_Read_SCL(void)
{
    unsigned char state = LL_GPIO_IsInputPinSet(I2C_GPIO_PORT, I2C_SCL_PIN);
    return state;
}

__STATIC_INLINE void I2C_Write_Bit(bool bit)
{
    if (bit)
    {
        LL_GPIO_SetOutputPin(I2C_GPIO_PORT, I2C_SDA_PIN);
    }
    else
    {
        LL_GPIO_ResetOutputPin(I2C_GPIO_PORT, I2C_SDA_PIN);
    }
    DWT_Delay_us(2); // Đợi ổn định
}

__STATIC_INLINE unsigned char I2C_Read_Bit(void)
{
    unsigned char bit = LL_GPIO_IsInputPinSet(I2C_GPIO_PORT, I2C_SDA_PIN);
    return bit;
}
__STATIC_INLINE void I2C_Send_NACK(void)
{
    I2C_SCL_High();
}
__STATIC_INLINE void I2C_Send_ACK(void)
{
    I2C_SCL_Low();
}

/**
 * The function `I2C_Write_Byte` writes a byte of data over I2C communication by sending each bit
 * sequentially and then reading an acknowledgment bit.
 *
 * @param data The `data` parameter in the `I2C_Write_Byte` function is an unsigned char variable that
 * represents the byte of data to be written over the I2C communication protocol. The function writes
 * this byte of data bit by bit, starting from the most significant bit (MSB) and
 *
 * @return The function `I2C_Write_Byte` is returning the result of the function `I2C_Read_Bit()`,
 * which is either an ACK (acknowledgment) or NACK (not acknowledged) signal.
 */
bool I2C_Write_Byte(unsigned char data)
{
    unsigned char index;
    for (index = 0; index < 8; index++)
    {
        I2C_Write_Bit((data & 0x80) != 0); // MSB first
        data <<= 1;
    }
    return I2C_Read_Bit(); // return ACK or NACK
}

/**
 * The function `I2C_Read_Byte` reads a byte of data using I2C communication protocol.
 *
 * @param buffer The `buffer` parameter in the `I2C_Read_Byte` function is used to store the byte read
 * from the I2C bus. It is initialized to 0 and then each bit read from the bus is shifted into this
 * buffer byte.
 */
__STATIC_INLINE unsigned char I2C_Read_Byte()
{
    unsigned char buffer = 0;
    unsigned char index;
    for (index = 0; index < 8; index++)
    {
        buffer = (buffer << 1) | I2C_Read_Bit();
    }
    return buffer;
}

__STATIC_INLINE void i2c_set_sda_opendrain()
{
    LL_GPIO_SetPinMode(I2C_GPIO_PORT, I2C_SDA_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(I2C_GPIO_PORT, I2C_SDA_PIN, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinSpeed(I2C_GPIO_PORT, I2C_SDA_PIN, LL_GPIO_SPEED_FREQ_HIGH);
}

__STATIC_INLINE void i2c_set_scl_opendrain()
{
    LL_GPIO_SetPinMode(I2C_GPIO_PORT, I2C_SCL_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(I2C_GPIO_PORT, I2C_SCL_PIN, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinSpeed(I2C_GPIO_PORT, I2C_SCL_PIN, LL_GPIO_SPEED_FREQ_HIGH);
}

__STATIC_INLINE void i2c_set_sda_input()
{
    LL_GPIO_SetPinMode(I2C_GPIO_PORT, I2C_SDA_PIN, LL_GPIO_MODE_INPUT);
}

__STATIC_INLINE void i2c_enable_sda_falling()
{
    LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_7);
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_7);
}

__STATIC_INLINE void i2c_enable_scl_rising()
{
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_6);
}

__STATIC_INLINE void i2c_disable_scl_rising()
{
    LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_6);
    LL_GPIO_SetPinPull(I2C_GPIO_PORT, I2C_SCL_PIN, LL_GPIO_PULL_NO);
}

__STATIC_INLINE void i2c_disable_sda_falling()
{
    LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_7);
}

// Bring to file.h later
typedef enum
{
    I2C_IDLE,
    I2C_ADDRESS_RECEIVING,
    I2C_SET_SDA_INPUT_ONLY,
    I2C_DATA_RECEIVING,
} I2C_State;

I2C_State i2c_state = I2C_IDLE;

uint8_t count_bit = 0;
unsigned char Slave_Address = 0x00;
unsigned char Slave_rxdata[10] = {0};
unsigned char index_rxdata = 0;
bool start_condtion = false;
unsigned char bit;

void check_start_condition()
{
    if (I2C_Read_SCL() && !I2C_Read_SDA())
    {
        start_condtion = true;
        i2c_state = I2C_ADDRESS_RECEIVING;
        i2c_disable_sda_falling();
        i2c_enable_scl_rising();
    }
}

void I2C_Event_Take()
{

    bit = I2C_Read_Bit();
    if (start_condtion)
    {
        switch (i2c_state)
        {
        case I2C_IDLE:
            break;
        case I2C_ADDRESS_RECEIVING:
            Slave_Address = (Slave_Address << 1) | bit;
            if (++count_bit == 8)
            {
                i2c_set_sda_opendrain();
                if (Slave_Address >> 1 == 0x55)
                {
                    I2C_SDA_Low(); // Send ACK
                    i2c_state = I2C_SET_SDA_INPUT_ONLY;
                    count_bit = 0;
                }
                else
                {
                    I2C_SDA_High();
                    i2c_state = I2C_IDLE;
                    count_bit = 0;
                    start_condtion = false;
                    i2c_disable_scl_rising();
                    i2c_enable_sda_falling();
                    Slave_Address = 0x00;
                    for (int i = 0; i < 3; i++)
                    {
                        Slave_rxdata[i] = 0;
                    }
                }
            }
            break;
        case I2C_SET_SDA_INPUT_ONLY:
            i2c_set_sda_input();
            i2c_state = I2C_DATA_RECEIVING;
            break;
        case I2C_DATA_RECEIVING:
            Slave_rxdata[index_rxdata] = (Slave_rxdata[index_rxdata] << 1) | bit;
            if (++count_bit % 8 == 0)
            {
                i2c_set_sda_opendrain();
                I2C_SDA_Low(); // Send ACK
                DWT_Delay_us(150);
                I2C_SDA_High();
                i2c_set_sda_input();
                if (++index_rxdata == 8)//receive n-1 byte, here n=9
                {
                    count_bit = 0;
                    index_rxdata = 0;
                    start_condtion = false;
                    uart_printf("add=0x%02X\r\n", Slave_Address >> 1);
                    for (int i = 0; i <= 7; i++)
                    {
                        uart_printf("d%d=0x%02X\r\n", i, Slave_rxdata[i]);
                        Slave_rxdata[i] = 0;
                    }
                    i2c_disable_scl_rising();
                    i2c_enable_sda_falling();
                }
            }
            break;
        default:
            break;
        }
    }
}
