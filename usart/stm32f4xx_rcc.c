#include "config.h"

#if defined(STM32F4) && defined(ENABLE_USART)

#ifndef assert_param
#define assert_param(x)
#endif

#include "stm32f4xx_rcc.h"

static __I uint8_t APBAHBPrescTable[16] = {0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};

/**
  * @brief  Forces or releases Low Speed APB (APB1) peripheral reset.
  * @param  RCC_APB1Periph: specifies the APB1 peripheral to reset.
  *          This parameter can be any combination of the following values:
  *            @arg RCC_APB1Periph_TIM2:   TIM2 clock
  *            @arg RCC_APB1Periph_TIM3:   TIM3 clock
  *            @arg RCC_APB1Periph_TIM4:   TIM4 clock
  *            @arg RCC_APB1Periph_TIM5:   TIM5 clock
  *            @arg RCC_APB1Periph_TIM6:   TIM6 clock
  *            @arg RCC_APB1Periph_TIM7:   TIM7 clock
  *            @arg RCC_APB1Periph_TIM12:  TIM12 clock
  *            @arg RCC_APB1Periph_TIM13:  TIM13 clock
  *            @arg RCC_APB1Periph_TIM14:  TIM14 clock
  *            @arg RCC_APB1Periph_LPTIM1: LPTIM1 clock (STM32F410xx and STM32F413_423xx devices) 
  *            @arg RCC_APB1Periph_WWDG:   WWDG clock
  *            @arg RCC_APB1Periph_SPI2:   SPI2 clock
  *            @arg RCC_APB1Periph_SPI3:   SPI3 clock
  *            @arg RCC_APB1Periph_SPDIF:  SPDIF RX clock (STM32F446xx devices) 
  *            @arg RCC_APB1Periph_USART2: USART2 clock
  *            @arg RCC_APB1Periph_USART3: USART3 clock
  *            @arg RCC_APB1Periph_UART4:  UART4 clock
  *            @arg RCC_APB1Periph_UART5:  UART5 clock
  *            @arg RCC_APB1Periph_I2C1:   I2C1 clock
  *            @arg RCC_APB1Periph_I2C2:   I2C2 clock
  *            @arg RCC_APB1Periph_I2C3:   I2C3 clock
  *            @arg RCC_APB1Periph_FMPI2C1:FMPI2C1 clock
  *            @arg RCC_APB1Periph_CAN1:   CAN1 clock
  *            @arg RCC_APB1Periph_CAN2:   CAN2 clock
  *            @arg RCC_APB1Periph_CEC:    CEC clock(STM32F446xx devices)
  *            @arg RCC_APB1Periph_PWR:    PWR clock
  *            @arg RCC_APB1Periph_DAC:    DAC clock
  *            @arg RCC_APB1Periph_UART7:  UART7 clock
  *            @arg RCC_APB1Periph_UART8:  UART8 clock  
  * @param  NewState: new state of the specified peripheral reset.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void RCC_APB1PeriphResetCmd(uint32_t RCC_APB1Periph, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_RCC_APB1_PERIPH(RCC_APB1Periph));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    RCC->APB1RSTR |= RCC_APB1Periph;
  }
  else
  {
    RCC->APB1RSTR &= ~RCC_APB1Periph;
  }
}

/**
  * @brief  Forces or releases High Speed APB (APB2) peripheral reset.
  * @param  RCC_APB2Periph: specifies the APB2 peripheral to reset.
  *          This parameter can be any combination of the following values:
  *            @arg RCC_APB2Periph_TIM1:   TIM1 clock
  *            @arg RCC_APB2Periph_TIM8:   TIM8 clock
  *            @arg RCC_APB2Periph_USART1: USART1 clock
  *            @arg RCC_APB2Periph_USART6: USART6 clock
  *            @arg RCC_APB2Periph_ADC1:   ADC1 clock
  *            @arg RCC_APB2Periph_ADC2:   ADC2 clock
  *            @arg RCC_APB2Periph_ADC3:   ADC3 clock
  *            @arg RCC_APB2Periph_SDIO:   SDIO clock
  *            @arg RCC_APB2Periph_SPI1:   SPI1 clock
  *            @arg RCC_APB2Periph_SPI4:   SPI4 clock  
  *            @arg RCC_APB2Periph_SYSCFG: SYSCFG clock
  *            @arg RCC_APB2Periph_TIM9:   TIM9 clock
  *            @arg RCC_APB2Periph_TIM10:  TIM10 clock
  *            @arg RCC_APB2Periph_TIM11:  TIM11 clock
  *            @arg RCC_APB2Periph_SPI5:   SPI5 clock
  *            @arg RCC_APB2Periph_SPI6:   SPI6 clock
  *            @arg RCC_APB2Periph_SAI1:   SAI1 clock (STM32F42xxx/43xxx/446xx/469xx/479xx/413_423xx devices)
  *            @arg RCC_APB2Periph_SAI2:   SAI2 clock (STM32F446xx devices) 
  *            @arg RCC_APB2Periph_LTDC:   LTDC clock (STM32F429xx/439xx devices)
  *            @arg RCC_APB2Periph_DSI:    DSI clock (STM32F469_479xx devices)
  *            @arg RCC_APB2Periph_DFSDM1: DFSDM Clock (STM32F412xG and STM32F413_423xx Devices)
  *            @arg RCC_APB2Periph_DFSDM2: DFSDM2 Clock (STM32F413_423xx Devices)
  *            @arg RCC_APB2Periph_UART9:  UART9 Clock (STM32F413_423xx Devices)
  *            @arg RCC_APB2Periph_UART10: UART10 Clock (STM32F413_423xx Devices)
  * @param  NewState: new state of the specified peripheral reset.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_RCC_APB2_RESET_PERIPH(RCC_APB2Periph));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    RCC->APB2RSTR |= RCC_APB2Periph;
  }
  else
  {
    RCC->APB2RSTR &= ~RCC_APB2Periph;
  }
}

/**
  * @brief  Enables or disables the Low Speed APB (APB1) peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it. 
  * @param  RCC_APB1Periph: specifies the APB1 peripheral to gates its clock.
  *          This parameter can be any combination of the following values:
  *            @arg RCC_APB1Periph_TIM2:   TIM2 clock
  *            @arg RCC_APB1Periph_TIM3:   TIM3 clock
  *            @arg RCC_APB1Periph_TIM4:   TIM4 clock
  *            @arg RCC_APB1Periph_TIM5:   TIM5 clock
  *            @arg RCC_APB1Periph_TIM6:   TIM6 clock
  *            @arg RCC_APB1Periph_TIM7:   TIM7 clock
  *            @arg RCC_APB1Periph_TIM12:  TIM12 clock
  *            @arg RCC_APB1Periph_TIM13:  TIM13 clock
  *            @arg RCC_APB1Periph_TIM14:  TIM14 clock
  *            @arg RCC_APB1Periph_LPTIM1: LPTIM1 clock (STM32F410xx and STM32F413_423xx devices) 
  *            @arg RCC_APB1Periph_WWDG:   WWDG clock
  *            @arg RCC_APB1Periph_SPI2:   SPI2 clock
  *            @arg RCC_APB1Periph_SPI3:   SPI3 clock
  *            @arg RCC_APB1Periph_SPDIF:  SPDIF RX clock (STM32F446xx devices) 
  *            @arg RCC_APB1Periph_USART2: USART2 clock
  *            @arg RCC_APB1Periph_USART3: USART3 clock
  *            @arg RCC_APB1Periph_UART4:  UART4 clock
  *            @arg RCC_APB1Periph_UART5:  UART5 clock
  *            @arg RCC_APB1Periph_I2C1:   I2C1 clock
  *            @arg RCC_APB1Periph_I2C2:   I2C2 clock
  *            @arg RCC_APB1Periph_I2C3:   I2C3 clock
  *            @arg RCC_APB1Periph_FMPI2C1:FMPI2C1 clock
  *            @arg RCC_APB1Periph_CAN1:   CAN1 clock
  *            @arg RCC_APB1Periph_CAN2:   CAN2 clock
  *            @arg RCC_APB1Periph_CEC:    CEC clock (STM32F446xx devices)
  *            @arg RCC_APB1Periph_PWR:    PWR clock
  *            @arg RCC_APB1Periph_DAC:    DAC clock
  *            @arg RCC_APB1Periph_UART7:  UART7 clock
  *            @arg RCC_APB1Periph_UART8:  UART8 clock
  * @param  NewState: new state of the specified peripheral clock.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_RCC_APB1_PERIPH(RCC_APB1Periph));  
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    RCC->APB1ENR |= RCC_APB1Periph;
  }
  else
  {
    RCC->APB1ENR &= ~RCC_APB1Periph;
  }
}

/**
  * @brief  Enables or disables the High Speed APB (APB2) peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.
  * @param  RCC_APB2Periph: specifies the APB2 peripheral to gates its clock.
  *          This parameter can be any combination of the following values:
  *            @arg RCC_APB2Periph_TIM1:   TIM1 clock
  *            @arg RCC_APB2Periph_TIM8:   TIM8 clock
  *            @arg RCC_APB2Periph_USART1: USART1 clock
  *            @arg RCC_APB2Periph_USART6: USART6 clock
  *            @arg RCC_APB2Periph_ADC1:   ADC1 clock
  *            @arg RCC_APB2Periph_ADC2:   ADC2 clock
  *            @arg RCC_APB2Periph_ADC3:   ADC3 clock
  *            @arg RCC_APB2Periph_SDIO:   SDIO clock
  *            @arg RCC_APB2Periph_SPI1:   SPI1 clock
  *            @arg RCC_APB2Periph_SPI4:   SPI4 clock
  *            @arg RCC_APB2Periph_SYSCFG: SYSCFG clock
  *            @arg RCC_APB2Periph_EXTIT:  EXTIIT clock
  *            @arg RCC_APB2Periph_TIM9:   TIM9 clock
  *            @arg RCC_APB2Periph_TIM10:  TIM10 clock
  *            @arg RCC_APB2Periph_TIM11:  TIM11 clock
  *            @arg RCC_APB2Periph_SPI5:   SPI5 clock
  *            @arg RCC_APB2Periph_SPI6:   SPI6 clock
  *            @arg RCC_APB2Periph_SAI1:   SAI1 clock (STM32F42xxx/43xxx/446xx/469xx/479xx/413_423xx devices)
  *            @arg RCC_APB2Periph_SAI2:   SAI2 clock (STM32F446xx devices) 
  *            @arg RCC_APB2Periph_LTDC:   LTDC clock (STM32F429xx/439xx devices)
  *            @arg RCC_APB2Periph_DSI:    DSI clock (STM32F469_479xx devices)
  *            @arg RCC_APB2Periph_DFSDM1: DFSDM Clock (STM32F412xG and STM32F413_423xx Devices)
  *            @arg RCC_APB2Periph_DFSDM2: DFSDM2 Clock (STM32F413_423xx Devices)
  *            @arg RCC_APB2Periph_UART9:  UART9 Clock (STM32F413_423xx Devices)
  *            @arg RCC_APB2Periph_UART10: UART10 Clock (STM32F413_423xx Devices)
  * @param  NewState: new state of the specified peripheral clock.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_RCC_APB2_PERIPH(RCC_APB2Periph));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    RCC->APB2ENR |= RCC_APB2Periph;
  }
  else
  {
    RCC->APB2ENR &= ~RCC_APB2Periph;
  }
}

/**
  * @brief  Enables or disables the AHB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.   
  * @param  RCC_AHBPeriph: specifies the AHB1 peripheral to gates its clock.
  *          This parameter can be any combination of the following values:
  *            @arg RCC_AHB1Periph_GPIOA:       GPIOA clock
  *            @arg RCC_AHB1Periph_GPIOB:       GPIOB clock 
  *            @arg RCC_AHB1Periph_GPIOC:       GPIOC clock
  *            @arg RCC_AHB1Periph_GPIOD:       GPIOD clock
  *            @arg RCC_AHB1Periph_GPIOE:       GPIOE clock
  *            @arg RCC_AHB1Periph_GPIOF:       GPIOF clock
  *            @arg RCC_AHB1Periph_GPIOG:       GPIOG clock
  *            @arg RCC_AHB1Periph_GPIOG:       GPIOG clock
  *            @arg RCC_AHB1Periph_GPIOI:       GPIOI clock
  *            @arg RCC_AHB1Periph_GPIOJ:       GPIOJ clock (STM32F42xxx/43xxx devices) 
  *            @arg RCC_AHB1Periph_GPIOK:       GPIOK clock (STM32F42xxx/43xxx devices)  
  *            @arg RCC_AHB1Periph_CRC:         CRC clock
  *            @arg RCC_AHB1Periph_BKPSRAM:     BKPSRAM interface clock
  *            @arg RCC_AHB1Periph_CCMDATARAMEN CCM data RAM interface clock
  *            @arg RCC_AHB1Periph_DMA1:        DMA1 clock
  *            @arg RCC_AHB1Periph_DMA2:        DMA2 clock
  *            @arg RCC_AHB1Periph_DMA2D:       DMA2D clock (STM32F429xx/439xx devices)  
  *            @arg RCC_AHB1Periph_ETH_MAC:     Ethernet MAC clock
  *            @arg RCC_AHB1Periph_ETH_MAC_Tx:  Ethernet Transmission clock
  *            @arg RCC_AHB1Periph_ETH_MAC_Rx:  Ethernet Reception clock
  *            @arg RCC_AHB1Periph_ETH_MAC_PTP: Ethernet PTP clock
  *            @arg RCC_AHB1Periph_OTG_HS:      USB OTG HS clock
  *            @arg RCC_AHB1Periph_OTG_HS_ULPI: USB OTG HS ULPI clock
  * @param  NewState: new state of the specified peripheral clock.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void RCC_AHB1PeriphClockCmd(uint32_t RCC_AHB1Periph, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_RCC_AHB1_CLOCK_PERIPH(RCC_AHB1Periph));

  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    RCC->AHB1ENR |= RCC_AHB1Periph;
  }
  else
  {
    RCC->AHB1ENR &= ~RCC_AHB1Periph;
  }
}

/**
  * @brief  Returns the frequencies of different on chip clocks; SYSCLK, HCLK, 
  *         PCLK1 and PCLK2.
  * 
  * @note   The system frequency computed by this function is not the real 
  *         frequency in the chip. It is calculated based on the predefined 
  *         constant and the selected clock source:
  * @note     If SYSCLK source is HSI, function returns values based on HSI_VALUE(*)
  * @note     If SYSCLK source is HSE, function returns values based on HSE_VALUE(**)
  * @note     If SYSCLK source is PLL, function returns values based on HSE_VALUE(**) 
  *           or HSI_VALUE(*) multiplied/divided by the PLL factors.         
  * @note     (*) HSI_VALUE is a constant defined in stm32f4xx.h file (default value
  *               16 MHz) but the real value may vary depending on the variations
  *               in voltage and temperature.
  * @note     (**) HSE_VALUE is a constant defined in stm32f4xx.h file (default value
  *                25 MHz), user has to ensure that HSE_VALUE is same as the real
  *                frequency of the crystal used. Otherwise, this function may
  *                have wrong result.
  *                
  * @note   The result of this function could be not correct when using fractional
  *         value for HSE crystal.
  *   
  * @param  RCC_Clocks: pointer to a RCC_ClocksTypeDef structure which will hold
  *          the clocks frequencies.
  *     
  * @note   This function can be used by the user application to compute the 
  *         baudrate for the communication peripherals or configure other parameters.
  * @note   Each time SYSCLK, HCLK, PCLK1 and/or PCLK2 clock changes, this function
  *         must be called to update the structure's field. Otherwise, any
  *         configuration based on this function will be incorrect.
  *    
  * @retval None
  */
void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks)
{
  uint32_t tmp = 0, presc = 0, pllvco = 0, pllp = 2, pllsource = 0, pllm = 2;
#if defined(STM32F412xG) || defined(STM32F413_423xx) || defined(STM32F446xx)  
  uint32_t pllr = 2;
#endif /* STM32F412xG || STM32F413_423xx || STM32F446xx */
  
  /* Get SYSCLK source -------------------------------------------------------*/
  tmp = RCC->CFGR & RCC_CFGR_SWS;
  
  switch (tmp)
  {
  case 0x00:  /* HSI used as system clock source */
    RCC_Clocks->SYSCLK_Frequency = HSI_VALUE;
    break;
  case 0x04:  /* HSE used as system clock  source */
    RCC_Clocks->SYSCLK_Frequency = HSE_VALUE;
    break;
  case 0x08:  /* PLL P used as system clock  source */
    
    /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLLM) * PLLN
    SYSCLK = PLL_VCO / PLLP
    */    
    pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
    pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
    
    if (pllsource != 0)
    {
      /* HSE used as PLL clock source */
      pllvco = (HSE_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
    }
    else
    {
      /* HSI used as PLL clock source */
      pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);      
    }
    
    pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >>16) + 1 ) *2;
    RCC_Clocks->SYSCLK_Frequency = pllvco/pllp;
    break;

#if defined(STM32F412xG) || defined(STM32F413_423xx) || defined(STM32F446xx)
  case 0x0C:  /* PLL R used as system clock  source */
    /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLLM) * PLLN
    SYSCLK = PLL_VCO / PLLR
    */    
    pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
    pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
    
    if (pllsource != 0)
    {
      /* HSE used as PLL clock source */
      pllvco = (HSE_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
    }
    else
    {
      /* HSI used as PLL clock source */
      pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);      
    }
    
    pllr = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLR) >>28) + 1 ) *2;
    RCC_Clocks->SYSCLK_Frequency = pllvco/pllr;    
    break;
#endif /* STM32F412xG || STM32F413_423xx || STM32F446xx */
    
  default:
    RCC_Clocks->SYSCLK_Frequency = HSI_VALUE;
    break;
  }
  /* Compute HCLK, PCLK1 and PCLK2 clocks frequencies ------------------------*/
  
  /* Get HCLK prescaler */
  tmp = RCC->CFGR & RCC_CFGR_HPRE;
  tmp = tmp >> 4;
  presc = APBAHBPrescTable[tmp];
  /* HCLK clock frequency */
  RCC_Clocks->HCLK_Frequency = RCC_Clocks->SYSCLK_Frequency >> presc;

  /* Get PCLK1 prescaler */
  tmp = RCC->CFGR & RCC_CFGR_PPRE1;
  tmp = tmp >> 10;
  presc = APBAHBPrescTable[tmp];
  /* PCLK1 clock frequency */
  RCC_Clocks->PCLK1_Frequency = RCC_Clocks->HCLK_Frequency >> presc;

  /* Get PCLK2 prescaler */
  tmp = RCC->CFGR & RCC_CFGR_PPRE2;
  tmp = tmp >> 13;
  presc = APBAHBPrescTable[tmp];
  /* PCLK2 clock frequency */
  RCC_Clocks->PCLK2_Frequency = RCC_Clocks->HCLK_Frequency >> presc;
}

#endif