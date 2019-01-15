/**
  ******************************************************************************
  * @file    Applications/Virtual_COM_Port/Inc/stm32l0xx_it.h
  * @author  AST/CL
  * @version V2.0.0
  * @date    Apr-2018
  * @brief   This file contains the headers of the interrupt handlers.
  *
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *     
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32L0xx_IT_H
#define STM32L0xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_nucleo.h"

/** @addtogroup PROJECTS
 * @{
 */

/** @addtogroup MULTI
 * @{
 */
 
/** @addtogroup APPLICATIONS
 * @{
 */

/** @addtogroup Virtual_COM_Port
 * @{
 */
		
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/** @addtogroup Virtual_COM_Port_PUBLIC_FUNCTIONS
 * @{
 */

/**
 * @brief  This function handles System tick timer.
 * @param  None
 * @retval None
 */ 
void SysTick_Handler(void);

/**
 * @brief  This function handles UART1 interrupt request.  
 * @param  None
 * @retval None
 * @Note   
 */
void USART1_IRQHandler(void);

/**
 * @brief  This function handles UART2 interrupt request.  
 * @param  None
 * @retval None
 * @Note   
 */
void USART2_IRQHandler(void);

/**
 * @brief  This function handles I2C1 event interrupt.  
 * @param  None
 * @retval None
 * @Note   
 */
void I2C1_EV_IRQHandler(void);

/**
 * @brief  This function handles I2C1 error interrupt.  
 * @param  None
 * @retval None
 * @Note   
 */
void I2C1_ER_IRQHandler(void);

/**
 * @brief  This function handles USER BUTTON event interrupt.  
 * @param  None
 * @retval None
 * @Note   
 */
void EXTI4_15_IRQHandler(void);

/**
 * @}
 */
  
/**
 * @}
 */
  
/**
 * @}
 */

/**
 * @}
 */
 
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* STM32L0xx_IT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
