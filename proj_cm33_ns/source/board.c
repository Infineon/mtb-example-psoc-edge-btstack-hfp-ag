/*******************************************************************************
* File Name: board.c
*
* Description: This file contains board supported API's.
*
* Related Document: See README.md
*
********************************************************************************
* (c) 2025-2026, Infineon Technologies AG, or an affiliate of Infineon
* Technologies AG. All rights reserved.
* This software, associated documentation and materials ("Software") is
* owned by Infineon Technologies AG or one of its affiliates ("Infineon")
* and is protected by and subject to worldwide patent protection, worldwide
* copyright laws, and international treaty provisions. Therefore, you may use
* this Software only as provided in the license agreement accompanying the
* software package from which you obtained this Software. If no license
* agreement applies, then any use, reproduction, modification, translation, or
* compilation of this Software is prohibited without the express written
* permission of Infineon.
*
* Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
* IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
* THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
* SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
* Infineon reserves the right to make changes to the Software without notice.
* You are responsible for properly designing, programming, and testing the
* functionality and safety of your intended application of the Software, as
* well as complying with any legal requirements related to its use. Infineon
* does not guarantee that the Software will be free from intrusion, data theft
* or loss, or other breaches ("Security Breaches"), and Infineon shall have
* no liability arising out of any Security Breaches. Unless otherwise
* explicitly approved by Infineon, the Software may not be used in any
* application where a failure of the Product or any consequences of the use
* thereof can reasonably be expected to result in personal injury.
*******************************************************************************/
/*******************************************************************************
* Header file includes
*******************************************************************************/
#include "cybsp.h"
#include "cy_retarget_io.h"
#include <FreeRTOS.h>
#include <task.h>
#include "board.h"
#include "app_bt_utils.h"
#include "event_groups.h"
#include "hfp_audio_gateway.h"
#include "mtb_syspm_callbacks_tcpwm.h"
#include "retarget_io_init.h"

/*******************************************************************************
* Macros
********************************************************************************/
#define PWM_DUTY_CYCLE_0                 (0U)
#define PWM_DUTY_CYCLE_50                (1000U)
#define PWM_DUTY_CYCLE_100               (2000U)

#define INTERRUPT_MASKED                 (1U)
#define BUTTON_SCAN_DELAY_MSEC           (10U)
#define SHORT_PRESS_DELAY_MSEC           (10U)
#define LONG_PRESS_DELAY_MSEC            (3000U)
#define DEBOUNCE_TIME_MS                 (300U)

#define BUTTON_INTR_PRIORITY             (5U)
#define BUTTON_TASK_STACK_SIZE           (512U)
#define BUTTON_TASK_PRIORITY             (2U)
#define NULL_PTR                         (NULL)

/*universal macro for zero*/
#define ZERO                             (0U)

#if (CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_DEEPSLEEP)

/* PWM Context for SysPm Callback */
mtb_syspm_tcpwm_deepsleep_context_t PWMdscontext =
{
    .channelNum = CYBSP_PWM_LED_CTRL_NUM,
};

/* SysPm callback parameter structure for PWM */
static cy_stc_syspm_callback_params_t PWMDSParams =
{
        .context   = &PWMdscontext,
        .base      = CYBSP_PWM_LED_CTRL_HW
};

/* SysPm callback structure for PWM */
static cy_stc_syspm_callback_t PWMDeepSleepCallbackHandler =
{
    .callback           = &mtb_syspm_tcpwm_deepsleep_callback,
    .skipMode           = CY_SYSPM_SKIP_CHECK_FAIL | CY_SYSPM_SKIP_CHECK_READY,
    .type               = CY_SYSPM_DEEPSLEEP,
    .callbackParams     = &PWMDSParams,
    .prevItm            = NULL,
    .nextItm            = NULL,
    .order              = SYSPM_CALLBACK_ORDER
};

#endif /*CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_DEEPSLEEP) */
/*******************************************************************************
* Global Variables
*******************************************************************************/
/*Flag to set based on USER BUTTON press */
bool gpio_intr_flag1 = FALSE;
bool gpio_intr_flag2 = FALSE;

/* Define the semaphore handle */
static SemaphoreHandle_t button_semaphore;

/* Variables for button debouncing */
volatile bool button_debouncing = false;
volatile uint32_t button_debounce_timestamp = ZERO;

/******************************************************************************
 *                          Function Declarations
 ******************************************************************************/
static void board_button_init(void);
static void board_button_event_handler(void);

/*******************************************************************************
* Function Definition
*******************************************************************************/

/*******************************************************************************
* Function Name: board_led_init
********************************************************************************
*
* Summary:
*   Initialize the User Led 3 with PWM
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
static void board_led_init(void)
{
    cy_rslt_t result = CY_TCPWM_SUCCESS;

    /* Initialize the TCPWM block */
    result = Cy_TCPWM_PWM_Init(CYBSP_PWM_LED_CTRL_HW,
            CYBSP_PWM_LED_CTRL_NUM,
            &CYBSP_PWM_LED_CTRL_config);

    /* PWM init failed. Stop program execution */
    if(CY_TCPWM_SUCCESS != result)
    {
        printf("Failed to initialize PWM for LED2.\r\n");
        handle_app_error();
    }

#if (CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_DEEPSLEEP)

    /* SysPm callback registration for PWM */
    Cy_SysPm_RegisterCallback(&PWMDeepSleepCallbackHandler);

#endif

    /* Enable the TCPWM block */
    Cy_TCPWM_PWM_Enable(CYBSP_PWM_LED_CTRL_HW,
            CYBSP_PWM_LED_CTRL_NUM);

    /* Start the PWM */
    Cy_TCPWM_TriggerStart_Single(CYBSP_PWM_LED_CTRL_HW,
            CYBSP_PWM_LED_CTRL_NUM);
}

/*******************************************************************************
* Function Name: user_button_init
********************************************************************************
*
* Summary:
*  Initialize the button with Interrupt
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void board_button_init(void)
{
    cy_stc_sysint_t btn_cfg =
    {
        .intrSrc = CYBSP_USER_BTN1_IRQ,
        .intrPriority = BUTTON_INTR_PRIORITY
    };

    /* Clear any pending interrupts for User Buttons */
    Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN1_PORT, CYBSP_USER_BTN1_PIN);
    Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN2_PORT, CYBSP_USER_BTN2_PIN);
    NVIC_ClearPendingIRQ(CYBSP_USER_BTN1_IRQ);
    NVIC_ClearPendingIRQ(CYBSP_USER_BTN2_IRQ);

    /* Initialize interrupt configuration structure for user Buttons */
    Cy_SysInt_Init(&btn_cfg, board_button_event_handler);

    /* Enable NVIC interrupt for user Buttons */
    NVIC_EnableIRQ(btn_cfg.intrSrc);
    
    xTaskCreate(board_button_event_task, "BtnTask", BUTTON_TASK_STACK_SIZE, 
            NULL_PTR, BUTTON_TASK_PRIORITY, NULL_PTR);
}

/*******************************************************************************
* Function Name: board_init
********************************************************************************
*
* Summary:
*  Initialize the board with LED's and Buttons
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void board_init(void)
{
    /* Initialize the USER LEDs with PWM*/
    board_led_init();
    board_button_init();
}

/*******************************************************************************
* Function Name: board_led_set_state
********************************************************************************
*
* Summary:
*  Set the led state over PWM
*
* Parameters:
*  value: ON/OFF state
*
* Return:
*  None
*
*******************************************************************************/
void board_led_set_state(bool value)
{
    Cy_TCPWM_PWM_SetCompare0(CYBSP_PWM_LED_CTRL_HW,
            CYBSP_PWM_LED_CTRL_NUM,value?PWM_DUTY_CYCLE_0:PWM_DUTY_CYCLE_100);
}

/*******************************************************************************
* Function Name: board_led_set_blink
********************************************************************************
*
* Summary:
*  Set the User led 3 duty cycle to 50%
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void board_led_set_blink(void)
{
    Cy_TCPWM_PWM_SetCompare0(CYBSP_PWM_LED_CTRL_HW,
            CYBSP_PWM_LED_CTRL_NUM,PWM_DUTY_CYCLE_50);
}

/*******************************************************************************
* Function Name: board_button_event_task
********************************************************************************
* Summary:
*  FreeRTOS task that waits for button press events signaled via an event group
*  and sends the appropriate AVRCP command based on the button action detected.
*
* Parameters:
*  pvParameters - Pointer to task parameters (unused in this implementation).
*
* Return:
*  None
*
*******************************************************************************/
void board_button_event_task(void *pv_parameters)
{
    (void) pv_parameters; /* unused */
    TickType_t last_button_press_time = ZERO;
    TickType_t long_press_start_time = ZERO;
    BaseType_t long_press_detected = pdFALSE;

    /* Create the semaphore */
    button_semaphore = xSemaphoreCreateBinary();
    for (;;)
    {
        /* Wait for the semaphore to be taken from the ISR */
        if (pdTRUE == xSemaphoreTake(button_semaphore, portMAX_DELAY))
        {
            if(true == gpio_intr_flag2)
            {
                gpio_intr_flag2 = false;
                last_button_press_time = xTaskGetTickCount();
                long_press_start_time = xTaskGetTickCount();
                long_press_detected = pdFALSE;

                while (Cy_GPIO_Read(CYBSP_SW4_PORT, CYBSP_SW4_PIN) ==
                                       CYBSP_BTN_PRESSED)
                {
                    if(!long_press_detected &&
                          ((xTaskGetTickCount() - long_press_start_time) >=
                          pdMS_TO_TICKS(LONG_PRESS_DELAY_MSEC)))
                    {
                        long_press_detected = pdTRUE;
                        /* Button 2 press is long */
                        hfp_ag_simulate_call_end();
                    }

                    vTaskDelay(pdMS_TO_TICKS(BUTTON_SCAN_DELAY_MSEC));
                }

                if (!long_press_detected &&
                    ((xTaskGetTickCount() - last_button_press_time) >=
                      pdMS_TO_TICKS(SHORT_PRESS_DELAY_MSEC)))
                {
                    /* Button 2 press is short */
                    hfp_ag_volume_down();
                }
            }
            else if(true == gpio_intr_flag1)
            {
                gpio_intr_flag1 = false;
                last_button_press_time = xTaskGetTickCount();
                long_press_start_time = xTaskGetTickCount();
                long_press_detected = pdFALSE;

                while (Cy_GPIO_Read(CYBSP_SW2_PORT, CYBSP_SW2_PIN) ==
                                       CYBSP_BTN_PRESSED)
                {
                    if(!long_press_detected &&
                          ((xTaskGetTickCount() - long_press_start_time) >=
                          pdMS_TO_TICKS(LONG_PRESS_DELAY_MSEC)))
                    {
                        long_press_detected = pdTRUE;
                        /* Button 1 press is long */
                        hfp_ag_simulate_call_start();
                    }

                    vTaskDelay(pdMS_TO_TICKS(BUTTON_SCAN_DELAY_MSEC));
                }

                if (!long_press_detected &&
                    ((xTaskGetTickCount() - last_button_press_time) >=
                      pdMS_TO_TICKS(SHORT_PRESS_DELAY_MSEC)))
                {
                    /* Button 1 press is short */
                    hfp_ag_volume_up();
                }
            }
        }
    }
}

/*******************************************************************************
 * Function Name: board_button_event_handler
 *******************************************************************************
 * Summary:
 *  Detects whether a button press is short, long, or double and sets the
 *  corresponding event flag. The 'button' parameter identifies which button
 *  triggered the logic.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  void
 ******************************************************************************/
static void board_button_event_handler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Check if the interrupt is from USER BUTTON 1 */
    if(INTERRUPT_MASKED == Cy_GPIO_GetInterruptStatusMasked(CYBSP_USER_BTN_PORT,
            CYBSP_USER_BTN_PIN))
    {
        /* Clear the interrupt and pending IRQ from NVIC*/
        Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_PIN);
        NVIC_ClearPendingIRQ(CYBSP_USER_BTN1_IRQ);

        /* Set the interrupt flag */
        gpio_intr_flag1 = true;
    }

    /* Check if the interrupt is from USER BUTTON 2 */
    if(INTERRUPT_MASKED == Cy_GPIO_GetInterruptStatusMasked(CYBSP_USER_BTN2_PORT,
            CYBSP_USER_BTN2_PIN))
    {
        /* Clear the interrupt and pending IRQ from NVIC*/
        Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN2_PORT, CYBSP_USER_BTN2_PIN);
        NVIC_ClearPendingIRQ(CYBSP_USER_BTN2_IRQ);

        if (!button_debouncing)
        {
            /* Set the debouncing flag */
            button_debouncing = true;

            /* Record the current timestamp */
            button_debounce_timestamp = (uint32_t) (xTaskGetTickCount()
                * portTICK_PERIOD_MS);
        }

        if (button_debouncing && (((xTaskGetTickCount() * portTICK_PERIOD_MS)) -
                button_debounce_timestamp <= DEBOUNCE_TIME_MS *
                portTICK_PERIOD_MS))
        {
              button_debouncing = false;
              /* Set the interrupt flag */
              gpio_intr_flag2 = true;
        }
    }

    /* Give Semaphore from ISR to the button task */
    xSemaphoreGiveFromISR(button_semaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* [] END OF FILE */
