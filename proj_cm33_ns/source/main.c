/*******************************************************************************
 * File Name: main.c
 *
 * Description: This is the source code for the Bluetooth HFP Audio Gateway
 *              Example for ModusToolbox.
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
/******************************************************************************
 * Includes
 *****************************************************************************/
#include "cy_pdl.h"
#include "cycfg.h"
#include "cybsp.h"
#include "cy_time.h"
#include "cyabs_rtos.h"
#include "cyabs_rtos_impl.h"
#include "FreeRTOS.h"
#include "task.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_cfg.h"
#include "app_bt_utils/app_bt_utils.h"
#include "hfp_audio_gateway.h"
#include "app_bt_bonding.h"
#include "app_bt_utils.h"
#include "retarget_io_init.h"
#include "board.h"
#include "hfp_audio_gateway.h"
#include "app_pdm_pcm.h"
#include "app_i2s.h"

/******************************************************************************
 * Macros
 *****************************************************************************/
/* The timeout value in microsecond used to wait for core to be booted */
#define CM55_BOOT_WAIT_TIME_US            (10U)

/* UART function parameter value to wait forever */
#define BT_STACK_HEAP_SIZE                (0X1000U)
#define BT_TASK_STACK_SIZE                (1024U)
#define BT_TASK_PRIORITY                  (configMAX_PRIORITIES - (6U))
#define TASK_DELAY_50MS                   (50U)
#define TIMER_INTERRUPT_PRIORITY          (7U)

/* Enabling or disabling a MCWDT requires a wait time of upto 2 CLK_LF cycles
 * to come into effect. This wait time value will depend on the actual CLK_LF
 * frequency set by the BSP.
 */
#define LPTIMER_0_WAIT_TIME_USEC          (62U)

/* Define the LPTimer interrupt priority number. '1' implies highest priority.*/
#define APP_LPTIMER_INTERRUPT_PRIORITY    (1U)
#define TASK_DELAY_10MS                   (10U)

/* App boot address for CM55 project */
#define CM55_APP_BOOT_ADDR                (CYMEM_CM33_0_m55_nvm_START + \
                                                CYBSP_MCUBOOT_HEADER_SIZE)
 
/* The timeout value in microsecond used to wait for the CM55 core to be booted.
 * Use value 0U for infinite wait till the core is booted successfully.
 */
#define CM55_BOOT_WAIT_TIME_USEC           (10U)
#define DEVICE_COUNT_MIN                   (0U)

#define BDA_ADDR_BYTE_0                    (0U)
#define BDA_ADDR_BYTE_1                    (1U)
#define BDA_ADDR_BYTE_2                    (2U)
#define BDA_ADDR_BYTE_3                    (3U)
#define BDA_ADDR_BYTE_4                    (4U)
#define BDA_ADDR_BYTE_5                    (5U)
#define CONSTANT_ZERO                      (0U)
#define CONSTANT_ONE                       (1U)

/****************************************************************************
 * Global variables
 ***************************************************************************/
wiced_bt_device_address_t bda_str;
uint8_t uart_response;

/* RTC HAL object */
static mtb_hal_rtc_t rtc_obj;

/****************************************************************************
 * Static and extern variables
 ***************************************************************************/
static TaskHandle_t bt_task_handle;

/* LPTimer HAL object */
static mtb_hal_lptimer_t lptimer_obj;
extern device_info_t device_info_list[MAX_DEVICES];
extern int16_t device_count;
extern int8_t connection_status;
extern volatile bool already_connected;

/*****************************************************************************
 * Function Prototypes
 *****************************************************************************/
static void application_start(void *task_params);
static void bt_task_create(void);

/*****************************************************************************
 * Function Definitions
 *****************************************************************************/
/*******************************************************************************
* Function Name: setup_clib_support
********************************************************************************
* Summary:
*    1. This function configures and initializes the Real-Time Clock (RTC).
*    2. It then initializes the RTC HAL object to enable CLIB support library 
*       to work with the provided Real-Time Clock (RTC) module.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
static void setup_clib_support(void)
{
    /* RTC Initialization */
    Cy_RTC_Init(&CYBSP_RTC_config);
    Cy_RTC_SetDateAndTime(&CYBSP_RTC_config);

    /* Initialize the ModusToolbox CLIB support library */
    mtb_clib_support_init(&rtc_obj);
}

/*******************************************************************************
* Function Name: lptimer_interrupt_handler
********************************************************************************
* Summary:
* Interrupt handler function for LPTimer instance.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
static void lptimer_interrupt_handler(void)
{
    mtb_hal_lptimer_process_interrupt(&lptimer_obj);
}

/*******************************************************************************
* Function Name: setup_tickless_idle_timer
********************************************************************************
* Summary:
* 1. This function first configures and initializes an interrupt for LPTimer.
* 2. Then it initializes the LPTimer HAL object to be used in the RTOS
*    tickless idle mode implementation to allow the device enter deep sleep
*    when idle task runs. LPTIMER_0 instance is configured for CM33 CPU.
* 3. It then passes the LPTimer object to abstraction RTOS library that
*    implements tickless idle mode
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
static void setup_tickless_idle_timer(void)
{
    /* Interrupt configuration structure for LPTimer */
    cy_stc_sysint_t lptimer_intr_cfg =
    {
        .intrSrc = CYBSP_CM33_LPTIMER_0_IRQ,
        .intrPriority = APP_LPTIMER_INTERRUPT_PRIORITY
    };

    /* Initialize the LPTimer interrupt and specify the interrupt handler. */
    cy_en_sysint_status_t interrupt_init_status =
                                    Cy_SysInt_Init(&lptimer_intr_cfg,
                                                    lptimer_interrupt_handler);

    /* LPTimer interrupt initialization failed. Stop program execution. */
    if(CY_SYSINT_SUCCESS != interrupt_init_status)
    {
        handle_app_error();
    }

    /* Enable NVIC interrupt. */
    NVIC_EnableIRQ(lptimer_intr_cfg.intrSrc);

    /* Initialize the MCWDT block */
    cy_en_mcwdt_status_t mcwdt_init_status =
                                    Cy_MCWDT_Init(CYBSP_CM33_LPTIMER_0_HW,
                                                &CYBSP_CM33_LPTIMER_0_config);

    /* MCWDT initialization failed. Stop program execution. */
    if(CY_MCWDT_SUCCESS != mcwdt_init_status)
    {
        handle_app_error();
    }

    /* Enable MCWDT instance */
    Cy_MCWDT_Enable(CYBSP_CM33_LPTIMER_0_HW,
                    CY_MCWDT_CTR_Msk,
                    LPTIMER_0_WAIT_TIME_USEC);

    /* Setup LPTimer using the HAL object and desired configuration as defined
     * in the device configurator. */
    cy_rslt_t result = mtb_hal_lptimer_setup(&lptimer_obj,
                                            &CYBSP_CM33_LPTIMER_0_hal_config);

    /* LPTimer setup failed. Stop program execution. */
    if(CY_RSLT_SUCCESS != result)
    {
        handle_app_error();
    }

    /* Pass the LPTimer object to abstraction RTOS library that implements
     * tickless idle mode
     */
    cyabs_rtos_set_lptimer(&lptimer_obj);
}

/*****************************************************************************
 * Function Name: main()
 ******************************************************************************
 * Summary:
 *   1. Entry point to the application.
 *   2. Set device configuration and start BT application RTOS tasks
 *   3. Enables CM55
 *
 * Parameters:
 *   None
 *
 * Return:
 *   result status
 *
 ******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize and Verify the BSP initialization */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (CY_RSLT_SUCCESS != result)
    {
        handle_app_error();
    }

    /* Setup CLIB support library. */
    setup_clib_support();

    /* Setup the LPTimer instance for CM33 CPU. */
    setup_tickless_idle_timer();

    /* Initialize retarget-io middleware */
    init_retarget_io();

    /* \x1b[2J\x1b[;H - ANSI ESC sequence to clear screen */
    printf("\x1b[2J\x1b[;H");
    printf( "\r\n********************************************************" );
    printf( "\r\n         PSOC EDGE MCU: HFP Audio Gateway           \r\n" );
    printf( "********************************************************\r\n" );

    /* Initialize USER Button and USER LED */
    board_init();

    /* Initialize the device used by kv-store for performing
     * read/write operations to the NVM
     */
    app_kv_store_init();

    if(BUTTON_PRESSED == Cy_GPIO_Read(CYBSP_USER_BTN1_PORT, CYBSP_USER_BTN1_PIN))
    {
         /* Reset Kv-store library, this will clear the keys in the nvm */
         reset_link_keys();
         handle_app_error();
    }

    /* TLV codec initiailization */
    app_tlv_codec_init();

    /* I2S initialization */
    app_i2s_init();

    /* Initialize the PDM-PCM block */
    app_pdm_pcm_init();

    /* Creating BT task */
    bt_task_create();

    /* Enable CM55. */
    /* CM55_APP_BOOT_ADDR must be updated if CM55 memory layout is changed.*/
    Cy_SysEnableCM55(MXCM55, CM55_APP_BOOT_ADDR, CM55_BOOT_WAIT_TIME_USEC);

    /* Enable global interrupts */
    __enable_irq();

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();

    /* Should never get here */
    handle_app_error();
}

/******************************************************************************
 * Function Name: uart_get_data()
 *******************************************************************************
 * Summary:
 *  This function reads a single byte from the UART interface, 
 *  blocking until data is available.
 *
 * Parameters:
 *  value : uart data
 *
 * Return:
 *  None
 *
 ******************************************************************************/

static void uart_get_data(uint8_t *value)
{

    uint32_t read_value = Cy_SCB_UART_Get(SCB2);
    while (CY_SCB_UART_RX_NO_DATA == read_value)
    {
        read_value = Cy_SCB_UART_Get(SCB2);
        vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_10MS));
    }
    *value = (uint8_t)read_value;
}

/******************************************************************************
 * Function Name: print_stored_devices(void)
 *******************************************************************************
 * Summary:
 *  Prints the Stored BT devices after scanning
 *
 ******************************************************************************/
static void print_stored_devices(void)
{
    for (int i = CONSTANT_ZERO; i < device_count; i++)
    {
        printf("Device %d: ", i + CONSTANT_ONE);
        printf("BD_ADDR: ");
        for (int j = CONSTANT_ZERO; j < BD_ADDR_LEN; j++)
        {
            if(j != (BD_ADDR_LEN -CONSTANT_ONE))
                printf("%02X:", device_info_list[i].remote_bd_addr[j]);
            else
                printf("%02X", device_info_list[i].remote_bd_addr[j]);
        }
        printf("\tName: %s", device_info_list[i].eir_data);
        printf("\n");
    }
}

/******************************************************************************
 * Function Name: application_start()
 *******************************************************************************
 * Summary:
 *  Starts BT stack and runs the BT thread monitoring for connection status.
 *
 * Parameters:
 *  Task parameters.
 *
 * Return:
 *  None
 *
 ******************************************************************************/
static void application_start(void *task_params)
{
    wiced_result_t wiced_result = WICED_BT_SUCCESS;

    /* Switch OFF the LED */
    board_led_set_state(LED_OFF);

    /* Register call back and configuration with stack */
    wiced_result = wiced_bt_stack_init(hfp_audio_gateway_management_callback, 
            &hfp_audio_gateway_cfg_settings);

    WICED_BT_TRACE("Initializing BT Stack - HFP Audio Gateway Start\n");

    /* Check if stack initialization was successful */
    if (WICED_BT_SUCCESS == wiced_result)
    {
        /* Create a buffer heap, make it the default heap.  */
        p_default_heap = wiced_bt_create_heap("app", NULL, BT_STACK_HEAP_SIZE,
                NULL, WICED_TRUE);
        WICED_BT_TRACE("Creating heap \r\n");

        if ((WICED_BT_SUCCESS == wiced_result) && (NULL != p_default_heap))
        {
            WICED_BT_TRACE("heap creation successful\r\\n");
            fprintf(stdout, "Loading Bluetooth Stack...\r");
        }
        else /* Exit App if stack init was not successful or heap creation 
        failed */
        {
            fprintf(stderr, "Bluetooth Stack Initialization or heap creation failed!! Exiting App...\n\r");
            handle_app_error();
        }
    }
    printf("Loading Bluetooth Stack & tasks...\n\r");

    bda_str[BDA_ADDR_BYTE_0]=BDA_ADDRESS_BYTE_0;
    bda_str[BDA_ADDR_BYTE_1]=BDA_ADDRESS_BYTE_1;
    bda_str[BDA_ADDR_BYTE_2]=BDA_ADDRESS_BYTE_2;
    bda_str[BDA_ADDR_BYTE_3]=BDA_ADDRESS_BYTE_3;
    bda_str[BDA_ADDR_BYTE_4]=BDA_ADDRESS_BYTE_4;
    bda_str[BDA_ADDR_BYTE_5]=BDA_ADDRESS_BYTE_5;

    for (;;)
    {
        if (initiate_bt_device_connection && !already_connected)
        {
            if (DEVICE_COUNT_MIN < device_count)
            {
                print_stored_devices();

                printf("Enter the device number you wish to connect to:\n");

                /*Get user input from UART*/
                uart_get_data(&uart_response);    

                if ('0' < uart_response)
                {
                    int idx = uart_response - '1';

                    if (idx <= device_count)
                    {
                        printf("Connecting to: %s \r\n", 
                            device_info_list[idx].eir_data);
                        wiced_result = hfp_audio_gateway_bt_set_visibility(
                            WICED_TRUE, WICED_TRUE);
                        if (WICED_BT_SUCCESS == wiced_result)
                        {
                            wiced_result = hfp_audio_gateway_command_connect(
                                device_info_list[idx].remote_bd_addr, 
                                BD_ADDR_LEN);
                            if (WICED_BT_SUCCESS == wiced_result)
                            {
                                printf("Connection Request sent. Waiting for HFP Hands-Free device to get connected...\r\n");
                                initiate_bt_device_connection = false;
                            }
                            else
                            {
                                printf("Error: Failed to connect \r\n");
                            }
                        }
                        else
                        {
                            printf("Error: Failed to set the Bluetooth visibility \r\n");
                        }
                    }
                    else
                    {
                        printf("Invalid device number. Initiating re-scan...\r\n");
                        device_count = DEVICE_COUNT_MIN;
                        initiate_bt_device_connection = false;
                        hfp_audio_gateway_bt_set_pairability(true);
                        hfp_audio_gateway_bt_inquiry (true);
                    }
                }
            }
            else
            {
                printf("No Bluetooth sink devices found. Would you like to retry? (y/n): \r\n");
                uart_get_data(&uart_response);
                if (uart_response == 'y' ||  uart_response == 'Y')
                {
                    printf("Re-scanning..\r\n");
                    device_count = DEVICE_COUNT_MIN;
                    initiate_bt_device_connection = false;
                    hfp_audio_gateway_bt_set_pairability(true);
                    hfp_audio_gateway_bt_inquiry (true);
                }
                else
                {
                    printf("Scan cancelled...\r\n");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_50MS));

        if (HFP_PEER_CONNECTED == connection_status)
        {
            board_led_set_state(LED_ON);
        }
        else if (HFP_PEER_DISCONNECTED == connection_status)
        {
            board_led_set_blink();
        }
    }
}

/******************************************************************************
 * Function Name: bt_task_create()
 *******************************************************************************
 * Summary:
 *  BT task creation function wrapper
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
 ******************************************************************************/
static void bt_task_create(void)
{
    BaseType_t status;
    status = xTaskCreate(application_start, "BT task", BT_TASK_STACK_SIZE,
            NULL, BT_TASK_PRIORITY, &bt_task_handle);
    if (pdPASS != status)
    {
        printf("Error in starting BT task \n");
    }
}


/* [] END OF FILE */
