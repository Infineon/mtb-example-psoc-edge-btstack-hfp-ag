/******************************************************************************
* File Name:   hfp_audio_gateway.c
*
* Description: Source file for HFP Audio Gateway task.
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

#ifdef  WICED_BT_TRACE_ENABLE
#include "wiced_bt_trace.h"
#endif
#include "hfp_audio_gateway.h"
#include "cybsp.h"
#include "wiced_bt_hfp_ag.h"
#include "wiced_bt_hfp_hf.h"
#include "app_bt_bonding.h"
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "board.h"
#include "app_i2s.h"
#include "app_pdm_pcm.h"
#include "timers.h"

/******************************************************************************
 * Macros
 *****************************************************************************/
#define HFP_AUDIO_GATEWAY_NVRAM_ID          (0U)
#define WICED_HS_EIR_BUF_MAX_SIZE           (264U)
#define MAX_KEY_SIZE                        (16U)
#define INQUIRY_DURATION                    (5U)

#define SCO_TASK_STACK_SIZE                 (1024U)
#define SCO_TASK_PRIORITY                   (configMAX_PRIORITIES - (6U))
#define SCO_TASK_DELAY_2MS                  (2U)

/* 8kHz for CVSD */
#define SAMPLE_RATE                         (8000U)

/* 16-bit PCM */
#define BYTES_PER_SAMPLE                    (2U)

/* Mono */
#define CHANNELS                            (1U)

#define AT_ANSWER_CALL_EVT_STR              "ATA"
#define AT_DECLINE_CALL_EVT_STR             "AT+CHUP"
#define AT_MIC_VOLUME_EVT_STR               "AT+VGM="
#define AT_SPK_VOLUME_EVT_STR               "AT+VGS="

#define CALL_RING_TIMER_PERIOD_MS           (2000U)
#define SCO_AUDIO_TIMER_PERIOD_MS           (2U)
#define HFP_AG_TIMER_BLOCK_TIME             pdMS_TO_TICKS(100)
#define HFP_AG_VOLUME_MAX_LEN               (2U)
#define HFP_AG_VOLUME_MAX                   (15U)
#define HFP_AG_DECIMAL_BASE                 (10U)
#define HFP_AG_VOLUME_INIT                  (0U)
#define HFP_AG_VOLUME_INVALID               (-1)
#define HFP_AG_INDEX_INIT                   (0U)
#define PHONE_NUM                           "\"+919876543210\""

#define CONSTANT_ZERO                       (0U)
#define CONSTANT_ONE                        (1U)
#define CONSTANT_TWO                        (2U)
#define CONSTANT_THREE                      (3U)
#define CONSTANT_FOUR                       (4U)
#define CONSTANT_FIVE                       (5U)
#define CONSTANT_TEN                        (10U)
#define CONSTANT_HUNDRED                    (100U)
#define CALL_TYPE_INTERNATIONAL             (145U)
#define COMMAND_STRING_SIZE                 (128U)

#ifndef M_PI
#define M_PI                                3.14159265358979323846
#endif

/****************************************************************************
* Static variables
* **************************************************************************/
static uint8_t spk_volume = 8;   /* start at mid-level (0–15) */
static uint16_t ag_connection_handle = 0xFFFF;
static char cmd_str[COMMAND_STRING_SIZE];
static wiced_bt_hfp_hf_active_call_t ag_call_status;
static wiced_bt_hfp_hf_callsetup_state_t ag_call_state;
static TimerHandle_t call_ring_timer_handle = NULL;
static TaskHandle_t sco_task_handle;

/* ***************************************************************************
* Global variables
* **************************************************************************/
volatile bool already_connected = false;
int8_t connection_status;
uint8_t pincode[WICED_PIN_CODE_LEN] = { 0x30, 0x30, 0x30, 0x30 };
uint8_t  status_flag = CONSTANT_ZERO;
uint8_t  bond_index = CONSTANT_ZERO;
uint8_t app_hello_sensor_notify_client_char_config[] = {0x00, 0x00};
int16_t device_count = CONSTANT_ZERO;
uint16_t active_sco_index = 0xFFFF;

bool agIs_call_active = false;
volatile bool initiate_bt_device_connection = false;

wiced_bt_heap_t *p_default_heap = NULL;
wiced_bt_device_link_keys_t nv_key_ram;

/* service control blocks */
wiced_bt_hfp_ag_session_cb_t  ag_scb[HFP_AG_NUM_SCB];

/* Array to store connection information */
device_info_t device_info_list[MAX_DEVICES];

wiced_bt_voice_path_setup_t ag_sco_path = {
    .path = WICED_BT_SCO_OVER_PCM,
    .p_sco_data_cb = NULL
};

/******************************************************************************
 * Extern variables and functions
 *****************************************************************************/
extern const uint8_t hfp_audio_gateway_sdp_db[HFP_AUDIO_GATEWAY_SDP_DB_SIZE];
extern const wiced_bt_cfg_settings_t hfp_audio_gateway_cfg_settings;

/*******************************************************************************
* Function prototypes
*******************************************************************************/
static void hfp_ag_simulate_call_ring_send_callback(TimerHandle_t xTimer);
static const char *hfag_get_ag_event_name( wiced_bt_hfp_ag_event_t event );
static void hfp_ag_event_callback( wiced_bt_hfp_ag_event_t evt, uint16_t handle, 
                                    wiced_bt_hfp_ag_event_data_t *p_data );
static void hfag_sco_data_app_callback(uint16_t sco_channel, uint16_t length, 
                                       uint8_t* p_data);
static void hfag_sco_task(void *task_params);

/******************************************************************************
 * Function definitions
 ******************************************************************************/

/******************************************************************************
 * Function Name: hfp_audio_gateway_write_eir
 *******************************************************************************
 * Summary:
 *  Prepare extended inquiry response data.  Current version publishes
 *  audio source services.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
 ******************************************************************************/
void hfp_audio_gateway_write_eir( void )
{
    uint8_t *pBuf;
    uint8_t *p;
    uint8_t *p_tmp;
    uint8_t nb_uuid = CONSTANT_ZERO;
    uint8_t length;

    /* Allocating a buffer from the public pool */
    pBuf = (uint8_t*) wiced_bt_get_buffer ( WICED_HS_EIR_BUF_MAX_SIZE);

    if (!pBuf)
    {
        WICED_BT_TRACE ("Buffer allocation for EIR Data failed \n");
        return;
    }

    p = pBuf;

    length = (uint8_t)strlen((char*)hfp_audio_gateway_cfg_settings.device_name);
    UINT8_TO_STREAM (p, length + CONSTANT_ONE);
    UINT8_TO_STREAM (p, BT_EIR_COMPLETE_LOCAL_NAME_TYPE);
    memcpy (p, hfp_audio_gateway_cfg_settings.device_name, length);
    p += length;

    /* Add other BR/EDR UUIDs */
    p_tmp = p;
    p++;
    UINT8_TO_STREAM (p, BT_EIR_COMPLETE_16BITS_UUID_TYPE);
    UINT16_TO_STREAM (p, UUID_SERVCLASS_AUDIO_SOURCE);
    nb_uuid++;

    /* Now, we can update the UUID Tag's length */
    UINT8_TO_STREAM (p_tmp, (nb_uuid * LEN_UUID_16) + CONSTANT_ONE);

    /* Last Tag */
    UINT8_TO_STREAM (p, CONSTANT_ZERO);

    /* print EIR data */
    printf( "Current device's EIR (Extended Inquiry Response):");
    for(int i = CONSTANT_ZERO; i < MIN( p-( uint8_t* )pBuf,CONSTANT_HUNDRED );
        i++)
    {
        printf("%c",(char)pBuf[i+CONSTANT_ONE]);
    }
    wiced_bt_dev_write_eir( pBuf, (uint16_t)(p - pBuf) );

    /* Allocated buffer not anymore needed. Free it */
    wiced_bt_free_buffer (pBuf);
}


/******************************************************************************
 * Function Name: hfp_audio_gateway_write_nvram
 *******************************************************************************
 * Summary:
 *  Function to write information in NVRAM. but it will not be stored.
 *
 * Parameters:
 *  nvram_id: NVRAM Id
 *  data_len: Length of the data to be written
 *  p_data: Data to be written
 *
 * Return:
 *          Number of bytes written
 *
 *****************************************************************************/
uint16_t hfp_audio_gateway_write_nvram(int nvram_id, int data_len, void *p_data)
{
    uint16_t bytes_written = CONSTANT_ZERO;
    uint8_t i=CONSTANT_ZERO;

    if (NULL != p_data)
    {
        bytes_written=data_len;
        memcpy(&nv_key_ram,p_data,sizeof(wiced_bt_device_link_keys_t));
        WICED_BT_TRACE("Link Key is ");
        for (i=CONSTANT_ZERO; i<LINK_KEY_LEN; i++) {
            WICED_BT_TRACE("%02x ",nv_key_ram.key_data.br_edr_key[i]);
        }
        WICED_BT_TRACE("\r\n");

        WICED_BT_TRACE("NVRAM ID:%d written :%d bytes\n", nvram_id, 
            bytes_written);
    }

    return (bytes_written);
}

/* ****************************************************************************
 * Function Name: a_store_device_info
 ******************************************************************************
 * Summary:
 *  Function to store BT device information ( BT device Address, name ,
 *  RSSI value
 *
 * Parameters:
 *  p_inquiry_result - Inquiry result consisting BD address,
 *                      RSSI and Device class
 *  p_eir_data - EIR data
 *
 * Return:
 *  None
 *
 * ***************************************************************************/
void a_store_device_info(wiced_bt_dev_inquiry_scan_result_t *p_inquiry_result,
                                uint8_t *p_eir_data)
{
    if (device_count >= MAX_DEVICES)
    {
        printf("Device info storage is full. Cannot store more devices.\n");
        return;
    }

    /* Store the Bluetooth Device Address */
    memcpy(device_info_list[device_count].remote_bd_addr,
            p_inquiry_result->remote_bd_addr, BD_ADDR_LEN);

    /* Store the EIR data if available */
    if (p_eir_data != NULL)
    {
        uint8_t len = *p_eir_data; // First byte is the length of the EIR data
        if (len > CONSTANT_ZERO && len < MAX_EIR_LEN)
        {
            memcpy(device_info_list[device_count].eir_data, p_eir_data, len);
        }
    }

    /* Store the RSSI value */
    device_info_list[device_count].rssi = p_inquiry_result->rssi;

    /* Increment the device count */
    device_count++;
}

/* ****************************************************************************
 * Function Name: hfp_audio_gateway_bt_inquiry_result_cback
 ******************************************************************************
 * Summary:
 *  Handle Inquiry result callback from the stack, format and
 *  send event over UART
 *
 * Parameters:
 *  p_inquiry_result - Inquiry result consisting BD address,
 *                      RSSI and Device class
 *  p_eir_data - EIR data
 *
 * Return:
 *  None
 *
 * ***************************************************************************/
void hfp_audio_gateway_bt_inquiry_result_cback(
    wiced_bt_dev_inquiry_scan_result_t *p_inquiry_result,
    uint8_t *p_eir_data)
{
    if (p_inquiry_result == NULL)
    {
        WICED_BT_TRACE( "Scanning complete.\r\n");
        hfp_audio_gateway_bt_inquiry (CONSTANT_ZERO);
        initiate_bt_device_connection = true;
    }
    else
    {
        a_store_device_info(p_inquiry_result, p_eir_data);
    }
}

/* ****************************************************************************
 * Function Name: hfp_audio_gateway_bt_inquiry
 ******************************************************************************
 * Summary:
 *  Handle Inquiry command from user
 *
 * Parameters:
 *  enable - Enable Inquiry if 1, Cancel if 0
 *
 * Return:
 *  wiced_result_t: result of start or cancel inquiry operation initiation
 *
 * ***************************************************************************/
wiced_result_t hfp_audio_gateway_bt_inquiry( uint8_t enable )
{
    wiced_result_t           result;
    wiced_bt_dev_inq_parms_t params;

    if(enable)
    {
        memset(&params, CONSTANT_ZERO, sizeof( params ));

        params.mode             = BTM_GENERAL_INQUIRY;
        params.duration         = INQUIRY_DURATION;
        params.filter_cond_type = BTM_CLR_INQUIRY_FILTER;

        result = wiced_bt_start_inquiry(&params, 
            &hfp_audio_gateway_bt_inquiry_result_cback);
        if (result == WICED_BT_PENDING)
        {
            result = WICED_BT_SUCCESS;
        }
        WICED_BT_TRACE("Inquiry started:%d\n", result);
    }
    else
    {
        result = wiced_bt_cancel_inquiry();
        WICED_BT_TRACE("Cancel inquiry:%d\n", result);
    }
    return result;
}

/* ****************************************************************************
 * Function Name: hfp_audio_gateway_bt_set_visibility
 ******************************************************************************
 * Summary:
 *  Handle Set Visibility command
 *
 * Parameters:
 *  discoverability: Discoverable if 1, Non-discoverable if 0
 *  connectability: Connectable if 1, Non-connectable if 0
 *
 * Return:
 *  wiced_result_t: result of set visibility operation
 *
 * ***************************************************************************/
wiced_result_t hfp_audio_gateway_bt_set_visibility(uint8_t discoverability, 
    uint8_t connectability)
{
    wiced_result_t status = WICED_BT_SUCCESS;

    if ((CONSTANT_ONE < discoverability) || (CONSTANT_ONE < connectability))
    {
        WICED_BT_TRACE( "Invalid Input \n");
        status = WICED_BT_ERROR;
    }
    else if ((CONSTANT_ZERO != discoverability) &&
        (CONSTANT_ZERO == connectability))
    {
        /* we cannot be discoverable and not connectable */
        WICED_BT_TRACE("we cannot be discoverable and not connectable \n");
        status = WICED_BT_ERROR;
    }
    else
    {
        wiced_bt_dev_set_discoverability((CONSTANT_ZERO != discoverability) ? 
            BTM_GENERAL_DISCOVERABLE : BTM_NON_DISCOVERABLE,
            BTM_DEFAULT_DISC_WINDOW,
            BTM_DEFAULT_DISC_INTERVAL);

        wiced_bt_dev_set_connectability((CONSTANT_ZERO != connectability) ? 
            WICED_TRUE : WICED_FALSE,
            BTM_DEFAULT_CONN_WINDOW,
            BTM_DEFAULT_CONN_INTERVAL);
    }
    return status;
}

/* ****************************************************************************
 * Function Name: hfp_audio_gateway_bt_set_pairability
 ******************************************************************************
 * Summary:
 *  Handle Set Pairability command
 *
 * Parameters:
 *  pairing_allowed: Pairing allowed if 1, not allowed if 0
 *
 * Return:
 *   None
 *
 * ***************************************************************************/
void hfp_audio_gateway_bt_set_pairability( uint8_t pairing_allowed )
{
    wiced_bt_set_pairable_mode(pairing_allowed, TRUE);
    WICED_BT_TRACE( "Set the pairing allowed to %d \n", pairing_allowed );
}

/*******************************************************************************
 * Function Name: reset_link_keys
 *******************************************************************************
 * Summary: Reset and erase all stored link keys for BLE
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
 ******************************************************************************/
void reset_link_keys(void)
{
    /* Reset kv-store library, this will clear the NVM */
    if(CY_RSLT_SUCCESS == mtb_kvstore_reset(&kvstore_obj))
    {
        printf("Successfully reset kv-store library, \n Please reset the device to generate new keys!\n");
    }
    else
    {
        printf("failed to reset kv-store library\n");
    }

    /* Clear peer link keys and identity keys structure */
    memset(&bond_info, CONSTANT_ZERO, sizeof(bond_info));
    memset(&identity_keys, CONSTANT_ZERO, sizeof(identity_keys));
}


/* ****************************************************************************
 * Function Name: hfp_audio_gateway_command_connect
 ******************************************************************************
 * Summary:
 *   Connects to the A2DP Sink with the given BD-Address
 *
 * Parameters:
 *   bd_addr: Remote BD Address
 *  len: Length of the BD-address.
 *
 * Return:
 *  status: result of a2dp connect API
 *
 * ***************************************************************************/
wiced_result_t hfp_audio_gateway_command_connect(
    wiced_bt_device_address_t bd_addr, uint32_t len)
{
    wiced_result_t status = WICED_BT_SUCCESS;

    if(already_connected)
    {
        status = WICED_BT_MAX_CONNECTIONS_REACHED;
    }
    else
    {
        WICED_BT_TRACE("Connecting to BDA: ");
        if(ENABLE_BT_VERBOSE_LOGS)
        {
            print_bd_address(bd_addr);
        }
        wiced_bt_hfp_ag_connect(bd_addr);

        already_connected = true;
    }

    return status;
}

/*******************************************************************************
 * Function Name: hfag_print_hfp_context
 *******************************************************************************
 * Summary:
 *   This Function prints the current HFAG connection status
 *
 * Parameters:
 *   NONE
 *
 * Return:
 *   NONE
 *
 ******************************************************************************/
void hfag_print_hfp_context(void)
{
    uint8_t i = CONSTANT_ZERO;
    printf("\n---------------HFAG CONNECTION DETAILS-----------------------\n");
    printf("BD ADDRESS \t\t APPLICATION HANDLE \t SCO INDEX\n");
    for ( i = CONSTANT_ZERO; i < HFP_AG_NUM_SCB; i++ )
    {
        printf("%02X %02X %02X %02X %02X %02X \t %X \t\t\t %X \n",
                                        ag_scb[i].hf_addr[CONSTANT_ZERO],
                                        ag_scb[i].hf_addr[CONSTANT_ONE],
                                        ag_scb[i].hf_addr[CONSTANT_TWO],
                                        ag_scb[i].hf_addr[CONSTANT_THREE],
                                        ag_scb[i].hf_addr[CONSTANT_FOUR],
                                        ag_scb[i].hf_addr[CONSTANT_FIVE],
                                        ag_scb[i].app_handle,
                                        ag_scb[i].sco_idx);
    }
    printf("---------------------------------------------------------------\n");
}

/******************************************************************************
 * Function Name: ag_get_volume
 ******************************************************************************
 * Summary:
 * Parses the input string to extract the volume level.
 * Validates that the length and volume value are within allowed limits.
 *
 * Parameters:
 * char *str  : Input string containing the volume value.
 * int len    : Length of the input string.
 *
 * Return:
 * int        : Parsed volume value (0–15) on success,
 *              or -1 on error (invalid length or volume).
 ******************************************************************************/
int ag_get_volume(char *str, int len)
{
    uint8_t volume = HFP_AG_VOLUME_INIT;
    uint8_t index = HFP_AG_INDEX_INIT;

    if (HFP_AG_VOLUME_MAX_LEN < len)
    {
        printf("[ag_get_volume] Invalid Length %d\n", len);
        volume = HFP_AG_VOLUME_INVALID;
    }
    else
    {
        while (len--)
        {
            volume = (volume * HFP_AG_DECIMAL_BASE) + (str[index] - '0');
            index++;
        }
    
        if (HFP_AG_VOLUME_MAX < volume)
        {
            printf("[ag_get_volume] Invalid volume %d\n", volume);
            volume = HFP_AG_VOLUME_INVALID;
        }
    }

    return volume;
}


/******************************************************************************
 * Function Name: hfp_audio_gateway_init
 ******************************************************************************
 * Summary:
 *  Initializes the Hands-Free Profile (HFP) Audio Gateway.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
 ******************************************************************************/
void hfp_audio_gateway_init( void )
{
    BaseType_t status;
    wiced_bt_hfp_ag_session_cb_t *p_scb = &ag_scb[CONSTANT_ZERO];
    wiced_bt_dev_status_t result;
    int i;

    memset(p_scb, CONSTANT_ZERO,
        sizeof( wiced_bt_hfp_ag_session_cb_t ) * HFP_AG_NUM_SCB);

    for ( i = CONSTANT_ZERO; i < HFP_AG_NUM_SCB; i++, p_scb++ )
    {
        p_scb->app_handle = ( uint16_t ) ( i + CONSTANT_ONE );

        if(i == CONSTANT_ZERO)
            p_scb->hf_profile_uuid = UUID_SERVCLASS_HF_HANDSFREE;
        else
            p_scb->hf_profile_uuid = UUID_SERVCLASS_HEADSET;
    }

    wiced_bt_hfp_ag_startup(&ag_scb[CONSTANT_ZERO], HFP_AG_NUM_SCB, 
        BT_AUDIO_HFP_SUPPORTED_FEATURES, hfp_ag_event_callback);

    /* Set up the SCO path to be routed over HCI to the app */
    ag_sco_path.path = WICED_BT_SCO_OVER_HCI;
    ag_sco_path.p_sco_data_cb = &hfag_sco_data_app_callback;
    result = wiced_bt_sco_setup_voice_path(&ag_sco_path);

    WICED_BT_TRACE("[%s] SCO Setting up voice path = %d\n",__func__, result);

    status = xTaskCreate(hfag_sco_task, "SCO task", SCO_TASK_STACK_SIZE,
            NULL, SCO_TASK_PRIORITY, &sco_task_handle);
    if (pdPASS != status)
    {
        printf("Error in starting BT task \n");
    }
}

/******************************************************************************
 * Function Name: hfp_audio_gateway_management_callback
 *******************************************************************************
 * Summary:
 *   This is a Bluetooth stack event handler function to receive management
 *   events from the Bluetooth stack and process as per the application.
 *
 * Parameters:
 *   wiced_bt_management_evt_t event : BLE event code of one byte length
 *   wiced_bt_management_evt_data_t *p_event_data: Pointer to BTStack management
 *   event structures
 *
 * Return:
 *  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
 *
 ******************************************************************************/

wiced_result_t hfp_audio_gateway_management_callback(
    wiced_bt_management_evt_t event,
    wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_bt_ble_advert_mode_t          *p_mode; /* Advertisement Mode */
    wiced_bt_device_address_t           bda = {CONSTANT_ZERO};
    wiced_result_t                      result = WICED_BT_SUCCESS;

    wiced_bt_power_mgmt_notification_t *p_power_mgmt_notification;
    wiced_bt_dev_encryption_status_t   *p_encryption_status;
    wiced_bt_dev_pairing_cplt_t        *p_pairing_cmpl;
    int                                 pairing_result;

    WICED_BT_TRACE("[HFP Audio Gateway management callback] %s\n",
        get_bt_event_name(event));

    switch( event )
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:

            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }

            if (WICED_BT_SUCCESS == p_event_data->enabled.status)
            {
                /* Bluetooth is enabled */
                wiced_bt_dev_read_local_addr(bda);
                WICED_BT_TRACE("Local Bluetooth Address (PSOC kit): ");
                if(ENABLE_BT_VERBOSE_LOGS)
                {
                    print_bd_address(bda);
                }

                /* Set as connectable */
                result = hfp_audio_gateway_bt_set_visibility (WICED_FALSE,
                    WICED_TRUE);

                /* Check for existing data in KV store for a bonded device */
                if(CY_RSLT_SUCCESS == app_bt_restore_bond_data())
                {
                    WICED_BT_TRACE("Keys found in NVM, Adding it to Address Resolution DB\n");
                    /* Load previous paired keys for address resolution */
                    app_bt_add_devices_to_address_resolution_db();
                }

                hfp_audio_gateway_write_eir();

                /* create SDP records */
                wiced_bt_sdp_db_init((uint8_t*) hfp_audio_gateway_sdp_db,
                        sizeof(hfp_audio_gateway_sdp_db));

                hfp_audio_gateway_init();

                hfp_audio_gateway_bt_set_pairability(true);

                /* Enable the Inquiry for scanning the devices. */
                hfp_audio_gateway_bt_inquiry (true);

                printf("\nScanning for BT HFP Hands-Free devices...\n");
            }
            else
            {
                WICED_BT_TRACE("Bluetooth Enable failed \n");
            }
            break;

        case BTM_DISABLED_EVT:
            WICED_BT_TRACE("Bluetooth Disabled \n");
            break;

        case BTM_SECURITY_FAILED_EVT:
            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }
            WICED_BT_TRACE("Security failed: %d / %d\n",
                    p_event_data->security_failed.status,
                    p_event_data->security_failed.hci_status);
            break;

        case BTM_PIN_REQUEST_EVT:
            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }

            wiced_bt_dev_pin_code_reply(*p_event_data->pin_request.bd_addr,
                    WICED_BT_SUCCESS,
                    WICED_PIN_CODE_LEN, 
                    (uint8_t *)&pincode[CONSTANT_ZERO]);
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }
            /* If this is just_works pairing, accept.
             * Otherwise send event to the MCU to confirm the same value.
             */

            WICED_BT_TRACE("User confirmation request for device address: ");

            if(ENABLE_BT_VERBOSE_LOGS)
            {
                print_bd_address(p_event_data->user_confirmation_request.bd_addr);
            }

            if (p_event_data->user_confirmation_request.just_works)
            {
                WICED_BT_TRACE("User confirmation req. is just_works.\r\n");
                wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS, 
                    p_event_data->user_confirmation_request.bd_addr);
            }
            else
            {
                WICED_BT_TRACE("User confirmation req. is not supported.\r\n");
                wiced_bt_dev_confirm_req_reply(WICED_BT_UNSUPPORTED, 
                    p_event_data->user_confirmation_request.bd_addr);
            }
            break;

        case BTM_PASSKEY_NOTIFICATION_EVT:
            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }
            WICED_BT_TRACE("Passkey: %lu, BDA: ", 
                p_event_data->user_passkey_notification.passkey);

            if(ENABLE_BT_VERBOSE_LOGS)
            {
                print_bd_address(p_event_data->user_passkey_notification.bd_addr);
            }
            wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS,
                    p_event_data->user_passkey_notification.bd_addr );
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }

            p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap =
                    BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_br_edr_request.oob_data =
                    BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_br_edr_request.auth_req = 
                    BTM_AUTH_ALL_PROFILES_NO;
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }

            p_event_data->pairing_io_capabilities_ble_request.local_io_cap  =
                    BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data =
                    BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req =
                    BTM_LE_AUTH_REQ_SC_BOND;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size =
                    MAX_KEY_SIZE;
            p_event_data->pairing_io_capabilities_ble_request.init_keys =
                    BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys =
                    BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }

            p_pairing_cmpl = &p_event_data->pairing_complete;
            if(BT_TRANSPORT_BR_EDR == p_pairing_cmpl->transport)
            {
                pairing_result = p_pairing_cmpl->pairing_complete_info.br_edr.status;
            }
            else
            {
                pairing_result = p_pairing_cmpl->pairing_complete_info.ble.reason;
            }
            WICED_BT_TRACE("Pairing Result: %d\n", pairing_result);

            if (WICED_BT_SUCCESS == pairing_result )
            {
                /* Update Num of bonded devices and next free slot in slot 
                data */
                app_bt_update_slot_data();

                /* Enable here if bonding request API is called so that 
                connection happens after pairing. */
                WICED_BT_TRACE("Connection Request sent. Waiting for paired device to get connected...\r\n");
            }

            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }
            p_encryption_status = &p_event_data->encryption_status;

            WICED_BT_TRACE("Encryption status result:%u, BDA: ",
                    p_encryption_status->result );
            if(ENABLE_BT_VERBOSE_LOGS)
            {
                print_bd_address(p_encryption_status->bd_addr);
            }
            
            /* Check and retreive the index of the bond data of the device that
             * got connected */
            /* This call will return BOND_INDEX_MAX if the device is not found*/
            bond_index = app_bt_find_device_in_nvm(
                p_event_data->encryption_status.bd_addr);
            if (BOND_INDEX_MAX > bond_index)
            {
                app_bt_restore_bond_data();
                app_bt_restore_cccd();
                
                /*Set CCCD value from the value that was previously saved in 
                NVRAM*/
                app_hello_sensor_notify_client_char_config[CONSTANT_ZERO] = 
                    peer_cccd_data[bond_index];

                WICED_BT_TRACE("Bond info present in NVM for device: ");
                if(ENABLE_BT_VERBOSE_LOGS)
                {
                    print_bd_address(p_event_data->encryption_status.bd_addr);
                }
            }
            else
            {
                WICED_BT_TRACE("No Bond info present in NVM for device: ");
                if(ENABLE_BT_VERBOSE_LOGS)
                {
                    print_bd_address(p_event_data->encryption_status.bd_addr);
                }
                bond_index = CONSTANT_ZERO;
            }
            break;

        case BTM_SECURITY_REQUEST_EVT:
            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }

            wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr,
                    WICED_BT_SUCCESS );
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }
            WICED_BT_TRACE("Paired device link key size:%d, BDA: ",
                    sizeof(wiced_bt_device_link_keys_t));

            if(ENABLE_BT_VERBOSE_LOGS)
            {
                print_bd_address(p_event_data->paired_device_link_keys_update.bd_addr);
            }

            /* This application supports a single paired host, we can save keys
             * under the same NVRAM ID overwriting previous pairing if any */
            hfp_audio_gateway_write_nvram(HFP_AUDIO_GATEWAY_NVRAM_ID,
                    sizeof(wiced_bt_device_link_keys_t), 
                        &p_event_data->paired_device_link_keys_update);

            /* save device keys to NVRAM */
            if (CY_RSLT_SUCCESS == app_bt_save_device_link_keys(
                    &(p_event_data->paired_device_link_keys_update)))
            {
                printf("Successfully Bonded to ");
                print_bd_address(p_event_data->paired_device_link_keys_update.bd_addr);
            }
            else
            {
                printf("Failed to bond! \n");
            }

            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }
            WICED_BT_TRACE("Paired device link key size:%d, BDA: ",
                    sizeof(wiced_bt_device_link_keys_t));
            if(ENABLE_BT_VERBOSE_LOGS)
            {
                print_bd_address(p_event_data->paired_device_link_keys_request.bd_addr);
            }

            /* Need to search to see if the BD_ADDR we are
             * looking for is in NVRAM. If not, we return WICED_BT_ERROR
             * and the stack will generate keys and will then call
             * BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT so that they
             * can be stored
             */

            /* Assume the device won't be found.
             * If it is, we will set this back to WICED_BT_SUCCESS */
            result = WICED_BT_ERROR;

            bond_index = app_bt_find_device_in_nvm(
                p_event_data->paired_device_link_keys_request.bd_addr);
            if(BOND_INDEX_MAX > bond_index)
            {
                /* Copy the keys to where the stack wants it */
                memcpy(&(p_event_data->paired_device_link_keys_request),
                        &bond_info.link_keys[bond_index],
                        sizeof(wiced_bt_device_link_keys_t));
                result = WICED_BT_SUCCESS;

                WICED_BT_TRACE("Paired device Link Keys found. BDA: ");
                if(ENABLE_BT_VERBOSE_LOGS)
                {
                    print_bd_address(p_event_data->paired_device_link_keys_request.bd_addr);
                }
            }
            else
            {
                WICED_BT_TRACE("Paired device Link Keys not found in the database!\n\n");
                bond_index = CONSTANT_ZERO;
            }
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }
            /* Update of local privacy keys - save to NVRAM */
            if (CY_RSLT_SUCCESS != app_bt_save_local_identity_key(
                    p_event_data->local_identity_keys_update))
            {
                result = WICED_BT_ERROR;
            }else
            {
                result = WICED_BT_SUCCESS;
            }

            break;

        case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }

            /* Read Local Identity Resolution Keys if present in NVRAM*/
            if(CY_RSLT_SUCCESS == app_bt_read_local_identity_keys())
            {
                memcpy(&(p_event_data->local_identity_keys_request),
                    &(identity_keys),sizeof(wiced_bt_local_identity_keys_t));
                WICED_BT_TRACE("Local Identity keys\n");
                if(ENABLE_BT_VERBOSE_LOGS)
                {
                    print_array(&identity_keys,
                            sizeof(wiced_bt_local_identity_keys_t));
                }
                result = WICED_BT_SUCCESS;
            }
            else
            {
                result = WICED_BT_ERROR;
            }
            break;

        case BTM_POWER_MANAGEMENT_STATUS_EVT:
            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }
            p_power_mgmt_notification = &p_event_data->power_mgmt_notification;

            WICED_BT_TRACE("Power management event status:%d, hci_status:%d, BDA:",
                    p_power_mgmt_notification->status,
                    p_power_mgmt_notification->hci_status);
            if(ENABLE_BT_VERBOSE_LOGS)
            {
                print_bd_address(p_power_mgmt_notification->bd_addr);
            }
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }
            p_mode = &p_event_data->ble_advert_state_changed;

            WICED_BT_TRACE("Advertisement state changed: %d\n", *p_mode);
            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }

            /* Connection parameters updated */
            if(WICED_SUCCESS == p_event_data->ble_connection_param_update.status)
            {
                WICED_BT_TRACE("Supervision Time Out = %d\n",
                    (p_event_data->ble_connection_param_update.supervision_timeout 
                        * CONSTANT_TEN));
            }
            break;

        case BTM_SCO_CONNECTED_EVT:
            WICED_BT_TRACE("BTM_SCO_CONNECTED_EVT\n");
            active_sco_index = p_event_data->sco_connected.sco_index;
            break;

        case BTM_SCO_DISCONNECTED_EVT:
            WICED_BT_TRACE("BTM_SCO_DISCONNECTED_EVT\n");
            active_sco_index = 0xFFFF;
            break;
            
        case BTM_SCO_CONNECTION_REQUEST_EVT:
        case BTM_SCO_CONNECTION_CHANGE_EVT:
            WICED_BT_TRACE("Recv SCO Bluetooth Management Event: 0x%x %s\n",
                event, get_bt_event_name(event));
            break;

        default:
            WICED_BT_TRACE("Unhandled Bluetooth Management Event: 0x%x %s\n",
                event, get_bt_event_name(event));
            break;
        }
        return result;
}

/******************************************************************************
 * Function Name: hfp_ag_volume_up
 ******************************************************************************
 * Summary:
 *  Increases the Audio Gateway (AG) speaker volume by one step.
 *  If the current volume is below the maximum (15), it is incremented
 *  and the new value is sent to the Hands-Free (HF) device using
 *  the AT+VGS command.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 ******************************************************************************/
void hfp_ag_volume_up(void)
{
    if (HFP_AG_VOLUME_MAX > spk_volume)
    {
        spk_volume++;
        printf("Setting HF's speaker volume gain to %d\r\n", spk_volume);
        wiced_bt_hfp_ag_send_VGS_to_hf(&ag_scb[CONSTANT_ZERO], spk_volume);
    }
}

/******************************************************************************
 * Function Name: hfp_ag_volume_down
 ******************************************************************************
 * Summary:
 *  Decreases the Audio Gateway (AG) speaker volume by one step.
 *  If the current volume is above CONSTANT_ZERO, it is decremented
 *  and the new value is sent to the Hands-Free (HF) device using
 *  the AT+VGS command.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 ******************************************************************************/
void hfp_ag_volume_down(void)
{
    if (spk_volume > CONSTANT_ZERO)
    {
        spk_volume--;
        printf("Setting HF's speaker volume gain to %d\r\n", spk_volume);
        wiced_bt_hfp_ag_send_VGS_to_hf(&ag_scb[CONSTANT_ZERO], spk_volume);
    }
}

/******************************************************************************
 * Function Name: hfp_ag_simulate_call_ring_timer_init
 ******************************************************************************
 * Summary:
 *  Initializes the call ring simulation timer for the HFP Audio Gateway.
 *
 * Parameters:
 *  None
 *
 * Return:
 *   None
 *
 ******************************************************************************/
void hfp_ag_simulate_call_ring_timer_init(void)
{
    if (call_ring_timer_handle != NULL)
    {
        return; /* Timer already created */
    }

    call_ring_timer_handle = xTimerCreate("Call RING Timer",
                                       pdMS_TO_TICKS(CALL_RING_TIMER_PERIOD_MS),
                                       pdTRUE, // auto-reload
                                       (void *)CONSTANT_ZERO,
                                       hfp_ag_simulate_call_ring_send_callback);

    if (call_ring_timer_handle == NULL)
    {
        printf("Failed to create timer!\n");
    }
}

/******************************************************************************
 * Function Name: hfp_ag_simulate_call_ring_timer_start
 ******************************************************************************
 * Summary:
 *  Starts the call ring simulation timer if it has been created.
 *  Once started, the timer periodically invokes the callback to
 *  simulate call ring notifications.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 ******************************************************************************/
void hfp_ag_simulate_call_ring_timer_start(void)
{
    if (call_ring_timer_handle != NULL)
    {
        xTimerStart(call_ring_timer_handle, CONSTANT_ZERO);
    }
}

/******************************************************************************
 * Function Name: hfp_ag_simulate_call_ring_timer_stop
 ******************************************************************************
 * Summary:
 *  Stops and deletes the call ring simulation timer.
 *
 * Parameters:
 *  None
 *
 * Return:
 *   None
 ******************************************************************************/
void hfp_ag_simulate_call_ring_timer_stop(void)
{
    if (call_ring_timer_handle != NULL)
    {
        xTimerStop(call_ring_timer_handle, HFP_AG_TIMER_BLOCK_TIME);
        xTimerDelete(call_ring_timer_handle, HFP_AG_TIMER_BLOCK_TIME);
        call_ring_timer_handle = NULL;
    }
}

/******************************************************************************
 * Function Name: hfp_ag_simulate_call_start
 ******************************************************************************
 * Summary:
 *  Simulates an incoming call on the HFP Audio Gateway.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 ******************************************************************************/
void hfp_ag_simulate_call_start(void)
{
    ag_call_status.idx = CONSTANT_ONE;
    ag_call_status.dir = WICED_BT_HFP_HF_INCOMING;
    ag_call_status.status = WICED_BT_HFP_HF_CALL_INCOMING;
    ag_call_status.mode = WICED_BT_HFP_HF_MODE_VOICE;
    ag_call_status.is_conference = false;
    
    strncpy((char *)ag_call_status.num, PHONE_NUM, 
        sizeof(ag_call_status.num) - CONSTANT_ONE);

    /*ensure null-termination*/ 
    ag_call_status.num[sizeof(ag_call_status.num) - CONSTANT_ONE] = '\0'; 
    ag_call_status.type = CALL_TYPE_INTERNATIONAL; /*International*/

    ag_call_state = WICED_BT_HFP_HF_CALLSETUP_STATE_INCOMING;

    printf("Simulating Incoming Call!!!\r\n");
    
    hfp_ag_simulate_call_ring_timer_init();

    /* Start sending RING every 2 sec */
    hfp_ag_simulate_call_ring_timer_start();

    wiced_bt_hfp_ag_send_RING_to_hf(&ag_scb[CONSTANT_ZERO]);

    /* Notify HF: Call setup = Incoming */
    snprintf(&cmd_str[CONSTANT_ZERO], COMMAND_STRING_SIZE, "+CIEV: %d,%d",
                            CONSTANT_TWO,
                            ag_call_state);
    wiced_bt_hfp_ag_send_cmd_str_to_hf(&ag_scb[CONSTANT_ZERO], 
        &cmd_str[CONSTANT_ZERO]);

    /* Optional: Send Caller ID if supported */
    snprintf(&cmd_str[CONSTANT_ZERO], COMMAND_STRING_SIZE, "+CLIP: %s,%d",
                            ag_call_status.num,
                            ag_call_status.type);
    wiced_bt_hfp_ag_send_cmd_str_to_hf(&ag_scb[CONSTANT_ZERO], 
        &cmd_str[CONSTANT_ZERO]);
}


/******************************************************************************
 * Function Name: hfp_ag_simulate_call_answer
 ******************************************************************************
 * Summary:
 *  Simulates answering an incoming call on the HFP Audio Gateway.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 ******************************************************************************/
void hfp_ag_simulate_call_answer(void)
{
    /* Stop ringing */
    hfp_ag_simulate_call_ring_timer_stop();
    
    printf("Incoming Call is accepted!\r\n");

    agIs_call_active = true;
    ag_call_state = WICED_BT_HFP_HF_CALLSETUP_STATE_IDLE;

    /* Update HF indicators: call active, setup finished */
    snprintf(&cmd_str[CONSTANT_ZERO], COMMAND_STRING_SIZE, "+CIEV: %d,%d",
                            CONSTANT_ONE,
                            agIs_call_active);
    wiced_bt_hfp_ag_send_cmd_str_to_hf(&ag_scb[CONSTANT_ZERO], 
        &cmd_str[CONSTANT_ZERO]);

    /* Notify HF: Call setup = Idle as it is answered. */
    snprintf(&cmd_str[CONSTANT_ZERO], COMMAND_STRING_SIZE, "+CIEV: %d,%d",
                            CONSTANT_TWO,
                            ag_call_state);
    wiced_bt_hfp_ag_send_cmd_str_to_hf(&ag_scb[CONSTANT_ZERO], 
        &cmd_str[CONSTANT_ZERO]);

    wiced_bt_hfp_ag_audio_open(ag_connection_handle);

    /* Enable I2S */
    app_i2s_enable();
    
    /* Activate and enable I2S TX interrupts */
    app_i2s_activate();
    app_pdm_pcm_activate();
}

/******************************************************************************
 * Function Name: hfp_ag_simulate_call_end
 ******************************************************************************
 * Summary:
 *  Simulates ending an active or ringing call on the HFP Audio Gateway.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 ******************************************************************************/
void hfp_ag_simulate_call_end(void)
{
    /* Stop RING if active */
    hfp_ag_simulate_call_ring_timer_stop();

    printf("Incoming Call ended!\r\n");

    agIs_call_active = false;
    ag_call_state = WICED_BT_HFP_HF_CALLSETUP_STATE_IDLE;

    /* Update HF indicators: call ended */
    snprintf(&cmd_str[CONSTANT_ZERO], COMMAND_STRING_SIZE, "+CIEV: %d,%d",
                            CONSTANT_ONE,
                            agIs_call_active);
    wiced_bt_hfp_ag_send_cmd_str_to_hf(&ag_scb[CONSTANT_ZERO], 
        &cmd_str[CONSTANT_ZERO]);

    snprintf(&cmd_str[CONSTANT_ZERO], COMMAND_STRING_SIZE, "+CIEV: %d,%d",
                            2,
                            ag_call_state);
    wiced_bt_hfp_ag_send_cmd_str_to_hf(&ag_scb[CONSTANT_ZERO], 
        &cmd_str[CONSTANT_ZERO]);

    wiced_bt_hfp_ag_audio_close(ag_connection_handle);

    /* Disable and de-activate I2S TX interrupts */
    app_i2s_deactivate();
    /* Disable I2S */
    app_i2s_disable();

   app_pdm_pcm_deactivate();
}

/******************************************************************************
 * Function Name: hfp_ag_simulate_call_ring_send_callback
 ******************************************************************************
 * Summary:
 *  Callback function for the RING timer.
 *
 * Parameters:
 *  xTimer - FreeRTOS timer handle (unused in this implementation).
 *
 * Return:
 *  None
 ******************************************************************************/
static void hfp_ag_simulate_call_ring_send_callback(TimerHandle_t xTimer)
{
    printf("Ring\r\n");
    /* Send RING repeatedly until answered/rejected */
    wiced_bt_hfp_ag_send_RING_to_hf(&ag_scb[CONSTANT_ZERO]);
}

/******************************************************************************
 * Function Name: hfag_get_ag_event_name
 ******************************************************************************
 * Summary:
 * The function converts the wiced_bt_hfp_ag_event_t enum value to its
 * corresponding string literal. This will help the programmer to debug easily
 * with log traces without navigating through the source code.
 *
 * Parameters:
 *  wiced_bt_hfp_ag_event_t event: Bluetooth HF AG event type
 *
 * Return:
 *  char *: String for wiced_bt_management_evt_t
 *
 ******************************************************************************/
static const char *hfag_get_ag_event_name( wiced_bt_hfp_ag_event_t event )
{
    switch ( (int)event )
    {
        CASE_RETURN_STR( WICED_BT_HFP_AG_EVENT_OPEN )
        CASE_RETURN_STR( WICED_BT_HFP_AG_EVENT_CLOSE )
        CASE_RETURN_STR( WICED_BT_HFP_AG_EVENT_CONNECTED )
        CASE_RETURN_STR( WICED_BT_HFP_AG_EVENT_AUDIO_OPEN )
        CASE_RETURN_STR( WICED_BT_HFP_AG_EVENT_AUDIO_CLOSE )
        CASE_RETURN_STR( WICED_BT_HFP_AG_EVENT_AT_CMD )
        CASE_RETURN_STR( WICED_BT_HFP_AG_EVENT_CLCC_REQ )
        CASE_RETURN_STR( WICED_BT_HFP_AG_EVENT_NREC_CMD )
        default:
          printf("Unknorn event: %d\n", event);
          break;
    }
    return NULL;
}

/*******************************************************************************
 * Function Name: hfag_event_cback
 *******************************************************************************
 * Summary:
 *   This is a Bluetooth stack event handler function to receive HFAG related
 *   events from the stack and process as per the application.
 *
 * Parameters:
 *   wiced_bt_hfp_ag_event_t evt: AG event
 *   uint16_t handle: app handle
 *   hfp_ag_event_t *p_data: event data
 *
 * Return:
 *   NONE
 *
 ******************************************************************************/
static void hfp_ag_event_callback(wiced_bt_hfp_ag_event_t evt, uint16_t handle,
            wiced_bt_hfp_ag_event_data_t *p_data)
{
    WICED_BT_TRACE("### %s: hdl = %d evt = %x: %s\n", __FUNCTION__, handle, 
        evt, hfag_get_ag_event_name( evt ));

    switch( evt )
    {
    case WICED_BT_HFP_AG_EVENT_OPEN:
        if ( NULL != p_data )
        {
            printf("----------------------------------------------------------\n");
            printf("WICED_BT_HFP_AG_EVENT_OPEN: Open status = %s\n", 
                (p_data->open.status == CONSTANT_ZERO) ? "Success" : "Failed");
            printf("----------------------------------------------------------\n");
        }
        connection_status = HFP_PEER_CONNECTED;
        break;

    case WICED_BT_HFP_AG_EVENT_CLOSE:
        connection_status = HFP_PEER_DISCONNECTED;
        hfag_print_hfp_context();
        break;

    case WICED_BT_HFP_AG_EVENT_CONNECTED:
        if ( NULL != p_data )
        {
            printf("----------------------------------------------------------\n");
            printf("WICED_BT_HFP_AG_EVENT_CONNECTED with Peer Features  %04lx\n",
                (unsigned long)p_data->conn.peer_features);
            printf("----------------------------------------------------------\n");
        }
        already_connected = true;
        initiate_bt_device_connection = false;
        ag_connection_handle = handle;
        hfag_print_hfp_context();
        break;

    case WICED_BT_HFP_AG_EVENT_AUDIO_OPEN:
        WICED_BT_TRACE("SCO Audio Opened\n");
        active_sco_index = CONSTANT_ZERO;
        hfag_print_hfp_context();
        break;

    case WICED_BT_HFP_AG_EVENT_AUDIO_CLOSE:
        WICED_BT_TRACE("SCO Audio Closed\n");
        active_sco_index = 0xFFFF;
        hfag_print_hfp_context();
        break;

    case WICED_BT_HFP_AG_EVENT_AT_CMD:
        if(strncmp((const char *)p_data->at_cmd.cmd_ptr, AT_SPK_VOLUME_EVT_STR,
             strlen(AT_SPK_VOLUME_EVT_STR)) == CONSTANT_ZERO)
        {
            int spk_evt_len = strlen(AT_SPK_VOLUME_EVT_STR);
            int volume = ag_get_volume(
                (char *)&p_data->at_cmd.cmd_ptr[spk_evt_len],
                p_data->at_cmd.cmd_len-spk_evt_len-CONSTANT_ONE);
            if (volume != -1)
            {
                spk_volume = volume;
                WICED_BT_TRACE("SPK Volume: [%d]\r\n", volume);
            }
        }

        if(strncmp((const char *)p_data->at_cmd.cmd_ptr, AT_MIC_VOLUME_EVT_STR,
                strlen(AT_MIC_VOLUME_EVT_STR)) == CONSTANT_ZERO)
        {
            int mic_evt_len = strlen(AT_MIC_VOLUME_EVT_STR);
            int volume = ag_get_volume((
                char *)&p_data->at_cmd.cmd_ptr[mic_evt_len],
                p_data->at_cmd.cmd_len-mic_evt_len-1);
            if (volume != -1)
            {
                WICED_BT_TRACE("MIC Volume: [%d]\r\n", volume);
            }
        }

        if(strncmp((const char *)p_data->at_cmd.cmd_ptr, AT_ANSWER_CALL_EVT_STR,
                strlen(AT_ANSWER_CALL_EVT_STR)) == CONSTANT_ZERO)
        {
            wiced_bt_hfp_ag_send_OK_to_hf(&ag_scb[CONSTANT_ZERO]);
            hfp_ag_simulate_call_answer();
        }

        if(strncmp((const char *)p_data->at_cmd.cmd_ptr,AT_DECLINE_CALL_EVT_STR, 
                strlen(AT_DECLINE_CALL_EVT_STR)) == CONSTANT_ZERO)
        {
            wiced_bt_hfp_ag_send_OK_to_hf(&ag_scb[CONSTANT_ZERO]);
            hfp_ag_simulate_call_end();
        }

        break;

    case WICED_BT_HFP_AG_EVENT_CLCC_REQ:
        snprintf(&cmd_str[CONSTANT_ZERO], COMMAND_STRING_SIZE, 
            "+CLCC: %d,%d,%d,%d,%d,%s,%d",
            ag_call_status.idx,
            ag_call_status.dir,
            ag_call_status.status,
            ag_call_status.mode,
            ag_call_status.is_conference,
            ag_call_status.num,
            ag_call_status.type);

        wiced_bt_hfp_ag_send_cmd_str_to_hf(&ag_scb[CONSTANT_ZERO], 
            &cmd_str[CONSTANT_ZERO]);
        break;

    default:
        break;
    }
}

/*******************************************************************************
 * Function Name: hfag_sco_data_app_callback
 *******************************************************************************
 * Summary:
 *   callback function called for incoming pcm data
 *
 * Parameters:
 *   uint16_t sco_channel : sco channel
 *   uint16_t length : SCO data callback length
 *   uint8_t* p_data : incoming SCO pcm data
 *
 * Return:
 *   NONE
 *
 ******************************************************************************/
static void hfag_sco_data_app_callback(uint16_t sco_channel, uint16_t length, 
        uint8_t* p_data)
{
    bt_sco_data_callback(sco_channel, p_data, length);
}

/*******************************************************************************
 * Function Name: hfag_sco_task
 *******************************************************************************
 * Summary:
 *   Task function for sending recieved SCO data to speaker at 2 MS interval.
 *
 * Parameters:
 *   Task parameters.
 *
 * Return:
 *   NONE
 *
 ******************************************************************************/
static void hfag_sco_task(void *task_params)
{
    for(;;)
    {
        if(already_connected)
        {
            sco_process_data();
        }

        vTaskDelay(pdMS_TO_TICKS(SCO_TASK_DELAY_2MS));
    }
}

/* [] END OF FILE */
