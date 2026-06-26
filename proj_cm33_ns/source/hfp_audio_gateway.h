/******************************************************************************
* File Name:   hfp_audio_gateway.h
*
* Description: Header file for HFP Audio Gateway task.
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
#ifndef HFP_AUDIO_GATEWAY_H
#define HFP_AUDIO_GATEWAY_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/******************************************************************************
* Includes
*****************************************************************************/
#include "app_bt_utils/app_bt_utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <stdbool.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <ctype.h>
#include <string.h>
#include "string.h"
#include "wiced_bt_stack.h"
#include "wiced_memory.h"
#include "wiced_timer.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_types.h"
#include "wiced_bt_l2c.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_trace.h"
#include "hcidefs.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_uuid.h"
#include "wiced_result.h"
#include "wiced_timer.h"
#include "wiced_bt_hfp_ag.h"

/******************************************************************************
* Macros
*****************************************************************************/
#define HFP_PEER_CONNECTED              (1U)
#define HFP_PEER_DISCONNECTED           (0U)

#define BDA_ADDRESS_BYTE_0              (0x2CU)
#define BDA_ADDRESS_BYTE_1              (0xFDU)
#define BDA_ADDRESS_BYTE_2              (0xB4U)
#define BDA_ADDRESS_BYTE_3              (0xE6U)
#define BDA_ADDRESS_BYTE_4              (0x6CU)
#define BDA_ADDRESS_BYTE_5              (0x45U)

/* Maximum number of devices to store */
#define MAX_DEVICES                     (9U)

/* Maximum length of EIR data */
#define MAX_EIR_LEN                     (240U)   

#define HFP_AUDIO_GATEWAY_SDP_DB_SIZE   (64U)
#define ENABLE_BT_VERBOSE_LOGS          (1U)

#define WICED_PIN_CODE_LEN              (4U)

/* Max simultaneous connections to HFs */
 #define HFP_AG_NUM_SCB                 (2U)

#if (BTM_WBS_INCLUDED == TRUE )
#define BT_AUDIO_HFP_SUPPORTED_FEATURES (HFP_AG_FEAT_VREC | \
                                         HFP_AG_FEAT_CODEC | \
                                         HFP_AG_FEAT_ESCO)
#define AG_SUPPORTED_FEATURES_ATT       (WICED_BT_HFP_AG_SDP_FEATURE_VRECG | \
                                         WICED_BT_HFP_AG_SDP_FEATURE_WIDEBAND_SPEECH)
#else
#define BT_AUDIO_HFP_SUPPORTED_FEATURES (HFP_AG_FEAT_VREC | HFP_AG_FEAT_ESCO)
#define AG_SUPPORTED_FEATURES_ATT       (WICED_BT_HFP_AG_SDP_FEATURE_VRECG)
#endif 

/******************************************************************************
* Structures
*****************************************************************************/
typedef struct {
    uint8_t remote_bd_addr[BD_ADDR_LEN];  /* Bluetooth Device Address */
    uint8_t eir_data[MAX_EIR_LEN];        /* EIR data */
    int8_t  rssi;                          /* RSSI value */
} device_info_t;

/*******************************************************************************
* Global Variables
*******************************************************************************/
extern int8_t connection_status;
extern wiced_bt_heap_t *p_default_heap;
extern const wiced_bt_cfg_settings_t hfp_audio_gateway_cfg_settings;
extern volatile bool initiate_bt_device_connection;

/*******************************************************************************
* Function prototypes
*******************************************************************************/
void hfp_audio_gateway_write_eir( void );
uint16_t hfp_audio_gateway_write_nvram( int nvram_id, int data_len, void *p_data);
void a_store_device_info(wiced_bt_dev_inquiry_scan_result_t *p_inquiry_result,
                         uint8_t *p_eir_data);
void hfp_audio_gateway_bt_inquiry_result_cback(
        wiced_bt_dev_inquiry_scan_result_t *p_inquiry_result,
        uint8_t *p_eir_data);
wiced_result_t hfp_audio_gateway_bt_inquiry( uint8_t enable );
wiced_result_t hfp_audio_gateway_bt_set_visibility( uint8_t discoverability,
        uint8_t connectability );
void hfp_audio_gateway_bt_set_pairability( uint8_t pairing_allowed );
void reset_link_keys(void);
wiced_result_t hfp_audio_gateway_command_connect(
        wiced_bt_device_address_t bd_addr, uint32_t len);
void hfag_print_hfp_context(void);
void hfp_audio_gateway_init( void );
wiced_result_t hfp_audio_gateway_management_callback(
        wiced_bt_management_evt_t event,
        wiced_bt_management_evt_data_t *p_event_data);
void hfp_ag_volume_up(void);
void hfp_ag_volume_down(void);
void hfp_ag_simulate_call_ring_timer_init(void);
void hfp_ag_simulate_call_ring_timer_start(void);
void hfp_ag_simulate_call_ring_timer_stop(void);
void hfp_ag_simulate_call_start(void);
void hfp_ag_simulate_call_answer(void);
void hfp_ag_simulate_call_end(void);

#endif  /* HFP_AUDIO_GATEWAY_H */

/* [] END OF FILE */

