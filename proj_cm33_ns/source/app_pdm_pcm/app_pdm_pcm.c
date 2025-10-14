/******************************************************************************
* File Name : app_pdm_pcm.c
*
* Description : Source file for PDM PCM.
********************************************************************************
* (c) 2025, Infineon Technologies AG, or an affiliate of Infineon
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
* Header Files
*******************************************************************************/
#ifdef  WICED_BT_TRACE_ENABLE
#include "wiced_bt_trace.h"
#endif
#include "app_pdm_pcm.h"
#include "cy_utils.h"
#include "wiced_bt_types.h"

/*******************************************************************************
* Macros
********************************************************************************/
#define PCM_FRAME_SIZE           (60U)               /* SCO payload per frame */
#define SCO_NOT_CONNECTED        (0xFFFF)
#define PCM_RING_BUFFER_SIZE     (PCM_FRAME_SIZE * 100U)     /* ~750ms buffer */
#define CONSTANT_ZERO            (0U)
#define CONSTANT_ONE             (1U)
#define CONSTANT_EIGHT           (8U)

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* Recorded audio buffer in shared SoC memory */
int16_t recorded_data[NUM_CHANNELS * BUFFER_SIZE] __attribute__((section(".cy_shared_socmem"))) = {CONSTANT_ZERO};
int32_t recorded_data_size = CONSTANT_ZERO;

static uint8_t pcm_tx_buf[PCM_FRAME_SIZE];                   /* SCO TX buffer */
static uint8_t pcm_ring_buffer[PCM_RING_BUFFER_SIZE];
static volatile uint32_t pcm_ring_write = CONSTANT_ZERO;
static volatile uint32_t pcm_ring_read  = CONSTANT_ZERO;

volatile int16_t *audio_data_ptr = NULL;

extern uint16_t active_sco_index;   /* Set during BTM_SCO_CONNECTED_EVT */
extern wiced_timer_t sco_timer;

/* PDM/PCM interrupt configuration */
const cy_stc_sysint_t PDM_IRQ_cfg = {
    .intrSrc      = (IRQn_Type)PDM_IRQ,
    .intrPriority = PDM_PCM_ISR_PRIORITY
};

/******************************************************************************
 * Function Name: app_pdm_pcm_init
 ******************************************************************************
 * Summary:
 *          Initializes the PDM-PCM hardware module.

 *
 * Parameters:
 *          None
 *
 * Return:
 *          None
 *
 ******************************************************************************/
void app_pdm_pcm_init(void)
{
    cy_en_pdm_pcm_gain_sel_t gain_scale = CY_PDM_PCM_SEL_GAIN_NEGATIVE_37DB;

    if (CY_PDM_PCM_SUCCESS != Cy_PDM_PCM_Init(PDM0, &CYBSP_PDM_config))
    {
        CY_ASSERT(0);
    }

    Cy_PDM_PCM_Channel_Enable(PDM0, LEFT_CH_INDEX);
    Cy_PDM_PCM_Channel_Enable(PDM0, RIGHT_CH_INDEX);

    Cy_PDM_PCM_Channel_Init(PDM0, &LEFT_CH_CONFIG,  (uint8_t)LEFT_CH_INDEX);
    Cy_PDM_PCM_Channel_Init(PDM0, &RIGHT_CH_CONFIG, (uint8_t)RIGHT_CH_INDEX);

    /* Set the gain for both left and right channels. */
    gain_scale = convert_db_to_pdm_scale((double)PDM_MIC_GAIN_VALUE);
    set_pdm_pcm_gain(gain_scale);

    Cy_PDM_PCM_Channel_ClearInterrupt(PDM0, RIGHT_CH_INDEX, CY_PDM_PCM_INTR_MASK);
    Cy_PDM_PCM_Channel_SetInterruptMask(PDM0, RIGHT_CH_INDEX, CY_PDM_PCM_INTR_MASK);

    if(CY_SYSINT_SUCCESS != Cy_SysInt_Init(&PDM_IRQ_cfg,&pdm_interrupt_handler))
    {
        CY_ASSERT(0);
    }
    NVIC_ClearPendingIRQ(PDM_IRQ_cfg.intrSrc);
    NVIC_EnableIRQ(PDM_IRQ_cfg.intrSrc);
}


/*******************************************************************************
 * Function Name: app_i2s_activate
 *******************************************************************************
* Summary: Activate I2S Tx interrupt and enable I2S transmission
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void app_pdm_pcm_activate(void)
{
    recorded_data_size = CONSTANT_ZERO;
    audio_data_ptr = recorded_data;

    Cy_PDM_PCM_Activate_Channel(PDM0, LEFT_CH_INDEX);
    Cy_PDM_PCM_Activate_Channel(PDM0, RIGHT_CH_INDEX);
}

/*******************************************************************************
 * Function Name: pdm_interrupt_handler
 *******************************************************************************
* Summary: PDM/PCM interrupt handler for processing audio data from microphone.
*          Handles RX trigger interrupts to read FIFO data and overflow 
*          conditions.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void pdm_interrupt_handler(void)
{
    volatile uint32_t int_stat;
    int_stat = Cy_PDM_PCM_Channel_GetInterruptStatusMasked(PDM0, RIGHT_CH_INDEX);

    if (CY_PDM_PCM_INTR_RX_TRIGGER & int_stat)
    {
        for (uint8_t i = CONSTANT_ZERO; i < RX_FIFO_TRIG_LEVEL; i++)
        {
            int32_t left  = (int32_t)Cy_PDM_PCM_Channel_ReadFifo(PDM0, 
                    LEFT_CH_INDEX);
            int32_t right = (int32_t)Cy_PDM_PCM_Channel_ReadFifo(PDM0, 
                    RIGHT_CH_INDEX);

            /* Mono mix: (L+R)/2 */
            int16_t pcm_sample = (int16_t)((left + right) / AUDIO_CHANNELS_TO_MIX);

            /* Write to circular buffer (little-endian) */
            uint32_t next = (pcm_ring_write + PCM_SAMPLE_BYTES) % PCM_RING_BUFFER_SIZE;
            if (next != pcm_ring_read) /* buffer not full */
            {
                pcm_ring_buffer[pcm_ring_write]     = pcm_sample & 0xFF;
                pcm_ring_buffer[pcm_ring_write + CONSTANT_ONE] = (pcm_sample >> CONSTANT_EIGHT) & 0xFF;
                pcm_ring_write = next;
            }
        }
        Cy_PDM_PCM_Channel_ClearInterrupt(PDM0, RIGHT_CH_INDEX, 
                CY_PDM_PCM_INTR_RX_TRIGGER);
    }

    if ((CY_PDM_PCM_INTR_RX_FIR_OVERFLOW | CY_PDM_PCM_INTR_RX_OVERFLOW |
         CY_PDM_PCM_INTR_RX_IF_OVERFLOW | CY_PDM_PCM_INTR_RX_UNDERFLOW) 
         & int_stat)
    {
        Cy_PDM_PCM_Channel_ClearInterrupt(PDM0, RIGHT_CH_INDEX, 
                CY_PDM_PCM_INTR_MASK);
    }
}

/*******************************************************************************
 * Function Name: sco_process_data
 *******************************************************************************
* Summary: Timer callback function for SCO audio transmission. Reads PCM data
*          from circular buffer and sends it via SCO connection.
*
* Parameters:
*  arg: Timer parameter (unused)
*
* Return:
*  None
*
*******************************************************************************/
void sco_process_data(void)
{
    if (active_sco_index == 0xFFFF)
    {
        return;
    }

    /* Check if enough data available */
    uint32_t available = (pcm_ring_write >= pcm_ring_read)
                         ? (pcm_ring_write - pcm_ring_read)
                         : (PCM_RING_BUFFER_SIZE - pcm_ring_read + pcm_ring_write);

    if (available < PCM_FRAME_SIZE)
        return; /* Not enough mic data yet */

    /* Prepare SCO frame */
    uint8_t sco_frame[PCM_FRAME_SIZE];
    for (uint16_t i = CONSTANT_ZERO; i < PCM_FRAME_SIZE; i++)
    {
        sco_frame[i] = pcm_ring_buffer[pcm_ring_read];
        pcm_ring_read = (pcm_ring_read + CONSTANT_ONE) % PCM_RING_BUFFER_SIZE;
    }

    /* Send via SCO output stream */
    wiced_bt_dev_status_t status = wiced_bt_sco_write_buffer(active_sco_index, 
            sco_frame, PCM_FRAME_SIZE);

    if (status != WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE("SCO write failed! status=%d\n", status);

        /* Roll back read pointer so we retry later */
        pcm_ring_read = (pcm_ring_read + PCM_RING_BUFFER_SIZE - PCM_FRAME_SIZE) 
                % PCM_RING_BUFFER_SIZE;
    }
}

/*******************************************************************************
 * Function Name: send_sco_audio_frame
 *******************************************************************************
* Summary: Sends a single SCO audio frame using pre-filled PCM buffer
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void send_sco_audio_frame(void)
{
    if (active_sco_index == SCO_NOT_CONNECTED)
        return;

    wiced_bt_dev_status_t status =
        wiced_bt_sco_write_buffer(active_sco_index, pcm_tx_buf, PCM_FRAME_SIZE);

    if (status != WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE("SCO write failed! status=%d\n", status);
    }
}

/*******************************************************************************
 * Function Name: app_pdm_pcm_deactivate
 *******************************************************************************
* Summary: Deactivates PDM/PCM channels to stop audio capture
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void app_pdm_pcm_deactivate(void)
{
    Cy_PDM_PCM_DeActivate_Channel(PDM0, LEFT_CH_INDEX);
    Cy_PDM_PCM_DeActivate_Channel(PDM0, RIGHT_CH_INDEX);
}

/*******************************************************************************
 * Function Name: convert_db_to_pdm_scale
 ********************************************************************************
 * Summary:
 * Converts dB to PDM scale (fixed scale from 0 to 31)
 * Refer
 *
 * Parameters:
 *  gain  : gain in dB
 * Return:
 *  Scale value
 *
 *******************************************************************************/
cy_en_pdm_pcm_gain_sel_t convert_db_to_pdm_scale(double db)
{
    if(db<=PDM_PCM_MIN_GAIN)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_103DB; 
    }
    else if(db>PDM_PCM_SEL_GAIN_NEGATIVE_103DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_97DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_97DB;
    }
     else if(db>PDM_PCM_SEL_GAIN_NEGATIVE_97DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_91DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_91DB;
    }
     else if(db>PDM_PCM_SEL_GAIN_NEGATIVE_91DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_85DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_85DB;
    }
     else if(db>PDM_PCM_SEL_GAIN_NEGATIVE_85DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_79DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_79DB;
    }
     else if(db>PDM_PCM_SEL_GAIN_NEGATIVE_79DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_73DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_73DB;
    }
     else if(db>PDM_PCM_SEL_GAIN_NEGATIVE_73DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_67DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_67DB;
    }
     else if(db>PDM_PCM_SEL_GAIN_NEGATIVE_67DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_61DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_61DB;
    }
     else if(db>PDM_PCM_SEL_GAIN_NEGATIVE_61DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_55DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_55DB;
    }
     else if(db>PDM_PCM_SEL_GAIN_NEGATIVE_55DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_49DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_49DB;
    }
     else if(db>PDM_PCM_SEL_GAIN_NEGATIVE_49DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_43DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_43DB;
    }
     else if(db>PDM_PCM_SEL_GAIN_NEGATIVE_43DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_37DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_37DB;
    }
     else if(db>PDM_PCM_SEL_GAIN_NEGATIVE_37DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_31DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_31DB;
    }
     else if(db>PDM_PCM_SEL_GAIN_NEGATIVE_31DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_25DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_25DB;
    }
     else if(db>PDM_PCM_SEL_GAIN_NEGATIVE_25DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_19DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_19DB;
    }
     else if(db>PDM_PCM_SEL_GAIN_NEGATIVE_19DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_13DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_13DB;
    }
     else if(db>PDM_PCM_SEL_GAIN_NEGATIVE_13DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_7DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_7DB;
    }
    else if(db>PDM_PCM_SEL_GAIN_NEGATIVE_7DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_1DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_1DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_1DB && db<=PDM_PCM_SEL_GAIN_5DB)
    {
        return CY_PDM_PCM_SEL_GAIN_5DB;
    }
     else if(db>PDM_PCM_SEL_GAIN_5DB && db<=PDM_PCM_SEL_GAIN_11DB)
    {
        return CY_PDM_PCM_SEL_GAIN_11DB;
    }
    else if(db>PDM_PCM_SEL_GAIN_11DB && db<=PDM_PCM_SEL_GAIN_17DB)
    {
        return CY_PDM_PCM_SEL_GAIN_17DB;
    }     
    else if(db>PDM_PCM_SEL_GAIN_17DB && db<=PDM_PCM_SEL_GAIN_23DB)
    {
        return CY_PDM_PCM_SEL_GAIN_23DB;
    } 
    else if(db>PDM_PCM_SEL_GAIN_23DB && db<=PDM_PCM_SEL_GAIN_29DB)
    {
        return CY_PDM_PCM_SEL_GAIN_29DB;
    } 
    else if(db>PDM_PCM_SEL_GAIN_29DB && db<=PDM_PCM_SEL_GAIN_35DB)
    {
        return CY_PDM_PCM_SEL_GAIN_35DB;
    } 
    else if(db>PDM_PCM_SEL_GAIN_35DB && db<=PDM_PCM_SEL_GAIN_41DB)
    {
        return CY_PDM_PCM_SEL_GAIN_41DB;
    } 
    else if(db>PDM_PCM_SEL_GAIN_41DB && db<=PDM_PCM_SEL_GAIN_47DB)
    {
        return CY_PDM_PCM_SEL_GAIN_47DB;
    } 
    else if(db>PDM_PCM_SEL_GAIN_47DB && db<=PDM_PCM_SEL_GAIN_53DB)
    {
        return CY_PDM_PCM_SEL_GAIN_53DB;
    } 
    else if(db>PDM_PCM_SEL_GAIN_53DB && db<=PDM_PCM_SEL_GAIN_59DB)
    {
        return CY_PDM_PCM_SEL_GAIN_59DB;
    } 
    else if(db>PDM_PCM_SEL_GAIN_59DB && db<=PDM_PCM_SEL_GAIN_65DB)
    {
        return CY_PDM_PCM_SEL_GAIN_65DB;
    } 
    else if(db>PDM_PCM_SEL_GAIN_65DB && db<=PDM_PCM_SEL_GAIN_71DB)
    {
        return CY_PDM_PCM_SEL_GAIN_71DB;
    } 
    else if(db>PDM_PCM_SEL_GAIN_71DB && db<=PDM_PCM_SEL_GAIN_77DB)
    {
        return CY_PDM_PCM_SEL_GAIN_77DB;
    } 
    else if(db>PDM_PCM_SEL_GAIN_77DB && db<=PDM_PCM_SEL_GAIN_83DB)
    {
        return CY_PDM_PCM_SEL_GAIN_83DB;
    } 
    else if(db>PDM_PCM_MAX_GAIN)
    {
        return CY_PDM_PCM_SEL_GAIN_83DB;
    }
    else 
    {
        return (cy_en_pdm_pcm_gain_sel_t) PDM_MIC_GAIN_VALUE;
    }
}

/*******************************************************************************
 * Function Name: set_pdm_pcm_gain
 *******************************************************************************
 * 
 * Set PDM scale value for gain.
 *
 ******************************************************************************/
void set_pdm_pcm_gain(cy_en_pdm_pcm_gain_sel_t gain)
{
    Cy_PDM_PCM_SetGain(PDM0, RIGHT_CH_INDEX, gain);
    Cy_PDM_PCM_SetGain(PDM0, LEFT_CH_INDEX, gain);
}

/* [] END OF FILE */

