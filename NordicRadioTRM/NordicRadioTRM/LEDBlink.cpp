/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif	
#include "nrf_esb.h"
#include <string.h>
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_esb_error_codes.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_saadc.h" // Needed for SAADC
#include "nrf_drv_ppi.h"   // Needed for SAADC
#include "nrf_error.h"
#include "nrf_log.h"
#include "boards.h"
#include "nrf_log.h"
#include "nrf_drv_rtc.h"
#ifdef __cplusplus
}
#endif

#define LED_ON          0
#define LED_OFF         1

#define DATA_SIZE		57

typedef struct
{
	uint8_t CUID;
	uint8_t Slot1Status;
	uint8_t Slot1Suid;
	uint8_t Slot2Status;
	uint8_t Slot2Suid;
	uint8_t Slot3Status;
	uint8_t Slot3Suid;	
	uint8_t data[DATA_SIZE];
} SyncPayload;


#define SLOTS     12
#define SUID    0x23
#define CH_OPEN 0x00
#define CH_PAIR 0x01
#define CH_USED 0x02

/* Varible Declarations for RTC*/
#define RTC_CC_VALUE 8                            //Determines the RTC interrupt frequency and thereby the SAADC sampling frequency
const  nrf_drv_rtc_t           rtc = NRF_DRV_RTC_INSTANCE(2); /**< Declaring an instance of nrf_drv_rtc for RTC2. */
static bool                    m_saadc_initialized = false; 
/* END Varibles for RTC */
const nrf_drv_timer_t TIMER_LED = NRF_DRV_TIMER_INSTANCE(1);
/* Variable Declarations for SAADC */
#define SAMPLES_IN_BUFFER 2//5

static const nrf_drv_timer_t   m_timer = NRF_DRV_TIMER_INSTANCE(0);
static nrf_saadc_value_t       m_buffer_pool[2][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t       m_ppi_channel;
static uint32_t                m_adc_evt_counter;
/* END Variables for SAADC Code */

/* Function Declarations for SAADC */
void timer_handler(nrf_timer_event_t event_type, void* p_context);
void saadc_sampling_event_init(void);
void saadc_sampling_event_enable(void);
void saadc_sampling_event_disable(void);
void saadc_callback(nrf_drv_saadc_evt_t const * p_event);
void saadc_init(void);
/* End Function Declarations for SAADC */

/* Function Declarations for Radio */
void timer0_init(void);
void HandleMessage(void);
uint32_t SendPairPayload(void);
uint32_t SendDataPayload(void);
void HandleTransmitState(void);
void TransmitSlot(void);
/* END Function Declarations for Radio */

/* Variable Declarations for Radio */
#define NOT_PAIRED 0
#define PAIRING    1
#define PAIRED     2

uint8_t pair_status = NOT_PAIRED;
uint8_t slotCount = 0;
/* END Variable Declarations for Radio */

void nrf_esb_error_handler(uint32_t err_code, uint32_t line)
{
#if DEBUG //lint -e553
	while (true)
		;
#else
	//NVIC_SystemReset();
#endif

}

#define APP_ESB_ERROR_CHECK(err_code) if(err_code) nrf_esb_error_handler(err_code, __LINE__);

static nrf_esb_payload_t tx_payload;
static nrf_esb_payload_t rx_payload;
static uint8_t m_state[4];
const uint8_t leds_list[LEDS_NUMBER] = LEDS_LIST;
uint8_t led_nr;

static uint32_t rxCount = 0;
static uint32_t txCount = 0;
static uint32_t errCount = 0;

void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
	switch (p_event->evt_id)
	{
	case NRF_ESB_EVENT_TX_SUCCESS:
		NRF_LOG("SUCCESS\r\n");
		break;
	case NRF_ESB_EVENT_TX_FAILED:
		NRF_LOG("FAILED\r\n");
		(void) nrf_esb_flush_tx();
		break;
	case NRF_ESB_EVENT_RX_RECEIVED:
		while (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS);
		
		nrf_drv_timer_disable(&TIMER_LED);
		saadc_sampling_event_disable();

		
		m_adc_evt_counter = 0;
		HandleMessage();
		nrf_drv_timer_enable(&TIMER_LED);	
		saadc_sampling_event_enable();

		break;
	}
}


void clocks_start(void)
{
    // Start HFCLK and wait for it to start.
	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	NRF_CLOCK->TASKS_HFCLKSTART = 1;
	while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}


uint32_t esb_init(void)
{
	uint32_t err_code;
	uint8_t base_addr_0[4] = { 0xE7, 0xE7, 0xE7, 0xE7 };
	uint8_t base_addr_1[4] = { 0xC2, 0xC2, 0xC2, 0xC2 };
	uint8_t addr_prefix[8] = { 0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };

#ifndef NRF_ESB_LEGACY
	nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
#else // NRF_ESB_LEGACY
	nrf_esb_config_t nrf_esb_config         = NRF_ESB_LEGACY_CONFIG;
#endif // NRF_ESB_LEGACY
	nrf_esb_config.retransmit_count         = 0;
	nrf_esb_config.retransmit_delay			= 0;
	nrf_esb_config.selective_auto_ack       = true;
	nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
	nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_2MBPS;
	nrf_esb_config.event_handler            = nrf_esb_event_handler;
	nrf_esb_config.mode                     = NRF_ESB_MODE_PTX;
	nrf_esb_config.tx_mode					= NRF_ESB_TXMODE_MANUAL;
	nrf_esb_config.payload_length           = sizeof(SyncPayload);

	err_code = nrf_esb_init(&nrf_esb_config);
	VERIFY_SUCCESS(err_code);

	err_code = nrf_esb_set_base_address_0(base_addr_0);
	VERIFY_SUCCESS(err_code);

	err_code = nrf_esb_set_base_address_1(base_addr_1);
	VERIFY_SUCCESS(err_code);

	err_code = nrf_esb_set_prefixes(addr_prefix, 8);
	VERIFY_SUCCESS(err_code);

	tx_payload.length = sizeof(SyncPayload);
	tx_payload.pipe    = 0;
	tx_payload.noack = true;

	return NRF_SUCCESS;
}

void gpio_init(void)
{
	m_state[0] = LED_OFF;
	m_state[1] = LED_OFF;
	m_state[2] = LED_OFF;
	m_state[3] = LED_OFF;

	    //nrf_gpio_range_cfg_output(8, 31);
	LEDS_CONFIGURE(LEDS_MASK);

	nrf_gpio_pin_write(LED_1, m_state[0]);
	nrf_gpio_pin_write(LED_2, m_state[1]);
	nrf_gpio_pin_write(LED_3, m_state[2]);
	nrf_gpio_pin_write(LED_4, m_state[3]);
}


uint32_t logging_init(void)
{
	uint32_t err_code;
	err_code = NRF_LOG_INIT();
	return err_code;
}


void power_manage(void)
{
    // WFE - SEV - WFE sequence to wait until a radio event require further actions.
	__WFE();
	__SEV();
	__WFE();
}


int main(void)
{
	uint32_t err_code;
	
	/* Reduces current draw significantly. */
	NRF_POWER->DCDCEN = 1;
	
	err_code = logging_init();
	APP_ERROR_CHECK(err_code);
	
	// Clocks Initialize
	clocks_start();
	
	// ESB Initialize
	err_code = esb_init();
	APP_ESB_ERROR_CHECK(err_code);
	
	// SAADC Initialize
	saadc_sampling_event_init();
	saadc_init();
	//saadc_sampling_event_enable();

	NRF_LOG("Enhanced ShockBurst Radio Module running.\r\n");

	/* Starts in RX awaiting Sync Payload*/
	err_code = nrf_esb_start_rx();
	APP_ESB_ERROR_CHECK(err_code);
	
	gpio_init();
	//timer0_init();

	while (true)
	{
		power_manage();
	}
}
/*lint -restore */

/* Handles Message coming in from CRM */
void HandleMessage(void)
{
	uint32_t rxCountMod = 0;
	rxCountMod = rxCount % 600; // % 600 is Duty cycle of 6 seconds
	
	SyncPayload *syncRxPayload = (SyncPayload*)&rx_payload.data[0];
	SyncPayload *syncTxPayload = (SyncPayload*)&tx_payload.data[0];
	memcpy(syncTxPayload, syncRxPayload, sizeof(SyncPayload));
	
	if (syncRxPayload->Slot1Status != CH_USED && pair_status == NOT_PAIRED)
	{
		syncTxPayload->Slot1Suid = SUID;
		pair_status = PAIRING;
		nrf_gpio_pin_write(LED_1, LED_OFF);	
		nrf_gpio_pin_write(LED_2, LED_OFF);	
	}
	else if (syncRxPayload->Slot1Status == CH_USED && syncRxPayload->Slot1Suid == SUID)
	{
		pair_status = PAIRED;
		if (rxCountMod == 0 || rxCountMod < 6) // rxCountMod < 6 is ON time of 60ms
		{
			nrf_gpio_pin_write(LED_1, LED_ON);
			nrf_gpio_pin_write(LED_2, LED_ON);
		}
		else 
		{
			nrf_gpio_pin_write(LED_1, LED_OFF);
			nrf_gpio_pin_write(LED_2, LED_OFF);
		}
	}
	
	rxCount++;
}

/* Configure and operate TIMER0 */
/**
 * @brief Handler for timer events.
 */
void timer_led_event_handler(nrf_timer_event_t event_type, void* p_context)
{
	static uint32_t i;
	uint32_t errCode;
	
	slotCount++;	
	if (slotCount > SLOTS)
	{
		slotCount = 0;
		NRF_RADIO->SHORTS = 0;
		nrf_drv_timer_disable(&TIMER_LED);
		nrf_esb_start_rx();
		return;
	}
    
	switch (event_type)
	{
	case NRF_TIMER_EVENT_COMPARE0:
		if (pair_status != PAIRED)
		{
			errCode = SendPairPayload();
			APP_ESB_ERROR_CHECK(errCode);
		}
		else if (pair_status == PAIRED)
		{
			errCode = SendDataPayload();
			APP_ESB_ERROR_CHECK(errCode);
		}
		break;
        
	default:
	    //Do nothing.
		break;
	}    
}

void timer0_init(void)
{
	uint32_t time_us = 475; //Time(in microseconds) between consecutive compare events.
	uint32_t time_ticks;
	uint32_t err_code;
	//Configure TIMER_LED for generating simple light effect - leds on board will invert his state one after the other.
	err_code = nrf_drv_timer_init(&TIMER_LED, NULL, timer_led_event_handler);
	APP_ERROR_CHECK(err_code);
	
	time_ticks = nrf_drv_timer_us_to_ticks(&TIMER_LED, time_us);
    
	nrf_drv_timer_extended_compare(
	     &TIMER_LED,
		NRF_TIMER_CC_CHANNEL0,
		time_ticks,
		NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
		true);
}
/* END TIMER0 */

uint32_t SendPairPayload(void)
{
	uint32_t err_code;
	SyncPayload *syncTxPayload = (SyncPayload*)&tx_payload.data[0];
	tx_payload.length = sizeof(SyncPayload);
	tx_payload.noack = true;
	
	syncTxPayload->Slot1Suid = SUID;
	
	/* Stop the radio from RX */
	nrf_esb_stop_rx();
	
	nrf_esb_write_payload(&tx_payload);
	err_code = nrf_esb_start_tx();
	
	if (err_code == NRF_SUCCESS)
		txCount++;
	else
	{
		errCount++;
		nrf_esb_flush_tx();
	}
	
	return err_code;	
}

uint32_t SendDataPayload(void)
{
	uint32_t err_code;
	tx_payload.length = sizeof(SyncPayload);
	tx_payload.noack = true;
	memset(tx_payload.data, 0xAB, sizeof(SyncPayload));
	
	/* Stop the radio from RX */
	nrf_esb_stop_rx();
	
	nrf_esb_write_payload(&tx_payload);
	err_code = nrf_esb_start_tx();
	
	if (err_code == NRF_SUCCESS)
		txCount++;
	else
	{
		errCount++;
		nrf_esb_flush_tx();
	}
	
	return err_code;
}

/* SAADC (Successive Approx. Analog-to-Digital Converter) Implementation Code */
void timer_handler(nrf_timer_event_t event_type, void* p_context)
{

}

void saadc_sampling_event_init(void)
{
	ret_code_t err_code;
	err_code = nrf_drv_ppi_init();
	APP_ERROR_CHECK(err_code);

	err_code = nrf_drv_timer_init(&m_timer, NULL, timer_handler);
	APP_ERROR_CHECK(err_code);

	    /* setup m_timer for compare event every 400ms */
	uint32_t ticks = nrf_drv_timer_us_to_ticks(&m_timer, 63);
	nrf_drv_timer_extended_compare(&m_timer, NRF_TIMER_CC_CHANNEL0, ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
	nrf_drv_timer_enable(&m_timer);

	uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer, NRF_TIMER_CC_CHANNEL0);
	uint32_t saadc_sample_event_addr = nrf_saadc_task_address_get(NRF_SAADC_TASK_SAMPLE);

	    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
	err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
	APP_ERROR_CHECK(err_code);
    
	err_code = nrf_drv_ppi_channel_assign(m_ppi_channel, timer_compare_event_addr, saadc_sample_event_addr);
	APP_ERROR_CHECK(err_code);
}

void saadc_sampling_event_enable(void)
{
	ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);
	APP_ERROR_CHECK(err_code);
}

void saadc_sampling_event_disable(void)
{
	ret_code_t err_code = nrf_drv_ppi_channel_disable(m_ppi_channel);
	APP_ERROR_CHECK(err_code);
}

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
	if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
	{
		ret_code_t err_code;
     
		err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
		APP_ERROR_CHECK(err_code);
		
		m_adc_evt_counter++;
		HandleTransmitState();
	}
}

void saadc_init(void)
{
	ret_code_t err_code;
	
	nrf_saadc_channel_config_t channel_0_config =
	      NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
	channel_0_config.acq_time = NRF_SAADC_ACQTIME_20US;
	
	nrf_saadc_channel_config_t channel_1_config =
				NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);
	channel_1_config.acq_time = NRF_SAADC_ACQTIME_20US;
  
	err_code = nrf_drv_saadc_init(NULL, saadc_callback);
	APP_ERROR_CHECK(err_code);

	err_code = nrf_drv_saadc_channel_init(0, &channel_0_config);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_saadc_channel_init(1, &channel_1_config);
	APP_ERROR_CHECK(err_code);

	err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
	APP_ERROR_CHECK(err_code);
    
	err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
	APP_ERROR_CHECK(err_code);
}
/* END --- SAADC (Successive Approx. Analog-to-Digital Converter) Implementation Code */

/* RTC2 Implementation Code */
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
	uint32_t err_code;
	
	if (int_type == NRF_DRV_RTC_INT_COMPARE0)
	{		
		if (!m_saadc_initialized)
		{
			saadc_init();                                              //Initialize the SAADC. In the case when SAADC_SAMPLES_IN_BUFFER > 1 then we only need to initialize the SAADC when the the buffer is empty.
		}
		m_saadc_initialized = true;                                    //Set SAADC as initialized
		nrf_drv_saadc_sample();                                        //Trigger the SAADC SAMPLE task
			
		LEDS_INVERT(BSP_LED_0_MASK);                                   //Toggle LED1 to indicate SAADC sampling start
			
		err_code = nrf_drv_rtc_cc_set(&rtc, 0, RTC_CC_VALUE, true);       //Set RTC compare value. This needs to be done every time as the nrf_drv_rtc clears the compare register on every compare match
		APP_ERROR_CHECK(err_code);
		nrf_drv_rtc_counter_clear(&rtc);                               //Clear the RTC counter to start count from zero
	}
}

static void rtc_config(void)
{
	uint32_t err_code;

	    //Initialize RTC instance
	err_code = nrf_drv_rtc_init(&rtc, NULL, rtc_handler);              //Initialize the RTC with callback function rtc_handler. The rtc_handler must be implemented in this applicaiton. Passing NULL here for RTC configuration means that configuration will be taken from the nrf_drv_config.h file.
	APP_ERROR_CHECK(err_code);

	err_code = nrf_drv_rtc_cc_set(&rtc, 0, RTC_CC_VALUE, true);           //Set RTC compare value to trigger interrupt. Configure the interrupt frequency by adjust RTC_CC_VALUE and RTC2_CONFIG_FREQUENCY constant in the nrf_drv_config.h file
	APP_ERROR_CHECK(err_code);

	    //Power on RTC instance
	nrf_drv_rtc_enable(&rtc);                                          //Enable RTC
}

/* END --- RTC2 Implementation Code */

#define START_TX   1
#define TX_DELAY   8 // 8 = 62.5us * 8 = 500 us
#define START_RX   150
void HandleTransmitState(void)
{
	uint32_t adc_evt_count_mod;
	
	adc_evt_count_mod = m_adc_evt_counter % 160;
	
	switch (adc_evt_count_mod)
	{
	case START_TX:
	case START_TX + TX_DELAY * 1:
	case START_TX + TX_DELAY * 2:
	case START_TX + TX_DELAY * 3:
	case START_TX + TX_DELAY * 4:
	case START_TX + TX_DELAY * 5:
	case START_TX + TX_DELAY * 6:
	case START_TX + TX_DELAY * 7:
	case START_TX + TX_DELAY * 8:
	case START_TX + TX_DELAY * 9:
	case START_TX + TX_DELAY * 10:
	case START_TX + TX_DELAY * 11:
		TransmitSlot();
		break;
	case START_RX:
		NRF_RADIO->SHORTS = 0;
		nrf_drv_timer_disable(&TIMER_LED);
		nrf_esb_start_rx();
		break;
	default:
		break;
	}
	
	if (m_adc_evt_counter > 1280) // Missed a lot of Sync payloads
	{
		pair_status = NOT_PAIRED;
		nrf_gpio_pin_write(LED_1, LED_OFF);
		nrf_gpio_pin_write(LED_2, LED_OFF);
	}
}

void TransmitSlot(void)
{
	uint32_t errCode;
	
	if (pair_status != PAIRED)
	{
		errCode = SendPairPayload();
		APP_ESB_ERROR_CHECK(errCode);
	}
	else if (pair_status == PAIRED)
	{
		errCode = SendDataPayload();
		APP_ESB_ERROR_CHECK(errCode);
	}
}