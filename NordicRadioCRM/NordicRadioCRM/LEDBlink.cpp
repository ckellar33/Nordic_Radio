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
#ifdef __cplusplus
extern "C" {
#endif
#include "nrf_esb.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_error.h"
#include "nrf_esb_error_codes.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_drv_timer.h"
#include "boards.h"
#include "nrf_log.h"
#include "app_util.h"

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

uint32_t SendSyncPayload(void);
void timer0_init(void);
void HandleMessage(void);

#define SLOTS     12

#define CH_OPEN 0x00
#define CH_PAIR 0x01
#define CH_USED 0x02

#define RESET_MEMORY_TEST_BYTE  (0x0DUL)        /**< Known sequence written to a special register to check if this wake up is from System OFF. */
#define RAM_RETENTION_OFF       (0x00000003UL)  /**< The flag used to turn off RAM retention on nRF52. */

#define BTN_PRESSED     0                       /**< Value of a pressed button. */
#define BTN_RELEASED    1                       /**< Value of a released button. */

//#define NRF_ESB_LEGACY

/*lint -save -esym(40, BUTTON_1) -esym(40, BUTTON_2) -esym(40, BUTTON_3) -esym(40, BUTTON_4) -esym(40, LED_1) -esym(40, LED_2) -esym(40, LED_3) -esym(40, LED_4) */

static nrf_esb_payload_t tx_payload;// = NRF_ESB_CREATE_PAYLOAD(0, 0x01, 0x00);
static nrf_esb_payload_t rx_payload;
static uint32_t button_state_1;
static uint32_t button_state_2;
static uint32_t button_state_3;
static uint32_t button_state_4;

static int8_t missedCount = SLOTS;
static uint32_t rxCount = 0;
static uint32_t txCount = 0;
static uint32_t errCount = 0;

const uint8_t leds_list[LEDS_NUMBER] = LEDS_LIST;

void nrf_esb_error_handler(uint32_t err_code, uint32_t line)
{
#if DEBUG //lint -e553
	while (true)
		;
#else
//	NVIC_SystemReset();
#endif

}

#define APP_ESB_ERROR_CHECK(err_code) if(err_code) nrf_esb_error_handler(err_code, __LINE__);

void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
	uint32_t err_code;
	
	switch (p_event->evt_id)
	{
	case NRF_ESB_EVENT_TX_SUCCESS:
		/* Set to RX mode right after TX Sync payload */
		err_code = nrf_esb_start_rx();
		APP_ESB_ERROR_CHECK(err_code); break;
		break;
	case NRF_ESB_EVENT_TX_FAILED:
		(void) nrf_esb_flush_tx();
		break;
	case NRF_ESB_EVENT_RX_RECEIVED:
	    // Get the most recent element from the RX FIFO.
		while (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS);		
		HandleMessage();
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
	nrf_esb_config.mode                     = NRF_ESB_MODE_PRX;
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
	
	SyncPayload *syncPayload = (SyncPayload*)&tx_payload.data[0];
	
	syncPayload->CUID = tx_payload.pipe;
	syncPayload->Slot1Status = CH_OPEN;
	syncPayload->Slot1Suid = 0x11;
	syncPayload->Slot2Status = CH_OPEN;
	syncPayload->Slot2Suid = 0x12;
	syncPayload->Slot3Status = CH_OPEN;
	syncPayload->Slot3Suid = 0x13;
	
	tx_payload.length  = sizeof(SyncPayload);
	tx_payload.pipe    = 0;
	tx_payload.noack = true;

	return NRF_SUCCESS;
}

void gpio_init(void)
{
	nrf_gpio_cfg_sense_input(BUTTON_1, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
	nrf_gpio_cfg_sense_input(BUTTON_2, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
	nrf_gpio_cfg_sense_input(BUTTON_3, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
	nrf_gpio_cfg_sense_input(BUTTON_4, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
	
	    // Workaround for PAN_028 rev1.1 anomaly 22 - System: Issues with disable System OFF mechanism
	nrf_delay_ms(1);

	LEDS_CONFIGURE(LEDS_MASK);
	
	nrf_gpio_pin_write(LED_1, LED_OFF);
	nrf_gpio_pin_write(LED_2, LED_OFF);
	nrf_gpio_pin_write(LED_3, LED_OFF);
	nrf_gpio_pin_write(LED_4, LED_OFF);
}

uint32_t logging_init(void)
{
	uint32_t err_code;
	err_code = NRF_LOG_INIT();
	return err_code;
}

int main(void)
{
	uint32_t err_code;
	
	err_code = logging_init();
	APP_ERROR_CHECK(err_code);
	
	// Initialize
	clocks_start();
	err_code = esb_init();
	APP_ESB_ERROR_CHECK(err_code);
	
	NRF_LOG("Enhanced ShockBurst Radio Module running.\r\n");

	gpio_init();
	timer0_init();

	while (true)
	{
		button_state_1 = nrf_gpio_pin_read(BUTTON_1);
		if (button_state_1 == BTN_PRESSED)
		{
			SendSyncPayload();
		}
	}
	
}
/*lint -restore */


uint32_t SendSyncPayload(void)
{
	uint32_t err_code;
	
	tx_payload.length = sizeof(SyncPayload);
	nrf_esb_write_payload(&tx_payload);
	err_code = nrf_esb_start_tx();
	
	if (err_code == NRF_SUCCESS)
		txCount++;
	else
		errCount++;
	
	return err_code;
	
}

/**
 * @brief Handler for timer events.
 */
void timer_led_event_handler(nrf_timer_event_t event_type, void* p_context)
{
	static uint32_t i;
	uint32_t err_code;
	SyncPayload *syncTxPayload = (SyncPayload*)&tx_payload.data[0];
	
	switch (event_type)
	{
	case NRF_TIMER_EVENT_COMPARE0:
		/* Set to TX mode right before TX Sync payload */
		err_code = nrf_esb_stop_rx();
		
		if (missedCount > 0)
		{
			missedCount = SLOTS;
			syncTxPayload->Slot1Suid = 0x11;
			syncTxPayload->Slot1Status = CH_OPEN;
			/* Turn off paired indicator */
			nrf_gpio_pin_write(LED_1, LED_OFF);
		}

		err_code = SendSyncPayload();
		APP_ESB_ERROR_CHECK(err_code);
		break;
	default:
	    //Do nothing.
		break;
	} 
}

const nrf_drv_timer_t TIMER_LED = NRF_DRV_TIMER_INSTANCE(0);

void timer0_init(void)
{
	uint32_t time_ms = 10; //Time(in miliseconds) between consecutive compare events.
	uint32_t time_ticks;
	uint32_t err_code;
	//Configure TIMER_LED for generating simple light effect - leds on board will invert his state one after the other.
	err_code = nrf_drv_timer_init(&TIMER_LED, NULL, timer_led_event_handler);
	APP_ERROR_CHECK(err_code);
	
	time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_LED, time_ms);
    
	nrf_drv_timer_extended_compare(
	     &TIMER_LED,
		NRF_TIMER_CC_CHANNEL0,
		time_ticks,
		NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
		true);
    
	nrf_drv_timer_enable(&TIMER_LED);	
}

/* Handles messages coming from TRM */
void HandleMessage(void)
{
	SyncPayload *syncRxPayload = (SyncPayload*)rx_payload.data;
	SyncPayload *syncTxPayload = (SyncPayload*)tx_payload.data;
	
	if (syncTxPayload->Slot1Status == CH_OPEN)
	{
		syncTxPayload->Slot1Status = CH_USED;
		syncTxPayload->Slot1Suid = syncRxPayload->Slot1Suid;
		/* Turn on LED1 when paired */
		nrf_gpio_pin_write(LED_1, LED_ON);
	}
	
	if (rx_payload.pipe == 0)
	{
		missedCount--;
		if (missedCount < 0)
			missedCount = 0;
	}
	rxCount++;
}