/**
 * Copyright (c) 2017, Łukasz Marcin Podkalicki <lpodkalicki@gmail.com>
 * ESP32/016
 * WiFi Sniffer.
 */

#include <string.h>	

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_wifi_types.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"

#include "nvs_flash.h"
#include "esp_log.h"
#include "driver/gpio.h"


// BLE 

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "esp_log.h"


#define LED_GPIO_PIN GPIO_NUM_4
#define WIFI_CHANNEL_MAX (13)
#define WIFI_CHANNEL_SWITCH_INTERVAL (500)

static wifi_country_t wifi_country = {.cc = "FR", .schan = 1, .nchan = 13, .policy = WIFI_COUNTRY_POLICY_AUTO};

typedef struct
{
	unsigned frame_ctrl : 16;
	unsigned duration_id : 16;
	uint8_t addr1[6]; /* receiver address */
	uint8_t addr2[6]; /* sender address */
	uint8_t addr3[6]; /* filtering address */
	unsigned sequence_ctrl : 16;
	uint8_t addr4[6]; /* optional */
} wifi_ieee80211_mac_hdr_t;

typedef struct
{
	wifi_ieee80211_mac_hdr_t hdr;
	uint8_t payload[0]; /* network data ended with 4 bytes csum (CRC32) */
} wifi_ieee80211_packet_t;

static esp_err_t event_handler(void *ctx, system_event_t *event);
static void wifi_sniffer_init(void);
static void wifi_sniffer_set_channel(uint8_t channel);
static const char *wifi_sniffer_packet_type2str(wifi_promiscuous_pkt_type_t type);
static void wifi_sniffer_packet_handler(void *buff, wifi_promiscuous_pkt_type_t type);

void ble_sniffer_init(void);

void app_main(void)
{
	uint8_t level = 0, channel = 1;

	/* setup */
	wifi_sniffer_init();
	gpio_set_direction(LED_GPIO_PIN, GPIO_MODE_OUTPUT);

	ble_sniffer_init();

	/* loop */
	while (true)
	{
		gpio_set_level(LED_GPIO_PIN, level ^= 1);
		vTaskDelay(WIFI_CHANNEL_SWITCH_INTERVAL / portTICK_PERIOD_MS);
		wifi_sniffer_set_channel(channel);
		channel = (channel % WIFI_CHANNEL_MAX) + 1;
	}
}

esp_err_t
event_handler(void *ctx, system_event_t *event)
{

	return ESP_OK;
}

static wifi_promiscuous_filter_t ctl_filter;




void wifi_sniffer_init(void)
{

	nvs_flash_init();
	tcpip_adapter_init();

	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
	
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(esp_wifi_set_country(&wifi_country)); /* set country for channel range [1, 13] */
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));



	ESP_ERROR_CHECK(esp_wifi_start());


	esp_wifi_set_promiscuous(true);
	esp_wifi_set_promiscuous_rx_cb(&wifi_sniffer_packet_handler);

	ctl_filter.filter_mask = WIFI_PROMIS_CTRL_FILTER_MASK_ALL | WIFI_PROMIS_FILTER_MASK_ALL;

	ESP_ERROR_CHECK(esp_wifi_set_promiscuous_ctrl_filter(&ctl_filter));
	ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&ctl_filter));
	

	ble_sniffer_init();
	
}

void wifi_sniffer_set_channel(uint8_t channel)
{
	esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
}

const char *
wifi_sniffer_packet_type2str(wifi_promiscuous_pkt_type_t type)
{
	switch (type)
	{
	case WIFI_PKT_MGMT:
		return "MGMT";
	case WIFI_PKT_DATA:
		return "DATA";
	default:
	case WIFI_PKT_MISC:
		return "MISC";
	}
}

void wifi_sniffer_packet_handler(void *buff, wifi_promiscuous_pkt_type_t type)
{

	const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buff;
	const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload;
	const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;

	//
	// filter a mac address for sending elements
	//
	uint8_t pfdaddr[6] = {0x2A, 0x5B, 0xE7, 0xE8, 0x42, 0x6C};
	
    bool found1 = true;
    for (int i = 0 ; i< 6 ; i ++) {
        if (hdr->addr3[i] != pfdaddr[i])
             found1 = false;
    }

	// sender address
	bool found2 = true;
	for (int i = 0; i < 6; i++)
	{
		if (hdr->addr2[i] != pfdaddr[i])
			found2 = false;
	}

	// receiver address
	bool found3 = true;
	for (int i = 0; i < 6; i++)
	{
		if (hdr->addr1[i] != pfdaddr[i])
			found3 = false;
	}

/*
	if (!(found2))
	{
		return;
	}
*/

	// send to serial
	printf("PACKET,TYPE=%s,CHAN=%02d,RSSI=%02d,"
		   "RCVR=%02x:%02x:%02x:%02x:%02x:%02x,"
		   "SNDR=%02x:%02x:%02x:%02x:%02x:%02x,"
		   "FILT=%02x:%02x:%02x:%02x:%02x:%02x\n",
		   wifi_sniffer_packet_type2str(type),
		   ppkt->rx_ctrl.channel,
		   ppkt->rx_ctrl.rssi,
		   /* ADDR1 */
		   hdr->addr1[0], hdr->addr1[1], hdr->addr1[2],
		   hdr->addr1[3], hdr->addr1[4], hdr->addr1[5],
		   /* ADDR2 */
		   hdr->addr2[0], hdr->addr2[1], hdr->addr2[2],
		   hdr->addr2[3], hdr->addr2[4], hdr->addr2[5],
		   /* ADDR3 */
		   hdr->addr3[0], hdr->addr3[1], hdr->addr3[2],
		   hdr->addr3[3], hdr->addr3[4], hdr->addr3[5]);
}

//////////////////////////////////////////////////////////////////////////////////
// BLE Handling

static  esp_ble_scan_params_t ble_scan_params;
static  int scan_duration = 10;


// forward
void
gap_event_handler_static(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
  
void ble_sniffer_init(void) {

  ble_scan_params.scan_type = BLE_SCAN_TYPE_PASSIVE;
  ble_scan_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
  ble_scan_params.scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL;
  ble_scan_params.scan_interval = 0x50;
  ble_scan_params.scan_window = 0x30;

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  esp_bt_controller_init(&bt_cfg);
  esp_bt_controller_enable(ESP_BT_MODE_BLE);
  esp_bluedroid_init();
  esp_bluedroid_enable();

  esp_ble_gap_register_callback(gap_event_handler_static);
  esp_ble_gap_set_scan_params(&ble_scan_params);

}


static const char* tag = "gap_handler";

#define SCAN_DURATION_SEC 10

void
gap_event_handler_static(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
  switch (event)
    {
      case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        {
          ESP_LOGI(tag, "Scan param set complete, start scanning.");
          esp_ble_gap_start_scanning(scan_duration);
          break;
        }

      case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
          {
            ESP_LOGE(tag, "Scan start failed.");
          }
        else
          {
            ESP_LOGI(tag, "Scan start successfully.");
          }
        break;

      case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
          {
            ESP_LOGE(tag, "Scan stop failed.");
          }
        else
          {
            ESP_LOGI(tag, "Scan stop successfully.");
          }
        break;

      case ESP_GAP_BLE_SCAN_RESULT_EVT:
        {
          switch (param->scan_rst.search_evt)
            {
              case ESP_GAP_SEARCH_INQ_RES_EVT:
                {

					 uint8_t *new_bda=param->scan_rst.bda;
					
					char buffer[ESP_BD_ADDR_LEN * 2 +1];
					memset(&buffer[0], 0x00, ESP_BD_ADDR_LEN * 2 +1);
					for (uint8_t i = 0; i < ESP_BD_ADDR_LEN ; i ++ ) {
						sprintf(&buffer[i*2], "%02x", new_bda[i]);
					}

					char ble_adv_buffer[(ESP_BLE_ADV_DATA_LEN_MAX + ESP_BLE_SCAN_RSP_DATA_LEN_MAX ) * 2 + 1];
					memset(&ble_adv_buffer[0], 0x00, (ESP_BLE_ADV_DATA_LEN_MAX + ESP_BLE_SCAN_RSP_DATA_LEN_MAX) * 2 +1);

					for (int i = 0; i < (param->scan_rst.adv_data_len ) ; i ++ ) {
						sprintf(&ble_adv_buffer[i*2], "%02x", param->scan_rst.ble_adv[i]);
					}


					printf("BLE,ADDR=%s,RSSI=%02d,ADVDATA=%s\n",buffer, param->scan_rst.rssi , ble_adv_buffer);
                
				  break;
                }

              case ESP_GAP_SEARCH_INQ_CMPL_EVT:
                {
                  ESP_LOGI(tag, "Scan completed, restarting.");
                  esp_ble_gap_set_scan_params(&ble_scan_params);
                  break;
                }

              default:
                ESP_LOGI(tag, "Unhandled scan result %d.", param->scan_rst.search_evt);
                break;
            }
          break;
        }
      default:
        break;
    }
}
