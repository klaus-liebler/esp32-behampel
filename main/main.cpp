#include <stdio.h>
#include "sdkconfig.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <esp_system.h>
#include <esp_spi_flash.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <sys/param.h>
#include <nvs_flash.h>
#include <esp_netif.h>

#include "network.h"
#include <esp_http_server.h>
#include <mqtt_client.h>

#include <driver/ledc.h>
#include <driver/i2c.h>
#include <driver/rmt.h>
#include <bme280.hh>
#include <owb.h>
#include <owb_rmt.h>
#include <ds18b20.h>
#include <i2c.hh>

#include "ws2812_strip.hh"
#include "esp_log.h"

#include "ringtoneplayer.hh"

#define TAG "main"

//Diverse konstante Werte werden gesetzt
constexpr gpio_num_t PIN_LED_STRIP = GPIO_NUM_26;
constexpr gpio_num_t PIN_ONEWIRE = GPIO_NUM_14;
constexpr gpio_num_t PIN_I2C_SDA = GPIO_NUM_22;
constexpr gpio_num_t PIN_I2C_SCL = GPIO_NUM_21;
constexpr uint8_t I2C_PORT = 1;

constexpr size_t LED_NUMBER = 8;
constexpr uint32_t TIMEOUT_FOR_LED_STRIP = 1000;
constexpr rmt_channel_t CHANNEL_WS2812 = RMT_CHANNEL_0;
constexpr rmt_channel_t CHANNEL_ONEWIRE_TX = RMT_CHANNEL_1;
constexpr rmt_channel_t CHANNEL_ONEWIRE_RX = RMT_CHANNEL_2;

constexpr uint8_t CCS811_ADDR = 0x5A; //or 0x5B

//Managementobjekt für die RGB-LEDs
WS2812_Strip<LED_NUMBER> *strip = NULL;

//Managementobjekt für den CO2-Sensor
//TODO

//Managementobjekt für den Temperatur/Luftdruck/Luftfeuchtigkeitssensor
BME280 *bme280;

//Managementobjekte für den präzisen Temperatursensor, der an Pin 14 (GPIO_NUM_14) hängt
DS18B20_Info *ds18b20_info = NULL;
owb_rmt_driver_info rmt_driver_info;
OneWireBus *owb;

//Managementobjekte für die Sound-Wiedergabe
Ringtoneplayer ringtoneplayer;

//Managementobjekt Webserver
httpd_handle_t server=NULL;

//Managementobjekt MQTT client
esp_mqtt_client_handle_t mqttClient;

//"Datenmodell" durch einfache globale Variablen
float temperature, humidity, pressure, co2;

char jsonBuffer[300];

bool hasAlreadePlayedTheWarnSound = false;

long lastMsg = 0;
char msg[50];
int value = 0;

#include "index.html"

esp_err_t handle_get_root(httpd_req_t *req) //browser get application itself
{
  httpd_resp_set_type(req, "text/html");
  ESP_ERROR_CHECK(httpd_resp_send(req, index_html, -1)); // -1 = use strlen()
  return ESP_OK;
}

esp_err_t handle_get_data(httpd_req_t *req)
{
  httpd_resp_set_type(req, "application/json");
  ESP_ERROR_CHECK(httpd_resp_send(req, jsonBuffer, -1)); // -1 = use strlen()
  return ESP_OK;
}

static const httpd_uri_t get_data = {
    .uri = "/data",
    .method = HTTP_GET,
    .handler = handle_get_data,
    .user_ctx = 0,
};

static const httpd_uri_t get_root = {
    .uri = "/*",
    .method = HTTP_GET,
    .handler = handle_get_root,
    .user_ctx = 0,
};

static const esp_mqtt_client_config_t mqtt_cfg={
  .uri = CONFIG_MQTT_SERVER,
  .port = CONFIG_MQTT_PORT,
  .username = CONFIG_MQTT_USER,
  .password = CONFIG_MQTT_PASS,
  .refresh_connection_after_ms=0,
};



int64_t GetMillis64()
{
  return esp_timer_get_time() / 1000ULL;
}

uint64_t lastSensorUpdate = 0;
uint64_t lastMQTTUpdate = 0;

extern "C" void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
  ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
  esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
  switch ((esp_mqtt_event_id_t)event_id)
  {
  case MQTT_EVENT_CONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
    break;
  case MQTT_EVENT_DISCONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
    break;
  case MQTT_EVENT_PUBLISHED:
    ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_ERROR:
    ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
    break;
  default:
    ESP_LOGI(TAG, "Other event id:%d", event->event_id);
    break;
  }
}



//Funktion wird automatisch vom Framework einmalig beim einschalten bzw nach Reset aufgerufen
extern "C" void app_main()
{
  //Willkommens-Meldung auf dem Bildschirm des angeschlossenen PCs ("serielle Konsole") ausgeben
  ESP_LOGI(TAG, "W-HS IoT BeHampel starting up...");

  //Starte das WIFI-Netzwerk
  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(connectBlocking());

  //Verbindung steht. Starte jetzt dem HTTP-Server und den MQTT-Client
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.uri_match_fn = httpd_uri_match_wildcard;
  const char *hostnameptr;
  tcpip_adapter_get_hostname(TCPIP_ADAPTER_IF_STA, &hostnameptr);
  ESP_ERROR_CHECK(httpd_start(&server, &config));
  ESP_LOGI(TAG, "HTTPd successfully started for website http://%s:%d", hostnameptr, config.server_port);
  ESP_ERROR_CHECK(httpd_register_uri_handler(server, &get_data));
  ESP_ERROR_CHECK(httpd_register_uri_handler(server, &get_root));

   mqttClient = esp_mqtt_client_init(&mqtt_cfg);
  /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
  esp_mqtt_client_register_event(mqttClient, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
  esp_mqtt_client_start(mqttClient);
  

  //Konfiguriert die I2C-Schnittstelle zur Anbindung der Sensoren BME280 und CCS811
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = PIN_I2C_SDA;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = PIN_I2C_SCL;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = 100000;
  i2c_param_config(I2C_PORT, &conf);
  ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0));
  ESP_ERROR_CHECK(I2C::Init());

  //Baut die Verbindung mit dem BME280 auf (unter Nutzung der zuvor konfigurierten I2C-Schnittstelle)
  uint32_t bme280ReadoutIntervalMs = UINT32_MAX;
  bme280 = new BME280(I2C_PORT, BME280_ADRESS::PRIM);
  if (bme280->Init(&bme280ReadoutIntervalMs) == ESP_OK)
  {
    bme280->TriggerNextMeasurement();
    ESP_LOGI(TAG, "I2C: BME280 successfully initialized.");
  }
  else
  {
    ESP_LOGW(TAG, "I2C: BME280 not found");
  }

  //Konfiguriere die OneWire-Schnittstelle und direkt auch den einen daran angeschlossenen Temperattursensor
  ESP_LOGI(TAG, "Start DS18B20:");
  owb = owb_rmt_initialize(&rmt_driver_info, PIN_ONEWIRE, CHANNEL_ONEWIRE_TX, CHANNEL_ONEWIRE_RX);
  owb_use_crc(owb, true); // enable CRC check for ROM code
  OneWireBus_ROMCode rom_code;
  owb_status status = owb_read_rom(owb, &rom_code);
  if (status == OWB_STATUS_OK)
  {
    ds18b20_info = ds18b20_malloc();                                 // heap allocation
    ds18b20_init_solo(ds18b20_info, owb);                            // only one device on bus
    ds18b20_use_crc(ds18b20_info, true);                             // enable CRC check on all reads
    ds18b20_set_resolution(ds18b20_info, DS18B20_RESOLUTION_10_BIT); //10bit -->187ms Conversion time
    ds18b20_convert_all(owb);
    ESP_LOGI(TAG, "1Wire: DS18B20 successfully initialized");
  }
  else
  {
    ESP_LOGW(TAG, "1Wire: An error occurred reading DS18B20 ROM code: %d", status);
  }

  //Konfiguriert den 8fach-RGB-LED-Strip
  strip = new WS2812_Strip<LED_NUMBER>(CHANNEL_WS2812);
  ESP_ERROR_CHECK(strip->Init(PIN_LED_STRIP));
  ESP_ERROR_CHECK(strip->Clear(TIMEOUT_FOR_LED_STRIP));
  strip->SetPixel(0, CRGB::Red);
  strip->SetPixel(1, CRGB::Green);
  strip->SetPixel(2, CRGB::Blue);
  strip->Refresh(TIMEOUT_FOR_LED_STRIP);
  ESP_LOGI(TAG, "LED: WS2812_Strip successfully initialized (if you see R G B)");

  ringtoneplayer.Setup(LEDC_TIMER_2, LEDC_CHANNEL_2, GPIO_NUM_25);
  ringtoneplayer.PlaySong(1);
  ringtoneplayer.Loop();

  // Die ganze Initializierung und Konfiguration ist an dieser Stelle zu Ende (puuuh...) - ab hier beginnt die "Endlos-Arbeits-Schleife"

  while (true)
  {
    //Der Klingelton-Player muss permanent prüfen, ob er eine neue Note auf dem Lautsprecher ausgeben sollte. Das tut er hier
    ringtoneplayer.Loop();

    //Hole die aktuelle Zeit
    uint64_t now = GetMillis64();

    //Hole alle 5 Sekunden Messdaten von den Sensoren
    if (now - lastSensorUpdate > 5000)
    {
      //Zuerst vom BME280
      float dummy;
      bme280->GetDataAndTriggerNextMeasurement(&dummy, &pressure, &humidity);
      pressure/=100;

      //Hole die Temperatur vom präzisen OneWire-Sensor und starte direkt den nächsten Messzyklus
      ds18b20_read_temp(ds18b20_info, &(temperature));
      ds18b20_convert_all(owb);

      //...und bastele aus den Messdaten ein JSON-Datenpaket zusammen
      const char *state = co2 < 800.0 ? "Gut" : "Schlecht";
      snprintf(jsonBuffer, sizeof(jsonBuffer), "{\"state\":\"%s\",\"temperature\":%.1f,\"humidity\":%.0f, \"pressure\":%.0f, \"co2\":%.0f}", state, temperature, humidity, pressure, co2);
      ESP_LOGI(TAG, "%s", jsonBuffer);

      //...und führe die Lampen-Sound-Logik aus
      //Errechne ausgehend vom CO2-Messwert die anzuzeigende Farbe (RGB-Darstellung), https://www.rapidtables.com/web/color/RGB_Color.html
      //#####################
      //BITTE AB HIER VERÄNDERN
      //#####################

      strip->SetPixel(0, CRGB::Blue);
      strip->SetPixel(1, CRGB::Blue);
      strip->SetPixel(2, CRGB::Blue);
      ESP_ERROR_CHECK(strip->Refresh(TIMEOUT_FOR_LED_STRIP));

      //#####################
      //BITTE AB HIER NICHTS MEHR VERÄNDERN
      //#####################

      //gebe außerdem der Sound-Wiedergabe vor, was Sie zu machen hat (Sound Starten bzw zurücksetzen)
      if (co2 > 1000)
      {
        if (!hasAlreadePlayedTheWarnSound)
        {
          ringtoneplayer.PlaySong(2);
          hasAlreadePlayedTheWarnSound = true;
        }
      }
      else
      {
        hasAlreadePlayedTheWarnSound = false;
      }

      lastSensorUpdate = now;
    }
    //Schreibe alle 20 Sekunden die aktuellen Messwerte per MQTT raus
    if (now - lastMQTTUpdate > 20000 && mqttClient)
    {
      ESP_LOGI(TAG, "Publishing to MQTT Topic %s", CONFIG_MQTT_TOPIC);
      esp_mqtt_client_publish(mqttClient, CONFIG_MQTT_TOPIC, jsonBuffer, 0, 0, 0);
      lastMQTTUpdate = now;
    }
  }
}