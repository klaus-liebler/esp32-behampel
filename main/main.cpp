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
#include <esp_http_server.h>
#include <mqtt_client.h>
#include <driver/ledc.h>
#include <driver/i2c.h>
#include <driver/rmt.h>


#include "network.h"
#include <bme280.hh>
#include <ccs811.hh>
#include <owb.h>
#include <owb_rmt.h>
#include <ds18b20.h>
#include <i2c.hh>

#include "ws2812_strip.hh"
#include "esp_log.h"

#include "Alarm.mp3.h"
#include "mp3player.hh"

#define TAG "main"

//Diverse konstante Werte werden gesetzt
//Hier wird festgelegt, an welchen Anschlüssen der CPU welche anderen Bauteile angeschlossen sind.
//Der ESP32 ist da sehr flexibel, und kann fast(!) alle Funktionen an fast(!) alle Pins legen
constexpr gpio_num_t PIN_LED_STRIP = GPIO_NUM_26;
constexpr gpio_num_t PIN_ONEWIRE = GPIO_NUM_14;
constexpr gpio_num_t PIN_I2C_SDA = GPIO_NUM_22;
constexpr gpio_num_t PIN_I2C_SCL = GPIO_NUM_21;


constexpr size_t LED_NUMBER = 8;
constexpr uint32_t TIMEOUT_FOR_LED_STRIP = 1000;
//Festlegung, welche internen Funktionseinheiten verwendet werden sollen
constexpr uint8_t I2C_PORT = 1;
constexpr rmt_channel_t CHANNEL_WS2812 = RMT_CHANNEL_0;
constexpr rmt_channel_t CHANNEL_ONEWIRE_TX = RMT_CHANNEL_1;
constexpr rmt_channel_t CHANNEL_ONEWIRE_RX = RMT_CHANNEL_2;


//Managementobjekt für die RGB-LEDs
WS2812_Strip<LED_NUMBER> *strip = NULL;

//Managementobjekt für den Temperatur/Luftdruck/Luftfeuchtigkeitssensor
BME280 *bme280;

//Managementobjekt für den Luftqualitätssensor
CCS811Manager *ccs811;

//Managementobjekte für den präzisen 1Wire-Temperatursensor
DS18B20_Info *ds18b20_info = NULL;
owb_rmt_driver_info rmt_driver_info;
OneWireBus *owb;

//Managementobjekte für die Sound-Wiedergabe
MP3Player mp3player;

//Managementobjekte für den Webserver
httpd_handle_t server=NULL;

//Managementobjekt MQTT client
esp_mqtt_client_handle_t mqttClient;

//"Datenmodell" durch einfache globale Variablen
float temperature{0};
float humidity{0};
float pressure{0};
uint16_t co2{0}; // 16bit = 65536 verschiedene Werte. Weil "unsigned", wird das als Zahlen von 0 bis 65535 interpretiert

//"Schmierblatt", um ein JSON-Datenpaket zusammenbauen zu können (max. 300 Zeichen lang...)
char jsonBuffer[300];

//Merkt sich, ob der Warnton beim Überschreiten eines kritischen Messwertes bereits ausgegeben wurde. 
//Verhindert mit der entsprechenden Logik (s.u.), dass die Warnung in jedem Schleifendurchlauf ausgegeben wird
bool hasAlreadyPlayedTheWarnSound = false;

//Bindet die Datei index.html als Rohtext-Variable ein
#include "index.html"


//Konfiguration des Webservers
//letztlich die Festlegung, was gemacht werden soll, wenn eine bestimmte URL aufgerufen wird
esp_err_t handle_get_root(httpd_req_t *req)
{
  httpd_resp_set_type(req, "text/html");
  ESP_ERROR_CHECK(httpd_resp_send(req, index_html, -1)); // -1 = use strlen()
  return ESP_OK;
}
static const httpd_uri_t get_root = {
    .uri = "/*", //wenn immer ein Browser mit irgendeiner (*!) Adresse den labathome anfragt..
    .method = HTTP_GET, //...und zwar mit dem "Standardanfragetyp" "GET"
    .handler = handle_get_root, //...dann mache bitte das, was in dieser Funktion steht
    .user_ctx = 0,
};

esp_err_t handle_get_data(httpd_req_t *req)
{
  httpd_resp_set_type(req, "application/json");
  //Nachdem die eigentliche HTML-Seite (s.o.) an den Browser übertragen wurde, macht dieser eine eigene spezielle Datenverbindung mit labathome auf, über die
  //sekündlich die aktuellen Messwerte übertragen werden. Diese Messwerte stehen im sog. JSON-Format im jsonBuffer (oben als "Schmierblatt" bezeichnet)
  ESP_ERROR_CHECK(httpd_resp_send(req, jsonBuffer, -1)); // -1 = use strlen()
  return ESP_OK;
}
static const httpd_uri_t get_data = {
    .uri = "/data", //wenn der Browser aber speziell die Adresse /data abfragt
    .method = HTTP_GET,
    .handler = handle_get_data, //dann soll dieses hier passieren
    .user_ctx = 0,
};

#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
//Konfiguration des MQTT-Clients
static const esp_mqtt_client_config_t mqtt_cfg={
  .uri = CONFIG_MQTT_SERVER,
  .port = CONFIG_MQTT_PORT,
  .username = CONFIG_MQTT_USER,
  .password = CONFIG_MQTT_PASS,
  .refresh_connection_after_ms=0,
};

//Hilfsfunktion, um die abgelaufene Zeit in ms zu bekommen
int64_t GetMillis64()
{
  return esp_timer_get_time() / 1000ULL;
}

//Variablen merken sich, wann eine Aktion zuletzt ausgeführt wurde
uint64_t lastSensorUpdate = 0;
uint64_t lastMQTTUpdate = 0;

//Diese Funktion wird vom MQTT-Client aufgerufen, wenn "etwas spannendes" passiert.
//Das Ereignis wird einfach nur mal ausgegeben
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



//Funktion wird automatisch vom Framework einmalig beim Einschalten bzw nach Reset aufgerufen
extern "C" void app_main()
{
  //Willkommens-Meldung auf dem Bildschirm des angeschlossenen PCs ("serielle Konsole") ausgeben
  ESP_LOGI(TAG, "W-HS IoT BeHampel starting up...");

  //Starte das WIFI-Netzwerk
  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(connectBlocking());

  //Verbindung steht. 
  //Starte jetzt den HTTP-Server
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.uri_match_fn = httpd_uri_match_wildcard;
  const char *hostnameptr;
  tcpip_adapter_get_hostname(TCPIP_ADAPTER_IF_STA, &hostnameptr);
  ESP_ERROR_CHECK(httpd_start(&server, &config));
  ESP_LOGI(TAG, "HTTPd successfully started for website http://%s:%d", hostnameptr, config.server_port);
  ESP_ERROR_CHECK(httpd_register_uri_handler(server, &get_data));
  ESP_ERROR_CHECK(httpd_register_uri_handler(server, &get_root));

  //Starte jetzt den MQTT-Client
  mqttClient = esp_mqtt_client_init(&mqtt_cfg);
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

  //CCS811
  ccs811 = new CCS811Manager((i2c_port_t)I2C_PORT, (gpio_num_t)GPIO_NUM_NC, CCS811::ADDRESS::ADDR0);
  if(ccs811->Init()==ESP_OK){
    ccs811->Start(CCS811::MODE::_1SEC);
    ESP_LOGI(TAG, "I2C: CCS811 successfully initialized and started.");
  }
   else{
    ESP_LOGW(TAG, "I2C: CCS811 not found");
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
  ESP_LOGI(TAG, "LED: WS2812_Strip successfully initialized (if you see a red, green and blue)");

  //Konfiguriert die Tonwiedergabe
  mp3player.SetupInternalDAC();
  //mp3player.Play(Alarm_mp3, sizeof(Alarm_mp3));
  //mp3player.Loop();

  // Die ganze Initializierung und Konfiguration ist an dieser Stelle zu Ende (puuuh...) - ab hier beginnt die "Endlos-Arbeits-Schleife"

  while (true)
  {
    //Der Musik-Player muss permanent prüfen, ob er eine neue Note auf dem Lautsprecher ausgeben sollte. Das tut er hier
    //mp3player.Loop();

    //Hole die aktuelle Zeit
    uint64_t now = GetMillis64();

    //Hole alle 5 Sekunden Messdaten von den Sensoren
    if (now - lastSensorUpdate > 5000)
    {
      //Zuerst vom BME280
      float dummyF;
      bme280->GetDataAndTriggerNextMeasurement(&dummyF, &pressure, &humidity);
      pressure=pressure/100;

      uint16_t dummyU16[3];
      //dann vom CCS811
      ccs811->Read(&co2, dummyU16, dummyU16+1, dummyU16+2);

      //Hole die Temperatur vom präzisen OneWire-Sensor und starte direkt den nächsten Messzyklus
      ds18b20_read_temp(ds18b20_info, &(temperature));
      ds18b20_convert_all(owb);

      //...und bastele aus den Messdaten ein JSON-Datenpaket zusammen
      const char *state = co2 < 800.0 ? "Gut" : "Schlecht";
      snprintf(jsonBuffer, sizeof(jsonBuffer), "{\"state\":\"%s\",\"temperature\":%.1f,\"humidity\":%.0f, \"pressure\":%.0f, \"co2\":%.0d}", state, temperature, humidity, pressure, co2);
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
        if (!hasAlreadyPlayedTheWarnSound)
        {
          //mp3player.Play(Alarm_mp3, sizeof(Alarm_mp3));
          hasAlreadyPlayedTheWarnSound = true;
        }
      }
      else
      {
        hasAlreadyPlayedTheWarnSound = false;
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
    //Muss aus technischen Gründen einmal im Durchlauf aufgerufen werden, weil sonst der "Watchdog" nicht ausgeführt werden kann
    vTaskDelay(1);
  }
}

