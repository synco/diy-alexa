#include <Arduino.h>
#include <WiFi.h>
#include <driver/i2s.h>
#include <esp_task_wdt.h>
#include "I2SMicSampler.h"
#include "ADCSampler.h"
#include "I2SOutput.h"
#include "config.h"
#include "Application.h"
#include "SPIFFS.h"
#include "IntentProcessor.h"
#include "Speaker.h"
#include "IndicatorLight.h"
#include <ESP32Ping.h>
#include <ezTime.h>

#include <GxEPD2_BW.h> // including both doesn't use more code or ram
#include <GxEPD2_3C.h> // including both doesn't use more code or ram
#include <Fonts/FreeMonoBold24pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMono24pt7b.h>
#include <Fonts/FreeMono18pt7b.h>
#include <Fonts/FreeMono12pt7b.h>
#include <Fonts/FreeMono9pt7b.h>

// select the display class and display driver class in the following file (new style):
#include "GxEPD2_display_selection_new_style.h"

#include <Adafruit_BMP280.h>
#include "SampleBuff.h"

Adafruit_BMP280 bmp = Adafruit_BMP280();

#define TEMP_ADJ            -4.0
#define PRES_ADJ            2380    // Pa (std pressure difference at 200m alt)

#define DECIMATE_SAMPLES     10    // min
#define GRAPH_WIDTH         144   // = 60 / DECIMATE_SAMPLES * 24

SampleBuff hist_temperature = SampleBuff(GRAPH_WIDTH, DECIMATE_SAMPLES);
SampleBuff hist_pressure = SampleBuff(GRAPH_WIDTH, DECIMATE_SAMPLES);


// i2s config for using the internal ADC
i2s_config_t adcI2SConfig = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_LSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0};

// i2s config for reading from both channels of I2S
i2s_config_t i2sMemsConfigBothChannels = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_MIC_CHANNEL,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0};

// i2s microphone pins
i2s_pin_config_t i2s_mic_pins = {
    .bck_io_num = I2S_MIC_SERIAL_CLOCK,
    .ws_io_num = I2S_MIC_LEFT_RIGHT_CLOCK,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_MIC_SERIAL_DATA};

// i2s speaker pins
i2s_pin_config_t i2s_speaker_pins = {
    .bck_io_num = I2S_SPEAKER_SERIAL_CLOCK,
    .ws_io_num = I2S_SPEAKER_LEFT_RIGHT_CLOCK,
    .data_out_num = I2S_SPEAKER_SERIAL_DATA,
    .data_in_num = I2S_PIN_NO_CHANGE};

#define GRAPH_YRANGE     60    // 80
#define GRAPH_YOFFSET    154   // 174
void plot_hist(SampleBuff* sb, float ymin, float ymax, uint16_t color, bool solid=true, uint8_t thickness=1)
{
  float* bmin = sb->getMinBuff();
  float* bmax = sb->getMinBuff();
  for (uint16_t b=0; b<GRAPH_WIDTH; b++) {
    float m;
    if ((!isnan(bmin[b])) && (!isnan(bmax[b]))) {
      m = (bmin[b] - ymin) * GRAPH_YRANGE / (ymax - ymin);
      uint16_t yl = (uint16_t)(GRAPH_YOFFSET - m);
      m = (bmax[b] - ymin) * GRAPH_YRANGE / (ymax - ymin);
      uint16_t yh = (uint16_t)(GRAPH_YOFFSET - m);
      if (solid || (b%(2*thickness) < thickness)) {
        display.drawFastVLine(b,yh-thickness/2,(yl-yh)+thickness,color);
      }
    }
  }
}

void getDaysTideData(uint8_t* dm, uint16_t len)
{

}


void pingTask(void *param)
{
  sensors_event_t temp_event, pressure_event;
  Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
  Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

  static bool pinged = true;
  static bool partialredraw = false;
  display.firstPage();

  Timezone myTZ;
  // Provide official timezone names
  // https://en.wikipedia.org/wiki/List_of_tz_database_time_zones
  myTZ.setLocation(F("Pacific/Auckland"));
  Serial.print(F("New Zealand:     "));
  Serial.println(myTZ.dateTime());
  
  uint32_t c = (myTZ.hour() * 60 + myTZ.minute()) / DECIMATE_SAMPLES;
  hist_temperature.setCount(c);
  hist_pressure.setCount(c);

  while (true)
  {
    myTZ.setLocation(F("Pacific/Auckland"));

    bool ping = Ping.ping("www.google.com", 3);
    if (ping != pinged) {
      if (ping) {
        Serial.println("NOTE: Network up");
      }
      else {
        Serial.println("WARN: Network down");
      }
    }
    pinged = ping;

    bmp_temp->getEvent(&temp_event);
    float temp_adj = temp_event.temperature + TEMP_ADJ;
    hist_temperature.addSample(temp_adj);
    Serial.print(F("Raw Temperature = "));
    Serial.print(temp_event.temperature);
    Serial.println(" *C");
    
    bmp_pressure->getEvent(&pressure_event);
    float pres_adj = pressure_event.pressure + PRES_ADJ/100;
    hist_pressure.addSample(pres_adj);
    Serial.print(F("Raw Pressure = "));
    Serial.print(pressure_event.pressure);
    Serial.println(" hPa");

    {    
      display.fillScreen(GxEPD_WHITE);
      display.setRotation(1);

      // Show Clock
      display.setCursor(60,32);
      display.setTextColor(GxEPD_BLACK);
      display.setFont(&FreeMonoBold24pt7b);
      display.print(myTZ.dateTime("H:i"));

      // Show Day of Week
      display.setFont(&FreeMonoBold12pt7b);
      display.setTextColor(GxEPD_RED);
      display.setCursor(80,56);
      display.print(myTZ.dateTime("l"));

      // Show Date
      display.setTextColor(GxEPD_BLACK);
      display.setCursor(55,78);
      display.print(myTZ.dateTime("d M Y"));
      // Update date only
      //display.displayWindow(0,0,display.width(),80);
      
      // Show Temp
      display.setFont(&FreeMonoBold18pt7b);
      display.setTextColor(GxEPD_RED);
      display.setCursor(144,120);
      display.print(temp_adj,1);
      display.setFont(&FreeMonoBold9pt7b);
      display.print("*C");
      // Show Temp min/max
      display.setCursor(160,136);
      display.print(hist_temperature.getAllMin(), 1);
      display.print("/");
      display.print(hist_temperature.getAllMax(), 1);

      // Show Pressure
      display.setFont(&FreeMonoBold12pt7b);
      display.setTextColor(GxEPD_BLACK);
      display.setCursor(144,156);
      display.print(pres_adj, 1);
      display.setFont(&FreeMonoBold9pt7b);
      display.print("hPa");
      // Show Pressure min/max
      display.setCursor(160,172);
      display.print(hist_pressure.getAllMin(), 0);
      display.print("/");
      display.print(hist_pressure.getAllMax(), 0);

      {
        //display.drawFastHLine(0, GRAPH_YOFFSET-GRAPH_YRANGE, GRAPH_WIDTH, GxEPD_BLACK);
        //display.drawFastHLine(0, GRAPH_YOFFSET, GRAPH_WIDTH, GxEPD_BLACK);
        display.setFont(&FreeMono9pt7b);
        float ymin, ymax;
        hist_pressure.getBuffMinMax(ymin, ymax);
        plot_hist(&hist_pressure, ymin-1, ymax+1, GxEPD_BLACK, false, 2);
        display.setTextColor(GxEPD_BLACK);
        display.setCursor(1,100);
        display.print(ymax, 0);
        display.setCursor(1,173);
        display.print(ymin, 0);

        hist_temperature.getBuffMinMax(ymin, ymax);
        plot_hist(&hist_temperature, ymin-1, ymax+1, GxEPD_RED, true, 4);
        display.setTextColor(GxEPD_RED);
        display.setCursor(70,100);
        display.print(ymax, 1);
        display.setCursor(70,173);
        display.print(ymin, 1);
      }
    }

    if ((myTZ.minute() % 10) == 0) {
      partialredraw = false;
    }
    if (partialredraw) {
      //display.displayWindow(0,0,display.width(),80);
      // Only redraw clock
      display.displayWindow(56,0,144,40);   // loose fit
    }
    else {
      display.display(partialredraw);
    }
    partialredraw = true;
    
    uint32_t d = (60 - myTZ.second()) * 1000;
    delay(d);
  }
}


// This task does all the heavy lifting for our application
void applicationTask(void *param)
{
  Application *application = static_cast<Application *>(param);

  const TickType_t xMaxBlockTime = pdMS_TO_TICKS(100);
  while (true)
  {
    // wait for some audio samples to arrive
    uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    if (ulNotificationValue > 0)
    {
      application->run();
    }
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting up");

  display.init();
  Serial.print("Display: ");
  Serial.print(display.width());
  Serial.print("x");
  Serial.println(display.height());

  bmp.begin(BMP280_ADDRESS_ALT);
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  // launch WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PSWD);
  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Sync clock - debug with: setDebug(INFO);    
  waitForSync();

  Serial.printf("Total heap: %d\n", ESP.getHeapSize());
  Serial.printf("Free heap: %d\n", ESP.getFreeHeap());

  // startup SPIFFS for the wav files
  SPIFFS.begin();
  // make sure we don't get killed for our long running tasks
  esp_task_wdt_init(10, false);

  // start up the I2S input (from either an I2S microphone or Analogue microphone via the ADC)
#ifdef USE_I2S_MIC_INPUT
  // Direct i2s input from INMP441 or the SPH0645
  I2SSampler *i2s_sampler = new I2SMicSampler(i2s_mic_pins, false);
#else
  // Use the internal ADC
  I2SSampler *i2sSampler = new ADCSampler(ADC_UNIT_1, ADC_MIC_CHANNEL);
#endif

  // start the i2s speaker output
  I2SOutput *i2s_output = new I2SOutput();
  i2s_output->start(I2S_NUM_1, i2s_speaker_pins);
  Speaker *speaker = new Speaker(i2s_output);

  // indicator light to show when we are listening
  IndicatorLight *indicator_light = new IndicatorLight();

  // and the intent processor
  IntentProcessor *intent_processor = new IntentProcessor(speaker);
  intent_processor->addDevice("kitchen", GPIO_NUM_0);
  intent_processor->addDevice("bedroom", GPIO_NUM_12);
  intent_processor->addDevice("table", GPIO_NUM_0);

  // create our application
  Application *application = new Application(i2s_sampler, intent_processor, speaker, indicator_light);

  // set up the i2s sample writer task
  TaskHandle_t applicationTaskHandle;
  xTaskCreate(applicationTask, "Application Task", 8192, application, 1, &applicationTaskHandle);

  // start sampling from i2s device - use I2S_NUM_0 as that's the one that supports the internal ADC
#ifdef USE_I2S_MIC_INPUT
  i2s_sampler->start(I2S_NUM_0, i2sMemsConfigBothChannels, applicationTaskHandle);
#else
  i2sSampler->start(I2S_NUM_0, adcI2SConfig, applicationTaskHandle);
#endif

  xTaskCreate(pingTask, "Ping Task", 8192, NULL, 1, NULL);
}


void loop()
{
  vTaskDelay(1000);
}