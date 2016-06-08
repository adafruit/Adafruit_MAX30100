#include <Wire.h>
#include "Adafruit_MAX30100.h"

Adafruit_MAX30100 pulse = Adafruit_MAX30100();
#define MAX30100_IRQPIN 12

// make sure this is right!
#define SECONDS_OF_SAMPLES  (5)
#define SAMPLE_RATE         (50)
#define MAX_SAMPLES         (SECONDS_OF_SAMPLES * SAMPLE_RATE)

uint32_t fifo_buffer[16];
uint32_t databufferptr = 0;
uint16_t databuffer_ir[MAX_SAMPLES];
uint16_t databuffer_red[MAX_SAMPLES];

void setup() {
  while (!Serial);
  delay(100);
  Serial.begin(115200);
  Serial.println("MAX30100 test!");

  Serial.print("I2C buffer length: "); Serial.println(SERIAL_BUFFER_SIZE);
  if (SERIAL_BUFFER_SIZE < 64) {
    Serial.println("Buffer must be 64 bytes long at least!");
  }
  if (!pulse.begin()) {
    Serial.println("Failed to find MAX30100");
    while (1);
  }
  Serial.println("MAX sensor found");
  pinMode(MAX30100_IRQPIN, INPUT);
  
  /***************** Set & check SpO2 sample rate */
  pulse.setSpO2SampleRate(MAX30100_SPO2SR_50HZ);

  max30100_spo2_samplerate_t rate = pulse.getSpO2SampleRate();
  uint16_t ratehz = 0;
  switch (rate) {
    case MAX30100_SPO2SR_50HZ: ratehz = 50; break;
    case MAX30100_SPO2SR_100HZ: ratehz = 100; break;
    case MAX30100_SPO2SR_167HZ: ratehz = 167; break;
    case MAX30100_SPO2SR_200HZ: ratehz = 200; break;
    case MAX30100_SPO2SR_400HZ: ratehz = 400; break;
    case MAX30100_SPO2SR_600HZ: ratehz = 600; break;
    case MAX30100_SPO2SR_800HZ: ratehz = 800; break;
    case MAX30100_SPO2SR_1000HZ: ratehz = 1000; break;
  }
  Serial.print("SpO2 sample rate = "); Serial.print(ratehz); Serial.println(" Hz");

  /***************** Set & check LED pulse width */
  pulse.setLEDpulseWidth(MAX30100_LEDPW_1600US);

  max30100_led_pulsewidth_t width = pulse.getLEDpulseWidth();
  uint16_t pulsewidth = 0;
  switch (width) {
    case  MAX30100_LEDPW_200US: pulsewidth = 200; break;
    case  MAX30100_LEDPW_400US: pulsewidth = 400; break;
    case  MAX30100_LEDPW_800US: pulsewidth = 800; break;
    case  MAX30100_LEDPW_1600US: pulsewidth = 1600; break;
  }
  Serial.print("LED pulse width = "); Serial.print(pulsewidth); Serial.println(" uS");

  /***************** Set & check LED current */
  pulse.setRedLEDcurrent(MAX30100_LEDCURRENT_30_6MA);
  pulse.setIRLEDcurrent(MAX30100_LEDCURRENT_11MA);
  max30100_led_current_t red = pulse.getRedLEDcurrent();
  max30100_led_current_t ir = pulse.getIRLEDcurrent();

  float ledcurrent = 0;
  switch (red) {
    case MAX30100_LEDCURRENT_0MA:    ledcurrent = 0; break;
    case MAX30100_LEDCURRENT_4_4MA:  ledcurrent = 4.4; break;
    case MAX30100_LEDCURRENT_7_6MA:  ledcurrent = 7.6; break;
    case MAX30100_LEDCURRENT_11MA:   ledcurrent = 11; break;
    case MAX30100_LEDCURRENT_14_2MA: ledcurrent = 14.2; break;
    case MAX30100_LEDCURRENT_17_4MA: ledcurrent = 17.4; break;
    case MAX30100_LEDCURRENT_20_8MA: ledcurrent = 20.8; break;
    case MAX30100_LEDCURRENT_24MA:   ledcurrent = 24; break;
    case MAX30100_LEDCURRENT_27_1MA: ledcurrent = 27.1; break;
    case MAX30100_LEDCURRENT_30_6MA: ledcurrent = 30.6; break;
    case MAX30100_LEDCURRENT_33_8MA: ledcurrent = 33.8; break;
    case MAX30100_LEDCURRENT_37MA:   ledcurrent = 37; break;
    case MAX30100_LEDCURRENT_40_2MA: ledcurrent = 40.2; break;
    case MAX30100_LEDCURRENT_43_6MA: ledcurrent = 43.6; break;
    case MAX30100_LEDCURRENT_46_8MA: ledcurrent = 46.8; break;
    case MAX30100_LEDCURRENT_50MA:   ledcurrent = 50; break;    
  }
  Serial.print("Red LED current = "); Serial.print(ledcurrent); Serial.println(" mA");

  switch (ir) {
    case MAX30100_LEDCURRENT_0MA:    ledcurrent = 0; break;
    case MAX30100_LEDCURRENT_4_4MA:  ledcurrent = 4.4; break;
    case MAX30100_LEDCURRENT_7_6MA:  ledcurrent = 7.6; break;
    case MAX30100_LEDCURRENT_11MA:   ledcurrent = 11; break;
    case MAX30100_LEDCURRENT_14_2MA: ledcurrent = 14.2; break;
    case MAX30100_LEDCURRENT_17_4MA: ledcurrent = 17.4; break;
    case MAX30100_LEDCURRENT_20_8MA: ledcurrent = 20.8; break;
    case MAX30100_LEDCURRENT_24MA:   ledcurrent = 24; break;
    case MAX30100_LEDCURRENT_27_1MA: ledcurrent = 27.1; break;
    case MAX30100_LEDCURRENT_30_6MA: ledcurrent = 30.6; break;
    case MAX30100_LEDCURRENT_33_8MA: ledcurrent = 33.8; break;
    case MAX30100_LEDCURRENT_37MA:   ledcurrent = 37; break;
    case MAX30100_LEDCURRENT_40_2MA: ledcurrent = 40.2; break;
    case MAX30100_LEDCURRENT_43_6MA: ledcurrent = 43.6; break;
    case MAX30100_LEDCURRENT_46_8MA: ledcurrent = 46.8; break;
    case MAX30100_LEDCURRENT_50MA:   ledcurrent = 50; break;    
  }
  Serial.print("IR LED current = "); Serial.print(ledcurrent); Serial.println(" mA");

  pulse.startRead();

}



void loop() {
  databufferptr = 0;
  while (databufferptr < MAX_SAMPLES) {
    if (digitalRead(MAX30100_IRQPIN)) continue;
    // some data ready!
    uint32_t fifodata = pulse.readRegister32(MAX30100_FIFO_DATA_REG);
    databuffer_ir[databufferptr] = fifodata >> 16;
    databuffer_red[databufferptr] = fifodata & 0xFFFF;
    databufferptr++;
  }
  Serial.println("Data read!");
  for (uint16_t i=0; i<MAX_SAMPLES; i++) {
    Serial.println(databuffer_ir[i]);
  }
  return;
  
  databufferptr = 0;
  while ((databufferptr+16) < MAX_SAMPLES) {
    while (digitalRead(MAX30100_IRQPIN)); // wait until IRQ goes low!
    // read 8 bytes of fifo
    pulse.readFIFO(fifo_buffer, 8);
    for (uint8_t i=0; i<8; i++) {
      databuffer_ir[databufferptr+i] = fifo_buffer[i] >> 16;
      databuffer_red[databufferptr+i] = fifo_buffer[i] & 0xFFFF;
    }
    databufferptr += 8;
  }
  Serial.println("Data read!");
  //Serial.print("Temperature = "); Serial.println(pulse.readTemperature(), 4);

  for (uint16_t i=0; i<MAX_SAMPLES; i++) {
    Serial.println(databuffer_red[i]);
    
  }
  delay(1000);
}
