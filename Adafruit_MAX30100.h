/**************************************************************************/
/*!
    @file     Adafruit_MAX30100.h
    @author   Limor Fried (Adafruit Industries)
    @license  BSD (see license.txt)

    This is a library for the Adafruit Max30100 breakout board
    ----> https://www.adafruit.com/products/xxxx

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define MAX30100_DEFAULT_ADDRESS         (0x57)


#define MAX30100_IRQSTAT_REG             0x00
#define MAX30100_IRQSTAT_PWRREADY        0x01
#define MAX30100_IRQSTAT_SPO2READY       0x10
#define MAX30100_IRQSTAT_HRREADY         0x20
#define MAX30100_IRQSTAT_TEMPREADY       0x40
#define MAX30100_IRQSTAT_AFULL           0x80

#define MAX30100_IRQENABLE_REG           0x01
#define MAX30100_IRQENABLE_SPO2READY     0x10
#define MAX30100_IRQENABLE_HRREADY       0x20
#define MAX30100_IRQENABLE_TEMPREADY     0x40
#define MAX30100_IRQENABLE_AFULL         0x80

#define MAX30100_FIFO_WRPTR_REG          0x02
#define MAX30100_FIFO_OFLOW_REG          0x03
#define MAX30100_FIFO_RDPTR_REG          0x04
#define MAX30100_FIFO_DATA_REG           0x05

#define MAX30100_MODECFG_REG             0x06
#define MAX30100_MODECFG_HRONLY          0x02
#define MAX30100_MODECFG_HRSP02          0x03
#define MAX30100_MODECFG_TEMPEN          0x08
#define MAX30100_MODECFG_RESET           0x40
#define MAX30100_MODECFG_SHDN            0x80

#define MAX30100_SPO2CFG_REG             0x07
#define MAX30100_SPO2CFG_HIGHRES         0x40

#define MAX30100_LEDCFG_REG              0x09

typedef enum
{
  MAX30100_SPO2SR_50HZ         = 0b000,
  MAX30100_SPO2SR_100HZ        = 0b001,
  MAX30100_SPO2SR_167HZ        = 0b010,
  MAX30100_SPO2SR_200HZ        = 0b011,
  MAX30100_SPO2SR_400HZ        = 0b100,
  MAX30100_SPO2SR_600HZ        = 0b101,
  MAX30100_SPO2SR_800HZ        = 0b110,
  MAX30100_SPO2SR_1000HZ       = 0b111,

} max30100_spo2_samplerate_t;

typedef enum
{
  MAX30100_LEDPW_200US         = 0b00,
  MAX30100_LEDPW_400US         = 0b01,
  MAX30100_LEDPW_800US         = 0b10,
  MAX30100_LEDPW_1600US        = 0b11,
} max30100_led_pulsewidth_t;


typedef enum
{
  MAX30100_LEDCURRENT_0MA      = 0b0000,
  MAX30100_LEDCURRENT_4_4MA    = 0b0001,
  MAX30100_LEDCURRENT_7_6MA    = 0b0010,
  MAX30100_LEDCURRENT_11MA     = 0b0011,
  MAX30100_LEDCURRENT_14_2MA   = 0b0100,
  MAX30100_LEDCURRENT_17_4MA   = 0b0101,
  MAX30100_LEDCURRENT_20_8MA   = 0b0110,
  MAX30100_LEDCURRENT_24MA     = 0b0111,
  MAX30100_LEDCURRENT_27_1MA   = 0b1000,
  MAX30100_LEDCURRENT_30_6MA   = 0b1001,
  MAX30100_LEDCURRENT_33_8MA   = 0b1010,
  MAX30100_LEDCURRENT_37MA     = 0b1011,
  MAX30100_LEDCURRENT_40_2MA   = 0b1100,
  MAX30100_LEDCURRENT_43_6MA   = 0b1101,
  MAX30100_LEDCURRENT_46_8MA   = 0b1110,
  MAX30100_LEDCURRENT_50MA     = 0b1111,
} max30100_led_current_t;

#define MAX30100_TEMPDATAINT_REG         0x16
#define MAX30100_TEMPDATAFRAC_REG        0x17

#define MAX30100_PARTREV_REG        0xFE
#define MAX30100_PARTID_REG         0xFF
#define MAX30100_PARTID_VAL         0x11

/*=========================================================================*/

class Adafruit_MAX30100 {
 public:
  Adafruit_MAX30100(void);
  bool begin(void);

  float readTemperature(void);

  void setSpO2SampleRate(max30100_spo2_samplerate_t rate);
  max30100_spo2_samplerate_t getSpO2SampleRate(void);

  void setLEDpulseWidth(max30100_led_pulsewidth_t pw);
  max30100_led_pulsewidth_t getLEDpulseWidth(void);

  void setRedLEDcurrent(max30100_led_current_t i);
  max30100_led_current_t getRedLEDcurrent(void);
  void setIRLEDcurrent(max30100_led_current_t i);
  max30100_led_current_t getIRLEDcurrent(void);

  void startRead(void);
  uint8_t readFIFO(uint32_t *fifoptr, uint8_t num);

  uint32_t readRegister32(uint8_t reg);

 private:
  
  uint8_t readRegister8(uint8_t reg);
  uint16_t readRegister16(uint8_t reg);
  void writeRegister8(uint8_t reg, uint8_t value);

  int8_t  _i2caddr;
};

/**************************************************************************/


