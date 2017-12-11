#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#define MAX_ADDR 							0x57

#define INT_STATUS1						0x00
#define INT_STATUS2						0x01
#define INT_ENABLE1						0x02
#define INT_ENABLE2						0x03

#define FIFO_WR_PTR						0x04
#define FIFO_OVERFLOW					0x05
#define FIFO_RD_PTR						0x06
#define FIFO_DATA							0x07

#define CONF_FIFO							0x08
#define CONF_MODE							0x09
#define CONF_SPO2							0x0A
#define CONF_LED_PULSE_RED		0x0C
#define CONF_LED_PULSE_IR			0x0D
#define CONF_LED_PULSE_GREEN	0x0E
#define CONF_LED_PULSE_PILOT	0x10
#define CONF_SLOT1						0x11
#define CONF_SLOT2						0x12

#define TEMP_INTEGER					0x1F
#define TEMP_FRACTION					0x20
#define TEMP_CONFIG						0x21

#define PROXIMITY_INT_TH			0x30
#define DEV_REV_ID						0xFE
#define DEV_PART_ID						0xFF

#define SAMPLE_AVG_OFF				0x00
#define SAMPLE_AVG_2					0x01
#define SAMPLE_AVG_4					0x02
#define SAMPLE_AVG_8					0x03
#define SAMPLE_AVG_16					0x04
#define SAMPLE_AVG_32					0x05

#define FIFO_A_FULL_OFF				0x00
#define FIFO_A_FULL_ON				0x0F
#define FIFO_ROLLOVER_OFF			0x00
#define FIFO_ROLLOVER_ON			0x01

#define MODE_RED							0x02
#define MODE_RED_IR						0x03
#define MODE_RED_IR_GREEN			0x07

#define ADC_RANGE_MIN					0x00
#define ADC_RANGE_LOW					0x01
#define ADC_RANGE_HIGH				0x02
#define ADC_RANGE_MAX					0x03

#define SAMPLE_RATE_50				0x00 //samples per second
#define SAMPLE_RATE_100				0x01
#define SAMPLE_RATE_200				0x02
#define SAMPLE_RATE_400				0x03
#define SAMPLE_RATE_800				0x04
#define SAMPLE_RATE_1000			0x05
#define SAMPLE_RATE_1600			0x06
#define SAMPLE_RATE_3200			0x07

#define LED_PULSE_WIDTH_69		0x00 //affects ADC resolution -> 15,16,17,18 bits
#define LED_PULSE_WIDTH_118		0x01
#define LED_PULSE_WIDTH_215		0x02
#define LED_PULSE_WIDTH_411		0x03

#define LED_PULSE_AMP_MIN			0x01 //max 50mA, min 0.2mA
#define LED_PULSE_AMP_LOW			0x40
#define LED_PULSE_AMP_MEDIUM	0x7F
#define LED_PULSE_AMP_HIGH		0xC0
#define LED_PULSE_AMP_MAX			0xFF

#define LED_SLOT_OFF					0x00
#define LED_SLOT_RED					0x01
#define LED_SLOT_IR						0x02
#define LED_SLOT_GREEN				0x03
#define LED_SLOT_RED_PILOT		0x05
#define LED_SLOT_IR_PILOT			0x06
#define LED_SLOT_GREEN_PILOT	0x07


#define CHECK_ERROR(e) if(e != NRF_SUCCESS) return e;

typedef struct {
	bool almostFullFIFO;
	bool newFIFOdata;
	bool ambientOverflow;
	bool proximityThReached;
	bool dieTempRdy;
} InterruptSettings, *InterruptSettingsPtr;

typedef struct {
	uint8_t sampleAverage:3;
	bool rollOver;
	uint8_t fifoAlmostFull:4;
} FIFOSettings;

typedef struct  {
	uint8_t adcRange:2;
	uint8_t sampleRate:3;
	uint8_t ledPulseWidth:2;
	uint8_t ledRedAmp;
	uint8_t ledIrAmp;
	uint8_t ledGreenAmp;
	uint8_t ledPilotAmp;
	uint8_t timeSlot1:3;
	uint8_t timeSlot2:3;
	uint8_t timeSlot3:3;
	uint8_t timeSlot4:3;
} OperationalSettings, *OperationalSettingsPtr;

typedef struct {
	uint8_t head;
	uint8_t tail;
	uint32_t redLed[32];
	uint32_t irLed[32];
	uint32_t greenLed[32];
	
} MeasurementBuffer;

typedef struct {
	uint8_t mode:3;
	uint8_t proximity;
	FIFOSettings *fifoSettings;
	InterruptSettings *intSettings;
	OperationalSettings *opSettings;
	MeasurementBuffer *meaBuf;
} Device;

static ret_code_t write_reg_8(nrf_drv_twi_t instance, uint8_t reg, uint8_t data);
static ret_code_t read_reg(nrf_drv_twi_t instance, uint8_t reg, uint8_t *data);

static ret_code_t set_int_reg(nrf_drv_twi_t instance, InterruptSettings *intSettings);
static ret_code_t set_fifo_reg(nrf_drv_twi_t instance, FIFOSettings *fifoSettings);
static ret_code_t set_op_reg(nrf_drv_twi_t instance, OperationalSettings *opSettings);
static ret_code_t set_dev_reg(nrf_drv_twi_t instance, Device *device);

extern InterruptSettings *max30105_setup_interrupt(bool almostFullFIFO, bool newFIFOdata, bool ambientOverflow, bool proximityThReached, bool dieTempRdy);
extern FIFOSettings *max30105_setup_fifo(uint8_t sampleAverage, bool rollOver, uint8_t fifoAlmostFull);
extern OperationalSettings *max30105_setup_operational(uint8_t adcRange, uint8_t sampleRate, uint8_t ledPulseWidth, uint8_t ledRedAmp, uint8_t ledIrAmp, uint8_t ledGreenAmp, uint8_t ledPilotAmp, uint8_t timeSlot1, uint8_t timeSlot2, uint8_t timeSlot3, uint8_t timeSlot4);
extern Device *max30105_setup_device(uint8_t mode, uint8_t proximity, InterruptSettings *intSettings, FIFOSettings *fifoSettings, OperationalSettings *opSettings, MeasurementBuffer *meaBuf);
extern MeasurementBuffer *max30105_setup_measurement_buffer(void);

extern ret_code_t max30105_configure_device(nrf_drv_twi_t instance, Device *device);
extern ret_code_t max30105_print_reg_map(nrf_drv_twi_t instance);
extern ret_code_t max30105_read_temperature(nrf_drv_twi_t instance, float *data);
extern ret_code_t max30105_read_fifo(nrf_drv_twi_t instance, Device* device);
extern ret_code_t max30105_sleep(nrf_drv_twi_t instance, Device* device);
extern ret_code_t max30105_wake_up(nrf_drv_twi_t instance, Device* device);
extern ret_code_t max30105_reset(nrf_drv_twi_t instance);

