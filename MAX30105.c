#include "MAX30105.h"

static ret_code_t write_reg_8(nrf_drv_twi_t instance, uint8_t reg, uint8_t data){
	
	ret_code_t err_code;
	
	uint8_t tab[2] = {reg, data};
	err_code = nrf_drv_twi_tx(&instance, MAX_ADDR, tab, sizeof(tab), false);
	
	return err_code;
}

static ret_code_t read_reg(nrf_drv_twi_t instance, uint8_t reg, uint8_t *data){
	
	ret_code_t err_code;
	uint8_t reg_p[1] = {reg};
	
	err_code = nrf_drv_twi_tx(&instance, MAX_ADDR, reg_p, sizeof(reg_p), false);
	CHECK_ERROR(err_code);
	err_code = nrf_drv_twi_rx(&instance, MAX_ADDR, data, sizeof(data));
	
	return err_code;
}

static ret_code_t set_int_reg(nrf_drv_twi_t instance, InterruptSettings *intSettings){
	
	ret_code_t err_code;
	uint8_t data;
	
	data = 0xF0 & ((intSettings->almostFullFIFO<<7) | (intSettings->newFIFOdata<<6) | (intSettings->ambientOverflow<<5) | (intSettings->proximityThReached<<4));
	err_code = write_reg_8(instance, INT_ENABLE1, data);
	data = 0x02 & (intSettings->dieTempRdy<<1);
	err_code = write_reg_8(instance, INT_ENABLE2, data);	

	return err_code;
}

static ret_code_t set_fifo_reg(nrf_drv_twi_t instance, FIFOSettings *fifoSettings){
	
	ret_code_t err_code;
	uint8_t data;
	
	data = 0xFF & ((fifoSettings->sampleAverage<<5) | (fifoSettings->rollOver<<4) | fifoSettings->fifoAlmostFull);
	err_code = write_reg_8(instance, CONF_FIFO, data);
	
	return err_code;
}

static ret_code_t set_op_reg(nrf_drv_twi_t instance, OperationalSettings *opSettings){
	
	ret_code_t err_code;
	uint8_t data;
	
	data = 0x7F & ((opSettings->adcRange<<5) | (opSettings->sampleRate<<2) | (opSettings->ledPulseWidth));
	err_code = write_reg_8(instance, CONF_SPO2, data);
	
	data = opSettings->ledRedAmp;
	err_code = write_reg_8(instance, CONF_LED_PULSE_RED, data);
	data = opSettings->ledIrAmp;
	err_code = write_reg_8(instance, CONF_LED_PULSE_IR, data);
	data = opSettings->ledGreenAmp;
	err_code = write_reg_8(instance, CONF_LED_PULSE_GREEN, data);	
	data = opSettings->ledPilotAmp;
	err_code = write_reg_8(instance, CONF_LED_PULSE_PILOT, data);
	
	data = 0x77 &((opSettings->timeSlot2<<4) | opSettings->timeSlot1);
	err_code = write_reg_8(instance, CONF_SLOT1, data);
	data = 0x77 &((opSettings->timeSlot4<<4) | opSettings->timeSlot3);
	err_code = write_reg_8(instance, CONF_SLOT2, data);
	
	return err_code;
}

static ret_code_t set_dev_reg(nrf_drv_twi_t instance, Device *device){
	
	ret_code_t err_code;
	uint8_t data;
	
	data = 0x07 & (device->mode);
	err_code = write_reg_8(instance, CONF_MODE, data);
	data = 0xFF & (device->proximity);
	err_code = write_reg_8(instance, PROXIMITY_INT_TH, data);
	
	return err_code;
}

extern InterruptSettings *max30105_setup_interrupt(bool almostFullFIFO, bool newFIFOdata, bool ambientOverflow, bool proximityThReached, bool dieTempRdy){
	
	InterruptSettings *intSettings = malloc(sizeof(InterruptSettings));
	intSettings->almostFullFIFO = almostFullFIFO;
	intSettings->newFIFOdata = newFIFOdata;
	intSettings->ambientOverflow = ambientOverflow;
	intSettings->proximityThReached = proximityThReached;
	intSettings->dieTempRdy = dieTempRdy;
	
	return intSettings;
}

extern FIFOSettings *max30105_setup_fifo(uint8_t sampleAverage, bool rollOver, uint8_t fifoAlmostFull){
	
	FIFOSettings *fifoSettings = malloc(sizeof(FIFOSettings));
	fifoSettings->sampleAverage = sampleAverage;
	fifoSettings->rollOver = rollOver;
	fifoSettings->fifoAlmostFull = fifoAlmostFull;
	
	return fifoSettings;
}

extern OperationalSettings *max30105_setup_operational(uint8_t adcRange, uint8_t sampleRate, uint8_t ledPulseWidth, uint8_t ledRedAmp, uint8_t ledIrAmp, uint8_t ledGreenAmp, uint8_t ledPilotAmp, uint8_t timeSlot1, uint8_t timeSlot2, uint8_t timeSlot3, uint8_t timeSlot4){
																					
	OperationalSettings *opSettings = malloc(sizeof(OperationalSettings));
	opSettings->adcRange = adcRange;
	opSettings->sampleRate = sampleRate;
	opSettings->ledPulseWidth = ledPulseWidth;
	opSettings->ledRedAmp = ledRedAmp;
	opSettings->ledIrAmp = ledIrAmp;
	opSettings->ledGreenAmp = ledGreenAmp;
	opSettings->ledPilotAmp = ledPilotAmp;
	opSettings->timeSlot1 = timeSlot1;
	opSettings->timeSlot2 = timeSlot2;
	opSettings->timeSlot3 = timeSlot3;
	opSettings->timeSlot4 = timeSlot4;			
																					
	return opSettings;
}

extern Device *max30105_setup_device(uint8_t mode, uint8_t proximity, InterruptSettings *intSettings, FIFOSettings *fifoSettings, OperationalSettings *opSettings, MeasurementBuffer *meaBuf){
	
	Device *device = malloc(sizeof(Device));
	device->mode = mode;
	device->proximity = proximity;
	device->intSettings = intSettings;
	device->fifoSettings = fifoSettings;
	device->opSettings = opSettings;
	device->meaBuf = meaBuf;
	
	return device;
}

extern MeasurementBuffer *max30105_setup_measurement_buffer(void){
	
	MeasurementBuffer *meaBuff = malloc(sizeof(MeasurementBuffer));
	meaBuff->head = 0;
	meaBuff->tail = 0;
	
	return meaBuff;
}

extern ret_code_t max30105_configure_device(nrf_drv_twi_t instance, Device *device){
	
	ret_code_t err_code;
	

	CHECK_ERROR(set_int_reg(instance, device->intSettings));
	
	err_code = set_fifo_reg(instance, device->fifoSettings);
	CHECK_ERROR(err_code);
	
	err_code = set_op_reg(instance, device->opSettings);
	CHECK_ERROR(err_code);
	
	err_code = set_dev_reg(instance, device);
	CHECK_ERROR(err_code);
	
	return err_code;
}

extern ret_code_t max30105_print_reg_map(nrf_drv_twi_t instance) {
	
	ret_code_t err_code;
	uint8_t reg[1] = {0};
	uint8_t data[1] = {0};
	
	printf("\n\n\rMAP:");
	for(uint16_t i = 0; i <= 0xFF; i++){
		if((i>=0x31 && i<=0xFD) || (i>=0x13 && i<=0x1E) || (i>=0x22 && i<=0x2F) || i == 0x0B || i == 0x0F)
			continue;
		
		reg[0] = i;
		err_code = nrf_drv_twi_tx(&instance, MAX_ADDR, reg, sizeof(reg), false);
		CHECK_ERROR(err_code);
		err_code = nrf_drv_twi_rx(&instance, MAX_ADDR, data, sizeof(data));
		CHECK_ERROR(err_code);
		
		if((i%4) == 0)
			printf("\n\r");
		printf("%02x: %02x\t\t",i, data[0]);
	}
	
	return err_code;
}

extern ret_code_t max30105_read_temperature(nrf_drv_twi_t instance, float *data){
	ret_code_t err_code;
	err_code = write_reg_8(instance, TEMP_CONFIG, 0x01);
	CHECK_ERROR(err_code);
	uint8_t response;
	
	while(1){
		err_code = read_reg(instance, TEMP_CONFIG, &response);
		CHECK_ERROR(err_code);
		
		if ((response & 0x01) == 0) break;
		nrf_delay_ms(1);
	}
	
	err_code = read_reg(instance, TEMP_INTEGER, &response);
	CHECK_ERROR(err_code);
	int8_t tempInt = (int8_t)response;
	err_code = read_reg(instance, TEMP_FRACTION, &response);
	CHECK_ERROR(err_code);
	uint8_t tempFrac = response;
	float *result_p = data;
	*result_p = (float)tempInt + ((float)tempFrac * 0.0625f);
	
	return err_code;
}

extern ret_code_t max30105_read_fifo(nrf_drv_twi_t instance, Device* device){
	
	ret_code_t err_code;
	uint8_t rdPtr;
	uint8_t wrPtr;
	uint8_t numOfChannels;
	uint32_t adcResolution;

	err_code = read_reg(instance, FIFO_RD_PTR, &rdPtr);
	CHECK_ERROR(err_code);
	err_code = read_reg(instance, FIFO_WR_PTR, &wrPtr);
	CHECK_ERROR(err_code);
	
	if(rdPtr != wrPtr){
		
		if(device->mode == MODE_RED)
			numOfChannels = 1;
		else if(device->mode == MODE_RED_IR)
			numOfChannels = 2;
		else if(device->mode == MODE_RED_IR_GREEN)
			numOfChannels = 3;
		else
			numOfChannels = 0;	
		
		if(device->opSettings->ledPulseWidth == LED_PULSE_WIDTH_411)
			adcResolution = 0x3FFFF;
		else if(device->opSettings->ledPulseWidth == LED_PULSE_WIDTH_215)
			adcResolution = 0x1FFFF;
		else if(device->opSettings->ledPulseWidth == LED_PULSE_WIDTH_118)
			adcResolution = 0xFFFF;
		else
			adcResolution = 0x7FFF;
		
		int8_t numOfSamples = wrPtr - rdPtr;
		if(numOfSamples < 0)
			numOfSamples += 32;
		uint8_t bytesToRead = numOfSamples*numOfChannels*3;

		uint8_t reg_p = FIFO_DATA;
		err_code = nrf_drv_twi_tx(&instance, MAX_ADDR, &reg_p, sizeof(reg_p), false);
		CHECK_ERROR(err_code);
		
		uint8_t result[bytesToRead];
		err_code = nrf_drv_twi_rx(&instance, MAX_ADDR, result, sizeof(result));
		CHECK_ERROR(err_code);
		
		uint32_t tempLong;
		uint8_t temp[4];
		
		for(int i = 0; i < numOfSamples; i++){
			device->meaBuf->head++;
			device->meaBuf->head %= 32;
			for(int k = 0; k < numOfChannels; k++){
				
				temp[3] = 0;
				temp[2] = result[(i*3*numOfChannels)+(k*3)];
				temp[1] = result[(i*3*numOfChannels)+(k*3)+1];
				temp[0] = result[(i*3*numOfChannels)+(k*3)+2];
				
				memcpy(&tempLong, temp, sizeof(tempLong));
				tempLong &= adcResolution;
				
				if(k == 0)
					device->meaBuf->redLed[device->meaBuf->head] = tempLong;
				if(k == 1)
					device->meaBuf->irLed[device->meaBuf->head] = tempLong;
				if(k == 2)
					device->meaBuf->greenLed[device->meaBuf->head] = tempLong;
			}
		}
	}
	return err_code;
}

extern ret_code_t max30105_sleep(nrf_drv_twi_t instance, Device* device){
	
	ret_code_t err_code;
	
	uint8_t sleep = device->mode | 0x80;
	uint8_t reg_p[2] = {CONF_MODE, sleep};
	err_code = nrf_drv_twi_tx(&instance, MAX_ADDR, reg_p, sizeof(reg_p), false);
	
	return err_code;
}

extern ret_code_t max30105_wake_up(nrf_drv_twi_t instance, Device* device){
	
	ret_code_t err_code;
	
	uint8_t wakeUp = device->mode & 0x07;
	uint8_t reg_p[2] = {CONF_MODE, wakeUp};
	err_code = nrf_drv_twi_tx(&instance, MAX_ADDR, reg_p, sizeof(reg_p), false);
	
	return err_code;
}

extern ret_code_t max30105_reset(nrf_drv_twi_t instance){
	
	ret_code_t err_code;
	uint8_t reset = 0x40;
	
	uint8_t reg_p[2] = {CONF_MODE, reset};
	err_code = nrf_drv_twi_tx(&instance, MAX_ADDR, reg_p, sizeof(reg_p), false);
	uint8_t reg_p1[2] = {FIFO_WR_PTR, 0x00};
	err_code = nrf_drv_twi_tx(&instance, MAX_ADDR, reg_p1, sizeof(reg_p1), false);	
	uint8_t reg_p2[2] = {FIFO_RD_PTR, 0x00};
	err_code = nrf_drv_twi_tx(&instance, MAX_ADDR, reg_p2, sizeof(reg_p2), false);
	
	return err_code;
}
