#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"
#include "nordic_common.h"
#include "boards.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#include "MAX30105.h"

#define UART_TX_BUF_SIZE 256
#define UART_RX_BUF_SIZE 256

#define TWI_INSTANCE_ID 0


static volatile bool m_xfer_done = false;
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

void uart_error_handle(app_uart_evt_t * p_event){
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context){
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                printf("SOME DATA HANDLER\r\n");
            }
            m_xfer_done = true;
            break;
        default:
            break;
    }
}

void twi_init (void){
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = 31,
       .sda                = 30,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

void uart_init (void){
    uint32_t err_code;

    bsp_board_leds_init();

    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_ENABLED,
          false,
          UART_BAUDRATE_BAUDRATE_Baud115200
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

    APP_ERROR_CHECK(err_code);
}

Device *init_device(uint8_t mode, uint8_t proxy){
	
	InterruptSettings *intSettings = max30105_setup_interrupt(0,0,0,0,0);
	FIFOSettings *fifoSettings = max30105_setup_fifo(SAMPLE_AVG_32,FIFO_ROLLOVER_ON,FIFO_A_FULL_OFF);	
	OperationalSettings *opSettings = max30105_setup_operational( ADC_RANGE_MIN, SAMPLE_RATE_1000, LED_PULSE_WIDTH_411,
																																				LED_PULSE_AMP_LOW, LED_PULSE_AMP_LOW, LED_PULSE_AMP_LOW, LED_PULSE_AMP_MEDIUM,
																																				LED_SLOT_RED_PILOT, LED_SLOT_IR_PILOT, LED_SLOT_GREEN_PILOT, LED_SLOT_OFF);
	
	MeasurementBuffer *meaBuf = max30105_setup_measurement_buffer();	
	
	Device *device = max30105_setup_device(mode, proxy, intSettings, fifoSettings, opSettings, meaBuf);
	
	return device;
}

void measurement_circle(nrf_drv_twi_t instance, Device *dev, uint32_t count, uint8_t sampleAvg, uint8_t adcRange, uint8_t sampleRate, uint8_t pulseWidth, uint8_t amplitude){
	

	Device *device = dev;
	device->fifoSettings->sampleAverage = sampleAvg;
	device->opSettings->adcRange = adcRange;
	device->opSettings->sampleRate = sampleRate;
	device->opSettings->ledPulseWidth = pulseWidth;
	device->opSettings->ledPilotAmp = amplitude;
	
	max30105_reset(instance);
	max30105_configure_device(instance, device);
	nrf_delay_ms(5);
//	max30105_wake_up(instance,device);
//	nrf_delay_ms(5);	
	
	uint32_t i = 0;
	while(i < count){
		
		max30105_read_fifo(instance, device);
		
		if(device->meaBuf->tail != device->meaBuf->head){
			
			printf("%d\t%d\t%d\n\r", device->meaBuf->redLed[device->meaBuf->tail], device->meaBuf->irLed[device->meaBuf->tail], device->meaBuf->greenLed[device->meaBuf->tail]);	
			nrf_delay_us(10);
			device->meaBuf->tail++;
			device->meaBuf->tail %= 32;
			i++;
		}
	}
}

int main(void)
{
	uart_init();
	twi_init();
	Device *firstDevice = init_device(MODE_RED_IR_GREEN, 0x0F);
	
  printf("\r\n<><><><><>Start!<><><><><>\r\n");
	
	max30105_reset(m_twi);
	nrf_delay_ms(20);
	
	max30105_configure_device(m_twi, firstDevice);
	nrf_delay_ms(20);
	
	float temperature;
	max30105_read_temperature(m_twi, &temperature);
	printf("\n\rTEMPERATURE: %f\n\r", temperature);
	nrf_delay_ms(20);
	
	measurement_circle(m_twi, firstDevice, 10000, SAMPLE_AVG_OFF, ADC_RANGE_MIN, SAMPLE_RATE_3200, LED_PULSE_WIDTH_69, 0x03);

/*	
  printf("\r\nRED:\t\tIR:\t\tGREEN:r\n");	
	for(uint8_t l = 1; l < 250; l+=5){
		printf("\n\r LED_PULSE_AMP_PILOT: %d\n\r", l);
		
		for(uint8_t j = 0; j <= 0x07; j++){
			printf("\n\r SAMPLE_RATE: %d\n\r", j);
			
			for(uint8_t k = 0; k <= 0x03; k++){
				printf("\n\r LED_PULSE_WIDTH: %d\n\r", k);

				for(uint8_t i = 0; i <= 0x03; i++){
					printf("\n\r ADC_RANGE: %d\n\r", i);
					
					measurement_circle(m_twi, firstDevice, 10, SAMPLE_AVG_OFF, i, j, k, l);
				}
			}
		}
	}
*/
  printf("\r\n<><><><><>End!<><><><><>\r\n");
	while(true){
	
	}
}
