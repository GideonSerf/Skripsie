#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/timer.h"
#include "driver/uart.h"
#include <driver/adc.h>
#include "sdkconfig.h"

#include "esp_adc_cal.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "afsk.h"

volatile static uint8_t current_byte;
volatile static uint16_t current_sample_in_baud;          // 1 bit = SAMPLES_PER_BAUD samples
volatile static bool go = false;                         // Modem is on
volatile static uint16_t phase_delta;                    // 1200/2200 for standard AX.25
volatile static uint16_t phase;                          // Fixed point 9.7 (2PI = TABLE_SIZE)
volatile static uint16_t packet_pos;                     // Next bit to be sent out
static uint8_t sample_fifo[SAMPLE_FIFO_SIZE];   // queue of samples
volatile static uint8_t sample_fifo_head = 0;            // empty when head == tail
volatile static uint8_t sample_fifo_tail = 0;
volatile static uint32_t sample_overruns = 0;

volatile static unsigned int afsk_packet_size = 0;
volatile static const uint8_t *afsk_packet;

#define TX_DELAY 100
#define NOP() asm volatile ("nop")

#define MAX 4096.0
#define VREF 1096

#define BUF_SIZE (2048)

bool AFSK_ISR();

void afsk_init()
{
	gpio_reset_pin(TX_SELECT);
	gpio_set_direction(TX_SELECT,GPIO_MODE_OUTPUT);

	gpio_reset_pin(TXD);

	ledc_timer_config_t pwm_timer = {
	        .duty_resolution = PWM_RES, // resolution of PWM duty
	        .freq_hz = 62500,                      // frequency of PWM signal
	        .speed_mode = LEDC_HIGH_SPEED_MODE,           // timer mode
	        .timer_num = TX_TIMER,            // timer index
	        .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
	    };

	ESP_ERROR_CHECK(ledc_timer_config(&pwm_timer));

	ledc_channel_config_t pwm_channel = {
			.channel    = LEDC_CHANNEL_0,
			.duty       = 255,
			.gpio_num   = TXD,
			.speed_mode = LEDC_HIGH_SPEED_MODE,
			.hpoint     = 0,
			.timer_sel  = TX_TIMER,
	};
	ESP_ERROR_CHECK(ledc_channel_config(&pwm_channel));

	timer_config_t timer_config = {
	        .divider = TIMER_DIVIDER,
	        .counter_dir = TIMER_COUNT_UP,
	        .counter_en = TIMER_PAUSE,
	        .alarm_en = TIMER_ALARM_EN,
	        .auto_reload = true,
	    };

	    timer_init(TIMER_GROUP_0, TIMER_0, &timer_config);
	    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
	    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0,TIMER_COUNTER);

	    timer_enable_intr(TIMER_GROUP_0, TIMER_0);

	    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, AFSK_ISR, 0, 0);
}

void afsk_timer_start()
{
	  ESP_ERROR_CHECK(timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0));
	  ESP_ERROR_CHECK(timer_start(TIMER_GROUP_0, TIMER_0));
}

void afsk_timer_stop()
{
	ESP_ERROR_CHECK(timer_pause(TIMER_GROUP_0, TIMER_0));
}

inline static bool afsk_is_fifo_full()
{
  return (((sample_fifo_head + 1) % SAMPLE_FIFO_SIZE) == sample_fifo_tail);
}

inline static void afsk_fifo_in(uint8_t s)
{
  sample_fifo[sample_fifo_head] = s;
  sample_fifo_head = (sample_fifo_head + 1) % SAMPLE_FIFO_SIZE;
}

inline static uint8_t afsk_fifo_out()
{
  uint8_t s = sample_fifo[sample_fifo_tail];
  sample_fifo_tail = (sample_fifo_tail + 1) % SAMPLE_FIFO_SIZE;
  return s;
}

inline static bool afsk_is_fifo_empty()
{
  return (sample_fifo_head == sample_fifo_tail);
}

inline void afsk_output_sample(uint8_t s)
{
	ledc_set_duty(LEDC_HIGH_SPEED_MODE,LEDC_CHANNEL_0,s);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);

}

void afsk_send(const uint8_t *buffer, int len)
{
  afsk_packet_size = len;
  afsk_packet = buffer;
}

bool afsk_flush()
{
  while (! afsk_is_fifo_full()) {
    // If done sending packet
    if (packet_pos == afsk_packet_size) {
      go = false;         // End of transmission
    }
    if (go == false) {
      if (afsk_is_fifo_empty()) {
        afsk_timer_stop();  // Disable modem
        gpio_set_level(TX_SELECT, 1);    // Release PTT
        return false;       // Done
      } else {
        return true;
      }
    }

    // If sent SAMPLES_PER_BAUD already, go to the next bit
    if (current_sample_in_baud < (1 << 8)) {    // Load up next bit
      if ((packet_pos & 7) == 0) {         // Load up next byte
        current_byte = afsk_packet[packet_pos >> 3];
      } else {
        current_byte = current_byte / 2;  // ">>1" forces int conversion
      }
      if ((current_byte & 1) == 0) {
        // Toggle tone (1200 <> 2200)
        phase_delta ^= (PHASE_DELTA_1200 ^ PHASE_DELTA_2200);
      }
    }

    phase += phase_delta;
    uint8_t s = afsk_read_sample((phase >> 7) & (TABLE_SIZE - 1));


    afsk_fifo_in(s);

    current_sample_in_baud += (1 << 8);
    if (current_sample_in_baud >= SAMPLES_PER_BAUD) {
      packet_pos++;
      current_sample_in_baud -= SAMPLES_PER_BAUD;
    }
  }

  return true;  // still working
}

void afsk_start()
{
  phase_delta = PHASE_DELTA_1200;
  phase = 0;
  packet_pos = 0;
  current_sample_in_baud = 0;
  go = true;

  // Prime the fifo
  afsk_flush();

  // Key the radio
  gpio_set_level(TX_SELECT, 0);
  vTaskDelay(10 / portTICK_PERIOD_MS);
  // Start transmission
  afsk_timer_start();
}

bool AFSK_ISR()
{
  if (afsk_is_fifo_empty()) {
    if (go) {
      sample_overruns++;
    }
  } else {

    afsk_output_sample(afsk_fifo_out());
  }
  return false;
}

struct s_address {
	char callsign[7];
	unsigned char ssid;
};

typedef struct s_environment{
	int tempV;
	int pressV;
	float tempC;
	float pressKpa;
} environment;


#define S_CALLSIGN      "GIDEON"
#define S_CALLSIGN_ID   11

// Destination callsign: APRS (with SSID=0) is usually okay.
#define D_CALLSIGN      "GROUND"
#define D_CALLSIGN_ID   0

// Module constants
#define MAX_PACKET 512  // bytes

// Module globals
static uint16_t crc;
static uint8_t ones_in_a_row;
static uint8_t packet[MAX_PACKET];
static unsigned int packet_size;

// Module functions
static void
update_crc(uint8_t a_bit)
{
  crc ^= a_bit;
  if (crc & 1)
    crc = (crc >> 1) ^ 0x8408;  // X-modem CRC poly
  else
    crc = crc >> 1;
}

static void
send_byte(uint8_t a_byte)
{
  uint8_t i = 0;
  while (i++ < 8) {
    uint8_t a_bit = a_byte & 1;
    a_byte >>= 1;
    update_crc(a_bit);
    if (a_bit) {
      // Next bit is a '1'
      if (packet_size >= MAX_PACKET * 8)  // Prevent buffer overrun
        return;
      packet[packet_size >> 3] |= (1 << (packet_size & 7));
      packet_size++;
      if (++ones_in_a_row < 5) continue;
    }
    // Next bit is a '0' or a zero padding after 5 ones in a row
    if (packet_size >= MAX_PACKET * 8)    // Prevent buffer overrun
      return;
    packet[packet_size >> 3] &= ~(1 << (packet_size & 7));
    packet_size++;
    ones_in_a_row = 0;
  }
}

// Exported functions
void
ax25_send_byte(uint8_t a_byte)
{
  // Wrap around send_byte, but prints debug info
  send_byte(a_byte);
}

void
ax25_send_flag()
{
  uint8_t flag = 0x7e;
  int i;
  for (i = 0; i < 8; i++, packet_size++) {
    if (packet_size >= MAX_PACKET * 8)  // Prevent buffer overrun
      return;
    if ((flag >> i) & 1)
      packet[packet_size >> 3] |= (1 << (packet_size & 7));
    else
      packet[packet_size >> 3] &= ~(1 << (packet_size & 7));
  }
}

void
ax25_send_string(const char *string)
{
  int i;
  for (i = 0; string[i]; i++) {
    ax25_send_byte(string[i]);
  }
}

void
ax25_send_header(const struct s_address *addresses, int num_addresses)
{
  int i, j;
  packet_size = 0;
  ones_in_a_row = 0;
  crc = 0xffff;

  // Send flags during TX_DELAY milliseconds (8 bit-flag = 8000/1200 ms)
  for (i = 0; i < TX_DELAY * 3 / 20; i++) {
    ax25_send_flag();
  }

  for (i = 0; i < num_addresses; i++) {
    // Transmit callsign
    for (j = 0; addresses[i].callsign[j]; j++)
      send_byte(addresses[i].callsign[j] << 1);
    // Transmit pad
    for ( ; j < 6; j++)
      send_byte(' ' << 1);
    // Transmit SSID. Termination signaled with last bit = 1
    if (i == num_addresses - 1)
      send_byte(('0' + addresses[i].ssid) << 1 | 1);
    else
      send_byte(('0' + addresses[i].ssid) << 1);
  }

  // Control field: 3 = APRS-UI frame
  send_byte(0x03);

  // Protocol ID: 0xf0 = no layer 3 data
  send_byte(0xf0);
}

void
ax25_send_footer()
{
  // Save the crc so that it can be treated it atomically
  uint16_t final_crc = crc;

  // Send the CRC
  send_byte(~(final_crc & 0xff));
  final_crc >>= 8;
  send_byte(~(final_crc & 0xff));

  // Signal the end of frame
  ax25_send_flag();
}

void
ax25_flush_frame()
{
  // Key the transmitter and send the frame
  afsk_send(packet, packet_size);
  afsk_start();
}

const struct s_address addresses[] = {
    {D_CALLSIGN, D_CALLSIGN_ID},  // Destination callsign
    {S_CALLSIGN, S_CALLSIGN_ID}
};

#define PRESS ADC1_CHANNEL_2
#define TEMP ADC2_CHANNEL_8
#define AF ADC1_CHANNEL_7

#define getTempV(i) adc2_get_raw(TEMP,ADC_WIDTH_BIT_12, &i)
#define getPressV(i) i= adc1_get_raw(PRESS)


static xQueueHandle s_timer_queue;

bool TX_ISR()
{
	BaseType_t high_task_awoken = pdFALSE;

	bool tx = true;

	xQueueSendFromISR(s_timer_queue, &tx, &high_task_awoken);

	return high_task_awoken == pdTRUE;
}

#define top_thresh 3000
#define bottom_thresh 500

uint16_t receive_buffer[6];
uint8_t sample_head;

char* message_buffer[50];
uint8_t message_head;

uint16_t holder;

bool change;

uint8_t bit_received;

inline uint16_t RX_avg()
{
	int i=0, c = 0;
	for(i=0;i<6;i++)
	{
		c+=receive_buffer[i];
	}
	return c/6;
}

inline void move_on()
{
	int i=0;
	for(i=0;i<5;i++)
	{
		receive_buffer[i]=receive_buffer[i+1];
	}
}

bool RX_ISR()
{
	receive_buffer[sample_head] = adc1_get_raw(AF);
	if(sample_head==5)
	{
		holder = RX_avg();
		if(holder<bottom_thresh)
		{
			bit_received = 0b0000010;
			sample_head = 0;
		}
		else if(holder>top_thresh)
		{
			bit_received = 0b0000011;
			sample_head = 0;
		}
		else
		{
			bit_received= 0;
			move_on();
		}
		change = true;
	}
	else
	{
		sample_head++;
	}
}

void receive(void)
{
	change = false;

	bit_received = 0;

	sample_head = 0;
	message_head = 0;

	timer_config_t timer_config = {
		    	        .divider = 8,
		    	        .counter_dir = TIMER_COUNT_UP,
		    	        .counter_en = TIMER_PAUSE,
		    	        .alarm_en = TIMER_ALARM_EN,
		    	        .auto_reload = true,
		    	    };

			timer_init(TIMER_GROUP_1, TIMER_0, &timer_config);
			timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0);
			timer_set_alarm_value(TIMER_GROUP_1, TIMER_0,11111);

			timer_enable_intr(TIMER_GROUP_1, TIMER_1);

			timer_isr_callback_add(TIMER_GROUP_1, TIMER_1, RX_ISR, 0, 0);

			  ESP_ERROR_CHECK(timer_set_counter_value(TIMER_GROUP_1, TIMER_1, 0));
			  ESP_ERROR_CHECK(timer_start(TIMER_GROUP_1, TIMER_1));
			  char received = (char)0;
			  bool start_bit = false;
			  uint8_t c = 0;
			  while(1)
			  {
				  if(change)
				  {
					  if(bit_received)
					  {
						if(start_bit)
						{
							if(c<8)
							{
							received<<=1;
							received|=(bit_received&1);
							c++;
							}
							else
							{
								if(bit_received&1)
								{
									message_buffer[message_head] = received;
									if(message_head == 49)
									{
										for(int i =0;i<49;i++) message_buffer[i]=message_buffer[i+1];
									}
									else
									message_head++;
								}
								start_bit = false;
								c=0;
							}
						}
						else
						{
							if(bit_received&1)
							{
								start_bit = true;
							}
						}
					  }
					  change = false;
				  }
			  }

}

void startup(void)
{
	afsk_init();
	adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten(PRESS,ADC_ATTEN_DB_6); //Pressure


	adc2_config_channel_atten(TEMP,ADC_ATTEN_DB_0);

	adc1_config_channel_atten(AF,ADC_ATTEN_DB_6);

	uart_config_t uart_config = {
	        .baud_rate = 9600,
	        .data_bits = UART_DATA_8_BITS,
	        .parity    = UART_PARITY_DISABLE,
	        .stop_bits = UART_STOP_BITS_1,
	        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	        .source_clk = UART_SCLK_APB,
	    };
	    int intr_alloc_flags = 0;

	    ESP_ERROR_CHECK(uart_driver_install(0, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
	    ESP_ERROR_CHECK(uart_param_config(0, &uart_config));
	    ESP_ERROR_CHECK(uart_set_pin(0, 1 , 3 , UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

	    timer_config_t timer_config = {
	    	        .divider = 2000,
	    	        .counter_dir = TIMER_COUNT_UP,
	    	        .counter_en = TIMER_PAUSE,
	    	        .alarm_en = TIMER_ALARM_EN,
	    	        .auto_reload = true,
	    	    };

		timer_init(TIMER_GROUP_1, TIMER_0, &timer_config);
		timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0);
		timer_set_alarm_value(TIMER_GROUP_1, TIMER_0,40000);

		timer_enable_intr(TIMER_GROUP_1, TIMER_0);

		timer_isr_callback_add(TIMER_GROUP_1, TIMER_0, TX_ISR, 0, 0);

		  ESP_ERROR_CHECK(timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0));
		  ESP_ERROR_CHECK(timer_start(TIMER_GROUP_1, TIMER_0));
}

char str[100];
char str2[100];
char pretoken[100];
char t[100];



static void echo_task(void *arg)
{
    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    bool previous = false;

    pretoken[0] = '\0';

    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(0, data, BUF_SIZE, 100 / portTICK_RATE_MS);
        data[len] = (uint8_t)'+';
        // Write data back to the UART

        if(!len && !previous)
        {
        	data[0] = (uint8_t)'=';
        	data[1] = (uint8_t)'\n';
        	len = 2;
        	previous = true;
        }
        else if(len)
        {
        	data[len] = 0;
        	char* token = strtok((char*)data,"\n");
        	while(token!=NULL)
        	{
        	if(token[0]!='$')
        	{
        		sprintf(t,"%s%s",pretoken,token);
        	}
        	else
        	{
        		sprintf(t,"%s",token);
        	}
        	strcpy(pretoken,token);
        	if(!strncmp(t,"$GNGG",5))
        	{

        		strcpy(str2,t);
        	}

        	      token = strtok(NULL,"\n");
        }
        previous = false;
    }}}


environment env;

void app_main(void)
{

	s_timer_queue = xQueueCreate(10, sizeof(bool));

	startup();

	xTaskCreate(echo_task, "uart_echo_task", 2048, NULL, 10, NULL);

    float x,y;


	gpio_set_level(TX_SELECT, 1);
	bool tx;
    while(1)
    {


    	xQueueReceive(s_timer_queue, &tx, portMAX_DELAY);

    	getTempV(env.tempV);

    	x = VREF*(env.tempV/MAX);

    	env.tempC = (x-480.25)/6.25;

    	getPressV(env.pressV);

    	y = (env.pressV/MAX)*2194;
    	env.pressKpa = (y/5000 +  0.00842)/ 0.002421;


    	sprintf(str,"T:%f\tP:%f\n",env.tempC,env.pressKpa);
    	//printf("%sV:%f\tV:%f\n",str,x,y);
    	//printf("%s",str2);
    	printf("%f,",env.tempC);

      ax25_send_header(addresses, sizeof(addresses)/sizeof(struct s_address));
      ax25_send_string(str);
      ax25_send_footer();
      ax25_flush_frame();
      while (afsk_flush()) {

        }
      ax25_send_header(addresses, sizeof(addresses)/sizeof(struct s_address));
            ax25_send_string(str2);
            ax25_send_footer();
            ax25_flush_frame();
            while (afsk_flush()) {

              }
      vTaskDelay(2000 / portTICK_PERIOD_MS);
      tx = false;
      }

}

