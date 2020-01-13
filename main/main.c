#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "driver/periph_ctrl.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "tcpip_adapter.h"
#include "soc/mcpwm_periph.h"


typedef enum  {
	MOVING_DOWN = 1, MOVING_UP, RESTING_UP, RESTING_DOWN, HOMING, DROPOUT, STOPPED, UNDEFINED
} State;

#define PWM_OUTPUT_1        32  //Set GPIO 8 as PWM0A
#define PWM_OUTPUT_2        33  //Set GPIO 9 as PWM0B

#define BUTTON_UP			21
#define BUTTON_DOWN			22

#define GPIO_DIRECTION_1    26 //Set GPIO 11 as direction control
#define GPIO_DIRECTION_2    27 //Set GPIO 12 as direction control

#define ENDSTOP_1           16 //GPIO for motor 1 end stop
#define ENDSTOP_2           17 //GPIO for motor 2 end stop

#define PCNT_INPUT_SIG_1    34 // Pulse Input GPIO
#define PCNT_INPUT_SIG_2    35 // Pulse Input GPIO

#define DIRECTION_MOTOR_1   27  //Direction 1
#define DIRECTION_MOTOR_2   26  //Direction 2
#define RELAY_230V          25  //230V Enable
#define RELAY_SPARE         4   //Spare enable (not used)

#define GPIO_OUTPUT_PIN_SEL ((1ULL<<DIRECTION_MOTOR_1) | (1ULL<<DIRECTION_MOTOR_2) | (1ULL<<RELAY_230V) | (1ULL<<RELAY_SPARE))
#define GPIO_INPUT_PIN_SEL  ((1ULL<<ENDSTOP_1) | (1ULL<<ENDSTOP_2) | (1ULL<<BUTTON_UP) | (1ULL<<BUTTON_DOWN))

#define MOTOR_1 0x01
#define MOTOR_2 0x02

#define HOMING_SPEED 10
#define DROPOUT_SPEED 20
#define ADJUST_SPEED 20

#define NUM_ACCEL_STEPS 400
#define TOTAL_MOVE_COUNT 5000
//#define TOTAL_MOVE_COUNT 1000
#define MOVE_FRACTION TOTAL_MOVE_COUNT/(NUM_ACCEL_STEPS + 1)
#define MOVE_FRACTION_FLOAT (float)TOTAL_MOVE_COUNT/((float)NUM_ACCEL_STEPS + 1.0f)

#define EXAMPLE_ESP_WIFI_SSID      	"NET"
#define EXAMPLE_ESP_WIFI_PASS      	"NETtilfolket!"
#define EXAMPLE_ESP_MAXIMUM_RETRY  	10
#define PORT 						100

static EventGroupHandle_t s_wifi_event_group;
static const char *TAG = "Micro Elevator";
const int WIFI_CONNECTED_BIT = BIT0;
static int s_retry_num = 0;

//Handle for non volatile memory
nvs_handle_t flash_handle;

xQueueHandle ENCODER_evt_queue = NULL; // A queue to handle pulse counter events
xQueueHandle ENDSTOP_evt_queue = NULL;
xQueueHandle BUTTON_evt_queue = NULL;
xQueueHandle NET_evt_queue = NULL;

pcnt_isr_handle_t user_isr_handle_1 = NULL; //user's ISR service handle
pcnt_isr_handle_t user_isr_handle_2 = NULL; //user's ISR service handle

ledc_channel_config_t ledc_channel1;
ledc_channel_config_t ledc_channel2;

/* A sample structure to pass events from the PCNT
 * interrupt handler to the main program.*/
typedef struct {
	int unit;  // the PCNT unit that originated an interrupt
	uint32_t status; // information on the event type that caused the interrupt
} ENCODER_evt_t;

ENCODER_evt_t ENCODER_evt;
uint32_t ENDSTOP_evt;
uint32_t BUTTON_evt;
uint32_t NET_evt;

uint32_t Debounce_BUTTON_UP = 0;
uint32_t Debounce_BUTTON_DOWN = 0;

typedef struct {
	float motor_pwr_scale[3];
	int position_index_motor[3];
	int positions[NUM_ACCEL_STEPS];
	int speeds_down[NUM_ACCEL_STEPS];
	int speeds_up[NUM_ACCEL_STEPS];
} Accel_info;

Accel_info accel_info = {
		{ 0.0f, 0.30f, 1.0f },
		{ 0, 0, 0 },
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, 			//Will be filled up in the encoder setup
		{ 10, 20, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 20, 10 },
		{ 10, 20, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 20, 10 }
};

/*
Accel_info accel_info = {
		{ 0.0f, 1.0f, 0.3f },
		{ 0, 0, 0 },
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, 			//Will be filled up in the encoder setup
		{ 20, 20, 30, 40, 50, 60, 70, 80, 90, 100, 90, 80, 70, 60, 50, 40, 30, 20, 10, 0 },
		{ 20, 20, 30, 40, 50, 60, 70, 80, 90, 100, 90, 80, 70, 60, 50, 40, 30, 20, 10, 0 }
};
*/

size_t Accel_info_size = sizeof(Accel_info);

int16_t motorPosition[] = { 0, 0, 0 };

State ELEVATOR_STATE = STOPPED;

static uint32_t millis() {
    return esp_timer_get_time() / 1000;
}

void FillAccelSteps(){
	for(int i = 0; i < NUM_ACCEL_STEPS; i++){
		accel_info.speeds_down[i] = 100;
		accel_info.speeds_up[i] = 100;
		accel_info.positions[i] = MOVE_FRACTION;
	}

	accel_info.speeds_down[0] = 20;
	accel_info.speeds_down[1] = 60;

	accel_info.speeds_up[0] = 20;
	accel_info.speeds_up[1] = 60;

	accel_info.speeds_down[NUM_ACCEL_STEPS - 1] = 20;
	accel_info.speeds_down[NUM_ACCEL_STEPS - 2] = 60;

	accel_info.speeds_up[NUM_ACCEL_STEPS - 1] = 20;
	accel_info.speeds_up[NUM_ACCEL_STEPS - 2] = 60;
}

int OtherM(int motor){
	if( motor == MOTOR_1) return MOTOR_2;
	if( motor == MOTOR_2) return MOTOR_1;
	return 0;
}

bool BothEndstopsActivated(){
	if( gpio_get_level(ENDSTOP_1) && gpio_get_level(ENDSTOP_2) ){
		return true;
	}
	return false;
}

bool BothButtonsActivated(){
	if( gpio_get_level(BUTTON_DOWN) && gpio_get_level(BUTTON_DOWN) ){
		return true;
	}
	return false;
}

static void IRAM_ATTR ENDSTOP_isr_handler(void *arg) {
	uint32_t gpio_num = (uint32_t) arg;
	if (gpio_num == ENDSTOP_1) {
		ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
	}
	else if (gpio_num == ENDSTOP_2) {
		ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
	}
	xQueueSendFromISR(ENDSTOP_evt_queue, &gpio_num, NULL);
}

static void IRAM_ATTR BUTTON_isr_handler(void *arg) {

	uint32_t button = (uint32_t) arg;

	uint32_t ms = millis();

	if( (button == BUTTON_UP && ms - Debounce_BUTTON_UP > 50) || (button == BUTTON_DOWN && ms - Debounce_BUTTON_DOWN > 50)){

		if( ELEVATOR_STATE == HOMING || ELEVATOR_STATE == DROPOUT || BothButtonsActivated() ){	//Button presses kill motors
			ledc_stop(LEDC_LOW_SPEED_MODE, MOTOR_1, 0);
			ledc_stop(LEDC_LOW_SPEED_MODE, MOTOR_2, 0);
			ELEVATOR_STATE = STOPPED;
		}

		if( button == BUTTON_UP )   Debounce_BUTTON_UP = ms;
		if( button == BUTTON_DOWN ) Debounce_BUTTON_DOWN = ms;

		uint32_t gpio_num = (uint32_t) arg;
		xQueueSendFromISR(BUTTON_evt_queue, &gpio_num, NULL);
	}
}

static void IRAM_ATTR ENCODER_intr_handler(void *arg) {
	uint32_t intr_status = PCNT.int_st.val;
	int i;
	ENCODER_evt_t evt;
	portBASE_TYPE HPTaskAwoken = pdFALSE;

	for (i = 0; i < PCNT_UNIT_MAX; i++) {
		if (intr_status & (BIT(i))) {
			evt.unit = i;

			if( ELEVATOR_STATE == HOMING || ELEVATOR_STATE == DROPOUT){		//Sync motors when homing
				accel_info.position_index_motor[evt.unit]++;
			}
			else if( ELEVATOR_STATE == STOPPED ){							//Stop motor when adjusting position when STOPPED
				ledc_stop(LEDC_LOW_SPEED_MODE, evt.unit, 0);
			}
			else{
				//Kill the engines at the end of the line
				if (accel_info.position_index_motor[evt.unit] == NUM_ACCEL_STEPS) {
					//ledc_stop(LEDC_LOW_SPEED_MODE, evt.unit, 0);
					ledc_stop(LEDC_LOW_SPEED_MODE, evt.unit, 0);
				}
				else{
					accel_info.position_index_motor[evt.unit]++;
				}
			}

			/* Save the PCNT event type that caused an interrupt
			 to pass it to the main program */
			evt.status = PCNT.status_unit[i].val;
			PCNT.int_clr.val = BIT(i);
			xQueueSendFromISR(ENCODER_evt_queue, &evt, &HPTaskAwoken);
			if (HPTaskAwoken == pdTRUE) {
				portYIELD_FROM_ISR();
			}
		}
	}
}

static void NET_event_handler(void* arg, esp_event_base_t event_base,  int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:%s", ip4addr_ntoa(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

    }
}

void SetSpeed(int motor, int speed) {
	if (motor == MOTOR_1) {
		ledc_channel1.duty = accel_info.motor_pwr_scale[ MOTOR_1 ] * (1024 * speed) / 100;
		printf("SetSpeed motor: %i speed: %i\n", motor, ledc_channel1.duty);
		ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, ledc_channel1.duty);
		ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
	}
	else{
		ledc_channel2.duty = accel_info.motor_pwr_scale[ MOTOR_2 ] * (1024 * speed) / 100;
		printf("SetSpeed motor: %i speed: %i\n", motor, ledc_channel2.duty);
		ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, ledc_channel2.duty);
		ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
	}
}

void SetDirection(State direction) {

	printf("SetDirection: ");

	SetSpeed(MOTOR_1, 0);
	SetSpeed(MOTOR_2, 0);

	vTaskDelay(500 / portTICK_PERIOD_MS);

	accel_info.position_index_motor[MOTOR_1] = 0;
	accel_info.position_index_motor[MOTOR_2] = 0;

	if (direction == MOVING_UP) {
		printf("Moving UP\n");
		gpio_set_level(DIRECTION_MOTOR_1, 0);
		gpio_set_level(DIRECTION_MOTOR_2, 0);
	} else {
		printf("Moving DOWN\n");
		gpio_set_level(DIRECTION_MOTOR_1, 1);
		gpio_set_level(DIRECTION_MOTOR_2, 1);
	}
}

void WriteNVS(bool close) {
	// Write
	printf("Updating restart counter in NVS ... ");

	esp_err_t err;

	err = nvs_set_blob(flash_handle, "velocity_curves", &accel_info,
			Accel_info_size);
	printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

	// Commit written value.
	// After setting any values, nvs_commit() must be called to ensure changes are written
	// to flash storage. Implementations may write to storage at other times,
	// but this is not guaranteed.
	printf("Committing updates in NVS ... ");
	err = nvs_commit(flash_handle);
	printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

	if (close) {
		// Close
		nvs_close(flash_handle);
	}
}

static void tcp_server_task()
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    printf("tcp_server_task");

    while (1) {

        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);


        int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (listen_sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket bound, port %d", PORT);

        err = listen(listen_sock, 1);
        if (err != 0) {
            ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
        uint addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket accepted");

        while (1) {
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recv failed: errno %d", errno);
                break;
            }
            // Connection closed
            else if (len == 0) {
                ESP_LOGI(TAG, "Connection closed");
                break;
            }
            // Data received
            else {
                // Get the sender's ip address as string
                inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);

                uint32_t command;

                if( rx_buffer[0] == 'H' ){
                	ESP_LOGI(TAG, "Received HOME Command");
                	command = 'H';
                }
                else if( rx_buffer[0] == 'U' ){
                	ESP_LOGI(TAG, "Received UP Command");
                	command = 'U';
                }
                else if( rx_buffer[0] == 'D' ){
                	ESP_LOGI(TAG, "Received DOWN Command");
                	command = 'D';
                }
                else if( rx_buffer[0] == 'E' ){
					ESP_LOGI(TAG, "Received DROPOUT Command");
					command = 'E';
				}
                else if( rx_buffer[0] == 'S' ){
					ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
					ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
					ELEVATOR_STATE = STOPPED;
					ESP_LOGI(TAG, "Received STOP Command -> ELEVATOR_STATE = STOPPED");
					command = 'S';
				}
                else{
                	ESP_LOGI(TAG, "Received --=UNKNOWN=- Command");
                }
                xQueueSendFromISR(NET_evt_queue, &command, NULL);

                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);

                int err = send(sock, rx_buffer, len, 0);
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

static void SetupMotor_PWM() {
	printf("SetupMotor_PWM\n");

	esp_err_t error;

	// Prepare and then apply the LEDC PWM timer configuration
	ledc_timer_config_t ledc_timer1;
	ledc_timer1.speed_mode = LEDC_LOW_SPEED_MODE;
	ledc_timer1.timer_num = LEDC_TIMER_1;
	ledc_timer1.duty_resolution = LEDC_TIMER_10_BIT;
	ledc_timer1.freq_hz = 4000;
	ledc_timer1.clk_cfg = LEDC_AUTO_CLK;
	error = ledc_timer_config(&ledc_timer1);

	if (error != ESP_OK)
		printf("   Error 0x%.8X\n", error);

	ledc_timer_config_t ledc_timer2;
	ledc_timer2.speed_mode = LEDC_LOW_SPEED_MODE;
	ledc_timer2.timer_num = LEDC_TIMER_2;
	ledc_timer2.duty_resolution = LEDC_TIMER_10_BIT;
	ledc_timer2.freq_hz = 4000;
	ledc_timer2.clk_cfg = LEDC_AUTO_CLK;
	error = ledc_timer_config(&ledc_timer2);

	if (error != ESP_OK)
		printf("   Error 0x%.8X\n", error);

	// Prepare and then apply the LEDC PWM channel configuration
	ledc_channel1.speed_mode = LEDC_LOW_SPEED_MODE;
	ledc_channel1.channel = LEDC_CHANNEL_1;
	ledc_channel1.timer_sel = LEDC_TIMER_1;
	ledc_channel1.intr_type = LEDC_INTR_DISABLE;
	ledc_channel1.gpio_num = PWM_OUTPUT_1;
	ledc_channel1.duty = 0; // set duty at about 10%
	ledc_channel1.hpoint = 0;
	error = ledc_channel_config(&ledc_channel1);

	if (error != ESP_OK)
		printf("   Error 0x%.8X\n", error);

	ledc_channel2.speed_mode = LEDC_LOW_SPEED_MODE;
	ledc_channel2.channel = LEDC_CHANNEL_2;
	ledc_channel2.timer_sel = LEDC_TIMER_2;
	ledc_channel2.intr_type = LEDC_INTR_DISABLE;
	ledc_channel2.gpio_num = PWM_OUTPUT_2;
	ledc_channel2.duty = 0; // set duty at about 10%
	ledc_channel2.hpoint = 0;
	error = ledc_channel_config(&ledc_channel2);

	FillAccelSteps();

	if (error != ESP_OK)
		printf("   Error 0x%.8X\n", error);
}

static void SetupCounter() {

	printf("counter_init\n");

	pcnt_config_t pcnt_config1;

	/* Prepare configuration for the PCNT unit */
	// Set PCNT input signal and control GPIOs
	pcnt_config1.pulse_gpio_num = PCNT_INPUT_SIG_1;
	pcnt_config1.ctrl_gpio_num = -1;
	pcnt_config1.channel = PCNT_CHANNEL_0;
	pcnt_config1.unit = PCNT_UNIT_1,
	// What to do on the positive / negative edge of pulse input?
	pcnt_config1.pos_mode = PCNT_COUNT_INC;   // Count up on the positive edge
	pcnt_config1.neg_mode = PCNT_COUNT_INC; // Keep the counter value on the negative edge
	// What to do when control input is low or high?
	pcnt_config1.lctrl_mode = PCNT_MODE_KEEP; // Reverse counting direction if low
	pcnt_config1.hctrl_mode = PCNT_MODE_KEEP; // Keep the primary counter mode if high
	// Set the maximum and minimum limit values to watch
	pcnt_config1.counter_l_lim = 0;
	pcnt_config1.counter_h_lim = accel_info.positions[0];

	pcnt_config_t pcnt_config2;

	/* Prepare configuration for the PCNT unit */
	// Set PCNT input signal and control GPIOs
	pcnt_config2.pulse_gpio_num = PCNT_INPUT_SIG_2;
	pcnt_config2.ctrl_gpio_num = -1;
	pcnt_config2.channel = PCNT_CHANNEL_0;
	pcnt_config2.unit = PCNT_UNIT_2,
	// What to do on the positive / negative edge of pulse input?
	pcnt_config2.pos_mode = PCNT_COUNT_INC;   // Count up on the positive edge
	pcnt_config2.neg_mode = PCNT_COUNT_INC; // Keep the counter value on the negative edge
	// What to do when control input is low or high?
	pcnt_config2.lctrl_mode = PCNT_MODE_KEEP; // Reverse counting direction if low
	pcnt_config2.hctrl_mode = PCNT_MODE_KEEP; // Keep the primary counter mode if high
	// Set the maximum and minimum limit values to watch
	pcnt_config2.counter_l_lim = 0;
	pcnt_config2.counter_h_lim = accel_info.positions[0];

	/* Initialize PCNT unit */
	pcnt_unit_config(&pcnt_config1);
	pcnt_unit_config(&pcnt_config2);

	/* Configure and enable the input filter */
	pcnt_set_filter_value(PCNT_UNIT_1, 500);
	pcnt_filter_enable(PCNT_UNIT_1);
	pcnt_set_filter_value(PCNT_UNIT_2, 500);
	pcnt_filter_enable(PCNT_UNIT_2);

	/* Enable events on zero, maximum and minimum limit values */
	pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_H_LIM);
	pcnt_event_enable(PCNT_UNIT_2, PCNT_EVT_H_LIM);

	/* Initialize PCNT's counter */
	pcnt_counter_pause(PCNT_UNIT_1);
	pcnt_counter_clear(PCNT_UNIT_1);
	pcnt_counter_pause(PCNT_UNIT_2);
	pcnt_counter_clear(PCNT_UNIT_2);

	/* Register ISR handler and enable interrupts for PCNT unit */
	pcnt_isr_register(ENCODER_intr_handler, NULL, 0, &user_isr_handle_1);
	pcnt_intr_enable(PCNT_UNIT_1);
	pcnt_isr_register(ENCODER_intr_handler, NULL, 0, &user_isr_handle_2);
	pcnt_intr_enable(PCNT_UNIT_2);

	/* Everything is set up, now go to counting */
	pcnt_counter_resume(PCNT_UNIT_1);
	pcnt_counter_resume(PCNT_UNIT_2);

	ENCODER_evt_queue = xQueueCreate(10, sizeof(ENCODER_evt_t));

}

void SetupNVS() {
	// Initialize NVS
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		// NVS partition was truncated and needs to be erased
		// Retry nvs_flash_init
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	ESP_ERROR_CHECK(err);

	// Open
	printf("\n");
	printf("Opening Non-Volatile Storage (NVS) handle... ");

	err = nvs_open("storage", NVS_READWRITE, &flash_handle);
	if (err != ESP_OK) {
		printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
	} else {
		printf("Done\n");

		// Read
		printf("Reading restart counter from NVS ... ");
		err = nvs_get_blob(flash_handle, "velocity_curves", (void*) &accel_info,
				(size_t*) &Accel_info_size);

		switch (err) {
		case ESP_OK:
			printf("Done\n");
			break;
		case ESP_ERR_NVS_NOT_FOUND:
			printf("The value is not initialized yet!\n");
			break;
		default:
			printf("Error (%s) reading!\n", esp_err_to_name(err));
		}
	}
}

void SetupRelays() {
	gpio_config_t io_conf;
	//disable interrupt
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	//set as output mode
	io_conf.mode = GPIO_MODE_OUTPUT;
	//bit mask of the pins that you want to set,e.g.GPIO18/19
	io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
	//disable pull-down mode
	io_conf.pull_down_en = 0;
	//disable pull-up mode
	io_conf.pull_up_en = 0;
	//configure GPIO with the given settings
	gpio_config(&io_conf);
}

void SetupEndpointsAndButtons() {
	printf("SetupEndpoints\n");
	gpio_config_t io_conf;
	//interrupt of rising edge
	io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
	//bit mask of the pins, use GPIO4/5 here
	io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
	//set as input mode
	io_conf.mode = GPIO_MODE_INPUT;
	//enable pull-up mode
	io_conf.pull_up_en = 0;
	io_conf.pull_down_en = true;
	gpio_config(&io_conf);

	gpio_set_intr_type(ENDSTOP_1,   GPIO_INTR_POSEDGE);
	gpio_set_intr_type(ENDSTOP_2,   GPIO_INTR_POSEDGE);
	gpio_set_intr_type(BUTTON_UP,   GPIO_INTR_POSEDGE);
	gpio_set_intr_type(BUTTON_DOWN, GPIO_INTR_POSEDGE);

	ENDSTOP_evt_queue = xQueueCreate(10, sizeof(uint32_t));

	//install gpio isr service
	gpio_install_isr_service(0);

	gpio_isr_handler_add(ENDSTOP_1, ENDSTOP_isr_handler, (void*) ENDSTOP_1);
	gpio_isr_handler_add(ENDSTOP_2, ENDSTOP_isr_handler, (void*) ENDSTOP_2);

	BUTTON_evt_queue = xQueueCreate(10, sizeof(uint32_t));
	gpio_isr_handler_add(BUTTON_UP, BUTTON_isr_handler, (void*) BUTTON_UP);
	gpio_isr_handler_add(BUTTON_DOWN, BUTTON_isr_handler, (void*) BUTTON_DOWN);

}

void SetupNetwork(){	//Needs SetupNVS()

	s_wifi_event_group = xEventGroupCreate();

	tcpip_adapter_init();

	ESP_ERROR_CHECK(esp_event_loop_create_default());

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &NET_event_handler, NULL));
	ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &NET_event_handler, NULL));

	wifi_config_t wifi_config = {
		.sta = {
			.ssid = EXAMPLE_ESP_WIFI_SSID,
			.password = EXAMPLE_ESP_WIFI_PASS
		},
	};

	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
	ESP_ERROR_CHECK(esp_wifi_start() );

	ESP_LOGI(TAG, "connect to ap SSID:%s password:%s", EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
	xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
	ESP_LOGI(TAG, "SetupNetwork finished.");

	NET_evt_queue = xQueueCreate(10, sizeof(uint32_t));

}

void SetupDebounce(){
	Debounce_BUTTON_UP = millis();
	Debounce_BUTTON_DOWN = millis();
}

/* TESTS
void TEST_Endstops(){

	SetupEndpointsAndButtons();

	while(1){
		if (xQueueReceive(ENDSTOP_evt_queue, &ENDSTOP_evt, 200 / portTICK_PERIOD_MS) == pdTRUE) {
			printf("Endstop triggered: %i\n", ENDSTOP_evt);
		}
		else{
			printf(".");
		}
	}
}
void TEST_RELAYS() {
	SetupRelays();

	printf("----BOSS---- Starting!\n");

	while (1) {
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		printf("Relay low\n");
		gpio_set_level(DIRECTION_MOTOR_1, 0);
		gpio_set_level(DIRECTION_MOTOR_2, 0);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		printf("Relay high\n");
		gpio_set_level(DIRECTION_MOTOR_1, 1);
		gpio_set_level(DIRECTION_MOTOR_2, 1);
	}
}
void TEST_PWM() {
	SetupRelays();

	SetupMotor_PWM();

	while (1) {
		printf("Speed 20\n");
		SetSpeed(1, 20);
		vTaskDelay(1000 / portTICK_PERIOD_MS);

		printf("Speed 100\n");
		SetSpeed(1, 100);
		vTaskDelay(1000 / portTICK_PERIOD_MS);

		printf("Speed 0\n");
		SetSpeed(1, 0);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}
void TEST_TURN() {
	SetupRelays();

	SetupMotor_PWM();

	SetDirection(MOVING_DOWN);

	while (1) {

		printf("Speed 100\n");
		SetSpeed(1, 100);
		SetSpeed(2, 100);
		vTaskDelay(1000 / portTICK_PERIOD_MS);

		printf("Speed 0\n");
		SetSpeed(1, 0);
		SetSpeed(2, 0);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		SetDirection(MOVING_UP);

		printf("Speed 100\n");
		SetSpeed(1, 100);
		SetSpeed(2, 100);
		vTaskDelay(1000 / portTICK_PERIOD_MS);

		printf("Speed 0\n");
		SetSpeed(1, 0);
		SetSpeed(2, 0);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		SetDirection(MOVING_DOWN);
	}
}
void TEST_COUNT_EVENT() {

	SetupRelays();

	// Initialize LEDC to generate sample pulse signal
	SetupMotor_PWM();

	// Initialize PCNT event queue and PCNT functions
	SetupCounter();

	SetDirection(MOVING_DOWN);

	SetSpeed(1, 100);

	while (1) {
		if (xQueueReceive(ENCODER_evt_queue, &ENCODER_evt_queue, 200 / portTICK_PERIOD_MS) == pdTRUE) {
			if (ENCODER_evt.status & PCNT_STATUS_H_LIM_M) {
				printf("Hit PCNT_STATUS_H_LIM_M\n");
			}
		} else {
			pcnt_get_counter_value(PCNT_UNIT_1, &(motorPosition[1]));
			pcnt_get_counter_value(PCNT_UNIT_2, &(motorPosition[2]));
			printf("Current counter values : %d , %d\n", motorPosition[1],
					motorPosition[2]);
		}
	}
}
void TEST_BUTTONS(){

	printf("Testing buttons");

	SetupEndpointsAndButtons();
	while(1){
		if (xQueueReceive(BUTTON_evt_queue, &BUTTON_evt, 100 / portTICK_PERIOD_MS) == pdTRUE) {
			printf("Button %i presssed", BUTTON_evt);
		}
	}
}
void TEST_BUTTON_MOVE(){

	SetupRelays();

	SetupMotor_PWM();

	SetupCounter();

	SetupEndpointsAndButtons();

	printf("TEST_BUTTON_MOVE\n");

	while(1){
		if( ELEVATOR_STATE == RESTING_DOWN ){
			if (xQueueReceive(BUTTON_evt_queue, &BUTTON_evt, 100 / portTICK_PERIOD_MS) == pdTRUE) {
				if( BUTTON_evt == BUTTON_UP){
					printf("ELEVATOR_STATE == RESTING_DOWN -> MOVING UP\n");
					SetDirection(MOVING_UP);
					accel_info.position_index_motor[MOTOR_1] = 0;
					accel_info.position_index_motor[MOTOR_2] = 0;
					SetSpeed(MOTOR_1, accel_info.speeds_up[0]);
					SetSpeed(MOTOR_2, accel_info.speeds_up[0]);
					pcnt_set_event_value(MOTOR_1, PCNT_EVT_H_LIM, accel_info.positions[0]);
					pcnt_set_event_value(MOTOR_2, PCNT_EVT_H_LIM, accel_info.positions[0]);
				}
				else{
					printf("ELEVATOR_STATE == RESTING_DOWN - UNKNOWN EVENT: %i\n", BUTTON_evt);
				}
			}
		}
		else if(ELEVATOR_STATE == MOVING_UP){
			if (xQueueReceive(ENCODER_evt_queue, &ENCODER_evt, POSITION_POOL_TIME_MS) == pdTRUE) {
				pcnt_get_counter_value(ENCODER_evt.unit, &(motorPosition[ENCODER_evt.unit]));
				printf("	MOVING UP - Motor %i Position %i index %i\n", ENCODER_evt.unit,  motorPosition[ENCODER_evt.unit], accel_info.position_index_motor[ENCODER_evt.unit]);
				if (ENCODER_evt.status & PCNT_STATUS_H_LIM_M) {
					//SetSpeed(ENCODER_evt.unit, accel_info.speeds_down[ accel_info.position_index_motor[ENCODER_evt.unit] ]);

					pcnt_get_counter_value(MOTOR_1, &(motorPosition[MOTOR_1]));
					pcnt_get_counter_value(MOTOR_2, &(motorPosition[MOTOR_2]));

					float scale = 1.0f;

					if( ENCODER_evt.unit == MOTOR_1){
						//If unit 1 is ahead
						if( accel_info.position_index_motor[ MOTOR_1 ] > accel_info.position_index_motor[ MOTOR_2 ] && motorPosition[ MOTOR_2 ] < MOVE_FRACTION - 10){
							scale = (float)motorPosition[MOTOR_2]/MOVE_FRACTION_FLOAT;
						}
					}
					else {
						//If unit 1 is ahead
						if( accel_info.position_index_motor[ MOTOR_2 ] > accel_info.position_index_motor[ MOTOR_1 ] && motorPosition[ MOTOR_1 ] < MOVE_FRACTION - 10){
							scale = (float)motorPosition[MOTOR_2]/MOVE_FRACTION_FLOAT;
						}
					}
					SetSpeed(ENCODER_evt.unit, scale*accel_info.speeds_down[accel_info.position_index_motor[ MOTOR_1 ]]);
					printf("	MOVING UP - Motor %i - Scale: %f\n", ENCODER_evt.unit, scale);
				}
				else{
					printf("ELEVATOR_STATE == MOVING_UP - UNKNOWN EVENT: %i\n", ENCODER_evt.unit);
				}
			}
			if( xQueueReceive(ENDSTOP_evt_queue, &ENDSTOP_evt, 1 / portTICK_PERIOD_MS) == pdTRUE){
				printf("	MOVING UP - Motor %i reached end stop\n", ENCODER_evt.unit);
			}
		}
		else if(ELEVATOR_STATE == RESTING_UP){
			if (xQueueReceive(BUTTON_evt_queue, &BUTTON_evt, 100 / portTICK_PERIOD_MS) == pdTRUE) {
				if( BUTTON_evt == BUTTON_DOWN){
					printf("ELEVATOR_STATE == RESTING_UP -> MOVING DOWN\n");
					SetDirection(MOVING_DOWN);
					accel_info.position_index_motor[MOTOR_1] = 0;
					accel_info.position_index_motor[MOTOR_2] = 0;
					SetSpeed(MOTOR_1, accel_info.speeds_down[ accel_info.position_index_motor[MOTOR_1] ]);
					SetSpeed(MOTOR_2, accel_info.speeds_down[ accel_info.position_index_motor[MOTOR_2] ]);
					pcnt_counter_clear(MOTOR_1);
					pcnt_counter_clear(MOTOR_2);
					pcnt_set_event_value(MOTOR_1, PCNT_EVT_H_LIM, accel_info.positions[ accel_info.position_index_motor[MOTOR_1] ]);
					pcnt_set_event_value(MOTOR_2, PCNT_EVT_H_LIM, accel_info.positions[ accel_info.position_index_motor[MOTOR_2] ]);
					printf("	Motor 1: Index %i - Position %i - Speed %i \n", accel_info.position_index_motor[MOTOR_1], accel_info.positions[ accel_info.position_index_motor[MOTOR_1] ], accel_info.speeds_down[ accel_info.position_index_motor[MOTOR_1] ]);
					printf("	Motor 2: Index %i - Position %i - Speed %i \n", accel_info.position_index_motor[MOTOR_2], accel_info.positions[ accel_info.position_index_motor[MOTOR_2] ], accel_info.speeds_down[ accel_info.position_index_motor[MOTOR_2] ]);
				}
				else{
					printf("ELEVATOR_STATE == RESTING_UP - UNKNOWN EVENT: %i\n", BUTTON_evt);
				}
			}
		}
		else if(ELEVATOR_STATE == MOVING_DOWN){
			if (xQueueReceive(ENCODER_evt_queue, &ENCODER_evt, POSITION_POOL_TIME_MS) == pdTRUE) {
				if (ENCODER_evt.status & PCNT_STATUS_H_LIM_M) {

					pcnt_get_counter_value(MOTOR_1, &(motorPosition[MOTOR_1]));
					pcnt_get_counter_value(MOTOR_2, &(motorPosition[MOTOR_2]));

					printf("M1 Idx: %i Count: %i\n", accel_info.position_index_motor[ MOTOR_1 ], motorPosition[MOTOR_1]);
					printf("M2 Idx: %i Count: %i\n", accel_info.position_index_motor[ MOTOR_2 ], motorPosition[MOTOR_2]);

					if( ENCODER_evt.unit == MOTOR_1){
						if( accel_info.position_index_motor[ MOTOR_1 ] > accel_info.position_index_motor[ MOTOR_2 ]){
							SetSpeed(MOTOR_1, 0);
						}
						else{
							SetSpeed(MOTOR_1, accel_info.speeds_down[accel_info.position_index_motor[ MOTOR_1 ]]);
							SetSpeed(MOTOR_2, accel_info.speeds_down[accel_info.position_index_motor[ MOTOR_2 ]]);
						}
					}
					else {
						if( accel_info.position_index_motor[ MOTOR_2 ] > accel_info.position_index_motor[ MOTOR_1 ]){
							SetSpeed(MOTOR_2, 0);
						}
						else{
							SetSpeed(MOTOR_1, accel_info.speeds_down[accel_info.position_index_motor[ MOTOR_1 ]]);
							SetSpeed(MOTOR_2, accel_info.speeds_down[accel_info.position_index_motor[ MOTOR_2 ]]);
						}
					}
				}
				else{
					printf("ELEVATOR_STATE == MOVING_DOWN - UNKNOWN EVENT: %i\n", ENCODER_evt.unit);
				}
			}
			if( xQueueReceive(ENDSTOP_evt_queue, &ENDSTOP_evt, 1 / portTICK_PERIOD_MS) == pdTRUE){
				printf("	MOVING DOWN - Motor %i reached end stop\n", ENCODER_evt.unit);
			}
		}
		else{
			printf("ERROR UNDEFINED STATE\n");
		}
	}
}
void TEST(){
	while(1){
		printf("High\n");
		gpio_set_level(PWM_OUTPUT_1, 1);
		gpio_set_level(PWM_OUTPUT_2, 1);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		printf("Low\n");
		gpio_set_level(PWM_OUTPUT_1, 0);
		gpio_set_level(PWM_OUTPUT_2, 0);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}
void TEST_SPEEDS(){

	SetupRelays();

	SetupMotor_PWM();

	SetupCounter();

	SetupEndpointsAndButtons();

	printf("TEST_SPEEDS\n");

	bool up = true;

	while(1){

		if( up ){
			SetDirection(MOVING_DOWN);
		}
		else{
			SetDirection(MOVING_UP);
		}

		SetSpeed(MOTOR_1, accel_info.speeds_down[accel_info.position_index_motor[ MOTOR_1 ]]);
		SetSpeed(MOTOR_2, accel_info.speeds_down[accel_info.position_index_motor[ MOTOR_2 ]]);

		pcnt_counter_clear(MOTOR_1);
		pcnt_counter_clear(MOTOR_2);

		up = !up;

		while(accel_info.position_index_motor[ MOTOR_1 ] < NUM_ACCEL_STEPS && accel_info.position_index_motor[ MOTOR_2 ] < NUM_ACCEL_STEPS){

			if (xQueueReceive(ENCODER_evt_queue, &ENCODER_evt, 500 / portTICK_PERIOD_MS) == pdTRUE) {
				if (ENCODER_evt.status & PCNT_STATUS_H_LIM_M) {

					printf("M1 Idx: %i Count: %i\n", accel_info.position_index_motor[ MOTOR_1 ], motorPosition[MOTOR_1]);
					printf("M2 Idx: %i Count: %i\n", accel_info.position_index_motor[ MOTOR_2 ], motorPosition[MOTOR_2]);

					if( ENCODER_evt.unit == MOTOR_1){
						printf("	M1\n");
						if( accel_info.position_index_motor[ MOTOR_1 ] > accel_info.position_index_motor[ MOTOR_2 ]){
							SetSpeed(MOTOR_1, 0);
							printf("		Stop\n");
						}
						else{
							SetSpeed(MOTOR_1, accel_info.speeds_down[accel_info.position_index_motor[ MOTOR_1 ]]);
							SetSpeed(MOTOR_2, accel_info.speeds_down[accel_info.position_index_motor[ MOTOR_2 ]]);
						}
						pcnt_set_event_value(MOTOR_1, PCNT_EVT_H_LIM, accel_info.positions[ accel_info.position_index_motor[MOTOR_1] ]);
					}
					else {
						printf("	M2\n");
						if( accel_info.position_index_motor[ MOTOR_2 ] > accel_info.position_index_motor[ MOTOR_1 ]){
							SetSpeed(MOTOR_2, 0);
							printf("		Stop\n");
						}
						else{
							SetSpeed(MOTOR_1, accel_info.speeds_down[accel_info.position_index_motor[ MOTOR_1 ]]);
							SetSpeed(MOTOR_2, accel_info.speeds_down[accel_info.position_index_motor[ MOTOR_2 ]]);
						}
						pcnt_set_event_value(MOTOR_2, PCNT_EVT_H_LIM, accel_info.positions[ accel_info.position_index_motor[MOTOR_2] ]);
					}
				}
				else{
					printf("ELEVATOR_STATE == MOVING_DOWN - UNKNOWN EVENT: %i\n", ENCODER_evt.unit);
				}
			}
		}
	}
}
void TEST_NETWORK(){
	SetupNetwork();

	while(1){
		if (xQueueReceive(NET_evt_queue, &NET_evt, 100 / portTICK_PERIOD_MS) == pdTRUE) {
			if( NET_evt == 'U'){
				printf(" -> UP event\n");
			}
			if( NET_evt == 'D'){
				printf(" -> DOWN event\n");
			}
			if( NET_evt == 'H'){
				printf(" -> HOME event\n");
			}
			if( NET_evt == 'S'){
				printf(" -> STOP event\n");
			}
			if( NET_evt == 'E'){
				printf(" -> DROPOUT event\n");
			}
		}
	}
}
*/

void Init(){

	printf("->Init\n");

	SetupNVS();

	SetupRelays();

	SetupMotor_PWM();

	SetupCounter();

	SetupEndpointsAndButtons();

	SetupNetwork();

	FillAccelSteps();

	SetupDebounce();

	printf("<-Init\n");
}

void CheckNetState(){
	if (xQueueReceive(NET_evt_queue, &NET_evt, 0) == pdTRUE) {
		if( NET_evt == 'D' && ELEVATOR_STATE == RESTING_UP){
			ELEVATOR_STATE = MOVING_DOWN;
		}
		else if (NET_evt == 'U' && ELEVATOR_STATE == RESTING_DOWN ){
			ELEVATOR_STATE = MOVING_UP;
		}
		else if (NET_evt == 'H' && ELEVATOR_STATE != DROPOUT){
			ELEVATOR_STATE = HOMING;
		}
		else if (NET_evt == 'E' && ELEVATOR_STATE != HOMING){
			ELEVATOR_STATE = DROPOUT;
		}
		else if (NET_evt == 'S' ){
			ELEVATOR_STATE = STOPPED;
		}
		else{
			printf("ELEVATOR_STATE == RESTING_UP - UNKNOWN NET EVENT: %c\n", NET_evt);
		}
	}
}


float FindScale(){

	pcnt_get_counter_value(ENCODER_evt.unit, &(motorPosition[OtherM(ENCODER_evt.unit)]));

	if( accel_info.position_index_motor[ ENCODER_evt.unit ] > accel_info.position_index_motor[ OtherM(ENCODER_evt.unit) ] ){
		return (float)motorPosition[OtherM(ENCODER_evt.unit)]/MOVE_FRACTION_FLOAT;
	}
	return 1.0f;
}

void State_RESTING_DOWN(){
	if (xQueueReceive(BUTTON_evt_queue, &BUTTON_evt, 0) == pdTRUE && BUTTON_evt == BUTTON_UP) {
		ELEVATOR_STATE = MOVING_UP;
	}
	else{
		printf("ELEVATOR_STATE == RESTING_DOWN - UNKNOWN BUTTON EVENT: %i\n", BUTTON_evt);
	}

	CheckNetState();

	if( ELEVATOR_STATE == MOVING_UP){
		printf("ELEVATOR_STATE == RESTING_DOWN -> MOVING UP\n");
		SetDirection(MOVING_UP);
		accel_info.position_index_motor[MOTOR_1] = 0;
		accel_info.position_index_motor[MOTOR_2] = 0;
		SetSpeed(MOTOR_1, accel_info.speeds_up[ accel_info.position_index_motor[MOTOR_1] ]);
		SetSpeed(MOTOR_2, accel_info.speeds_up[ accel_info.position_index_motor[MOTOR_2] ]);
		pcnt_counter_clear(MOTOR_1);
		pcnt_counter_clear(MOTOR_2);
		pcnt_set_event_value(MOTOR_1, PCNT_EVT_H_LIM, accel_info.positions[ accel_info.position_index_motor[MOTOR_1] ]);
		pcnt_set_event_value(MOTOR_2, PCNT_EVT_H_LIM, accel_info.positions[ accel_info.position_index_motor[MOTOR_2] ]);
	}
	else{
		printf("ELEVATOR_STATE == RESTING_DOWN - UNKNOWN EVENT: %i\n", BUTTON_evt);
	}
}

void State_RESTING_UP(){
	if (xQueueReceive(BUTTON_evt_queue, &BUTTON_evt, 0) == pdTRUE && BUTTON_evt == BUTTON_DOWN) {
		ELEVATOR_STATE = MOVING_DOWN;
	}
	else{
		printf("ELEVATOR_STATE == RESTING_UP - UNKNOWN BUTTON EVENT: %i\n", BUTTON_evt);
	}

	CheckNetState();


	if( ELEVATOR_STATE == MOVING_DOWN){
		printf("ELEVATOR_STATE == RESTING_UP -> MOVING DOWN\n");
		SetDirection(MOVING_DOWN);
		accel_info.position_index_motor[MOTOR_1] = 0;
		accel_info.position_index_motor[MOTOR_2] = 0;
		SetSpeed(MOTOR_1, accel_info.speeds_down[ accel_info.position_index_motor[MOTOR_1] ]);
		SetSpeed(MOTOR_2, accel_info.speeds_down[ accel_info.position_index_motor[MOTOR_2] ]);
		pcnt_counter_clear(MOTOR_1);
		pcnt_counter_clear(MOTOR_2);
		pcnt_set_event_value(MOTOR_1, PCNT_EVT_H_LIM, accel_info.positions[ accel_info.position_index_motor[MOTOR_1] ]);
		pcnt_set_event_value(MOTOR_2, PCNT_EVT_H_LIM, accel_info.positions[ accel_info.position_index_motor[MOTOR_2] ]);
	}
}

void State_MOVING_UP() {

	CheckNetState();

	if (xQueueReceive(ENCODER_evt_queue, &ENCODER_evt, 0) == pdTRUE) {
		if (ENCODER_evt.status & PCNT_STATUS_H_LIM_M) {
			SetSpeed(ENCODER_evt.unit, FindScale() * accel_info.speeds_up[accel_info.position_index_motor[ ENCODER_evt.unit ]]);
		}
	}

	if( xQueueReceive(ENDSTOP_evt_queue, &ENDSTOP_evt, 0) == pdTRUE){
		printf("	MOVING UP - Motor %i reached end stop\n", ENCODER_evt.unit);
		if( BothEndstopsActivated() ){
			ELEVATOR_STATE = RESTING_UP;
		}
	}
}

void State_MOVING_DOWN() {

	CheckNetState();

	if (xQueueReceive(BUTTON_evt_queue, &BUTTON_evt, 0) == pdTRUE) {
		ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
		ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
		ELEVATOR_STATE = STOPPED;
	}

	if (xQueueReceive(ENCODER_evt_queue, &ENCODER_evt, 0) == pdTRUE) {
		if (ENCODER_evt.status & PCNT_STATUS_H_LIM_M) {
			SetSpeed(ENCODER_evt.unit, FindScale() * accel_info.speeds_down[accel_info.position_index_motor[ ENCODER_evt.unit ]]);
		}
		if(accel_info.position_index_motor[ENCODER_evt.unit] == NUM_ACCEL_STEPS && accel_info.position_index_motor[ OtherM(ENCODER_evt.unit) ] == NUM_ACCEL_STEPS){
			ELEVATOR_STATE = RESTING_DOWN;
		}
	}
}

bool adjusting = false;

void State_STOPPED(){

	if (xQueueReceive(BUTTON_evt_queue, &BUTTON_evt, 0) == pdTRUE && !adjusting) {
		if(BUTTON_evt == BUTTON_DOWN){
			gpio_set_level(DIRECTION_MOTOR_1, 0);
		}
		else{
			gpio_set_level(DIRECTION_MOTOR_1, 1);
		}
		pcnt_counter_clear(MOTOR_1);
		pcnt_counter_clear(MOTOR_2);
		pcnt_set_event_value(MOTOR_1, PCNT_EVT_H_LIM, MOVE_FRACTION);

		SetSpeed(MOTOR_1, ADJUST_SPEED);

		adjusting = true;
	}

	if (xQueueReceive(ENCODER_evt_queue, &ENCODER_evt, 0) == pdTRUE) {
		if (ENCODER_evt.status & PCNT_STATUS_H_LIM_M) {
			adjusting = false;
		}
	}
	CheckNetState();
}

void State_HOMING(){

	SetDirection(MOVING_UP);

	pcnt_set_event_value(MOTOR_1, PCNT_EVT_H_LIM, MOVE_FRACTION);
	pcnt_set_event_value(MOTOR_2, PCNT_EVT_H_LIM, MOVE_FRACTION);

	SetSpeed(MOTOR_1, HOMING_SPEED);
	SetSpeed(MOTOR_2, HOMING_SPEED);

	while(1){

		CheckNetState();

		if( xQueueReceive(ENDSTOP_evt_queue, &ENDSTOP_evt, 0) == pdTRUE){

			printf("	HOMING - Motor %i reached end stop\n", ENCODER_evt.unit);
			if( BothEndstopsActivated() ){
				ELEVATOR_STATE = RESTING_UP;
			}
		}
		if (ENCODER_evt.status & PCNT_STATUS_H_LIM_M) {
			if ( accel_info.position_index_motor[ENCODER_evt.unit] > accel_info.position_index_motor[ OtherM(ENCODER_evt.unit) ] ){
				SetSpeed(ENCODER_evt.unit, FindScale() * HOMING_SPEED);
			}
			else{
				SetSpeed(ENCODER_evt.unit, HOMING_SPEED);
			}
		}
	}
}

void State_DROPOUT(){
	SetDirection(MOVING_DOWN);

	pcnt_set_event_value(MOTOR_1, PCNT_EVT_H_LIM, MOVE_FRACTION);
	pcnt_set_event_value(MOTOR_2, PCNT_EVT_H_LIM, MOVE_FRACTION);

	SetSpeed(MOTOR_1, DROPOUT_SPEED);
	SetSpeed(MOTOR_2, DROPOUT_SPEED);

	while(1){

		CheckNetState();

		if (ENCODER_evt.status & PCNT_STATUS_H_LIM_M) {
			if ( accel_info.position_index_motor[ENCODER_evt.unit] > accel_info.position_index_motor[ OtherM(ENCODER_evt.unit) ] ){
				SetSpeed(ENCODER_evt.unit, FindScale() * HOMING_SPEED);
			}
			else{
				SetSpeed(ENCODER_evt.unit, DROPOUT_SPEED);
			}
		}
	}
}

void app_main() {

	printf("Starting Elevator Firmware\n");

	//TEST();
	//TEST_TURN();
	//TEST_RELAYS();
	//TEST_PWM();
	//TEST_BUTTON_MOVE();
	//TEST_SPEEDS();
	//TEST_NETWORK();

	Init();

	if( BothEndstopsActivated( )){
		printf("	Elevator at home position\n");
		ELEVATOR_STATE = RESTING_UP;
	}
	else{
		printf("	Elevator homing\n");
		ELEVATOR_STATE = HOMING;
	}

	while(1){
		if(ELEVATOR_STATE == MOVING_DOWN){
			State_MOVING_DOWN();
		}
		if(ELEVATOR_STATE == MOVING_UP){
			State_MOVING_UP();
		}
		if(ELEVATOR_STATE == RESTING_DOWN){
			State_RESTING_DOWN();
		}
		if(ELEVATOR_STATE == RESTING_UP){
			State_RESTING_UP();
		}
		if( ELEVATOR_STATE == STOPPED){
			State_STOPPED();
		}
		if( ELEVATOR_STATE == HOMING){
			State_HOMING();
		}
	}
}
