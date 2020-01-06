#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#include "freertos/portmacro.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "esp_log.h"

enum State {
	MOVING_DOWN = 1, MOVING_UP, RESTING_UP, RESTING_DOWN, UNDEFINED
};


typedef enum State State;

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

#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<DIRECTION_MOTOR_1) | (1ULL<<DIRECTION_MOTOR_2) | (1ULL<<RELAY_230V) | (1ULL<<RELAY_SPARE))
#define GPIO_INPUT_PIN_SEL  ((1ULL<<ENDSTOP_1) | (1ULL<<ENDSTOP_2) | (1ULL<<BUTTON_UP) | (1ULL<<BUTTON_DOWN))

#define MOTOR_1 0x01
#define MOTOR_2 0x02

#define NUM_ACCEL_STEPS 400
#define TOTAL_MOVE_COUNT 5000
//#define TOTAL_MOVE_COUNT 1000
#define MOVE_FRACTION TOTAL_MOVE_COUNT/(NUM_ACCEL_STEPS + 1)
#define MOVE_FRACTION_FLOAT (float)TOTAL_MOVE_COUNT/((float)NUM_ACCEL_STEPS + 1.0f)

#define POSITION_POOL_TIME_MS 10 / portTICK_PERIOD_MS
//Handle for non volatile memory
nvs_handle_t flash_handle;

xQueueHandle ENCODER_evt_queue = NULL; // A queue to handle pulse counter events
xQueueHandle ENDSTOP_evt_queue = NULL;
xQueueHandle BUTTON_evt_queue = NULL;

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

int ELEVATOR_STATE = RESTING_UP;

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

void setSpeed(int motor, int speed) {
	if (motor == MOTOR_1) {
		ledc_channel1.duty = accel_info.motor_pwr_scale[ MOTOR_1 ] * (1024 * speed) / 100;
		printf("setSpeed motor: %i speed: %i\n", motor, ledc_channel1.duty);
		ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, ledc_channel1.duty);
		ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
	}
	else{
		ledc_channel2.duty = accel_info.motor_pwr_scale[ MOTOR_2 ] * (1024 * speed) / 100;
		printf("setSpeed motor: %i speed: %i\n", motor, ledc_channel2.duty);
		ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, ledc_channel2.duty);
		ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
	}
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
	uint32_t gpio_num = (uint32_t) arg;
	xQueueSendFromISR(BUTTON_evt_queue, &gpio_num, NULL);
}

static void IRAM_ATTR ENCODER_intr_handler(void *arg) {
	uint32_t intr_status = PCNT.int_st.val;
	int i;
	ENCODER_evt_t evt;
	portBASE_TYPE HPTaskAwoken = pdFALSE;

	for (i = 0; i < PCNT_UNIT_MAX; i++) {
		if (intr_status & (BIT(i))) {
			evt.unit = i;

			//Kill the engines at the end of the line
			if (accel_info.position_index_motor[evt.unit] == NUM_ACCEL_STEPS) {
				//ledc_stop(LEDC_LOW_SPEED_MODE, evt.unit, 0);
				ledc_set_duty(LEDC_LOW_SPEED_MODE, evt.unit, 0);
				ledc_update_duty(LEDC_LOW_SPEED_MODE, evt.unit);
			}
			else{
				accel_info.position_index_motor[evt.unit]++;
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

static void SetupMotor_PWM(void) {
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

static void SetupCounter(void) {

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
	pcnt_set_filter_value(PCNT_UNIT_1, 100);
	pcnt_filter_enable(PCNT_UNIT_1);
	pcnt_set_filter_value(PCNT_UNIT_2, 100);
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

void setDirection(State direction) {

	printf("setDirection: ");

	accel_info.position_index_motor[1] = 0;
	accel_info.position_index_motor[2] = 0;

	if (direction == MOVING_UP) {
		printf("Moving UP\n");
		ELEVATOR_STATE = MOVING_UP;
		gpio_set_level(DIRECTION_MOTOR_1, 0);
		gpio_set_level(DIRECTION_MOTOR_2, 0);
	} else {
		printf("Moving DOWN\n");
		ELEVATOR_STATE = MOVING_DOWN;
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
		setSpeed(1, 20);
		vTaskDelay(1000 / portTICK_PERIOD_MS);

		printf("Speed 100\n");
		setSpeed(1, 100);
		vTaskDelay(1000 / portTICK_PERIOD_MS);

		printf("Speed 0\n");
		setSpeed(1, 0);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

void TEST_TURN() {
	SetupRelays();

	SetupMotor_PWM();

	setDirection(MOVING_DOWN);

	while (1) {

		printf("Speed 100\n");
		setSpeed(1, 100);
		setSpeed(2, 100);
		vTaskDelay(1000 / portTICK_PERIOD_MS);

		printf("Speed 0\n");
		setSpeed(1, 0);
		setSpeed(2, 0);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		setDirection(MOVING_UP);

		printf("Speed 100\n");
		setSpeed(1, 100);
		setSpeed(2, 100);
		vTaskDelay(1000 / portTICK_PERIOD_MS);

		printf("Speed 0\n");
		setSpeed(1, 0);
		setSpeed(2, 0);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		setDirection(MOVING_DOWN);
	}
}

void TEST_COUNT_EVENT() {

	SetupRelays();

	// Initialize LEDC to generate sample pulse signal
	SetupMotor_PWM();

	// Initialize PCNT event queue and PCNT functions
	SetupCounter();

	setDirection(MOVING_DOWN);

	setSpeed(1, 100);

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
					setDirection(MOVING_UP);
					accel_info.position_index_motor[MOTOR_1] = 0;
					accel_info.position_index_motor[MOTOR_2] = 0;
					setSpeed(MOTOR_1, accel_info.speeds_up[0]);
					setSpeed(MOTOR_2, accel_info.speeds_up[0]);
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
					//setSpeed(ENCODER_evt.unit, accel_info.speeds_down[ accel_info.position_index_motor[ENCODER_evt.unit] ]);

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
					setSpeed(ENCODER_evt.unit, scale*accel_info.speeds_down[accel_info.position_index_motor[ MOTOR_1 ]]);
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
					setDirection(MOVING_DOWN);
					accel_info.position_index_motor[MOTOR_1] = 0;
					accel_info.position_index_motor[MOTOR_2] = 0;
					setSpeed(MOTOR_1, accel_info.speeds_down[ accel_info.position_index_motor[MOTOR_1] ]);
					setSpeed(MOTOR_2, accel_info.speeds_down[ accel_info.position_index_motor[MOTOR_2] ]);
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
							setSpeed(MOTOR_1, 0);
						}
						else{
							setSpeed(MOTOR_1, accel_info.speeds_down[accel_info.position_index_motor[ MOTOR_1 ]]);
							setSpeed(MOTOR_2, accel_info.speeds_down[accel_info.position_index_motor[ MOTOR_2 ]]);
						}
					}
					else {
						if( accel_info.position_index_motor[ MOTOR_2 ] > accel_info.position_index_motor[ MOTOR_1 ]){
							setSpeed(MOTOR_2, 0);
						}
						else{
							setSpeed(MOTOR_1, accel_info.speeds_down[accel_info.position_index_motor[ MOTOR_1 ]]);
							setSpeed(MOTOR_2, accel_info.speeds_down[accel_info.position_index_motor[ MOTOR_2 ]]);
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
			setDirection(MOVING_DOWN);
		}
		else{
			setDirection(MOVING_UP);
		}

		setSpeed(MOTOR_1, accel_info.speeds_down[accel_info.position_index_motor[ MOTOR_1 ]]);
		setSpeed(MOTOR_2, accel_info.speeds_down[accel_info.position_index_motor[ MOTOR_2 ]]);

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
							setSpeed(MOTOR_1, 0);
							printf("		Stop\n");
						}
						else{
							setSpeed(MOTOR_1, accel_info.speeds_down[accel_info.position_index_motor[ MOTOR_1 ]]);
							setSpeed(MOTOR_2, accel_info.speeds_down[accel_info.position_index_motor[ MOTOR_2 ]]);
						}
						pcnt_set_event_value(MOTOR_1, PCNT_EVT_H_LIM, accel_info.positions[ accel_info.position_index_motor[MOTOR_1] ]);
					}
					else {
						printf("	M2\n");
						if( accel_info.position_index_motor[ MOTOR_2 ] > accel_info.position_index_motor[ MOTOR_1 ]){
							setSpeed(MOTOR_2, 0);
							printf("		Stop\n");
						}
						else{
							setSpeed(MOTOR_1, accel_info.speeds_down[accel_info.position_index_motor[ MOTOR_1 ]]);
							setSpeed(MOTOR_2, accel_info.speeds_down[accel_info.position_index_motor[ MOTOR_2 ]]);
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

void app_main() {

	//TEST();
	//TEST_TURN();
	//TEST_RELAYS();
	//TEST_PWM();
	//TEST_BUTTON_MOVE();
	TEST_SPEEDS();

	printf("Setup\n");

	SetupRelays();

	SetupMotor_PWM();

	SetupCounter();

	SetupEndpointsAndButtons();

	printf("Main loop\n");


	if (user_isr_handle_1) {
		//Free the ISR service handle.
		esp_intr_free(user_isr_handle_1);
		user_isr_handle_1 = NULL;
	}
	if (user_isr_handle_2) {
		//Free the ISR service handle.
		esp_intr_free(user_isr_handle_2);
		user_isr_handle_2 = NULL;
	}
}
