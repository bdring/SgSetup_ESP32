#include <MeanFilterLib.h>

/**
 * Author Barton Dring @buildlog
 * 
 * Used to find good values for TMC2130 Stallguard2 settings
 *
 * Output:
 * Typical: RPM:10 SGT:24 SGV:0442 TSTEP: 01407 SG:0 I:397
 * RPM = Current Spped of the motors
 * SGT = Current StallGuard Threashold
 * SGV = Current reported StallGaurd value
 * TSTEP = Time in microseconds between steps. Use this to determine TCOOLTHRS value
 * SG = Stall detected (1 or 0)
 * I = Motor current in mA 
 * 
 * Tips:
 * 
 * StallGuard only works in coolstep mode. It will not work in the quiet Stealthchop modes
 *
 * It only works reliably in a speed band. You will want to find this speed and 
 * use it for your homing. Start with about 1 RPS
 *
 * Play with RPM (+ & - keys) and StallGuard Threadhold (i & d keys) until you see high SGV  
 * while running and a SG=1 when stalling (or nearly stalling)
 *
 * Note the the TSTEP value and plug a slightly lower value in TCOOLTHRS. This will prevent SG=1 at
 * at slower values.
 * 
 */
 #include "print.h" // For sprintf support https://github.com/mpaland/printf
 
#define MOTOR_STEPS 200.0 // steps per rev typically 200 (1.8°) to 400 (0.9°)
#define MICROSTEPPING 16.0 
#define DRIVER_CURRENT_mA 400 // current in milliamps  
 
#define MAX_SPEED_RPS  60
#define MIN_SPEED_RPS  1
#define INIT_SPEED_RPS 1  // a target
#define INIT_SPEED_ACTUAL 0.1 // accel from this to target
#define SPEED_INCR_VAL 0.2 // change amount per +,- inputs
#define ACCEL_RATE 0.05  // speed changes by this much per DISPLAY_INTERVAL until target speed

#define DISPLAY_INTERVAL 10 // in milliseconds. How oftn to display values

#define INIT_SGT_VALUE 6

#define EN_PIN    GPIO_NUM_13

/*
#define DIR_PIN   GPIO_NUM_26// X
#define STEP_PIN  GPIO_NUM_12 
#define CS_PIN    GPIO_NUM_17 
*/
#define DIR_PIN   GPIO_NUM_25// Y
#define STEP_PIN  GPIO_NUM_14 
#define CS_PIN    GPIO_NUM_16 

#include <TMC2130Stepper.h>  // https://github.com/teemuatlut/TMC2130Stepper 
#include <TMC2130Stepper_REGDEFS.h> 
TMC2130Stepper driver = TMC2130Stepper(CS_PIN);

bool vsense;
volatile uint16_t speed_rps = INIT_SPEED_RPS; // a good starting point for a lot of motors
int8_t sg_value = INIT_SGT_VALUE;

float current_speed = INIT_SPEED_ACTUAL;
float target_speed = INIT_SPEED_ACTUAL;
bool steps_enabled = false;

MeanFilter<long> meanFilter(10);


uint16_t rms_current(uint8_t CS, float Rsense = 0.11) {
  return (float)(CS+1)/32.0 * (vsense?0.180:0.325)/(Rsense+0.02) / 1.41421 * 1000;
}

// used to get step interval from RPM
uint32_t step_interval_us(float rps) { 
	uint32_t steps_per_second = ((MOTOR_STEPS * MICROSTEPPING * rps));	
	return (1000000 / steps_per_second);
}


// setup timer
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; //this is for synchronization between the interrupt and loop() functions

//timer interrupt function
void IRAM_ATTR onTimer(){
  digitalWrite(STEP_PIN, HIGH);
  digitalWrite(STEP_PIN, LOW);
}
 
char buff[100];  // for sprintf

void setup() {
	// menu
	Serial.begin(115200); //init serial port and set baudrate    
    Serial.println("\r\nTMC2130 StallGuard2 test program\r\n");
	Serial.println("'+' = faster");
	Serial.println("'-' = slower");
	Serial.println("'1' = run");
	Serial.println("'0' = pause");
	Serial.println("'r' = reverse");
	Serial.println("'i' = inc sg");
	Serial.println("'d' = dec sg");	
	Serial.println("Send '1' character to begin (\r\n");	
	
	while(Serial.available() == 0) // what for a buuton push...allows reading menu
	{
	}
	
	pinMode(EN_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(CS_PIN, OUTPUT);
	pinMode(MISO, INPUT_PULLUP);
	
	digitalWrite(EN_PIN, HIGH); //deactivate driver (LOW active)
    digitalWrite(DIR_PIN, LOW); //LOW or HIGH
    digitalWrite(STEP_PIN, LOW);
    digitalWrite(CS_PIN, HIGH);
    SPI.begin();
    
	
	// Set timer interrupt that generates pulses
    timer = timerBegin(0, 80, true);  //set timer to microseconds divider (80 MHz / 80 = 1 MHz, as for 1 microsconds)
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, step_interval_us(current_speed), true);
	timerAlarmEnable(timer); //enable alarm    
	
  
	driver.push();
    driver.toff(3);
    driver.tbl(1);
	driver.stealthChop(1);
	driver.coolstep_min_speed(200); // 20bit max
    driver.hysteresis_start(4);
    driver.hysteresis_end(-2);
    driver.rms_current(DRIVER_CURRENT_mA); // mA
    driver.microsteps(16);
    driver.diag1_stall(1);
    driver.diag1_active_high(1);	
    
    driver.THIGH(0);
    driver.semin(5);
    driver.semax(2);
    driver.sedn(0b01);
    driver.sg_stall_value(sg_value);
  
	//TMC2130 outputs on (LOW active)
	digitalWrite(EN_PIN, LOW);

	vsense = driver.vsense(); 
	
	sprintf(buff, "IO In: %u\r\n", driver.IOIN() & 0xFF);
	Serial.print(buff);
	// wait for any key to begin so the user can read the prompt
	
	
}

void loop()
{
  static uint32_t last_time=0;
  uint32_t ms = millis();
  
  
  
  portENTER_CRITICAL(&timerMux);
  while(Serial.available() > 0) {
    int8_t read_byte = Serial.read();
    if (read_byte == '0')      { 
		timerAlarmDisable(timer);
		digitalWrite( EN_PIN, HIGH ); 
		steps_enabled = true;
		current_speed = INIT_SPEED_ACTUAL;
	}
    else if (read_byte == '1') { 
		timerAlarmEnable(timer); 
		digitalWrite( EN_PIN,  LOW );
		steps_enabled = true;
	}
    else if (read_byte == '+') {
      if (target_speed < MAX_SPEED_RPS) { 
        target_speed += SPEED_INCR_VAL;
		Serial.print("faster\r\n");
      }
    }
    else if (read_byte == '-') {
      if (target_speed > MIN_SPEED_RPS) { 
        target_speed -= SPEED_INCR_VAL; 
		Serial.print("slower\r\n");
      }

    }
	else if (read_byte == 'r') { // reverse
		digitalWrite(DIR_PIN, !digitalRead(DIR_PIN));		
	}
	else if (read_byte == 'i') { // increase stall value
		if (sg_value < 64) {
			sg_value++;
			driver.sg_stall_value(sg_value);
		}
		
	}
	else if (read_byte == 'd') {
		if (sg_value > -64) {
			sg_value--;
			driver.sg_stall_value(sg_value);
		}
		
	}
  }
  portEXIT_CRITICAL(&timerMux);
    
  if((ms-last_time) > DISPLAY_INTERVAL) //run every 0.01s
  {
    last_time = ms;
	
	if(steps_enabled) {
		if (fabs(current_speed - target_speed) >=  ACCEL_RATE) {
			if (current_speed < target_speed) {
				current_speed += ACCEL_RATE;
			}
			else {
				current_speed -= ACCEL_RATE;
			}
		}
		timerAlarmWrite(timer, step_interval_us(current_speed), true);
	}
	
	uint32_t TSTEP = driver.TSTEP();
	
	if (TSTEP != 0xFFFFF) {	// if driver is stepping
	
		uint32_t drv_status = driver.DRV_STATUS();	

   

    int mean = meanFilter.AddValue((drv_status & STALLGUARD_bm)>>STALLGUARD_bp);
	
		sprintf(buff, "RPS:%2.1f SGT:%02d SGV:%04d", current_speed, sg_value, (drv_status & SG_RESULT_bm)>>SG_RESULT_bp);
		Serial.print(buff);
		
		sprintf(buff, " TSTEP: %05d", TSTEP);
		Serial.print(buff);
		
		sprintf(buff, " SG: %d I:04%d \r\n", mean, DEC, (drv_status & CS_ACTUAL_bm)>>CS_ACTUAL_bp);
		Serial.print(buff);	
	}
  }
}
