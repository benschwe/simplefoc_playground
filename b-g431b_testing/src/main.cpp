#include <SimpleFOC.h>
#include "SimpleFOCDrivers.h"
#include "encoders/stm32hwencoder/STM32HWEncoder.h"

#define BUTTON PC10

// Motor instance
BLDCMotor motor = BLDCMotor(21);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003, -64.0/7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// Motor encoder
STM32HWEncoder encoder = STM32HWEncoder(1024, A_HALL1, A_HALL2); 
//Encoder encoder = Encoder(PB7, PB8, 1024);
//void doA(){encoder.handleA();}
//void doB(){encoder.handleB();}

// instantiate the commander
Commander command = Commander(Serial);
void onMotor(char* cmd){ command.motor(&motor, cmd); }

float temp_val_raw = 0.0;
float temp_degC = 0.0;

float vbus_val_raw = 0.0;
float vbus_V = 0.0;

float pot_val_V = 0.0;

long print_time_ms = 0;
float vel_sp_radspersec = 0.0;
float target = 0.0;

float vel = 0.0;
float current_target = 0.0;

static float Ntc2TempV(float ADCVoltage) 
{
	// Formula: https://www.giangrandi.org/electronics/ntc/ntc.shtml
	const float ResistorBalance = 4700.0;
	const float Beta  = 3425.0F;
	const float RoomTempI = 1.0F / 298.15F; //[K]
	const float Rt = ResistorBalance * ((3.3F / ADCVoltage)-1);
	const float R25 = 10000.0F;
	
	float T = 1.0F / ((log(Rt/R25) / Beta) + RoomTempI);
	T = T - 273.15;

	return T;
}


void setup() {

  // Button
  pinMode(BUTTON, INPUT);
  
  // initialise encoder hardware
  encoder._pinA = PB_6;
  encoder._pinB = PB_7_ALT1;
  //encoder.pullup = Pullup::USE_INTERN;
  encoder.init();
  
  // hardware interrupt enable
  //encoder.enableInterrupts(doA, doB);
  
  // link the motor to the encoder
  motor.linkSensor(&encoder);
  
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 18.0;
  driver.init();
  
  // link the motor and the driver
  motor.linkDriver(&driver);
  
  // link current sense and the driver
  currentSense.linkDriver(&driver);

  // initialize motor
  motor.init();

  currentSense.init();
  // no need for aligning
  currentSense.skip_align = true;
  motor.linkCurrentSense(&currentSense);
  
  // aligning voltage [V]
  motor.voltage_sensor_align = 0.6;

  // control loop type and torque mode 
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::velocity;
  motor.motion_downsample = 3.0;

  // velocity loop PID
  motor.PID_velocity.P = 0.4;
  motor.PID_velocity.I = 9.0;
  // Low pass filtering time constant 
  motor.LPF_velocity.Tf = 0.006;
  
  // angle loop PID
  motor.P_angle.P = 16.0;

  // Low pass filtering time constant 
  motor.LPF_angle.Tf = 0.002;
  
  // current q loop PID 
  motor.PID_current_q.P = 2.0;
  motor.PID_current_q.I = 200.0;
  motor.LPF_current_q.Tf = 0.002;
  
  // current d loop PID
  motor.PID_current_d.P = 2.0;
  motor.PID_current_d.I = 200.0;
  motor.LPF_current_d.Tf = 0.002;

  // Limits 
  motor.velocity_limit = 20.0;
  motor.voltage_limit = 10.0;
  motor.current_limit = 3.0;
  motor.PID_velocity.limit = motor.current_limit;

   // pwm modulation settings 
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.modulation_centered = 1.0;
  motor.motion_downsample = 8.0;

  // use monitoring with serial 
  Serial.begin(115200);

  // comment out if not needed
  motor.useMonitoring(Serial);
  
  // align encoder and start FOC
  motor.initFOC();

  // set the inital target value
  motor.target = 0.0;

  // define the motor id
  command.add('M', onMotor, "motor");

  print_time_ms = millis();

}

void loop() {

  // if (millis() - print_time_ms > 10){

  // //   vbus_val_raw = _readADCVoltageInline(A_VBUS, currentSense.params); 
  // //   vbus_V = vbus_val_raw * 10.7711;	
    
  // //   temp_val_raw = _readADCVoltageInline(A_TEMPERATURE, currentSense.params);  
  // //   temp_degC = Ntc2TempV(temp_val_raw);
    
  //   pot_val_V = _readADCVoltageInline(A_POTENTIOMETER, currentSense.params);   

  // //   // Current sensing measurements
  // //   PhaseCurrent_s phase_currents = currentSense.getPhaseCurrents();  
  // //   float motor_e_angle = motor.electricalAngle();   
  // //   DQCurrent_s dq_currents = currentSense.getFOCCurrents(motor_e_angle); 
    
  // //   // Serial.print(print_time_ms / 1000.0);
  // //   // Serial.print('\t');
  // //   // Serial.print(vbus_V);
  // //   // Serial.print('\t');
  // //   // Serial.print(temp_degC);
  // //   // Serial.print('\t');
  // //   // Serial.print(current_target);
  // //   // Serial.print('\t');
  // //   // Serial.print(dq_currents.d);
  // //   // Serial.print('\t');
  // //   // Serial.print(dq_currents.q);
  // //   // Serial.print('\t');
  // //   // Serial.print(phase_currents.a);
  // //   // Serial.print('\t');
  // //   // Serial.print(phase_currents.b);
  // //   // Serial.print('\t');
  // //   // Serial.println(phase_currents.c);

  // // Pot goes from 0 to 3.3

  // // Change current limit dynamically
  // //   //motor.current_limit = 0.4 * (pot_val_V / 3.3) + 0.10;  // amps
  // //   //motor.PID_velocity.limit = motor.current_limit;
  // //   //target = 40 * (pot_val_V / 3.3);

  // //   // Change velocity sp 
  //   target = pot_val_V * 36.0; 
  //   //motor.velocity_limit = vel_sp_radspersec;
  // //   //motor.P_angle.limit = vel_sp_radspersec;
    
  //   print_time_ms = millis();
  
  // }
  
  motor.loopFOC();
  motor.move();
  motor.monitor();
  command.run();
  
}

