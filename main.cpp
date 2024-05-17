#include <mbed.h>
#include <cstdio>
#include <iostream>
#include "PM2_Drivers.h"
#include "PinNames.h"
#include "HX711.h"

# define M_PI 3.14159265358979323846                    // Number pi
# define MAX_VOLTS 12.0                                 // Maximum Voltage of Board [V]
# define SDA_ADDR 18<<1                                 // SPI Address of Child-Device
# define GEAR_DIAMETER 44.6                               // mm

// Runtime Variables ================================================
constexpr int   main_task_period_ms = 50;                  // define main task period time in ms
bool do_execute_main_task = false;                      // Toggled via user button(blue), used to activate main Task
bool do_reset_all_once = false;                         // Toggled via user button(blue), used to reset certain variables once
bool positioned = false;
bool jail = false;
int vial_count = 8;
float x_pos = 430;

// Check Variables ==================================================
constexpr float weight_soll = 14.12;            // Weight in grams with cap and 7.5ml of liquid
constexpr float weight_closed_low = 13.40;       // Weight in grams with cap and 7.0ml of liquid
constexpr float weight_closed_high = 14.63;      // Weight in grams with cap and 8.0ml of liquid
constexpr float weight_opened_low = 12.1;       // Weight in grams without cap and 7.0ml of liquid
constexpr float weight_opened_high = 13.33;      // Weight in grams without cap and 8.0ml of liquid

// Load Cell Variables ==============================================
constexpr float loadCellMaxWeight_kg = 0.1;                 // Maximum tolerable Weight on Load Cell
constexpr float loadCellSensitivity_V_V = 0.0006;           // Load Cell sensitivity displayed usually in mV/V, here in V/V.
constexpr float amplifierVoltage_V = 5;                     // Amplifier Input/Vcc value in Volts
constexpr float amplifierGain = 128;                        // Amplifier Gain 
constexpr float amplifierDrift_V = 0.0002;                  // Amplifier Drift/Offset
constexpr float measurementInaccuracy = 1.0635;             // 100% plus offset of 6.35%

float readout = 0;                  // Raw, unadjusted readout of gauge
float loadCellVoltage_mv = 0;       // Adjusted readout of gauge (read - zero value)
float weight_g = 0;                 // Conversion results in kg
float amplifierOffset = 0;          // Zero value

// Motor Variables ==================================================
constexpr float sir_cumfrence = M_PI * GEAR_DIAMETER;
constexpr float one_turn = 0.17;
constexpr float l_factor = sir_cumfrence/one_turn;

constexpr float max_voltage = 6.0f;                    // define maximum voltage of battery packs, adjust this to 6.0f V if you only use one batterypack
constexpr float counts_per_turn_A = 48.0f * 195.3;       // define counts per turn at gearbox end: counts/turn * gearratio
constexpr float counts_per_turn_B = 48.0f * 390.6;
constexpr float kn_A = 73.0f / 6.0f;                   // define motor constant in RPM/V
constexpr float kn_B = 36.0f / 6.0f;  
constexpr float k_gear_A = 195.3f / 78.125f;           // define additional ratio in case you are using a dc motor with a different gear box, e.g. 100:1
constexpr float k_gear_B =  390.6f / 78.125f;
constexpr float kp = 0.2f;

// Output Variables =================================================
char faults[9] = {'0'};
static int error_count = 0;

// Global Objects ===================================================
/* PinMap:
BUTTON1 - on-board user button blue
LED1  - on-board LED 1
PA_7  - DigitalIn for limit switch

PB_10 - SPI2_SCK pin used for load cell SCK
PB_4  - SPI1_MISO pin used for load cell data transfer from amplifier to controller

PB_8  - I2C SCL pin used for communication with arduino
PB_9  - I2C SDA pin used for communication with arduino

PC_2  - AnalogOut for tube holder servo (A0)
PB_2  - DigitalOut for Z-Axis Servo (D0)
PC_8  - DigitalOut for clamp servo (D1)
PB_13 - Feed DC Motor (M1)
PA_9  - Spin DC Motor (M2)
PB_15 - DC Motor Ground for both M1 and M2
PA_6  - Feed DC Motor Encoder Signal A
PC_7  - Feed DC Motor Encoder Signal B
PB_6  - Spin DC Motor Encoder Signal A
PB_7  - Spin DC Motor Encoder Signal B
*/
HX711 gauge(amplifierVoltage_V,PB_10,PB_4,amplifierGain);
DebounceIn user_button(BUTTON1);                          // create InterruptIn interface object for user button
DebounceIn limit_switch(PA_7, PullUp);
DigitalOut user_led(LED1);
I2C com(I2C_SDA,I2C_SCL);

// Function Prototypes ==============================================
void user_button_pressed();                         // custom functions which gets executed when user button gets pressed
void limit_switch_event();
float voltToKilograms(float measuredVoltage, float maxLoad, float excitingVolt, float sensitivity); // Convert Volt output to Weight in kg
void mark_error(int index,char *errors);
void waitForReset();
void measure_weight();
void move_rack(float length, PositionController & positionController_A);

    bool run = false;

// main() runs in its own thread in the OS ==========================
int main(){

    // Interrput - Setup ============================================
    Timer main_task_timer;
    user_button.fall(&user_button_pressed);
    limit_switch.fall(&limit_switch_event);

    // Load Cell - Setup ============================================
    gauge.powerUp();
    gauge.setGain();
    gauge.setDrift(amplifierDrift_V);

    while(gauge.isReady() == false){ }
    thread_sleep_for(10);
    amplifierOffset = gauge.read();
    printf("\nOffset = %f\n",amplifierOffset);
    printf("loadcell init done\n");

    // Motor Setup: Servos [1 = Z-Axis // 2 = hold // 4 = crab Clamp]
    Servo servo_S1(PB_2);       // Big Servo - Z-Axis
    Servo servo_S2(PC_3);       // Servo Cap black - stick hold 
    Servo servo_S4(PC_5);       // Servo orange - crab clamp
    servo_S1.setPeriod_mus(20000);
    servo_S2.setPeriod_mus(20000);
    servo_S4.setPeriod_mus(20000);
    float servo_S1_angle = 0; // servo S1 normalized input: 0...1
    float servo_S2_angle = 0;
    float servo_S4_angle = 0.0f;

    //Servo1 start position
    servo_S1.enable();
    servo_S1.setNormalisedAngle(0.022f);
    thread_sleep_for(1000);
    servo_S1.disable();

    // hold start position
    servo_S2.enable();
    servo_S2.setNormalisedAngle(-0.12f);
    thread_sleep_for(1000);
    servo_S2.disable();

    // Crab Claw start position
    servo_S4.enable();
    servo_S4.setNormalisedAngle(0.105f);
    thread_sleep_for(1000);
    servo_S4.disable();
    printf("servo init done\n");

    // Motor Setup: DC [ A = Feed // B = Spin ]======================
    DigitalOut enable_motors (PB_15);                   // M-
    FastPWM pwm_A(PB_13);                               // A M+
    EncoderCounter encoder_A(PA_6, PC_7);               // Encoder Pins A
    FastPWM pwm_B(PA_10);                               // B M+
    EncoderCounter encoder_B(PA_0, PA_1);               // Encoder Pins B
    enable_motors = 1;

    PositionController positionController_A(counts_per_turn_A * k_gear_A, kn_A / k_gear_A, max_voltage, pwm_A, encoder_A);
    positionController_A.setSpeedCntrlGain(kp * k_gear_A);
    float max_speed_rps_A = 0.2f; // define maximum speed that the position controller is changig the speed, has to be smaller or equal to kn * max_voltage
    positionController_A.setMaxVelocityRPS(max_speed_rps_A);

    PositionController positionController_B(counts_per_turn_B * k_gear_B, kn_B / k_gear_B, max_voltage, pwm_B, encoder_B); // parameters adjusted to 100:1 gear, we need a different speed controller gain here
    positionController_B.setSpeedCntrlGain(kp * k_gear_B);   // adjust internal speed controller gain, this is just an example
    float max_speed_rps_B = 0.5f; // define maximum speed that the position controller is changig the speed, has to be smaller or equal to kn * max_voltage
    positionController_B.setMaxVelocityRPS(max_speed_rps_B);

    printf("dcm init done\n");

    // Code =========================================================
    main_task_timer.start();
    while (true){
    
        user_led = !user_led;

        main_task_timer.reset();

        // Servo Configuration loops
        // float a = 0.02;
        // while(true){
        //     printf("%f\n",a);
        //     servo_S1.enable();
        //     servo_S1.setNormalisedAngle(a);
        //     thread_sleep_for(100);
        //     servo_S1.disable();
        //     thread_sleep_for(100);
        //     while(!do_execute_main_task){
        //         thread_sleep_for(10);
        //     }
        //     do_execute_main_task = false;
        //     a += 0.002;
        // }

        if (do_execute_main_task){
            printf("is on\n");
            printf("x_pos = %f\n", x_pos);

            // move rack to zeroing position
            while(!positioned){
                move_rack(3, positionController_A);
            }
            thread_sleep_for(1000);

            // feed motor: move to first position weighing
            move_rack(-25, positionController_A);
            thread_sleep_for(1500);

            for(int i = 0; i < vial_count; i++){
                printf("\nNow weighing: Position %d\n",i + 1 );
                measure_weight();

                if(weight_g > weight_closed_low && weight_g < weight_closed_high){

                    // clamp pos                    
                    move_rack(15, positionController_A);
                    thread_sleep_for(500);

                    // Servo1 down
                    servo_S1.enable();
                    servo_S1.setNormalisedAngle(0.044f);
                    thread_sleep_for(1500);
                    servo_S1.disable();
                    thread_sleep_for(100);

                    // Servo2 hold
                    servo_S2.enable();
                    servo_S2.setNormalisedAngle(0.075f);
                    thread_sleep_for(1500);

                    // Crab Clamp Close
                    servo_S4.enable();
                    servo_S4.setNormalisedAngle(0.05f);
                    thread_sleep_for(1000);

                    // Twist Cap
                    float angle_tracker_b = positionController_B.getRotation();
                    positionController_B.setDesiredRotation((angle_tracker_b + 1.2*0.08f)*-1);
                    thread_sleep_for(4000);

                    // Servo1 up
                    servo_S1.enable();
                    servo_S1.setNormalisedAngle(0.022f);
                    thread_sleep_for(1500);
                    servo_S1.disable();
                    thread_sleep_for(100);

                    //Servo2 release
                    servo_S2.setNormalisedAngle(-0.15f);
                    thread_sleep_for(1000);
                    servo_S2.disable();
                    thread_sleep_for(50);

                    //Untwist Cap
                    angle_tracker_b = positionController_B.getRotation();
                    positionController_B.setDesiredRotation(angle_tracker_b + 1.2*0.08f);
                    thread_sleep_for(4000);

                    // deposite pos
                    move_rack(-18, positionController_A);
                    thread_sleep_for(1000);

                    //Servo4 Open clamp
                    servo_S4.setNormalisedAngle(0.105f);
                    thread_sleep_for(1000);
                    servo_S4.disable();
                    thread_sleep_for(50);
                    
                    // measure pos
                    move_rack(8, positionController_A);
                    thread_sleep_for(1000);

                    measure_weight();
                    
                    // next tube pos
                    move_rack(-49.5, positionController_A);
                    thread_sleep_for(1500);

                    if(weight_g < weight_opened_low || weight_g > weight_opened_high){
                        mark_error(i, faults);
                    }

                }else{
                    mark_error(i, faults);
                    move_rack(-49.5, positionController_A);
                    thread_sleep_for(1500);
                }
            }
            //Servo1 start position
            servo_S1.enable();
            servo_S1.setNormalisedAngle(0.02f);
            thread_sleep_for(1000);
            servo_S1.disable();
            thread_sleep_for(50);
                
            // Claw start position
            servo_S4.enable();
            servo_S4.setNormalisedAngle(0.105f);
            thread_sleep_for(1000);
            servo_S4.disable();
            thread_sleep_for(50);

            // write your faults to display
            com.write(SDA_ADDR,faults,error_count+1);
            
            // eject rack
            move_rack(-45, positionController_A);
            thread_sleep_for(500);
            printf("Process done\n");

        // End if do_execute_main_task
        }else{
            if (do_reset_all_once){
                do_reset_all_once = false;
                weight_g = 0.0f;
                loadCellVoltage_mv = 0.0f;
                printf("\nReset");
            }
        }

        // read timer and make the main thread sleep for the remaining time span (non blocking)
        int main_task_elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(main_task_timer.elapsed_time()).count();
        thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

void measure_weight(void){
    weight_g = 0;
    for(int j = 0 ; j < 5 ; j++){
        while(gauge.isReady() == true){}
        thread_sleep_for(50);
        readout = gauge.read();
        loadCellVoltage_mv = readout - amplifierOffset;
        weight_g += voltToKilograms(loadCellVoltage_mv, loadCellMaxWeight_kg, amplifierVoltage_V, loadCellSensitivity_V_V);
        thread_sleep_for(50);
    }
    weight_g /= 5;
    weight_g *= (measurementInaccuracy - 0.0194);
    if(weight_g < 0){
        weight_g *= -1;
    }

    printf("\nCurrent weight in grams: %f\n", weight_g);
    thread_sleep_for(1000);
}

void move_rack(float length, PositionController & positionController_A){
    x_pos += length;
    float revolutions = length/l_factor;
    float angle_tracker = positionController_A.getRotation();
    thread_sleep_for(50);
    positionController_A.setDesiredRotation((angle_tracker + revolutions));
}

void waitForReset(){
    while(true){ thread_sleep_for(20); }
}

void mark_error(int index,char *errors){
    errors[index] = ((char)(index + 49));
    error_count++;
    printf("Posiiotn %d is faulty!!\n", index + 1);
}

float voltToKilograms(float measuredVoltage, float maxLoad, float excitingVolt, float sensitivity){
    float weight = 0;
    weight = measuredVoltage * maxLoad/(excitingVolt * sensitivity);
    return weight;
}

void limit_switch_event(){
    positioned = true;
    x_pos = 0;
}

void user_button_pressed(){
    do_execute_main_task = !do_execute_main_task;   // do_execute_main_task if the button was pressed twice
    run = true;
    if (do_execute_main_task) do_reset_all_once = true;
}