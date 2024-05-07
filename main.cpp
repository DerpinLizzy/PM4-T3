#include <mbed.h>
#include <cstdio>
#include <iostream>
#include "PM2_Drivers.h"
#include "PinNames.h"
#include "HX711.h"

# define M_PI 3.14159265358979323846                    // Number pi
# define MAX_VOLTS 12.0                                 // Maximum Voltage of Board [V]
# define SDA_ADDR 18<<1
# define GEAR_RADIUS 44.6                               // mm

// Runtime Variables ================================================
bool do_execute_main_task = false;                      // Toggled via user button(blue), used to activate main Task
bool do_reset_all_once = false;                         // Toggled via user button(blue), used to reset certain variables once
bool jail = false;
int vial_count = 8;
float x_pos = 0;

// Load Cell Variables ==============================================
constexpr float loadCellMaxWeight_kg = 0.1;                 // Maximum tolerable Weight on Load Cell
constexpr float loadCellSensitivity_V_V = 0.0006;           // Load Cell sensitivity displayed usually in mV/V, here in V/V.
constexpr float amplifierVoltage_V = 5;                     // Amplifier Input/Vcc value in Volts
constexpr float amplifierGain = 128;                        // Amplifier Gain 
constexpr float amplifierDrift_V = 0.0002;                  // Amplifier Drift/Offset
constexpr float measurementInaccuracy = 1.0635;             // 100% plus offset of 6.35%
constexpr int   main_task_period_ms = 100;                  // define main task period time in ms

float readout = 0;                  // Raw, unadjusted readout of gauge
float loadCellVoltage_mv = 0;       // Adjusted readout of gauge (read - zero value)
float weight_g = 0;                 // Conversion results in kg
float amplifierOffset = 0;          // Zero value

// Check Variables ==================================================
constexpr float weight_soll = 13.87;
constexpr float weight_low_thresh = 13.4;
constexpr float weight_high_thresh = 14.8;
constexpr float weight_no_cap_low = 11.5;
constexpr float weight_no_cap_high = 13.3;

// Motor Variables ==================================================
constexpr float sir_cumfrence = M_PI * GEAR_RADIUS;

// Output Variables =================================================
char faults[9] = {'0'};
static int error_count = 0;

// Global Objects ===================================================
/* PinMap:
PB_10 - SPI2_SCK pin used for load cell SCK
PB_4  - SPI1_MISO pin used for load cell data transfer from amplifier to controller
PB_8  - I2C SCL pin used for communication with arduino
PB_9  - I2C SDA pin used for communication with arduino
PC_13 - on-board user button blue
PB-13 - DigitalIn for limit switch
LED1  - on-board LED 1
*/
HX711 gauge(amplifierVoltage_V,PB_10,PB_4,amplifierGain);
DebounceIn user_button(BUTTON1);                          // create InterruptIn interface object to evaluate user button falling and rising edge (no blocking code in ISR)
DebounceIn limit_switch(PB_13);
DigitalOut user_led(LED1);
I2C com(I2C_SDA,I2C_SCL);

// Function Prototypes ==============================================
void user_button_pressed();                         // custom functions which gets executed when user button gets pressed
void limit_switch_event();
float voltToKilograms(float measuredVoltage, float maxLoad, float excitingVolt, float sensitivity); // Convert Volt output to Weight in kg
void mark_error(int index,char *errors);
void waitForReset();
void measure_weight();

// main() runs in its own thread in the OS ==========================
int main(){

    // Interrput - Setup ============================================
    user_button.fall(&user_button_pressed);
    limit_switch.fall(&limit_switch_event);
    Timer main_task_timer;
   
    // Load Cell - Setup ============================================
    gauge.powerUp();
    gauge.setGain();
    gauge.setDrift(amplifierDrift_V);

    while(gauge.isReady() == true){}
    amplifierOffset = gauge.read();
    printf("\nOffset = %f",amplifierOffset);

    // Motor - Setup ================================================
    

    // Code =========================================================
    main_task_timer.start();
    while (true){
    
    user_led = !user_led;

    main_task_timer.reset();
        
        if (do_execute_main_task){
            do_execute_main_task = false;

            // feed dc motor: move to zeroing position
            // limit switch: zero X-axis
            // feed dc motor: move to first position weighing

            for(int i = 0; i < vial_count; i++){
                measure_weight();

                if(weight_g > weight_low_thresh && weight_g < weight_high_thresh){
                    
                    /* Servo task 2:
                        - feed dc: move rack backwards to clamp 
                        - bottom clamp servo: grip tube
                        - Z-axis servo: move top clamp down
                        - top clamp servo: grip cap
                        - Z-Axis servo: move up
                         & clamp dc motor: turn to unscrew cap
                        - feed dc motor: turn to deposite position
                        - top clamp servo: release cap
                        - feed dc motor: move tube to measure position
                    */

                    measure_weight();

                    // feed dc motor: move rack to next position

                    if(weight_g < weight_no_cap_low || weight_g > weight_no_cap_high){
                        mark_error(i, faults);
                    }

                }else{
                    mark_error(i, faults);
                    // feed DC motor turn to next position
                }
            }
            com.write(SDA_ADDR,faults,error_count+1);

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
        while(gauge.isReady() == false){}
        readout = gauge.read();
        loadCellVoltage_mv = readout - amplifierOffset;
        weight_g += voltToKilograms(loadCellVoltage_mv, loadCellMaxWeight_kg, amplifierVoltage_V, loadCellSensitivity_V_V);
    }
    weight_g /= 5;
    weight_g *= measurementInaccuracy;

    printf("\nLoadcell Readout: %f",readout);
    printf("\nAdjusted Readout: %f", loadCellVoltage_mv);
    printf("\nCurrent weight in grams: %f", weight_g);
}

void waitForReset(){
    while(true){}
}

void mark_error(int index,char *errors){
    errors[index] = ((char)(index + 1));
    error_count++;
}

float voltToKilograms(float measuredVoltage, float maxLoad, float excitingVolt, float sensitivity){
    float weight = 0;
    weight = measuredVoltage * maxLoad/(excitingVolt * sensitivity);
    return weight;
}

void limit_switch_event(){
    // stop feed motor
    thread_sleep_for(10);
    x_pos = 0;
}

void user_button_pressed(){
    do_execute_main_task = !do_execute_main_task;   // do_execute_main_task if the button was pressed twice
    if (do_execute_main_task) do_reset_all_once = true;
}