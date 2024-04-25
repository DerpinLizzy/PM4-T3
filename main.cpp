#include <mbed.h>
#include <cstdio>
#include <iostream>
#include "PM2_Drivers.h"
#include "PinNames.h"
#include "HX711.h"

# define M_PI 3.14159265358979323846                     // Number pi
# define WHEEL_DIAMETER 3.0                              // Radius of crawler [cm]

bool do_execute_main_task = false;                      // this variable will be toggled via the user button (blue button) and decides whether to execute the main task or not
bool do_reset_all_once = false;                         // this variable is used to reset certain variables and objects and shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(PC_13);                          // create InterruptIn interface object to evaluate user button falling and rising edge (no blocking code in ISR)
void user_button_pressed_fcn();                         // custom functions which get executed when user button gets pressed, definition below

// Convert Volt output to Weight in kg
float voltToKilograms(float measuredVoltage, float maxLoad, float excitingVolt, float sensitivity){
    float weight = 0;
    weight = measuredVoltage * maxLoad/(excitingVolt * sensitivity);
    return weight;
}

// main() runs in its own thread in the OS
int main(){
//===============================Variables======================================

    const float max_voltage = 12.0f;                    // define maximum voltage of battery packs, adjust this to 6.0f V if you only use one batterypack
    const float counts_per_turn = 20.0f * 78.125f;      // define counts per turn at gearbox end: counts/turn * gearratio
    const float kn = 180.0f / 12.0f;                    // define motor constant in RPM/V
    const float k_gear_A = 100.0f / 78.125f;            // define additional ratio in case you are using a dc motor with a different gear box, e.g. 100:1
    const float k_gear_B = 195.0f / 78.125f;
    const float kp = 0.1f;   
    
    // attach button fall function to user button object, button has a pull-up resistor
    user_button.fall(&user_button_pressed_fcn);
    const int main_task_period_ms = 50;                 // define main task period time in ms e.g. 50 ms -> main task runs 20 times per second
    Timer main_task_timer;                              // create Timer object which we use to run the main task every main_task_period_ms
   
    DigitalIn mechanical_button(PC_5);                  // create DigitalIn object to evaluate extra mechanical button, you need to specify the mode for proper usage, see below
    mechanical_button.mode(PullUp);                     // set pullup mode: sets pullup between pin and 3.3 V, so that there is a defined potential
   
    float angle_tracker = 0.0f;                                            
    float ir_distance_mV = 0.0f;                        // Sharp GP2Y0A41SK0F, 4-40 cm IR Sensor 
    float ir_distance_cm = 0.0f;                        // define variable to store measurement
    AnalogIn ir_distance_sensor(PC_2);                  // create AnalogIn object to read in infrared distance sensor, 0...3.3V are mapped to 0...1
   

    // Load Cell - Variables ==============================================
    const float loadCellMaxWeight_kg = 0.1;
    const float loadCellSensitivity_V_V = 0.0006;
    const float amplifierVoltage_V = 5;
    const float amplifierGain = 128;
    
    float loadCellVoltage_mv = 0;       // read from the amplifier baord is in mV
    float loadCellWeight_kg = 0;        // Conversion results in kg

    DigitalIn load_cell(PC_8);
    
    HX711 gauge(amplifierVoltage_V,PB_2,PC_8,amplifierGain);

    // Load Cell - Seup ===================================================
    printf("%d",load_cell.read());
    gauge.powerUp();
    gauge.setGain();
    printf("Drift: %d\nAnalog Power Supply: %f",gauge.getDrift(),gauge.getAvdd());

    // Code ===============================================================
    main_task_timer.start();
    while (true){

        main_task_timer.reset();

        if (do_execute_main_task){
            if(gauge.isReady()){
                loadCellVoltage_mv = gauge.read();
                loadCellWeight_kg = voltToKilograms(loadCellVoltage_mv, loadCellMaxWeight_kg, amplifierVoltage_V, loadCellSensitivity_V_V);
                printf("Current weight: %f",loadCellWeight_kg);
            }

            else{
                if (do_reset_all_once){
                    do_reset_all_once = false;
                    loadCellWeight_kg = 0.0f;
                    loadCellVoltage_mv = 0.0f;

                }          
            }
        }
        // End if do_execute_main_task   

        // read timer and make the main thread sleep for the remaining time span (non blocking)
        int main_task_elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(main_task_timer.elapsed_time()).count();
        thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);

    }           // End while loop
}            // End main

void user_button_pressed_fcn(){
    // do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
    if (do_execute_main_task) do_reset_all_once = true;
}

