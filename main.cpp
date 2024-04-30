#include <mbed.h>
#include <cstdio>
#include <iostream>
#include "PM2_Drivers.h"
#include "PinNames.h"
#include "HX711.h"

# define M_PI 3.14159265358979323846                    // Number pi
# define MAX_VOLTS 12.0                                  // Maximum Voltage of Board [V]

// Runtime Variables ==================================================
bool do_execute_main_task = false;                      // Toggled via user button(blue), used to activate main Task
bool do_reset_all_once = false;                         // Toggled via user button(blue), used to reset certain variables once
bool do_read_offset = false;                            // Toggled via suer button, used to read offset exactly once
static int buttonCounter = 0;                           // Used to count number of button presses


// Load Cell Variables ==============================================
const float loadCellMaxWeight_kg = 0.1;                 // Maximum tolerable Weight on Load Cell
const float loadCellSensitivity_V_V = 0.0006;           // Load Cell sensitivity displayed usually in mV/V, here in V/V.
const float amplifierVoltage_V = 5;                     // Amplifier Input/Vcc value in Volts
const float amplifierGain = 128;                        // Amplifier Gain 
const float amplifierDrift_V = 0.0002;                  // Amplifier Drift/Offset
const float measurementInaccuracy = 1.0635;               // 100% plus offset of 6.35%   

float readout = 0;                  // Raw, unadjusted readout of gauge
float loadCellVoltage_mv = 0;       // Adjusted readout of gauge (read - zero value)
float weight_g = 0;                 // Conversion results in kg
float amplifierOffset = 0;          // Zero value


// Global Objects ===================================================
/* PinMap: 
PB_10 / D6 - SPI2_SCK pin used for load cell SCK
PB_4 / D5  - SPI1_MISO pin used for load cell data transfer from amplifier to controller

PB_5 / D4  - SPI1_MOSI
PA_9 / D8  - SPI2_SCK
PB_6 / D10 - UART1, CAN2
PA_7 / D11 - SPI1_MOSI
PA_6 / D12 - SPI1_MISO
PA5 / D13  - SPI1_SCK

*/
HX711 gauge(amplifierVoltage_V,PB_10,PB_4,amplifierGain);
DebounceIn user_button(PC_13);                          // create InterruptIn interface object to evaluate user button falling and rising edge (no blocking code in ISR)
DigitalOut user_led(LED1);

// Function Prototypes ==============================================
void user_button_pressed();                         // custom functions which gets executed when user button gets pressed
float voltToKilograms(float measuredVoltage, float maxLoad, float excitingVolt, float sensitivity); // Convert Volt output to Weight in kg


// main() runs in its own thread in the OS =========================
int main(){

    // Variables ======================================


    // attach button fall function to user button object, button has a pull-up resistor
    user_button.fall(&user_button_pressed);
    const int main_task_period_ms = 100;               // define main task period time in ms e.g. 50 ms -> main task runs 20 times per second
    Timer main_task_timer;                              // create Timer object which we use to run the main task every main_task_period_ms
   
    // Load Cell - Seup ===================================================
    gauge.powerUp();
    gauge.setGain();
    gauge.setDrift(amplifierDrift_V);

    // Code ===============================================================
    main_task_timer.start();
    while (true){

        user_led = !user_led;

        main_task_timer.reset();
        
        if(do_read_offset){
            do_read_offset = false;

            while(gauge.isReady() == false){}
            amplifierOffset = gauge.read();
            printf("\nOffset = %f",amplifierOffset);
        }

        if (do_execute_main_task){
            do_execute_main_task = false;

            if(gauge.isReady()){

                weight_g = 0;

                // for(int i = 0 ; i < 5 ; i++){
                    // while(gauge.isReady() == false){}
                    readout = gauge.read();
                    loadCellVoltage_mv = readout - amplifierOffset;
                    weight_g += voltToKilograms(loadCellVoltage_mv, loadCellMaxWeight_kg, amplifierVoltage_V, loadCellSensitivity_V_V);
                // }

                weight_g = weight_g * measurementInaccuracy;
                printf("\nLoadcell Readout: %f",readout);
                printf("\nAdjusted Readout: %f", loadCellVoltage_mv);
                printf("\nCurrent weight in grams: %f", weight_g);

            }

            else{
                printf("\nLoad Cell or Board Not Ready");
                if (do_reset_all_once){
                    do_reset_all_once = false;

                    weight_g = 0.0f;
                    loadCellVoltage_mv = 0.0f;
                    printf("\nReset");

                }          
            }
        }
        // End if do_execute_main_task

        // read timer and make the main thread sleep for the remaining time span (non blocking)
        int main_task_elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(main_task_timer.elapsed_time()).count();
        thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

float voltToKilograms(float measuredVoltage, float maxLoad, float excitingVolt, float sensitivity){
    float weight = 0;
    weight = measuredVoltage * maxLoad/(excitingVolt * sensitivity);
    return weight;
}

void user_button_pressed(){
    buttonCounter++;
    if(buttonCounter == 1){
       do_read_offset = true;
    }else{
        do_execute_main_task = !do_execute_main_task;   // do_execute_main_task if the button was pressed twice
        if (do_execute_main_task) do_reset_all_once = true;
    }
}

