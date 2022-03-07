// Include Libraries
#include "Arduino.h"
#include "ACS712.h"


// Pin Definitions
#define offsetVoltage  2.39280758 // for ACS712 sensor
#define Sensitivity 0.185
#define solar_current_sense A3

float solar_current;


// define vars for testing menu


// Setup the essentials for your circuit to work. It runs first every time your circuit is powered with electricity.
void setup() 
{
    // Setup Serial which is useful for debugging
    // Use the Serial Monitor to view printed messages
    Serial.begin(9600);
    while (!Serial) ; // wait for serial port to connect. Needed for native USB
   
    //Manually calibarte the ACS712 current sensor.
    //Connet the ACS to your board, but do not connect the current sensing side.
    //Follow serial monitor instructions. This needs be done one time only.
    
}

// Main logic of your circuit. It defines the interaction between the components you selected. After setup, it runs over and over again, in an eternal loop.
void loop() 
{    
    solar_current = (((analogRead(solar_current_sense)*0.00488758) - offsetVoltage) / Sensitivity );
    Serial.println(analogRead(solar_current_sense));
    Serial.println(solar_current);
    delay(1000); 
    
}
