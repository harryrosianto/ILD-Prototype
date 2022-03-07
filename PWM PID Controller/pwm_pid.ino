
   // Arduino pins Connections----------------------------------------------------------------------------
  
    // A0 - Voltage divider to measure solar panel voltage
    // A1 - Voltage divider to measure battery voltage
    // A2 - ACS712 to monitor load current
    // A3 - ACS712 to monitor solar current
    // A4 - LCD SDA 
    // A5 - LCD SCL
  
    // D0- Not used1
    // D1- Not used
    // D2 - Control Load MOSFET Q2 
    // D3 - PWM Output to control MOSFET Q1 
    // D4 - Not Used
    // D5 - Battery Red LED       
    // D6 - Battery Green LED 
    // D7 - Battery Blue LED 
    // D8 - Load Red Led 
    // D9 - Load Green Led  
    // D10- Solar Red LED
    // D11- Solar Green LED  
    // D12- DS18B20 Temp. Sensor
    // D13-not used  


    // Include Libraries 
    
    #include <OneWire.h>
    #include <DallasTemperature.h>
    #include <LiquidCrystal_I2C.h>
    #include <PID_v1.h>
    #include <Wire.h>
    
    // Definitions :
    
    #define solar_volt_sense A0        // defining the analog pin A0 to read solar panel Voltage
    #define bat_volt_sense A1          //defining the analog pin A1 to read battery voltage
    #define load_current_sense A2      //defining the Analog pin A2 to measure load current
    #define solar_current_sense A3     //defining the Analog pin A2 to measure load current
    #define AVG_NUM 10                // number of iterations of the adc routine to average the adc readings
    #define ONE_WIRE_BUS 12           // Data wire of DS18B20 temp sensor is plugged into pin 3 on the Arduino
    #define pwm_pin 3                 // defining digital pin D3 to drive the main MOSFET Q1 @ 1KHz
    #define load_pin 2                // Defining load control pin to drive MOSFET Q2
    
    //Defining led pins for indication
    
    #define bat_low_led 5  
    #define bat_normal_led 6
    #define bat_full_led 7
    #define load_red_led 8 
    #define load_green_led 9 
    #define solar_red_led 10
    #define solar_green_led 11 
      
    #define BULK_CHRARGE_SP 14.4
    #define FLOAT_CHARGE_SP 13.5
    #define Charge_Restart_SP 13.2
    
    #define MIN_SOLAR_VOLT  10
    #define LVD 11.5
    #define LVR 12.5
    #define ABSORPTION_LIMIT 3600000    // 1 hour in milliseconds
    #define NIGHT_TIME 3600000          // 1 hour in milliseconds
    #define CHARGER_RESTART_TIME 600000 // 10 mins in milliseconds
    #define EVENING_TIME 300000         // 5mins in milliseconds
    #define MORNING_TIME 180000         // 3mins in milliseconds

    
    #define offsetVoltage  2.5 // for ACS712 sensor
    #define Sensitivity 0.100 // 100mV/A for ACS712-20A variant  

    //--------------------------------------------------------------------------------------------------------------------------
///////////////////////DECLARATION OF ALL BIT MAP ARRAY FOR FONTS////////////////////////////////////////////////////////////////
//--------------------------------------------------------------------------------------------------------------------------

      byte solar[8] = //icon for solar panel
      {
        0b11111,0b10101,0b11111,0b10101,0b11111,0b10101,0b11111,0b00000
      };
      byte battery[8] =  //icon for battery
      {
        0b01110,0b11011,0b10001,0b10001,0b10001,0b10001,0b10001,0b11111
      };
      
      byte energy[8] =  // icon for power
      {
        0b00010,0b00100,0b01000,0b11111,0b00010,0b00100,0b01000,0b00000
      };
      /*byte alarm[8] =  // icon for alarm
      {
       0b00000,0b00100,0b01110,0b01110,0b01110,0b11111,0b00000,0b00100
      };*/
      byte temp[8] = //icon for termometer
      {
       0b00100,0b01010,0b01010,0b01110,0b01110,0b11111,0b11111,0b01110
      };

      byte charge[8] = // icon for battery charge
      {
        0b01010,0b11111,0b10001,0b10001,0b10001,0b01110,0b00100,0b00100,
      };
      byte not_charge[8]=
      {
        0b00000,0b10001,0b01010,0b00100,0b01010,0b10001,0b00000,0b00000,
      };

 //---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  
  
    //Declaring the global variables
  
    float solar_volt;                      // solar panel voltage 
    float bat_volt;                        // battery voltage 
    float load_current;
    float solar_current;
    float temperature;                            // temperature
    float temp_change;                     // temperature difference between current and refrence ( 25 degC)
    float pwm_duty;                          // PWM Duty Cycle (0 to 1024)
    float bulk_charge_sp;                  // Bulk charging set point
    float error=0;                           // calculate the difference between battery voltage and bulk charge set point
    float float_charge_sp;                 // Float charging set point
    int load_status; // 0-off, 1- on 
    const int bat_type;                  //Flooded=0,AGM=1 

    unsigned long absorption_time;         // to keep track of today's time in absorption state
    unsigned long charger_millis;           // to keep track of time for charger
    unsigned long restart_time;
    unsigned long morn_timer;
    unsigned long even_timer;
    unsigned long load_millis;
    
    float load_watts;
    float load_wattHours;
    float solar_watts;
    float solar_wattHours;
    unsigned long time; 
    unsigned long last_time;
    unsigned long current_time;

    // Variables for PID Controller
    //Define Variables we'll be connecting to 
    double Setpoint, Input, Output;

  
    OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices
    DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature sensor 

    //Specify the links and initial tuning parameters
    PID myPID(&Input, &Output, &Setpoint,2,5,1, DIRECT); // Here Kp= 2 , Ki= 5 and Kd=1 //  aggKp=4, aggKi=0.2, aggKd=1;

   // Set the pins on the I2C chip used for LCD connections:
   //                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
   LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address // In my case 0x27
  
   enum charger_state {off, bulk,absorption,Float} charger_state ;  
   
//---------------------------------------- Set Up Function----------------------------------------------------------------------------------------------------------------------------------

    void setup() {
    
    Serial.begin(9600);                  // open the serial port at 9600 bps:
    pinMode(pwm_pin,OUTPUT);
    pinMode(load_pin,OUTPUT);
    pinMode(bat_low_led,OUTPUT);
    pinMode(bat_normal_led,OUTPUT);
    pinMode(bat_full_led,OUTPUT);
    pinMode(load_red_led,OUTPUT);
    pinMode(load_green_led,OUTPUT);
    pinMode(solar_red_led,OUTPUT);
    pinMode(solar_green_led,OUTPUT);
    
    TCCR2B = TCCR2B & B11111000 | 0x03;    // pin 3 PWM frequency of 980.39 Hz
    sensors.begin();

    //initialize the variables we're linked to
    Input = 12;
    Setpoint = 14.5;
    //turn the PID on
    myPID.SetMode(AUTOMATIC);
    
    //sensors.requestTemperatures();  // get temperature readings 
    //temperature = sensors.getTempCByIndex(0) ;    // 0 refers to the first IC on the wire 
    //get_setpoint();                      // get the temperature compensated charging set point
    
    //while ((temperature > 50 ) || (temperature < -20 ) || (bat_volt < 9)) // If the temperature is beyond the permisible limit or battery is damaged/disconnected 
    //{
    // Serial.println(" Error "); 
    //} 
    charger_state = off;          // start the charger from off
    charger_millis = millis() ;   // initialise the local clock
    load_millis =millis();
    

    lcd.begin(16,4);   // initialize the lcd for 16 chars 2 lines, turn on backlight
    lcd.clear();
    lcd.backlight(); // finish with backlight on  
    lcd.createChar(1,solar);
    lcd.createChar(2, battery);
    lcd.createChar(3, energy);
    //lcd.createChar(4,alarm);
    lcd.createChar(5,temp);
    lcd.createChar(6,charge);
    lcd.createChar(7,not_charge);
    delay(1000);                         // pause to properly start up the Arduino 
    }
  
  //-----------------------------------------------------------Main Loop---------------------------------------------------

    void loop() 
    {
    read_sensors_data();         // Reading voltage temp from the sensors
    get_setpoint();              // Reading temp compensated charging set point
    run_charger();
    load_control();
    power();  
    led_indication();
    lcd_display();
    serial_print();
      
   }
  //----------------------------------------------------------------Function Definition----------------------------------------
  //---------------------------------------------------------------------------------------------------------------------------
  ////////////////////////////////////READING DIFFERENT SENSORS DATA //////////////////////////////////////////////////////////
  //---------------------------------------------------------------------------------------------------------------------------
  /* 
     This function read the voltage dividers and temperature sensor (DS18B20 probe),gives row adc values in between 0-1023
     Then adc value is calibrated to get the actual voltages and currents
   */
  //------------------------------------------------------------------------------------------------------
////////////////// READS AND AVERAGES THE ANALOG INPUTS (SOLRAR VOLTAGE,BATTERY VOLTAGE)////////////////
//------------------------------------------------------------------------------------------------------

    int read_adc(int adc_parameter)
    {
      int sum = 0;
      int sample ;
      for (int i=0; i<AVG_NUM; i++) 
      { 
      //  analogWrite(pwm_pin,0);                 // disconnect battery to read accurate battery voltage                                    // loop through reading raw adc values AVG_NUM number of times  
        sample = analogRead(adc_parameter);    // read the input pin  
        sum += sample;                        // store sum for averaging
        delay(1);              // pauses for 1 milli seconds  
      }
       
      return(sum / AVG_NUM);                // divide sum by AVG_NUM to get average and return it
    }


//-------------------------------------------------------------------------------------------------------------
////////////////////////////////////READ THE DATA//////////////////////////////////////////////////////////////
//-------------------------------------------------------------------------------------------------------------
    void read_sensors_data(void){
    //--------------------------------Reading Voltages---------------
    
    //5V = ADC value 1024 => 1 ADC value = (5/1023)Volt= 0.00488758Volt
    // Vout=Vin*R2/(R1+R2) => Vin = Vout*(R1+R2)/R2   R1=100 and R2=20
    
     solar_volt = read_adc(solar_volt_sense)*0.00488758*(120/20);
     bat_volt   = read_adc(bat_volt_sense)*0.00488758*(120/20);

   //--------------------------------Reading Currents---------------
     
     load_current = (offsetVoltage - (analogRead(load_current_sense)*0.00488758))/0.185; 
     solar_current = (offsetVoltage - (analogRead(solar_current_sense)*0.00488758))/Sensitivity; 
      if (load_current <0)
     {
      load_current = 0;
     }
     if (solar_current <0)
     {
      solar_current = 0;
     }
   //--------------------------------Reading Temperature---------------

     sensors.requestTemperatures();  // get temperature readings 
     temperature = sensors.getTempCByIndex(0) ;    // 0 refers to the first IC on the wire 
     }
     
    //---------------------------------------------------------------------------------------------------------------------------
    ////////////////////////////////////CHARGE SET POINT ///////////////////////////////////////////////////////////////////////
    //---------------------------------------------------------------------------------------------------------------------------
    // temperature compensation = -20mv/degC
    // If temperature is above the room temp ;Charge set point should reduced
    // If temperature is bellow the room temp ;Charge set point should increased
     
      void get_setpoint(void)
      {
        
          read_sensors_data();         // for measuring voltage and currents from sensors
          temp_change =temperature-25.0;      // 25deg cel is taken as standard room temperature ( STC)
          
          // for floaded battery
          if (bat_type == 0){
           bulk_charge_sp  = BULK_CHRARGE_SP-(0.020*temp_change) ;
           float_charge_sp = FLOAT_CHARGE_SP-(0.020*temp_change) ;
          }
          // for AGM battery // set point is lowered to avoid excess gassing
          else{
            bulk_charge_sp =(BULK_CHRARGE_SP-0.2)-(0.020*temp_change) ;
            float_charge_sp=(FLOAT_CHARGE_SP-0.2)-(0.020*temp_change) ;
          }
        
       }
       
     
      //---------------------------------------------------------------------------------------------------------------------------
    //////////////////////////////////// MAIN CHARGING CYCLE ///////////////////////////////////////////////////////////////////////
    //---------------------------------------------------------------------------------------------------------------------------
   
  
    void run_charger(){
      
    switch (charger_state) {  
    
      case off : 
         if ( ( bat_volt < float_charge_sp) && ( solar_volt > ( bat_volt + 0.5) ) ){ 
              charger_millis = millis() ;
              charger_state = bulk ; 
          }
         else if ( ( bat_volt > float_charge_sp) && ( solar_volt > ( bat_volt + 0.5) ) && ( absorption_time > ABSORPTION_LIMIT ) ){ 
              charger_millis = millis() ;
              restart_time = 0 ;
              charger_state = Float ; 
          }
         else{
            if (millis() - charger_millis > NIGHT_TIME) absorption_time = 0 ; 
            pwm_duty = 0;
            analogWrite(pwm_pin, pwm_duty);  //generate PWM from D3 @ 0% duty // Shut down the charger 
          }
         break; // end of case off condition
      
      ///////////////////////////////////  STAGE-1 (BULK CHARGING)//////////////////////////////////////////////////////
      // During this stage the MOSFET is fully on by setting the duty cycle to 100%
      // Constant Current Charging
      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
     case bulk : 
         if (  solar_volt <  bat_volt ){ 
              charger_millis = millis() ;
              charger_state = off ; 
         }
         else if ((bat_volt > bulk_charge_sp) && ( solar_volt > ( bat_volt + 0.5) ) && ( absorption_time < ABSORPTION_LIMIT )){ 
              charger_millis = millis() ;
              charger_state = absorption ; 
         }
         else if ( ( bat_volt > float_charge_sp) && ( solar_volt > ( bat_volt + 0.5) ) && ( absorption_time > ABSORPTION_LIMIT )){ 
              charger_millis = millis() ;
              restart_time = 0 ;
              charger_state = Float ; 
         }
         else{
            pwm_duty = 255;
            analogWrite(pwm_pin,pwm_duty);   //generate PWM from D3 @ 100% duty // MOSFET Q1 is ON 
            if( bat_volt > bulk_charge_sp )
             {
              charger_state = absorption ;  
             }
         }
         break; // end of case bulk condition
      

    ///////////////////////////////////  STAGE-2 (ABSORPTION CHARGING)//////////////////////////////////////////////////////
    // During this stage the MOSFET is partially on by setting the duty cycle to between 0% and 100%
    // Constant voltage
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
    case absorption : 
         if (  solar_volt <  bat_volt ){ 
              charger_millis = millis() ;
              charger_state = off ; 
         }
         else if ( ( bat_volt > float_charge_sp) && ( solar_volt > ( bat_volt + 0.5) ) && ( absorption_time > ABSORPTION_LIMIT )){ 
              charger_millis = millis() ;
              charger_state = Float ; 
         }
         else{
           //  increment absorption timer and test for duration in absorption state
           absorption_time = absorption_time + millis() - charger_millis ;
           charger_millis = millis();
           
          /* error  = (bat_volt - bulk_charge_sp);      // duty cycle reduced when the battery voltage approaches the charge set point and vice versa
           if (abs(error) < 0.3){
             pwm_duty = pwm_duty - error*25.5;
            }
           else{
             pwm_duty = pwm_duty - error*255;
             }
             */

              Input = bat_volt;
              Setpoint = bulk_charge_sp;
              myPID.Compute();  // Compute PID Output 
              pwm_duty = Output*5 ; // Output = kp * error + ki * errSum + kd * dErr  
             if(pwm_duty < 0)pwm_duty=0;
             if(pwm_duty >255)pwm_duty=255;
             analogWrite(pwm_pin, pwm_duty); 
           
          }
        break; // end of case absorption condition
     
  
  

     ///////////////////////////////////  STAGE-3 (FLOAT CHARGING)//////////////////////////////////////////////////////
     // During this stage the MOSFET is partially on by setting the duty cycle between 0% and 100%
     // Constant Voltage Charging
     /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
    case Float : 
       if (  solar_volt <  bat_volt ){ 
            charger_millis = millis() ;
            charger_state = off ; 
        }
       else if ( ( bat_volt < Charge_Restart_SP) && ( solar_volt > ( bat_volt + 0.5) ) && (restart_time > CHARGER_RESTART_TIME)){ 
            charger_millis = millis() ;
            charger_state = bulk ; 
        }
       else if ( ( bat_volt >float_charge_sp)  && ( solar_volt > ( bat_volt + 0.5) ) && ( absorption_time < ABSORPTION_LIMIT )){ 
            charger_millis = millis() ;
            charger_state = absorption ; 
        }
       else{
           if (bat_volt > float_charge_sp){
           pwm_duty--;
           if(pwm_duty <0)pwm_duty=0;
           analogWrite(pwm_pin, pwm_duty);
           } 
           else{
           pwm_duty = 12.75; // setting duty cycle = 5% for trickle charge
          analogWrite(pwm_pin, pwm_duty);  //generate PWM from D3 @ 5% duty // Q1 is driving @ 5% duty cycle
           }
           if ( bat_volt < Charge_Restart_SP){
            restart_time = restart_time + millis() - charger_millis ;
            charger_millis = millis(); 
          }
       }
      break; // end of case float condition
      }
     } // end of run_charger

    //------------------------------------------------------------------------------------------------------------
    /////////////////////////////////POWER AND ENERGY CALCULATION //////////////////////////////////////////////
    //------------------------------------------------------------------------------------------------------------
    void power(void){
    last_time = current_time;
    current_time = millis();
    load_watts = load_current * bat_volt; //load Watts now
    solar_watts = solar_current * solar_volt; //solar Watts now
    load_wattHours = load_wattHours +  load_watts*(( current_time -last_time) /3600000.0) ; // calculating energy in Watt-Hour
    solar_wattHours = solar_wattHours+  solar_watts*(( current_time -last_time) /3600000.0) ; // calculating energy in Watt-Hou
    }

     //---------------------------------------------------------------------------------------------------------------------------
     //////////////////////////////////// LOAD CONTROL ///////////////////////////////////////////////////////////////////////
     //---------------------------------------------------------------------------------------------------------------------------

      void load_control(){
        
    read_sensors_data();               // for measuring voltages sensors
    if ( (solar_volt < 8.0) && ( bat_volt > LVD ) ){
     //msec = millis();
    //evening_time =  msec - last_msec; //Calculate how long has past since last call of this function
    //Serial.print("Evening Time:");
    //Serial.println(evening_time/60000);
    // if (((evening_time) > 1000) && ( bat_volt > LVD )){ // check if the solar panel voltage falls below 8V for more than 5mins and battery voltage is above LVD
     
    load_status = 1;
    digitalWrite(load_pin,HIGH);  // load will turn on during evening
    //Serial.println("LOAD is ON ");
    // }
    //last_msec = millis(); //Store 'now' for next time
    }
    else if ( solar_volt > MIN_SOLAR_VOLT){
    //unsigned long morning_time=millis();
    //Serial.print("Morning Time:");
    //Serial.println(morning_time/60000);
    //if ((millis()- morning_time) > 1000){ // check if the solar panel voltage rise above minimum solar volt ( 10V ) for more than 53mins
    load_status = 0;
    digitalWrite(load_pin,LOW); // load will turn off during morning
    //Serial.println("LOAD is OFF ");
   //  }
    }
      }
     
      //---------------------------------------------------------------------------------------------------------------------------
     //////////////////////////////////// LED INDICATION ///////////////////////////////////////////////////////////////////////
     ///////////This function display the current state of charge of the battery,Charging status and load status via LED////////
     //--------------------------------------------------------------------------------------------------------------------------
     
     void led_indication(void){

    //Solar leds indication
    //----------------------------------------
    
    if (solar_volt > bat_volt)
      {  
          digitalWrite(solar_red_led,LOW);
          digitalWrite(solar_green_led,HIGH);  // Sun Light is available and charger is ready for charging
        //  Serial.println("SOL_GREEN_LED ON");
      } 
      else 
      {
          digitalWrite(solar_green_led,LOW);
          digitalWrite(solar_red_led,HIGH);  //Sun light is not available for charging
        //  Serial.println("SOL_RED_LED ON");
      } 

  //Battery leds indication
  //---------------------------------------------
  
   if(bat_volt > LVR)
  {   
      leds_off_all();
      digitalWrite(bat_normal_led,HIGH);  // battery voltage is healthy
  } 
  else if(bat_volt > float_charge_sp ) 
  {
      leds_off_all();
      digitalWrite(bat_full_led,HIGH);  //battery is fully charged
  }
   else
  {
      leds_off_all();
      digitalWrite(bat_low_led,HIGH);  // battery voltage low
  }

   //Load leds indication
    
    if(load_status==1)
    {
      digitalWrite(load_red_led, LOW);
      digitalWrite(load_green_led ,HIGH);
    }
    else if(load_status==0)
    {
      digitalWrite(load_green_led, LOW); 
      digitalWrite(load_red_led ,HIGH);
    }
  } 
   //------------------------------------------------------------------------------------------------------
//////////////////////// TURN OFF ALL THE LED///////////////////////////////////////////////////////////
//------------------------------------------------------------------------------------------------------
void leds_off_all(void)
{ 
  
  digitalWrite(bat_low_led,LOW);
  digitalWrite(bat_normal_led,LOW);
  digitalWrite(bat_full_led,LOW);
   
}
      
     //---------------------------------------------------------------------------------------------------------------------------
     //////////////////////////////////// SERIAL PRINTING  ///////////////////////////////////////////////////////////////////////
     //--------------------------------------------------------------------------------------------------------------------------
     
    void serial_print(){
    //Serial.print("Temp: " );
    //Serial.print ( temperature);
    //Serial.print (",");
    //Serial.print("     Bulk Voltage SP: "); 
    Serial.print(bulk_charge_sp);
    Serial.print(" ");
    //Serial.print (",");
    //Serial.print("     Float Voltage SP: ");
    Serial.print(float_charge_sp);
    Serial.print(" ");
    //Serial.print (","); 
    //Serial.print("     Solar Volt: ");  
    //Serial.print(solar_volt);
    //Serial.print (",");
    //Serial.print("     Solar Current: ");  
    //Serial.print(solar_current);
    //Serial.print (",");
    //Serial.print("     Bat Volt: "); 
    Serial.print(bat_volt);
    Serial.print(" ");
    //Serial.print (",");
    //Serial.print("     Load Current: ");  
    //Serial.print(load_current);
    //Serial.print("     PWM : ");
    Serial.println((pwm_duty /255 )*100);  // convert a number between 0 -1024  to  0-1024 then convert float to intiger 
    //Serial.print (",");
    //Serial.print("%");
    //Serial.print("   Charge : ");
    //if (charger_state == off) Serial.print("off");
    //else if (charger_state == bulk) Serial.print("bulk");
    //else if (charger_state == absorption) Serial.print("absorption");
    //else if (charger_state == Float) Serial.print("Float");
    //Serial.print (",");
    //Serial.print("     Absorp time:");
    //Serial.print(absorption_time);
    //Serial.print (",");
    //Serial.print("     Restart time:");
    //Serial.print(restart_time);
    //Serial.println(",");
    
    //Serial.println("************************************");
    //delay(100);
    }

    //------------------------------------------------------------------------------------------------------
//////////////////////// LCD DISPLAY///////////////////////////////////////////////////////////////////
//------------------------------------------------------------------------------------------------------
void lcd_display()
{
 // Display Solar Panel Parameters
 lcd.setCursor(0, 0);
 lcd.write(1);
 lcd.setCursor(2, 0);
 lcd.print(solar_volt,1);
 lcd.print("V");
 lcd.setCursor(7, 0);
 lcd.print(solar_current,1);
 lcd.print("A");
 lcd.setCursor(12,0);
 lcd.print(solar_watts,1);
 lcd.print("W");

 // Display Battery Parameters
 lcd.setCursor(0,1);
 lcd.write(2);
 lcd.setCursor(2, 1);
 lcd.print(bat_volt,1);
 lcd.print("V");
 lcd.setCursor(7, 1);
 lcd.write(5);
 lcd.setCursor(8, 1);
 lcd.print(temperature,0);
 lcd.write(0b11011111);
 //lcd.print("C");

 lcd.setCursor(12, 1);
 lcd.write(2);
 if((charger_state==1) | (charger_state== 2) | (charger_state== 3))
 {
 lcd.write(6);
 }
 else
 {
 lcd.write(7);
 }

 // Display Load Parameters

 lcd.setCursor(0,2);
 lcd.print("L");
 lcd.setCursor(2,2);
 lcd.print(load_current,1);
 lcd.print("A");
 lcd.setCursor(7,2);
 lcd.print(load_watts,1); 
 lcd.print("W");
 lcd.setCursor(12,2);
 if(load_status==1)
    {
      lcd.print("   "); // clear the OFF
      lcd.setCursor(12,2);
      lcd.print("ON"); 
    }
    else if(load_status==0)
    {
      lcd.print("OFF"); 
    }

 // Display Energy 
 lcd.setCursor(0,3);
 lcd.write(3);
 lcd.setCursor(2,3);
 lcd.print(solar_wattHours,1);
 lcd.print("WH");  
 lcd.setCursor(10,3);
 lcd.print(load_wattHours,1);
 lcd.print("WH");
  
}
  
