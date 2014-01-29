#include <EEPROM.h>

#include <Narcoleptic.h>

/*
 * CutDownTimer
 *
 * V5.00 Adds support for the BaseModule's GPS functions. It will ask
 * the user about the maximum altitude of the flight and supply this to
 * the BaseModule.
 *
 * V3.11 Now No longer Sends the Query 'Q'
 *
 * V3.01 Now avoids character conflict with base module, to allow
 *       the use of a third Xbee to program both the base module and 
 *       Cutdown arduino
 *
 * V3.00 Some fixes for high XBee current and workarounds for
 *       lack of single point setup control in alpha base station.
 * - Put XBee to sleep when not needed. Required hardware change.
 * - Use variable LED flash count to signal ready for input from base
 *   allows setup without terminal connection.
 * - Programming mode until cutter cap is nearly charged.
 *
 * Lou Nigra
 * Adler Planetarium - Far Horizons
 *
 * Brendan Batliner and Milan Shah
 * Illinois Mathematics and Science Academy - SIR Program
 * 
 * Controls:
 *  Raw voltage tied to charger ON switch
 *  Programming mode when serial port is connected
 *  Timer activate/de-activate momentary contact switch
 *
 * Indicators:
 *  LED no blink: Charging => Programming mode inhibited.
 *  LED slow blink: Armed => valid countdown value, ready to start. 
 *  LED fast blink: Active => Timer is counting down.
 *  LED long Off: Standby => Waiting for programming window to open.
 *  LED single three flashes: Programming mode => Window open for "non-D"
 *                          input to enter programming mode.
 *
 *  The following sequence follows if a non-D character is entered while
 *  the window is open:
 *
 *  LED single six flashes: Timer value: => Ready for timer value entry.
 *  LED three flashes: Confirmation: => Ready for y or n
 *  LED slow blink: Armed (as above)
 *
 * State Machine:
 *  RESET
 *    -> TTY
 *  TTY
 *    timeout -> SLEEP
 *    valid value -> armed -> SLEEP
 *    non-valid value -> standby -> SLEEP
 *  SLEEP
 *    wakeUp!
 *      standby? => TTY
 *      active? 
 *        switch? -> armed => SLEEP
 *        !switch? -> timerUpdate
 *      armed?
 *        switch? -> active    
 */
// General constants 
float vRef = 3.3;
int cutPin = 10;
int vTempPin = 5;
int vBattPin = 4;
int pwrDnPin = 7;
int cutChgDisablePin = 8;
float vBattRange = vRef * 4.092;
int vCutCapPin = 3;
float vCutCapRange = vRef * 2.0;
int vBackupCapPin = 2;
float vBackupCapRange = vRef * 2.0;
int ledPin = 13;
int modePin = 9;
float timeOhFactor = 0.0333; //Empirical with 3 second activeSleepTime
int maxEepromAddr = 1023; //ATMega328
String dataValues = "2*(T+75)(C), vB*20(V), vC*20(V)";

// Variable declarations;
int cut;
int cutDelayMins;
int ttyPollTime;
int ttyWindowTimeSecs;
int standbySleepTime;
int armedSleepTime;
int activeSleepTime;
int sleepTime;
int sampleCount;
int sampleNum;
int sampleTime;
float cutTimerMins;
float maxAlt;
float maxRadius;
float vCharged;
boolean isCharged;
boolean ledState;
int ledFlashTime;
int dataSampleInterval;
int sensType;
boolean standby;
boolean active;
boolean switchArmed;
boolean chgEnable;
int eepromAddr;

void setup()
{
  pinMode(cutPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(modePin, INPUT_PULLUP );
  pinMode(pwrDnPin, OUTPUT);
  pinMode(cutChgDisablePin, OUTPUT);
  chgEnable = true;
  setCutChg( chgEnable );
  setCut( false );
  setLED( false );
  setPwrDown( true );
  sensType = 0; //LM60
  active = false;
  standby = true;
  cutDelayMins = 0;
  cutTimerMins = 0;
  ttyPollTime = 6000; //ms
  ttyWindowTimeSecs = 10; //secs
  standbySleepTime = 30000; //ms
  armedSleepTime = 12000; //ms
  activeSleepTime = 3000; //ms
  sampleTime = 60; //ms
  float temp = ( 1000.0 * sampleTime ) / ( 1.0 * activeSleepTime );
  sampleCount = (int)( 0.5 + temp);
  sampleNum = 1;
  ledFlashTime = 10; //ms
  switchArmed = false;
  vCharged = 0.0; //When used for testing purposes set this to 0.0
  Serial.begin(9600);
  Serial.flush();

}

void loop() // run over and over again
{ 
  if (!isCharged ) waitForCharge();
  
  setPwrDown(false);
  delay(100);
  if (standby) {
    flashLED( ledFlashTime, 3 );
    switchArmed = false;
    cutDelayMins = getTTY( ttyPollTime, ttyWindowTimeSecs );
    if (cutDelayMins <= 0) {
      // standby mode
      standby = true;
      sleepTime = standbySleepTime;
    } else {
      // armed mode
      standby = false;
      active = false;
      sleepTime = armedSleepTime;
    }
  } else {
    flashLED( ledFlashTime, 1 );
    // Check for mode change via switch
    if ( getModeSwitch() ) {
      Serial.println("mode switch press detected");
      Serial.flush();
      if (switchArmed) {
        active = true;
        switchArmed = false;
        Serial.println("Timer is now active.");
        Serial.println("");
        Serial.println( "Min, T(C), Vbat(V), Vcut(V), Vbck(V)");
        Serial.flush();
        Serial.write('X');
        Serial.print(cutDelayMins);
        Serial.print(",");
        Serial.print(maxAlt);
        Serial.print(",");
        Serial.print(maxRadius);
        Serial.flush();
        eepromAddr = 1;
        EEPROM.write( 0, eepromAddr ); //Initial eeprom address

      } else {
        switchArmed = true;
        Serial.println( "Armed: Keep holding for one more flash to activate." );
        Serial.flush();
      }
    } else {
      switchArmed = false;
    }

    // Take action depending on active/armed mode
    if ( active ) {
      sleepTime = activeSleepTime;
      boolean temp = updateTimer() || cutdownReceived();
      if ( temp && chgEnable ) {
        chgEnable = false; // This branch only once
        setCutChg( chgEnable ); // Disable cut cap charging if cut is imminent.
        delay(100);
      }
      //Serial.print("updateTimer:");Serial.println( temp );Serial.flush();
      setCut( temp );
      if (sampleNum >= sampleCount) {
        float temp = readTemp( vTempPin, 0 );
        float vBatt = vBattRange * analogRead( vBattPin ) / 1024.0;       
        float vCutCap = vCutCapRange * analogRead( vCutCapPin ) / 1024.0;       
        float vBackupCap = vBackupCapRange * analogRead( vBackupCapPin ) / 1024.0;
        Serial.print( cutTimerMins );Serial.print( ", ");       
        Serial.print( temp );Serial.print( ", ");
        Serial.print( vBatt );Serial.print( ", ");
        Serial.print( vCutCap );Serial.print( ", ");
        Serial.print( vBackupCap );
        temp = 2.0 * ( temp + 75.0 ); // Shift temperature range
        // Constrain readings to byte values
        if ( temp > 255 ) temp = 255; if ( temp < 0 ) temp = 0;
        vBatt /= 0.05; if ( vBatt > 255 ) vBatt = 255; if ( vBatt < 0  ) vBatt = 0;
        vCutCap /= 0.05;if ( vCutCap > 255 ) vCutCap = 255; if ( vCutCap < 0  ) vCutCap = 0;
        vBackupCap /= 0.05; if ( vBackupCap > 255 ) vBackupCap = 255; if ( vBackupCap < 0  ) vBackupCap = 0;
        
        // If out of eeprom, keep overwriting the last set of samples
        if ( eepromAddr > maxEepromAddr ) eepromAddr = ( maxEepromAddr - 3 );
        EEPROM.write( eepromAddr, byte( temp ) );
        eepromAddr +=1;
        EEPROM.write( eepromAddr, byte( vBatt ) );
        eepromAddr +=1;
        EEPROM.write( eepromAddr, byte( vCutCap ) );
        eepromAddr +=1;
        EEPROM.write( eepromAddr, byte( vBackupCap ) );
        EEPROM.write( 0, byte(eepromAddr) );
        Serial.println();
        //Serial.print("Addr:");Serial.println( eepromAddr );
        //Serial.print("Stored Addr:");Serial.println( EEPROM.read( 0 ) );
        Serial.flush();
        eepromAddr += 1;
        sampleNum = 0;
      }
      sampleNum += 1;
    } else {
      cutTimerMins = 0;
      sleepTime = armedSleepTime;
    }
  }
  Serial.flush();
  setPwrDown( true );
  Narcoleptic.delay( sleepTime );
}

void setPwrDown( boolean state ) {
  digitalWrite( pwrDnPin, state );
}

void setCut( boolean state ) {
  digitalWrite( cutPin, state );
}

void setCutChg( boolean state ) {
  digitalWrite( cutChgDisablePin, !state );
}

boolean updateTimer() {
  cutTimerMins += ( ( 1 + timeOhFactor ) * activeSleepTime / 60000.0 );
  //Serial.print("cutTimerMins:");Serial.print(cutTimerMins);
  //Serial.print(" ");Serial.println(int(cutTimerMins + 0.5));Serial.flush();
  return( int(cutTimerMins) == cutDelayMins );
}

boolean cutdownReceived() {
  boolean isReceived = false;
  /*
  int incomingByte = 65;
  //Serial.println("Before");
  //Serial.flush();
  if (Serial.available() > 0) {
    //Serial.println("After");
    //Serial.flush();
    incomingByte = Serial.read();
    Serial.print(incomingByte);
    Serial.flush();
    if (incomingByte == 'C') {
      isReceived = true;
      Serial.println("Cutdown command received.");
      Serial.flush();
    }
  }
  */
  /*
  Serial.write('Q'); //Send a query over the serial
  Serial.flush();
  */
  //delay(100); //wait 100 milliseconds for a response
  for (int i = 0; i < 10; i++) 
  {
    if (Serial.available() > 0)
    {
      int incomingByte = Serial.read();
      if (incomingByte == 'C')
      {
        isReceived = true;
        Serial.println("Cutdown command received.");
        Serial.flush();
      }
    }
    delay(5);
  }
  
  return isReceived;
}

void setLED( boolean state ) {
  if ( state ) {
    digitalWrite( ledPin, HIGH );
  } else {
    digitalWrite( ledPin, LOW );
  }
}

void flashLED( int flashTime, int numFlashes ) {
  for ( int i = 0 ; i < numFlashes ; i++ ) {
    digitalWrite( ledPin, HIGH );
    delay( flashTime );
    digitalWrite( ledPin, LOW );
    delay(100);
  }
  
}

boolean isActive() {
  return( active );
}

void setActive() {
  active = true;
}

boolean isStandby() {
  return( standby );
}

boolean getModeSwitch() {
  // Mode switch is active low
  return( digitalRead( modePin ) == LOW );
}

float readTemp( int pin, int sensType ) {
  // Temperature Sensor constants:
  //   0  LM60
  //   1  MAX6605
  //   2  TMP36
  int mVoltsAtRefTemp[] = { 424, 744, 750 };
  int refTempC[] = { 0, 0, 25 };
  float mVperDegC[] = { 6.25, 11.9, 10.0 };

  int reading = analogRead(vTempPin);
  float mVolts = reading * vRef / 1.024;

  return( ( mVolts - mVoltsAtRefTemp[sensType] ) / 
            ( mVperDegC[sensType] ) + 
            refTempC[sensType]);
  
}

// Starts serial interface and waits for tty activity for a while to start
//  a dialog to get a new value for the global cutDelayMins cutdown time.
//  If no input, returns with value unmodified.
int getTTY( int pollTimeMs, int windowTimeSecs ) {
  int rcvdBytes[4];
  int rcvdBytesCnt;
  int timeDelay;
  float deadTime;
  boolean done;
  int inputByte;
  int timeOutCnt = 0;

  // Clear the input buffer
  while ( Serial.available() > 0 ) {
    Serial.read();
  }
  Serial.println(""); 
  Serial.println("Enter D to dump EEProm data, ");
  Serial.println("any other key to set up the timer...");
  Serial.flush();
  
  // Wait for an input, but only for windowTimeSecs
  deadTime = 0;
  done = false;
  while( Serial.available() <= 0 && !done ) {
    delay(pollTimeMs);
    flashLED(ledFlashTime, 3);
    delay(100);
    deadTime += pollTimeMs/ 1000.0;
    //Serial.print("deadTime=");Serial.println(deadTime);
    if ( deadTime > windowTimeSecs ) {
      Serial.println( "" );
      Serial.print( "Going back to sleep. Back in " );
      Serial.print( standbySleepTime / 1000 );Serial.println( " sec" );
      //Serial.flush();
      timeDelay = 0;
      done = true;
   }
  }
  
  if ( ( inputByte = Serial.read() ) == 68 ) {
    dumpData();
    return(0);
  } 
  while ( !done ) {
    // Clear the input buffer
    while ( Serial.available() > 0 ) {
      Serial.read();
    }
    Serial.println("");
    Serial.println("Ready for timer setting.");
    Serial.println("The timer won't start until MODE button" );
    Serial.println("pressed and held for two LED flashes.");
    Serial.println("");
    
    promptUserForData(&maxAlt, "max altitude", "feet");
    
    promptUserForData(&maxRadius, "max radius", "miles");
     
    Serial.print("Enter timer duration in minutes: ");
    done = false;
    rcvdBytesCnt = 0;
    boolean typing = true;
    flashLED( ledFlashTime, 6 );
    while ( typing ) {
      while ( ( inputByte = Serial.read() ) < 0 ) {
        //Serial.println(".");
        delay(100);
      }
      if ( inputByte <= 57 && inputByte >= 48) {
        if ( rcvdBytesCnt < 8 ) {
          rcvdBytes[rcvdBytesCnt] = inputByte;
          Serial.write( inputByte );
          rcvdBytesCnt += 1;
        }
      } else {
        if ( inputByte == 13 ) {
          typing = false;
          Serial.println("");
          timeDelay = 0;
          int weight = 1;
          for ( int i = 0; i < rcvdBytesCnt; i++ ) {
            timeDelay += ( rcvdBytes[rcvdBytesCnt-i-1] - 48 ) * weight;
            weight *= 10;
          }
          int hrs = (int)( timeDelay / 60.0 );
          int mins = timeDelay - 60 * hrs;
          Serial.print("You've entered ");
          Serial.print(timeDelay);
          Serial.print(" minutes, or ");
          Serial.print( hrs ); Serial.print( " hrs, " ); Serial.print( mins );
          Serial.println( " min." );
          int response = 0;
          Serial.print("Are you sure (y/n)? " );
          Serial.flush();
          flashLED( ledFlashTime, 3 );
          while ( response != 110 && response != 121 ) {
            response = Serial.read();
            //Serial.print(response);
          }
          Serial.write( response );Serial.println("");Serial.flush();
          if ( response == 121 ) {
            Serial.println( "" );
            Serial.println( "Timer now armed. Press and hold MODE button");
            Serial.print( "for two LED flashes (up to ");
            int time = (int)( 0.5 + 2.0 * armedSleepTime / 1000.0 );
            Serial.print( time );Serial.println( " secs) to start it.");
            Serial.flush();
            flashLED( ledFlashTime, 2 );
            done = true; 
          }
        }
      }
    }
  }
  return(timeDelay);  
}

void dumpData() {
  Serial.flush();
  Serial.println( dataValues );
  int lastAddr = int(EEPROM.read(0));
  int addr = 1;
  while ( addr <= lastAddr ) {
    Serial.print(EEPROM.read(addr));Serial.print(", ");
    Serial.print(EEPROM.read(addr+1));Serial.print(", ");
    Serial.print(EEPROM.read(addr+2));Serial.print(", ");
    Serial.println(EEPROM.read(addr+3));
    addr += 4;
  }
  Serial.println("End");
  Serial.flush();
}

void waitForCharge() {
  float vCutCap = vCutCap = vCutCapRange * analogRead( vCutCapPin ) / 1024.0;
  while ( vCutCap <= vCharged ) {
    Serial.print("Vcut = ");Serial.print(vCutCap);
    Serial.println("V. Waiting for full charge...");
    Serial.flush();
    delay(1000);
    flashLED( ledFlashTime, 1 );
    vCutCap = vCutCap = vCutCapRange * analogRead( vCutCapPin ) / 1024.0; 
  }
  isCharged = true; 
}

void promptUserForData(float * data, String dataName, String unit)
{
  Serial.print("Enter " + dataName + " of flight, in " + unit + ":");
    flashLED( ledFlashTime, 6 );
    *data = 0;
    boolean typing = true;
    while (typing) {
      while (Serial.available() > 0) {
        *data = Serial.parseFloat();
        Serial.println(*data); Serial.flush();
        if (*data != 0) {
          Serial.print("You've entered "); Serial.print(*data); Serial.println(" " + unit + " as the " + dataName + " for this flight."); Serial.print("Is this correct? (y/n)");
          Serial.flush();
          flashLED( ledFlashTime, 3 );
          int response = 0;
          while (response != 'y' && response != 'n') {
            response = Serial.read();
          }
          Serial.println(response);
          if (response == 'y') {
            Serial.println("Confirmed.");
            Serial.print(*data); Serial.println(" " + unit + "."); Serial.println("");
            Serial.flush();
            typing = false;
            while (Serial.available() > 0)
              Serial.read();
          }
          else {
            Serial.print("Enter " + dataName + " of flight, in " + unit);
            Serial.flush();
            while (Serial.available() > 0)
              Serial.read();
            *data = 0;
          }
        }
        else {
          Serial.println("Please enter a valid number.");
          Serial.flush();
        }
      }
    }
}

  

