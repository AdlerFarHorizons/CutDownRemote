Remote Cutdown Module

Placed between parachute and ballooon tether, cuts connection to the tether when certain conditions are met as determined by the Base Cutdown Module and triggered by wireless sigal. The remote module has a timer pre-set at launch that will trigger the cutdown if other criteria are not met or fail to get a trigger to the remote.

V5.00 Adds support for the BaseModule's GPS functions. It will ask
the user about the maximum altitude of the flight and supply this to
the BaseModule.

V3.11 Now No longer Sends the Query 'Q'

V3.01 Now avoids character conflict with base module, to allow
      the use of a third Xbee to program both the base module and 
      Cutdown arduino

V3.00 Some fixes for high XBee current and workarounds for
      lack of single point setup control in alpha base station.
- Put XBee to sleep when not needed. Required hardware change.
- Use variable LED flash count to signal ready for input from base
  allows setup without terminal connection.
- Programming mode until cutter cap is nearly charged.

Lou Nigra
Adler Planetarium - Far Horizons

Brendan Batliner and Milan Shah
Illinois Mathematics and Science Academy - SIR Program

Controls:
 Raw voltage tied to charger ON switch
 Programming mode when serial port is connected
 Timer activate/de-activate momentary contact switch

Indicators:
 LED no blink: Charging => Programming mode inhibited.
 LED slow blink: Armed => valid countdown value, ready to start. 
 LED fast blink: Active => Timer is counting down.
 LED long Off: Standby => Waiting for programming window to open.
 LED single three flashes: Programming mode => Window open for "non-D"
                         input to enter programming mode.

 The following sequence follows if a non-D character is entered while
 the window is open:

 LED single six flashes: Timer value: => Ready for timer value entry.
 LED three flashes: Confirmation: => Ready for y or n
 LED slow blink: Armed (as above)

State Machine:
 RESET
   -> TTY
 TTY
   timeout -> SLEEP
   valid value -> armed -> SLEEP
   non-valid value -> standby -> SLEEP
 SLEEP
   wakeUp!
     standby? => TTY
     active? 
       switch? -> armed => SLEEP
       !switch? -> timerUpdate
     armed?
       switch? -> active    

