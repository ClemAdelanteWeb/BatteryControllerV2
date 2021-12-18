/*
  BlueSeaLatchingRelay.h - Library for controlling Latching Relays on a Marine or Home battery system.
  Created by Clément Lambert, 6th march 2018.
  Released into the public domain.
*/
#include <Arduino.h>

#ifndef BlueSeaLatchingRelay_H
#define BlueSeaLatchingRelay_H



class BlueSeaLatchingRelay {
  
  public :
    BlueSeaLatchingRelay();
    byte state = 0;
    byte openPin;
    byte closePin;
    byte statePin;
    byte isReadyToOpen = 0;
    byte isReadyToClose = 0;
    byte isForceToOpen = 0;
    byte isForceToClose = 0;

    // Delay time before opening the Charge Relay™™
    // value in ms
    unsigned int delayBeforeOpening  = 0;

    // Begining time for the delay before opening
    // works only for setReadyToOpen, not for forceToOpen
    unsigned long delayBeforeOpeningStartMillis = 0;

    bool waitingForOpening = false;
    
    // Name of the Relay, usefull for logs
    String name;
    
    // Opening or closing time in ms
    int latchingDurationTime = 600;
    
    // Relay opened, no current flowing
    const byte RELAY_OPEN = 0;
    
    // Relay closed, current is flowing
    const byte RELAY_CLOSE = 1;
    
    void setClosed();
    
    void setOpened();
    
    void setReadyToClose();
    
    void setReadyToOpen();

    void forceToOpen();
    
    void forceToClose();
    
    void applyReadyActions();
        
    // Determine the beginning of a new cycle (run)
    // reset all "ready" states
    void startCycle();
  
    byte getState();

    static boolean isAnalogPin(uint8_t pin);
    
    static void customWrite(uint8_t pin, uint8_t val);

};

#endif

