//Tutorial ↓
/************************
* IC List:
* ATtiny85
* GPS , NEO-M6N(65.8 ma)/ATGM336H(33.1 ma , recommend)/TAU1201(102.3 ma)
#########################
Setting need:
Install ATtiny core use "https://github.com/SpenceKonde/ATTinyCore"
Use "ATtiny25/45/85(No Bootloader)" board
select"ATtiny85" and "16 mhz(PLL)"
Programmer use "Arduino as ISP"
And click "Burn Bootloader"(To set 16 mhz PLL) <This is very important!>
Next, Program your ATtiny85 and enjoy!
#########################
Wireing:
---------------
   >>>OLED<<<
VCC->PIN8
GND->PIN4
SCL->PIN7
SDA->PIN5
---------------
   >>>GPS<<<  
VCC->PIN6(My GPS just 33.1 ma,can no use MOSFIT)
GND->PIN4
TX ->PIN2
RX ->Not Used
---------------
   >>>BUTTON<<<
PIN->GND
OTHER PIN->PIN3
---------------
************************/
//This is open source and force free !
#include <TinyWireM.h>
#include <Tiny4kOLED.h>
#include <NeoSWSerial.h>
#include <string.h>
#include <stdio.h>
#include <avr/sleep.h>      // Deep sleep core library
#include <avr/interrupt.h>  // Interrupt control
#include <avr/wdt.h>        // Watchdog Timer (optional)
#include <avr/power.h>      // Used for shutting down peripherals
#include <avr/io.h>         // Used for register operations

// --- Time Zone Configuration ---
#define TIME_ZONE_OFFSET 8 // Example: +8 for CST/BJT

// --- OLED Settings ---
uint8_t width = 72;
uint8_t height = 40;

// --- GPS Module Settings ---
#define GPS_RX 3
#define GPS_TX -1 
// RX pin 3 corresponds to PB3/PCINT3
NeoSWSerial gpsSerial(GPS_RX, GPS_TX);

// --- Power and Control Pins ---
#define GPS_POWER_PIN 1      
#define SLEEP_BUTTON_PIN 4   // Connected to PB4 (PCINT4)

// --- Timing and Debounce ---
#define DEBOUNCE_DELAY 50      
#define SHORT_PRESS_MAX_TIME 500 

// --- VCC Update Rate ---
#define VCC_UPDATE_INTERVAL 2000 // Battery voltage updates every 2 seconds

// --- GPS Data Buffer & State ---
char nmeaSentence[82];
byte idx = 0;
char timeStr[9] = "08:00:00";
int currentSecond = -1;
volatile bool timeUpdated = false;
bool hasFix = false;

// --- Sleep State & Timers ---
volatile bool isLowPowerMode = false; 
volatile long buttonPressTime = 0; 
unsigned long lastVccUpdate = 0; 

// Function Declarations
void setGpsPower(bool on);
void setOledPower(bool on);
void parseGNRMC(char *sentence);
long readVccMillivolts();
void toggleLowPowerMode();


// --- ISRs ---
// WDT ISR (Empty, for safety)
ISR(WDT_vect) {
}


// --- Battery Voltage Measurement Function ---
/**
 * @brief Measures the VCC (chip supply voltage) using the internal 1.1V reference.
 * @return long VCC voltage in millivolts (mV).
 */
long readVccMillivolts() {
    // Set ADC to measure the 1.1V reference (channel 12, 1100b)
    ADMUX = _BV(MUX3) | _BV(MUX2); 

    ADCSRA |= _BV(ADSC); // Start ADC conversion
    while (ADCSRA & _BV(ADSC)); // Wait for conversion to finish

    int result = ADC; // Read ADC 10-bit result

    // VCC (mV) = (1100 * 1024) / ADC_Result
    return 1125376L / result; 
}


// --- GPS Power Control Function ---
void setGpsPower(bool on) {
    if (on) {
        digitalWrite(GPS_POWER_PIN, HIGH);
        // Start serial communication only if not in low power mode
        if (!isLowPowerMode) {
             gpsSerial.begin(9600); 
        }
    } else {
        digitalWrite(GPS_POWER_PIN, LOW);
        // Stop serial, disabling PCINT3 interrupt used by NeoSWSerial
        gpsSerial.end(); 
    }
}

// --- OLED Power Control Function ---
void setOledPower(bool on) {
    if (on) {
        oled.on();
        timeUpdated = true; 
    } else {
        oled.off();
    }
}


// --- Toggle Low Power/Deep Sleep Mode ---
void toggleLowPowerMode() {
    isLowPowerMode = !isLowPowerMode;
    
    if (isLowPowerMode) {
        // --- Enter Deep Sleep Mode ---
        
        // 1. Shut down high-power peripherals
        setOledPower(false); 
        setGpsPower(false); 
        oled.clear();
        
        // 2. Enable Pin Change Interrupt on the button pin (PCINT4) for wake-up
        GIMSK |= _BV(PCIE); 
        PCMSK |= _BV(PCINT4); 
        
        // 3. Set sleep mode
        set_sleep_mode(SLEEP_MODE_PWR_DOWN); 
        
        // 4. Go to sleep
        cli(); // Disable global interrupts
        sleep_enable(); 
        power_all_disable(); // Shut down clock/peripherals
        
        sei(); // Enable global interrupts (to allow PCINT to wake MCU)
        sleep_cpu(); // Zzzzz... 
        
        // --- Execution resumes here after wake-up ---
        sleep_disable(); 
        power_all_enable(); // Restore clock/peripherals
        PCMSK &= ~_BV(PCINT4); // Disable PCINT4 immediately
        
    } else {
        // --- Exit Low Power Mode / Resume Normal Operation ---
        
        setGpsPower(true); 
        
        // Re-initialize I2C/OLED
        TinyWireM.begin();
        oled.begin(width, height, sizeof(tiny4koled_init_72x40br), tiny4koled_init_72x40br);
        oled.setFont(FONT8X16); 
        setOledPower(true); 
        delay(300); 
    }
}


// Function Definition: Parses $GNRMC sentence and updates time 
void parseGNRMC(char *sentence) {
    if (strncmp(sentence, "$GNRMC", 6) != 0) return;

    char *p = sentence;
    byte field = 0;
    char *rawTime = NULL;
    char *validity = NULL;
    
    while (*p) {
        if (*p == ',') {
            field++;
            if (field == 1) {
                rawTime = p + 1;
            } else if (field == 2) {
                validity = p + 1;
                break;
            }
        }
        p++;
    }

    bool newFixStatus = (validity != NULL && *validity == 'A');

    if (newFixStatus != hasFix) {
        hasFix = newFixStatus;
        timeUpdated = true;
        if (!hasFix) {
            currentSecond = -1; 
        }
    }

    if (hasFix && rawTime != NULL && strlen(rawTime) >= 6) {
        int hh = (rawTime[0] - '0') * 10 + (rawTime[1] - '0');
        int mm = (rawTime[2] - '0') * 10 + (rawTime[3] - '0');
        int ss = (rawTime[4] - '0') * 10 + (rawTime[5] - '0');

        hh = (hh + TIME_ZONE_OFFSET) % 24;

        if (ss != currentSecond) {
            snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", hh, mm, ss);
            currentSecond = ss;
            timeUpdated = true; // Updates once per second
        }
    }
}


void setup() {
    // 1. Pin setup
    pinMode(GPS_POWER_PIN, OUTPUT);
    pinMode(SLEEP_BUTTON_PIN, INPUT_PULLUP); 

    // 2. Initial power state
    setGpsPower(true); 
    
    // 3. I2C/OLED Initialization
    TinyWireM.begin();
    oled.begin(width, height, sizeof(tiny4koled_init_72x40br), tiny4koled_init_72x40br);
    oled.setFont(FONT8X16); 
    oled.on();
    oled.clear();
    
    // 4. State initialization
    currentSecond = -1;
    timeUpdated = true;
    isLowPowerMode = false; 
    PCMSK &= ~_BV(PCINT4); 
    
    // 5. ADC Initialization (for VCC measurement)
    // Prescaler 128
    ADCSRA |= _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); 
    // Enable ADC (ADEN)
    ADCSRA |= _BV(ADEN); 
    // Start a dummy conversion to stabilize ADC
    ADCSRA |= _BV(ADSC);
    while (ADCSRA & _BV(ADSC));
}


void loop() {
    if (!isLowPowerMode) {
        // --- 1. GPS Sentence Reading & Parsing  ---
        while (gpsSerial.available()) {
            char c = gpsSerial.read();
            if (c == '$') idx = 0;
            if (idx < sizeof(nmeaSentence) - 1) {
                nmeaSentence[idx++] = c;
            }
            if (c == '\n' || c == '\r') {
                nmeaSentence[idx] = '\0';
                parseGNRMC(nmeaSentence);
                idx = 0;
            }
        }
    }


    // --- 2. Button Detection and Low Power Mode Toggle ---
    if (digitalRead(SLEEP_BUTTON_PIN) == LOW) {
        if (buttonPressTime == 0) {
            delay(DEBOUNCE_DELAY); 
            if (digitalRead(SLEEP_BUTTON_PIN) == LOW) {
                buttonPressTime = millis();
            }
        } 
        if (buttonPressTime != 0 && (millis() - buttonPressTime) > SHORT_PRESS_MAX_TIME) {
            buttonPressTime = 0; 
        }
    } 
    else if (buttonPressTime != 0) {
        delay(DEBOUNCE_DELAY); 
        if (digitalRead(SLEEP_BUTTON_PIN) == HIGH) {
            buttonPressTime = 0; 
            toggleLowPowerMode();
        } else {
            buttonPressTime = 0; 
        }
    }

    // --- 3. Conditional Refresh Logic  ---
    
    // Check if update is needed: 1) Time changed (1 Hz) OR 2) VCC update interval reached (2 seconds)
    bool shouldUpdate = timeUpdated || (millis() - lastVccUpdate >= VCC_UPDATE_INTERVAL);

    if (!isLowPowerMode && shouldUpdate) {
        oled.clear();
        
        // --- Read and format battery voltage (XX.XV format) ---
        long vccMv = readVccMillivolts();
        char battStr[6]; 
        
        long volts = vccMv / 1000;
        long tenths = (vccMv % 1000) / 100;

        snprintf(battStr, sizeof(battStr), "%ld.%ldV", volts, tenths);
        
        // --- Display Refresh ---
        
        oled.setFont(FONT8X16); 
        
        if (hasFix) {
            // Top row (Pages 0/1): Time
            oled.setCursor(0, 0); 
            oled.print(timeStr); 
            
            // Bottom row (Pages 2/3): Battery Voltage
            oled.setCursor(0, 2); 
            oled.print(battStr);
            
        } else {
            // No Fix:
            // Top row: Prompt (No Fix)
            oled.setCursor(0, 0);
            oled.print("No Fix");
            
            // Bottom row: Battery Voltage
            oled.setCursor(0, 2); 
            oled.print(battStr);
        }

        // Clear time update flag
        if (timeUpdated) {
            timeUpdated = false; 
        }
        
        // Update the VCC update timestamp
        lastVccUpdate = millis();
    }
}