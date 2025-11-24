#include <TinyWireM.h>
#include <Tiny4kOLED.h>
#include <NeoSWSerial.h>
#include <string.h>
#include <stdio.h>

// --- Time Zone Configuration ---
// Adjust this value to set the local time zone offset from UTC/GMT.
// Example: Taipei/Beijing is UTC+8. New York is UTC-5 (or -4 during DST).
// For UTC-X, use negative numbers, e.g., -5.
#define TIME_ZONE_OFFSET 8 

// --- OLED Settings ---
uint8_t width = 72;
uint8_t height = 40;

// --- GPS Module Settings ---
#define GPS_RX 3
#define GPS_TX 4
NeoSWSerial gpsSerial(GPS_RX, GPS_TX);

// --- GPS Data Buffer ---
char nmeaSentence[82];
byte idx = 0;

// --- Time Data and Display State ---
char timeStr[9] = "08:00:00";
int currentSecond = -1;
volatile bool timeUpdated = false; // Flag to indicate if time has changed

// Function Declarations
void parseGNRMC(char *sentence);


// Function Definition: Parses $GNRMC sentence and updates time 
void parseGNRMC(char *sentence) {
    // Check for $GNRMC header
    if (strncmp(sentence, "$GNRMC", 6) != 0) return; //when you use neo-m6n or some just gps, use "$GPRMC"

    // --- Resource Optimization: Manual pointer traversal instead of strtok ---
    char *p = sentence;
    byte field = 0;
    char *rawTime = NULL;
    
    // Iterate through the string to find the first comma (Field 1: UTC Time)
    while (*p) {
        if (*p == ',') {
            field++;
            if (field == 1) {
                rawTime = p + 1; // rawTime now points to the start of the time string
                break; 
            }
        }
        p++;
    }

    // Process raw time string
    if (rawTime != NULL && strlen(rawTime) >= 6) {
        // Extract HH, MM, SS from the character array
        int hh = (rawTime[0] - '0') * 10 + (rawTime[1] - '0');
        int mm = (rawTime[2] - '0') * 10 + (rawTime[3] - '0');
        int ss = (rawTime[4] - '0') * 10 + (rawTime[5] - '0');

        // Apply configurable TIME_ZONE_OFFSET
        hh = (hh + TIME_ZONE_OFFSET) % 24;

        // Conditional Update Logic (only update display if seconds changed)
        if (ss != currentSecond) {
            // Format to HH:MM:SS
            snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", hh, mm, ss);
            currentSecond = ss;
            timeUpdated = true; // Set flag for display loop
        }
    }
}


void setup() {
    // 1. Initialize GPS Serial Port
    gpsSerial.begin(9600);

    // 2. Initialize I2C/OLED
    TinyWireM.begin();

    // 3. Initialize OLED
    oled.begin(width, height, sizeof(tiny4koled_init_72x40br), tiny4koled_init_72x40br);
    oled.on();
    oled.setFont(FONT8X16); 

    // 4. [Optimization]: Remove double buffer initialization
    // Perform a single clear to ensure the initial screen is clean
    oled.clear();
    
    // 5. Initialize State Variables
    currentSecond = -1;
    timeUpdated = false; 
}


void loop() {
    // 1. GPS Sentence Reading & Parsing (Non-blocking)
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

    // --- Conditional Refresh Logic (Single Buffer) ---
    if (timeUpdated) {
        // 1. Draw new content directly to OLED display memory (Immediate draw)
        
        // The strategy relies on overwriting old content to avoid full screen clear,
        // which minimizes I2C traffic and maximizes speed/efficiency.
        oled.setCursor(5, 9); 
        oled.print(timeStr); 
        
        

        // 3. Reset flag
        timeUpdated = false; 
    }
    // No delay needed, allowing the loop to execute as fast as possible to check serial data.
}