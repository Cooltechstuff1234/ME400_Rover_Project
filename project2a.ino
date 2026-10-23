#include "lcdhelperv2.h"
#include "irhelper.h"
#include <math.h>
// B allows us to use colors
#define RCB_RED 0x001F
#define RCB_WHITE 0xFFFF
#define RCB_BLACK 0x0000
#define RCB_PURPLE      0x780F
//
//  Create an instance of the lcdhelper class named oLCD
//
lcdhelper oLCD(ILI9163_4L, 3, 2, 9, 10, 7);
//
//  Declare a sprite so we can display a rotated label on the TFT.
//
TFT_eSprite spr = TFT_eSprite(&oLCD); // Sprite object
//
//  Create an instance of irhelper class named oIR.
//
irhelper oIR;
// Get width of TFT display in pixels
int xScreen = 1;
// Get height of TFT display in pixels
int yScreen = 1;
// Define a spacer to represent margins and spacing between elements.
int mScreen = 2;
// B creates booleans that stay false and prevent options 2 and 4 until they are okay to run.
bool firstcomp = true;
bool thirdcomp = true;
// B define pins
const int RightOutputPin = 8;
const int LeftOutputPin = 10;
const int sw1 = 18;
const int sw2 = 17;
const int sw3 = 16;
const int sw4 = 15;
const int servo1 = 35;
const int servo2 = 36;
const int echo = 37;
const int trig = 38;
const int LeftMotorEPin = 46;
const int RightMotorEPin = 14;
const int LeftMotorPPin = 12;
const int RightMotorPPin = 13;
const int PWMLeft = 1;
const int PWMRight = 2;

int DutyCycle;

//
//  Define protypes for functions
//
void ShowDisplay(screen val, char optionstate, char keypressed, bool initdisplay = false);
// *************************************************************************************
// BEGIN: USER FUNCTION
// *************************************************************************************
void clearText(int32_t x, int32_t y, int32_t width, uint16_t color)
{
    int32_t xStart;

    if (x == CENTER)
        xStart = (oLCD.width() - width) / 2;
    else
        xStart = x;

    oLCD.fillRect(xStart, y - 6, width, 15, color);
}
//
//  Option1 code
//
void Option1(char optionstate, char keypressed, bool initdisplay)
{
    firstcomp = true;
    oLCD.print("DUTY CYCLE", CENTER, 10);
    oLCD.setBackColor(0x0000);
    oLCD.printNumI(keypressed.toInt(),CENTER,24,0,' ');
    oLCD.setBackColor(RCB_PURPLE);
    oLCD.print("PRESS", CENTER, 38);
    oLCD.print("<RETURN>", CENTER, 52);
    oLCD.print("WHEN", CENTER, 66);
    oLCD.print("READY", CENTER, 80);

}
//
//  Option2 code
//
void Option2(char optionstate, char keypressed, bool initdisplay)
{
    // DO NOT SHARE CODE WITH STUDENTS.  THIS IS A COMPETITION SO LETS SEE WHAT THEY COME UP.
}
//
//  Option3 code
//
void Option3(char optionstate, char keypressed, bool initdisplay)
{
    thirdcomp = true;
}
//
//  Option4 code
//
void Option4(char optionstate, char keypressed, bool initdisplay)
{
}
// *************************************************************************************
// END: USER FUNCTIONS
// *************************************************************************************
// *************************************************************************************
// BEGIN: CORE FUNCTIONALITY
// *************************************************************************************
//
//  Setup(): Perform initialization
//
void setup()
{
    Serial.begin(115200);
    // B Initialize output pins
    pinMode(LeftOutputPin, OUTPUT);
    pinMode(RightOutputPin, OUTPUT);
    // B Initialize input pins/buttons
    pinMode(sw1, INPUT_PULLUP);
    pinMode(sw2, INPUT_PULLUP);
    pinMode(sw3, INPUT_PULLUP);
    pinMode(sw4, INPUT_PULLUP);
    // B initialize servo pins
    pinMode(servo1, OUTPUT);
    pinMode(servo2, OUTPUT);
    // B initialize sensor pins
    pinMode(echo, INPUT_PULLUP);
    pinMode(trig, OUTPUT);
    // B initialize Enable pins
    pinMode(LeftMotorEPin, OUTPUT);
    pinMode(RightMotorEPin, OUTPUT);
    // B initialize Phase pins
    pinMode(LeftMotorPPin, OUTPUT);
    pinMode(RightMotorPPin, OUTPUT);
    // Initialize IR Communications
    oIR.IRInitialize();
    // Show main menu when Arduino starts
    // for the 1st time or on reset.
    ShowDisplay(SC_MAIN, ' ', ' ', true);
    // Get width of TFT display in pixels
    xScreen = oLCD.getViewportWidth();
    // Get height of TFT display in pixels
    yScreen = oLCD.getViewportHeight();
}

//
//  Loops(): Perform looping operations
//
void loop()
{
    unsigned long last_key_processed;
    // Determine which key was pressed.
    last_key_processed = oIR.GetKeyPressed();
    // Return to the main menu any time the
    // menu key is pressed.
    if (last_key_processed == KEY_MENU)
    {
        ShowDisplay(SC_MAIN, ' ', ' ');
    }
    else
    {
        if (oLCD.SCREEN_STATE == SC_MAIN)
        {
            if (last_key_processed == KEY_1)
            {
                ShowDisplay(SC_SUB1, 'A', ' ', false);
            }
            else if (last_key_processed == KEY_2)
            {
                ShowDisplay(SC_SUB2, 'A', ' ', false);
            }
            else if (last_key_processed == KEY_3)
            {
                ShowDisplay(SC_SUB3, 'A', ' ', false);
            }
            else if (last_key_processed == KEY_4)
            {
                ShowDisplay(SC_SUB4, 'A', ' ', false);
            }
        }
        if (oLCD.SCREEN_STATE == SC_SUB1)
        {
            // If the user presses one of the numeric keys, then display the key in the correct place on the screen.
            if (last_key_processed == KEY_0 || last_key_processed == KEY_1 || last_key_processed == KEY_2 || last_key_processed == KEY_3 || last_key_processed == KEY_4 ||
                last_key_processed == KEY_5 || last_key_processed == KEY_6 || last_key_processed == KEY_7 || last_key_processed == KEY_8 || last_key_processed == KEY_9 ||
                last_key_processed == KEY_BACK)
            {
                ShowDisplay((screen)oLCD.SCREEN_STATE, 'A', oIR.TranslateKey(last_key_processed), false);
            }
            else if (last_key_processed == KEY_RETURN)
            {
                ShowDisplay((screen)oLCD.SCREEN_STATE, 'B', oIR.TranslateKey(last_key_processed), false);
            }
        }
        else if (oLCD.SCREEN_STATE == SC_SUB2 && firstcomp == false)
        {
            Serial.print("Sub2 trace = ");
            Serial.println(last_key_processed);
            if (last_key_processed == KEY_RETURN)
            {
                ShowDisplay(SC_MAIN, ' ', ' ');
            }
            else
            {
                ShowDisplay(SC_SUB2, ' ', oIR.TranslateKey(last_key_processed), false);
            }
        }
        else if (oLCD.SCREEN_STATE == SC_SUB3)
        {
            Serial.print("Sub3 trace = ");
            Serial.println(last_key_processed);
            if (last_key_processed == KEY_RETURN)
            {
                ShowDisplay(SC_MAIN, ' ', ' ');
            }
            else
            {
                ShowDisplay(SC_SUB3, ' ', oIR.TranslateKey(last_key_processed), false);
            }
        }
        else if (oLCD.SCREEN_STATE == SC_SUB4 && firstcomp == false && thirdcomp == false)
        {
            if (last_key_processed == KEY_RETURN)
            {
                ShowDisplay(SC_MAIN, ' ', ' ');
            }
            else
            {
                ShowDisplay(SC_SUB4, ' ', oIR.TranslateKey(last_key_processed), false);
            }
        }
    }
    delay(1000);
}
//
//  Method to display menus on TFT display
//
void ShowDisplay(screen val, char optionstate, char keypressed, bool initdisplay)
{
    // Store screen state.
    oLCD.SCREEN_STATE = val;
    oLCD.OPTION_STATE = optionstate;
    int selectedpid = 0;

    if (val == SC_MAIN)
    {
        //
        //  Initialize the display
        //
        oLCD.LCDInitialize(LANDSCAPE, initdisplay);
        oLCD.print("SELECT OPTION", 10, 10);
        oLCD.print("1. CALIBRATE DRIVE", 10, 24);
        if (firstcomp == true)
        {
            oLCD.setColor(0x001F);
        }
        oLCD.print("2. SEEK TARGET", 10, 38);
        oLCD.setColor(0xFFFFE);
        oLCD.print("3. LOCATE TARGET", 10, 52);
        if (firstcomp == true && thirdcomp == true)
        {
            oLCD.setColor(0x001F);
        }
        oLCD.print("4. NAVIGATE", 10, 66);
        oLCD.setColor(0xFFFF);
    }
    if (val == SC_SUB1)
    {
        Serial.println("option1");
        Option1(optionstate, keypressed, initdisplay);
    }
    else if (val == SC_SUB2)
    {
        Serial.println("option2");
        Option2(optionstate, keypressed, initdisplay);
    }
    else if (val == SC_SUB3)
    {
        Serial.println("option3");
        Option3(optionstate, keypressed, initdisplay);
    }
    else if (val == SC_SUB4)
    {
        Serial.println("option4");
        Option4(optionstate, keypressed, initdisplay);
    }
}
// *************************************************************************************
// BEGIN: UTILITY FUNCTIONS
// *************************************************************************************
//
//  Function to move the servo to the specified angle in degrees
//
void servowrite(int pin, int angle, int numpulses)
{
    //  Move servo attached to pin to the specified angle.
    //  The numpulses shoul be set to approximately 3 for
    //  an angular movement of 2 or less degrees.  For a large
    //  angular change numpulses should be set to 10.
    //  Set the pulse height in microseconds associated
    //  with the servo angles of 0 and 180 degrees
    //
    int microzero = 500;
    int micro180 = 2400;
    int lenMicroSecondsOfPeriod = 20 * 1000;
    // pin for servo, angle to move, microseconds for zero degrees, microseconds for 180 degrees
    int lenMicroSecondsOfPulse = map(angle, 0, 180, microzero, micro180); // map angle to microseconds
    // Loop long enough for servo to see enough of waveform to fully
    // move to the new position.
    for (int i = 0; i < numpulses; i++)
    {
        // Servos work by sending a 20 ms pulse.
        // 1.0 ms at the start of the pulse will turn the servo to the 0 degree position
        // 1.5 ms at the start of the pulse will turn the servo to the 90 degree position
        // 2.0 ms at the start of the pulse will turn the servo to the 180 degree position
        // Turn voltage high to start the period and pulse
        digitalWrite(pin, HIGH);

        // Delay for the length of the pulse
        delayMicroseconds(lenMicroSecondsOfPulse);

        // Turn the voltage low for the remainder of the pulse
        digitalWrite(pin, LOW);

        // Delay this loop for the remainder of the period so we don't
        // send the next signal too soon or too late
        delayMicroseconds(lenMicroSecondsOfPeriod - lenMicroSecondsOfPulse);
    }
}
//
// Function for calculating the distance measured by the Ultrasonic sensor
//
int calculateDistance(int tPin, int ePin)
{
    // Variables for the duration and the distance
    long duration;
    int distance;
    digitalWrite(tPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(tPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(tPin, LOW);
    duration = pulseIn(ePin, HIGH, 15000); // Reads the echoPin, returns the sound wave travel time in microseconds
    distance = duration * 0.034 / 2;
    return distance;
}
//
//  Code to display y axis label (rotated 90 degrees)
//
void DisplayRotatedText(int x, int y)
{
    oLCD.setPivot(x / 2, y / 2); // Set pivot to middle of TFT screen
    // Create the Sprite
    spr.setColorDepth(16);     // Create an 8bpp Sprite of 60x30 pixels
    spr.createSprite(124, 30); // 8bpp requires 64 * 30 = 1920 bytes
    //
    // Set pivot relative to top left corner of Sprite (increasing y will push the text
    // closer to the left hand side of the screen.  Increasing x will move the label up and down)
    spr.setPivot(57, 85);
    spr.fillSprite(RCB_PURPLE); // Fill the Sprite with black

    spr.setTextColor(RCB_WHITE);              // Green text
    spr.setTextDatum(MC_DATUM);               // Middle centre datum
    spr.drawString("speed (rpm)", 60, 15, 2); // Plot text, font 4, in Sprite at 12, 15

    // rotate text on screen by 270 degrees
    spr.pushRotated(270);

    spr.deleteSprite();
}

//B stripped straight from exercise 5
void ProcessKeyPress(unsigned long int result)
{
    String response = "";
    // Return to the main menu any time the
    // menu key is pressed.
    if (result == KEY_MENU)
    {
        ShowDisplay(SC_MAIN,'A',' ', false);
    }
    else
    {
        if (oLCD.SCREEN_STATE == SC_MAIN)
        {
            if (result == KEY_1)  
            {
                ShowDisplay(SC_SUB1,'A',' ', false);  //AREA ESTIMATE
            }
            else if (result== KEY_2) 
            {
                ShowDisplay(SC_SUB2,'A',' ', false);  //PERIMETER ESTIMATE
            }
        }   
        else if (oLCD.SCREEN_STATE == SC_SUB1 || oLCD.SCREEN_STATE == SC_SUB2) //AREA ESTIMATE
        {
            // If the user presses one of the numeric keys, then display the key in the correct place on the screen.
            if (result == KEY_0  || result == KEY_1  || result == KEY_2  || result == KEY_3  || result == KEY_4  ||
                result == KEY_5  || result == KEY_6  || result == KEY_7  || result == KEY_8  || result == KEY_9)  
            {
                //Serial.print("t=");Serial.println(oLCD.OPTION_STATE);
                if ((oLCD.OPTION_STATE =='A') || (oLCD.OPTION_STATE == 'B'))
                {
                    ShowDisplay((screen)oLCD.SCREEN_STATE,'B',oIR.TranslateKey(result),false);  
                }
                else if ((oLCD.OPTION_STATE =='C') || (oLCD.OPTION_STATE == 'D'))
                {
                    ShowDisplay((screen)oLCD.SCREEN_STATE,'D',oIR.TranslateKey(result),false);  
                }
                else
                {
                    ShowDisplay((screen)oLCD.SCREEN_STATE,'F',oIR.TranslateKey(result),false);  
                }
                
            }
            else if (result== KEY_RETURN) 
            {
                if (oLCD.OPTION_STATE =='B')
                {
                    ShowDisplay((screen)oLCD.SCREEN_STATE,'C',' ', false);  //AREA ESTIMATE
                    //Serial.println("BINT");
                }
                else if (oLCD.OPTION_STATE =='D')
                {
                    ShowDisplay((screen)oLCD.SCREEN_STATE,'E',' ', false);  //AREA ESTIMATE
                    //Serial.println("DIVISIONINIT");
                }
                else if (oLCD.OPTION_STATE == 'F')
                {
                    ShowDisplay((screen)oLCD.SCREEN_STATE,'G',' ',false);  //AREA ESTIMATE
                    //Serial.println("CALCULATE");
                }
                else if (oLCD.OPTION_STATE == 'G')
                {
                    // Go To Main Menu if return pressed without both radius and division data.
                    ShowDisplay((screen)oLCD.SCREEN_STATE,'A',' ',false);  
                    //Serial.println("AINIT");
                }
                
            }
        }    
    }
}
// *************************************************************************************
// END: UTILITY FUNCTIONS
// *************************************************************************************
// *************************************************************************************
// START: OPTION1 SUPPORT
// *************************************************************************************

// *************************************************************************************
// END: OPTION1 SUPPORT
// *************************************************************************************

// *************************************************************************************
// START: OPTION2 SUPPORT
// *************************************************************************************

// *************************************************************************************
// END: OPTION2 SUPPORT
// *************************************************************************************
// *************************************************************************************
// START: OPTION3 SUPPORT
// *************************************************************************************
//
//  Option 3: Function to draw a box.
//
void DrawBox(int x1, int y1, int x2, int y2, uint32_t color)
{
    oLCD.fillRoundRect(x1, y1, x2, y2, 3, color);
}
// *************************************************************************************
// END: OPTION3 SUPPORT
// *************************************************************************************
// *************************************************************************************
// START: OPTION4 SUPPORT
// *************************************************************************************

// *************************************************************************************
// END: OPTION4 SUPPORT
// *************************************************************************************