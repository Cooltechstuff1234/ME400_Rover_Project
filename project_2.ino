#include "lcdhelperv2.h"
#include "irhelper.h"
#include <math.h>
 
// B allows us to use colors
#define RCB_RED 0x001F
#define RCB_WHITE 0xFFFF
#define RCB_BLACK 0x0000
#define RCB_PURPLE 0x780F
 
// *************************************************************************************
// BEGIN: GLOBAL VARIABLES
// *************************************************************************************
 
// Hardware objects
lcdhelper oLCD(ILI9163_4L, 3, 2, 9, 10, 7);
TFT_eSprite spr = TFT_eSprite(&oLCD); // Sprite object
irhelper oIR;
portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED; // This is to read the interrupt values with the RPM function to minimize skewed data.
//------------------------------------------------------------------
 
// Screen size and spacing
int xScreen = 1;
int yScreen = 1;
int mScreen = 2;
//------------------------------------------------------------------
 
// Pins
const int LeftOutputPin = 10;
const int RightOutputPin = 8;
const int LeftMotorEPin = 46;
const int RightMotorEPin = 14;
const int LeftMotorPPin = 12;
const int RightMotorPPin = 13;
const int sw1 = 18;
const int sw2 = 17;
const int sw3 = 16;
const int sw4 = 15;
const int servo1 = 35;
const int servo2 = 36;
const int trig = 38;
const int echo = 37;
//------------------------------------------------------------------
 
// setup(), Move() and Stop()
const int PWMLeft = 1;
const int PWMRight = 2;
const int pwmFrequency = 496;
const int pwmResolution = 8;
const int maxDutyCycle = (int)pow(2, pwmResolution) - 1;
//------------------------------------------------------------------
 
// loop(), ShowDisplay(), CheckExit() and Break()
bool firstcomp = true;
bool thirdcomp = true;
bool CascadeExit = false;
//------------------------------------------------------------------
 
// Option1() and Option4()
int DutyCycle;
//------------------------------------------------------------------
 
// Option2() and FollowUltra()
bool FoundTarget = false; // used in Option 2
double dt = 0;
//------------------------------------------------------------------
 
// Option3(), acuScan() and relAngle()
double angle_offset = 90;
 
double Tdist = 10000;
double Tdist_min_0 = 10000;
double Tdist_min_1 = 10000;
int Tangle_0;
int Tangle_1;
bool TargetFound = false; // used in Option 3
bool ScanDone = false;
//------------------------------------------------------------------
 
// Option3(), Option4() and Odomitry()
bool nav = true;
double TargetDir = 0;
double TargetDis = 0;
double CurrentDir = 0;
double CurrentDis = 0;
//------------------------------------------------------------------
 
// targetN(), Odomitry(), ReadTurnRPM() and PredictStop()
int cpr = 40;
//------------------------------------------------------------------
 
// onLeftEdge() and onRightEdge()
const unsigned long EncDeUS = 5000; // tune for your encoders
 
volatile long countL = 0;
volatile long countR = 0;
volatile unsigned long LLUs = 0; // Last left microseconds
volatile unsigned long LRUs = 0; // Last right microseconds
//------------------------------------------------------------------
 
// ReadRPM()
double taveL[20]; // time interval over the previous 10 counts
double taveR[20];
int count0 = 0;
int count1 = 0;
int count2 = 0;
int count3 = 0;
double dtL = 0; // Time differental for the RPM measureing
double dtR = 0;
double rpm_L = 0;
double rpm_R = 0;
//------------------------------------------------------------------
 
// ResetTurnRPM() and ReadTurnRPM()
const int TurnN = 2; // gaps averaged. 2 gaps = 3 counts. Keep it even: your gaps alternate short/long.
 
double ttaveL[TurnN]; // Previous gaps between counts on the left wheel
double ttaveR[TurnN]; // Previous gaps between counts on the right wheel
int tcount0 = 0;      // gaps stored, left
int tcount1 = 0;      // total counts
long tcount2 = 0;     // left count at the previous edge
long tcount3 = 0;     // right count at the previous edge
double tdtL = 0;      // time of last edge for the Left wheel
double tdtR = 0;      // time of last edge for the Right wheel
double tgapL = 0;     // average time per count for the Left wheel
double tgapR = 0;     // average time per count for the Right wheel
double trpm_L = 0;    // Left wheel rpm
double trpm_R = 0;    // Right wheel rpm
//------------------------------------------------------------------
 
// PredictStop()
const double CoastTime = 50000; // Time it takes for wheel to stop after cutoff
 
bool doneL = false; // Cuttof Left wheel
bool doneR = false; // Cutoff Right wheel
//------------------------------------------------------------------
 
// targetN()
double excessN = 0;
//------------------------------------------------------------------
 
// Odomitry()
double lastTravel = 0;
double delta = 0;
//------------------------------------------------------------------
 
// TranslateRobot()
const double wheelRatio = 1.0037; // right / left effective diameter, tuned from drift

int countoff = 0;
double error = 0;
unsigned long c = 0;
unsigned long t = 0;
double dt_1 = 0;
//------------------------------------------------------------------
 
// *************************************************************************************
// END: GLOBAL VARIABLES
// *************************************************************************************

//
//  Define protypes for functions
//

void ShowDisplay(screen val, char optionstate, char keypressed, bool initdisplay = false);
// Option 1
int targetN(double dist);
void flushIR(unsigned long ms);
void clearDisp();
void clearText(int32_t x, int32_t y, int32_t width, uint16_t color);
void UpdateSCR(int type, double a, double b, double c, double d);
int targetN(double dist);
void Move(int type, int err, double adj, int duty);
void Stop();
void DutyAdj(double &adj, double &error, int count0, int count1);
void TranslateRobot(double dist, double duty, int move, int SCR, int debug);
bool CheckExit();
// Option 2
double relAngle(double ang, double exof, int reltype);
// Option 3
void PrintRadarScreen(bool initdisplay);
void printLine(double distance, double angle, bool initial);
void ActualRadar(int radius, int i);
// Option 4
void DrawBox(int x, int y, int width, int height, uint32_t color);

void IRAM_ATTR onLeftEdge()
{
    unsigned long now = micros();
    if (now - LLUs >= EncDeUS)
    {
        countL++;
        LLUs = now;
    }
}
void IRAM_ATTR onRightEdge()
{
    unsigned long now = micros();
    if (now - LRUs >= EncDeUS)
    {
        countR++;
        LRUs = now;
    }
}

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
    firstcomp = false;
    char text[20];
    static String aval = "";
    char lastkeypressed;

    oLCD.LCDInitialize(PORTRAIT, initdisplay);

    oLCD.setTextColor(RCB_WHITE, RCB_PURPLE);
    sprintf(text, "DUTY CYCLE");
    oLCD.print(text, CENTER, 15);
    oLCD.print("PRESS", CENTER, 48);
    oLCD.print("<RETURN>", CENTER, 68);
    oLCD.print("WHEN", CENTER, 88);
    oLCD.print("READY", CENTER, 108);
    if (optionstate == 'A' && keypressed == ' ')
    {
        aval = "";
        clearText(CENTER, 32, 20, RCB_BLACK);
        Serial.println("help");
    }
    else if (keypressed == oIR.TranslateKey(KEY_BACK))
    {
        int change = (aval.toDouble() / 10);
        clearText(CENTER, 32, 20, RCB_BLACK);
        oLCD.setBackColor(RCB_BLACK);
        oLCD.printNumI(change, CENTER, 32, 3, ' ');
        aval = String(change);
        if (change == 0)
        {
            aval = "";
        }
    }
    else if (optionstate == 'B' && aval.toInt() < 40 || optionstate == 'B' && aval.toInt() > 70)
    {
        clearText(CENTER, 32, 20, RCB_BLACK);
        oLCD.setBackColor(RCB_BLACK);
        oLCD.printNumI(aval.toInt(), CENTER, 32, 3, ' ');
    }
    else if (optionstate == 'A' && keypressed != ' ')
    {
        if (aval.length() < 2)
        {
            if (aval == "" && keypressed != oIR.TranslateKey(KEY_0) && keypressed != oIR.TranslateKey(KEY_1) && keypressed != oIR.TranslateKey(KEY_2) && keypressed != oIR.TranslateKey(KEY_3) && keypressed != oIR.TranslateKey(KEY_8) && keypressed != oIR.TranslateKey(KEY_9))
            {
                aval = aval + keypressed;
                clearText(CENTER, 32, 20, RCB_BLACK);
                oLCD.setBackColor(RCB_BLACK);
                oLCD.printNumI(aval.toInt(), CENTER, 32, 3, ' ');
            }
            else if (aval == "0" && keypressed != oIR.TranslateKey(KEY_0) && keypressed != oIR.TranslateKey(KEY_1) && keypressed != oIR.TranslateKey(KEY_2) && keypressed != oIR.TranslateKey(KEY_3) && keypressed != oIR.TranslateKey(KEY_8) && keypressed != oIR.TranslateKey(KEY_9))
            {
                aval = aval + keypressed;
                clearText(CENTER, 32, 20, RCB_BLACK);
                oLCD.setBackColor(RCB_BLACK);
                oLCD.printNumI(aval.toInt(), CENTER, 32, 3, ' ');
            }
            else if (aval != "" && aval != "0")
            {
                aval = aval + keypressed;
                clearText(CENTER, 32, 20, RCB_BLACK);
                oLCD.setBackColor(RCB_BLACK);
                oLCD.printNumI(aval.toInt(), CENTER, 32, 3, ' ');
            }
        }
        clearText(CENTER, 32, 20, RCB_BLACK);
        oLCD.setBackColor(RCB_BLACK);
        oLCD.printNumI(aval.toInt(), CENTER, 32, 3, ' ');
    }
    else if (optionstate == 'B')
    {
        DutyCycle = aval.toInt();
        TranslateRobot(100, aval.toInt(), 2, 1, 0);
    }
}
//
//  Option2 code
//
void Option2(char optionstate, char keypressed, bool initdisplay)
{
    FollowUltra(); // Find Target
    if (CheckExit())
        return;
    while (!CheckExit()) // show result until RETURN/MENU
    {
        if (micros() - dt > 1000000)
        {
            UpdateSCR(3, 0, 0, 0, 0);
            dt = micros();
        }
    }
}
//
//  Option3 code
//
void Option3(char optionstate, char keypressed, bool initdisplay)
{

    if (ScanDone == false)
    {
        clearDisp();
        oLCD.setTextColor(RCB_WHITE, RCB_PURPLE);
        PrintRadarScreen(initdisplay);
        for (int i = 15; i < 165; i += 2)
        {

            servowrite(35, i, 3);

            if (i == 0)
            {
                delay(1000);
            }
            else
            {
                delay(100);
            }

            Tdist = calculateDistance(38, 37);

            if (Tdist < Tdist_min_0)
            {
                if (Tdist >= 5)
                {
                    Tdist_min_0 = Tdist;
                    Tangle_0 = i;
                }
            }
            if (i != 30, i != 60, i != 90, i != 120, i != 150)
            {
                ActualRadar(Tdist, i);
            }
            oLCD.setBackColor(RCB_BLACK);
            oLCD.printNumI(i, 65, 10, 3);
            oLCD.printNumI(Tdist, 90, 30, 3);
            //oLCD.setBackColor(RCB_BLACK);
            // String text = String(i);
            // String text1 = String(Tdist_min_0);
            // String text2 = String(Tdist);
            // oLCD.setBackColor(0x0000);
            // oLCD.print(text, 20, 20);
            // oLCD.print(text1, 20, 40);
            // oLCD.print(text2, 20, 60);
        }

        Tdist = 10000;
        for (int i = 165; i > 15; i--)
        {

            servowrite(35, i, 3);

            if (i == 0)
            {
                delay(1000);
            }
            else
            {
                delay(100);
            }

            Tdist = calculateDistance(38, 37);

            if (Tdist < Tdist_min_1)
            {
                if (Tdist >= 5)
                {
                    Tdist_min_1 = Tdist;
                    Tangle_1 = i;
                }
            }
            if (i != 30, i != 60, i != 90, i != 120, i != 150)
            {
                ActualRadar(Tdist, i);
            }
            oLCD.setBackColor(RCB_BLACK);
            oLCD.printNumI(i, 65, 10, 3);
            //oLCD.setBackColor(RCB_BLACK);
            oLCD.printNumI(Tdist, 90, 30, 3);
            //oLCD.setBackColor(RCB_BLACK);
            // String text = String(i);
            // String text1 = String(Tdist_min_1);
            // String text2 = String(Tdist);
            // oLCD.setBackColor(0x0000);
            // oLCD.print(text, 20, 20);
            // oLCD.print(text1, 20, 40);
            // oLCD.print(text2, 20, 60);
        }

        TargetDis = (Tdist_min_1 + Tdist_min_0) / 2;
        TargetDir = (Tangle_1 + Tangle_0) / 2;
        oLCD.setBackColor(RCB_BLACK);
        oLCD.printNumI(TargetDir, 65, 10, 3);
        //oLCD.setBackColor(RCB_BLACK);
        oLCD.printNumI(TargetDis, 90, 30, 3);
        oLCD.setBackColor(RCB_PURPLE);
        if (TargetDis <= 120 && TargetDis >= 5)
        {
            TargetFound = true;
        }
        else
        {
            TargetFound = false;
        }

        ScanDone = true;
    }

    // Explains the result of the scan
    if (TargetFound == true)
    {
        thirdcomp = false;
        DrawBox(35, 68, 90, 55, RCB_BLACK);
        oLCD.print("TARGET", CENTER, 76);
        String DispDir = String(TargetDir) + " DEGREES";
        oLCD.print(DispDir, CENTER, 90);
        String DispDis = String(TargetDis) + " CM";
        oLCD.print(DispDis, CENTER, 104);
    }
    else
    {
        DrawBox(35, 68, 90, 55, RCB_BLACK);
        oLCD.print("TARGET", CENTER, 76);
        oLCD.print("NOT", CENTER, 90);
        oLCD.print("FOUND", CENTER, 104);
    }
}
//
//  Option4 code
//
void Option4(char optionstate, char keypressed, bool initdisplay)
{
    CurrentDir = 0;
    CurrentDis = 0;
    if (nav == true)
    {
        // Intiazlies first Screen
        oLCD.LCDInitialize(PORTRAIT, initdisplay);
        oLCD.setTextColor(RCB_WHITE, RCB_PURPLE);
        oLCD.print("TARGET DIR", CENTER, 14);
        String DispDir = String(TargetDir) + " DEGREES";
        oLCD.print(DispDir, CENTER, 28);
        oLCD.print("TARGET DIS", CENTER, 42);
        String DispDis = String(TargetDis) + " CM";
        oLCD.print(DispDis, CENTER, 56);

        // Code to align the robot
        //TranslateRobot(22.62, 30, 3, 0, 0); // turn CCW
        TranslateRobot(90, 30, 3, 0, 0); // turn CCW
        delay(1000);
        TranslateRobot(90-13.4965637, 30, 4, 0, 0); // turn CW
        delay(1000);
        TranslateRobot(10.7375284, 30, 2, 0, 0); // move forward
        //TranslateRobot(13, 30, 2, 0, 0); // move forward

        // Pause with switch 1
        while (digitalRead(sw1) == HIGH)
        {
            delay(50);
        }
        delay(1000);

        // Code to turn the robot
        TranslateRobot(relAngle(CurrentDir, TargetDir, 1), 30, 5, 0, 0);

        // Intiazlies second Screen
        oLCD.LCDInitialize(PORTRAIT, initdisplay);
        oLCD.setTextColor(RCB_WHITE, RCB_PURPLE);
        oLCD.print("TARGET DIR", CENTER, 14);
        oLCD.print(DispDir, CENTER, 28);
        oLCD.print("TARGET DIS", CENTER, 42);
        oLCD.print(DispDis, CENTER, 56);
        oLCD.print("CURRENT DIR", CENTER, 70);
        String DispCDir = String(CurrentDir) + " DEGREES";

        oLCD.print(DispCDir, CENTER, 84);

        // Pause with switch 2
        while (digitalRead(sw2) == HIGH)
        {
            delay(50);
        }
        delay(1000);

        // Code to move towards Target
        TranslateRobot(TargetDis+4.48, DutyCycle, 2, 0, 0);

        // Intiazlies Third Screen
        oLCD.LCDInitialize(PORTRAIT, initdisplay);
        oLCD.setTextColor(RCB_WHITE, RCB_PURPLE);
        oLCD.print("TARGET DIR", CENTER, 14);
        oLCD.print(DispDir, CENTER, 28);
        oLCD.print("TARGET DIS", CENTER, 42);
        oLCD.print(DispDis, CENTER, 56);
        oLCD.print("CURRENT DIR", CENTER, 70);
        DispCDir = String(CurrentDir) + " DEGREES";
        oLCD.print(DispCDir, CENTER, 84);
        oLCD.print("CURRENT DIS", CENTER, 98);
        String DispCDis = String(CurrentDis) + " CM";
        oLCD.print(DispCDis, CENTER, 112);

        // Pause with switch 3
        while (digitalRead(sw3) == HIGH)
        {
            delay(50);
        }
        nav = false;
    }
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
    pinMode(LeftOutputPin, INPUT_PULLUP);
    pinMode(RightOutputPin, INPUT_PULLUP);
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
    ledcSetup(PWMLeft, pwmFrequency, pwmResolution);
    ledcSetup(PWMRight, pwmFrequency, pwmResolution);
    ledcAttachPin(LeftMotorEPin, PWMLeft);
    ledcAttachPin(RightMotorEPin, PWMRight);
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

    attachInterrupt(digitalPinToInterrupt(LeftOutputPin), onLeftEdge, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RightOutputPin), onRightEdge, CHANGE);

    // Intializing the servos
    servowrite(35, 90, 3);
    servowrite(36, 0, 3);

    DebugBegin();
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
        nav = true;
        ScanDone = false;
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
    if (CascadeExit)
    {
        CascadeExit = false; // reset so the next option can run
        nav = true;
        ScanDone = false;
        ShowDisplay(SC_MAIN, ' ', ' ');
    }
    if (last_key_processed == KEY_9)
    {
        firstcomp = false;
        thirdcomp = false;
        TargetDir = 70;
        TargetDis = 90;
        DutyCycle = 55;
    }
    // if(last_key_processed == KEY_5)
    // {
    //     TranslateRobot(90,30,5,1,1);
    //     delay(1000);
    //     TranslateRobot(-90,30,5,1,1);
    //     delay(1000);
    //     TranslateRobot(90,30,5,1,1);
    //     delay(1000);
    //     TranslateRobot(-90,30,5,1,1);
    //     delay(1000);
    //     TranslateRobot(45,30,5,1,1);
    //     delay(1000);
    //     TranslateRobot(-45,30,5,1,1);
    //     delay(1000);
    //     TranslateRobot(45,30,5,1,1);
    //     delay(1000);
    //     TranslateRobot(-45,30,5,1,1);
    //     delay(1000);
    //     TranslateRobot(25,30,5,1,1);
    //     delay(1000);
    //     TranslateRobot(-25,30,5,1,1);
    //     delay(1000);
    //     TranslateRobot(25,30,5,1,1);
    //     delay(1000);
    //     TranslateRobot(-25,30,5,1,1);
    //     delay(1000);
    //     TranslateRobot(5,30,5,1,1);
    //     delay(1000);
    //     TranslateRobot(-5,30,5,1,1);
    //     delay(1000);
    //     TranslateRobot(5,30,5,1,1);
    //     delay(1000);
    //     TranslateRobot(-5,30,5,1,1);
    //     delay(1000);
    // }
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

    oLCD.LCDInitialize(LANDSCAPE, initdisplay);
    oLCD.setTextColor(RCB_WHITE, RCB_PURPLE);
    oLCD.setBackColor(RCB_PURPLE);

    if (val == SC_MAIN)
    {
        //
        //  Initialize the display
        //
        oLCD.LCDInitialize(LANDSCAPE, initdisplay);
        oLCD.setTextColor(RCB_WHITE, RCB_PURPLE);
        oLCD.setBackColor(RCB_PURPLE);
        oLCD.print("SELECT OPTION", 10, 10);
        oLCD.print("1. CALIBRATE DRIVE", 10, 24);
        if (firstcomp == true)
        {
            oLCD.setColor(0x001F);
        }
        oLCD.print("2. SEEK TARGET", 10, 38);
        oLCD.setColor(0xFFFFE);
        oLCD.print("3. LOCATE TARGET", 10, 52);
        if (firstcomp == true || thirdcomp == true)
        {
            oLCD.setColor(0x001F);
        }
        oLCD.print("4. NAVIGATE", 10, 66);
        oLCD.setColor(0xFFFF);
    }
    if (val == SC_SUB1)
    {
        Option1(optionstate, keypressed, initdisplay);
    }
    else if (val == SC_SUB2 && firstcomp == false)
    {
        clearDisp();
        oLCD.print("PREPARING", CENTER, 10);
        oLCD.print("TO", CENTER, 24);
        oLCD.setBackColor(RCB_PURPLE);
        oLCD.print("ACQUIRE", CENTER, 38);
        oLCD.print("TARGET", CENTER, 52);

        Serial.println("option2");
        Option2(optionstate, keypressed, initdisplay);
    }
    else if (val == SC_SUB3)
    {
        Serial.println("option3");
        Option3(optionstate, keypressed, initdisplay);
    }
    else if (val == SC_SUB4 && firstcomp == false && thirdcomp == false)
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

// B stripped straight from exercise 5
void ProcessKeyPress(unsigned long int result)
{
    String response = "";
    // Return to the main menu any time the
    // menu key is pressed.
    if (result == KEY_MENU)
    {
        ShowDisplay(SC_MAIN, 'A', ' ', false);
    }
    else
    {
        if (oLCD.SCREEN_STATE == SC_MAIN)
        {
            if (result == KEY_1)
            {
                ShowDisplay(SC_SUB1, 'A', ' ', false); // AREA ESTIMATE
            }
            else if (result == KEY_2)
            {
                ShowDisplay(SC_SUB2, 'A', ' ', false); // PERIMETER ESTIMATE
            }
        }
        else if (oLCD.SCREEN_STATE == SC_SUB1 || oLCD.SCREEN_STATE == SC_SUB2) // AREA ESTIMATE
        {
            // If the user presses one of the numeric keys, then display the key in the correct place on the screen.
            if (result == KEY_0 || result == KEY_1 || result == KEY_2 || result == KEY_3 || result == KEY_4 ||
                result == KEY_5 || result == KEY_6 || result == KEY_7 || result == KEY_8 || result == KEY_9)
            {
                // Serial.print("t=");Serial.println(oLCD.OPTION_STATE);
                if ((oLCD.OPTION_STATE == 'A') || (oLCD.OPTION_STATE == 'B'))
                {
                    ShowDisplay((screen)oLCD.SCREEN_STATE, 'B', oIR.TranslateKey(result), false);
                }
                else if ((oLCD.OPTION_STATE == 'C') || (oLCD.OPTION_STATE == 'D'))
                {
                    ShowDisplay((screen)oLCD.SCREEN_STATE, 'D', oIR.TranslateKey(result), false);
                }
                else
                {
                    ShowDisplay((screen)oLCD.SCREEN_STATE, 'F', oIR.TranslateKey(result), false);
                }
            }
            else if (result == KEY_RETURN)
            {
                if (oLCD.OPTION_STATE == 'B')
                {
                    ShowDisplay((screen)oLCD.SCREEN_STATE, 'C', ' ', false); // AREA ESTIMATE
                    // Serial.println("BINT");
                }
                else if (oLCD.OPTION_STATE == 'D')
                {
                    ShowDisplay((screen)oLCD.SCREEN_STATE, 'E', ' ', false); // AREA ESTIMATE
                    // Serial.println("DIVISIONINIT");
                }
                else if (oLCD.OPTION_STATE == 'F')
                {
                    ShowDisplay((screen)oLCD.SCREEN_STATE, 'G', ' ', false); // AREA ESTIMATE
                    // Serial.println("CALCULATE");
                }
                else if (oLCD.OPTION_STATE == 'G')
                {
                    // Go To Main Menu if return pressed without both radius and division data.
                    ShowDisplay((screen)oLCD.SCREEN_STATE, 'A', ' ', false);
                    // Serial.println("AINIT");
                }
            }
        }
    }
}
// *************************************************************************************
// END: UTILITY FUNCTIONS
// *************************************************************************************

bool CheckExit()
{
    if (!CascadeExit)
    {
        unsigned long key = oIR.GetKeyPressed();
        if (key == KEY_RETURN || key == KEY_MENU)
        {
            CascadeExit = true;
            Stop();
        }
    }
    return CascadeExit;
}

void Break(bool &a)
{
    unsigned long key = oIR.GetKeyPressed();
    if (key == KEY_POWER)
    {
        Stop();
        if (a == true)
            a = false;
        else if (a == false)
            a = true;
    }
    if (key == KEY_RETURN || key == KEY_MENU || CascadeExit == true)
    {
        Stop();
        if (a == true)
            a = false;
        else if (a == false)
            a = true;
        CascadeExit = true;
    }
}

// *************************************************************************************
// START: OPTION1 SUPPORT
// *************************************************************************************
void ReadRPM(double &rpm, int side) // Reads an rpm over a longer interval great for long distance travel
{
    t = micros();
    if (side == 0) // Left
    {
        portENTER_CRITICAL(&encMux); // Insert data every count
        long c = countL;
        unsigned long e = LLUs;
        portEXIT_CRITICAL(&encMux);

        if (c - count2 > 0) // every new edge
        {
            if (count2 > 0) // skip the first edge of a move; its interval starts at the previous move
            {
                taveL[count0] = (e - dtL) / (c - count2);
                count0++;
            }
            dtL = e;
            count2 = c;
        }
        if (count0 >= 20)
        {
            double tsum = 0;

            for (int i = 0; i < 20; i++)
            {
                tsum += taveL[i];
            }

            tsum = tsum / 20;

            rpm = (60000) * (1000) / (tsum * 40);

            for (int i = 0; i < 19; i++)
            {
                taveL[i] = taveL[i + 1];
            }

            taveL[19] = 0;

            count0--;
        }
        if (t - dtL > 1000000) // Wheele stop set rpm to 0
        {
            rpm = 0;
            for (int i = 0; i < 20; i++)
            {
                taveL[i] = 0;
            }
        }
    }
    if (side == 1) // Right
    {
        portENTER_CRITICAL(&encMux); // Insert data every count
        long c = countR;
        unsigned long e = LRUs;
        portEXIT_CRITICAL(&encMux);

        if (c - count3 > 0) // every new edge
        {
            if (count3 > 0) // skip the first edge of a move; its interval starts at the previous move
            {
                taveR[count1] = (e - dtR) / (c - count3);
                count1++;
            }
            dtR = e;
            count3 = c;
        }
        if (count1 >= 20)
        {
            double tsum = 0;

            for (int i = 0; i < 20; i++)
            {
                tsum += taveR[i];
            }

            tsum = tsum / 20;

            rpm = ((60000) * (1000)) / (tsum * 40);

            for (int i = 0; i < 19; i++)
            {
                taveR[i] = taveR[i + 1];
            }

            taveR[19] = 0;

            count1--;
        }
        if (t - dtR > 1000000) // Wheele stop set rpm to 0
        {
            rpm = 0;
            for (int i = 0; i < 10; i++)
            {
                taveR[i] = 0;
            }
        }
    }
}

void ResetTurnRPM() // call at the start of every move
{
    tcount0 = 0;
    tcount1 = 0;
    tcount2 = 0;
    tcount3 = 0;
    tgapL = 0;
    tgapR = 0;
    trpm_L = 0;
    trpm_R = 0;
}

void ReadTurnRPM(double &rpm, int side) // Reads rpm over a short interval
{
    t = micros();
    if (side == 0) // Left
    {
        portENTER_CRITICAL(&encMux); // Insert data every count
        long c = countL;
        unsigned long e = LLUs;
        portEXIT_CRITICAL(&encMux);

        if (c - tcount2 > 0) // every new edge
        {
            if (tcount2 > 0) // skip the first edge of a move
            {
                ttaveL[tcount0] = (e - tdtL) / (c - tcount2);
                tcount0++;
            }
            tdtL = e;
            tcount2 = c;
        }
        if (tcount0 >= TurnN) // if enough gaps then average the turning time interval
        {
            double tsum = 0;

            for (int i = 0; i < TurnN; i++)
            {
                tsum += ttaveL[i];
            }

            tgapL = tsum / TurnN;

            rpm = 60000000.0 / (tgapL * cpr);

            for (int i = 0; i < TurnN - 1; i++) // slide the window along
            {
                ttaveL[i] = ttaveL[i + 1];
            }

            tcount0--;
        }
        if (tgapL > 0 && t - tdtL > 1.5 * tgapL) // if there is no edge where there should be one the wheel is slowing
        {
            rpm = 60000000.0 / ((t - tdtL) * cpr);
        }
        if (t - tdtL > 1000000) // if wheel stopped then set rpm to 0
        {
            rpm = 0;
            tgapL = 0;
            tcount0 = 0;
        }
    }
    if (side == 1) // Right
    {
        portENTER_CRITICAL(&encMux);
        long c = countR;
        unsigned long e = LRUs;
        portEXIT_CRITICAL(&encMux);

        if (c - tcount3 > 0) // every new edge
        {
            if (tcount3 > 0) // skip the first edge of a move
            {
                ttaveR[tcount1] = (e - tdtR) / (c - tcount3);
                tcount1++;
            }
            tdtR = e;
            tcount3 = c;
        }
        if (tcount1 >= TurnN) // enough gaps: average them
        {
            double tsum = 0;

            for (int i = 0; i < TurnN; i++)
            {
                tsum += ttaveR[i];
            }

            tgapR = tsum / TurnN;

            rpm = 60000000.0 / (tgapR * cpr);

            for (int i = 0; i < TurnN - 1; i++) // slide the window along
            {
                ttaveR[i] = ttaveR[i + 1];
            }

            tcount1--;
        }
        if (tgapR > 0 && t - tdtR > 1.5 * tgapR) // if there is no edge where there should be one the wheel is slowing
        {
            rpm = 60000000.0 / ((t - tdtR) * cpr);
        }
        if (t - tdtR > 1000000) // if wheel stopped then set rpm to 0
        {
            rpm = 0;
            tgapR = 0;
            tcount1 = 0;
        }
    }
}

//double OdoAngCorrection = 0;

// Uses a smaller interval rpm reading to predict where the wheels are going to stop this is used in turning because it requires signifigant precision to acheve 15 degree turns accuratly
bool PredictStop(int side, double exactTarg)
{
    portENTER_CRITICAL(&encMux); // Insert data every count
    long cnt; // passing the total counts 
    if (side == 0)
        cnt = countL; 
    if (side == 1)
        cnt = countR;
    unsigned long lastEdge; // setting the time sence the last count
    if (side == 0)
        lastEdge = LLUs;
    if (side == 1)
        lastEdge = LRUs;
    portEXIT_CRITICAL(&encMux);

    double rpm; // Per side rpm data passing
    if (side == 0)
        rpm = trpm_L;
    if (side == 1)
        rpm = trpm_R;

    if (rpm <= 0)
        return cnt >= exactTarg - 0.5; // if there is no rpm data yet retern if the count is greater than a tuned value

    double gap = 60000000.0 / (rpm * cpr);  // time per count at the rpm
    double sinceLast = micros() - lastEdge; // differental

    double position = (cnt - 0.5) + sinceLast / gap; // predicted position

    double coast = CoastTime / gap; // distance per time divided by time gives distance of coast of the robot.
    
    //OdoAngCorrection = CurrentDir - ();

    //double degrees = value / (3.14159 / 180); // convert deg to radians
    //value = (radians * 5.8097) / ((6.5 * 3.14159) / (cpr)); // b = dist/count

    return position + coast >= exactTarg; // return when the speed and distance components are >= to the desired target.
}

void flushIR(unsigned long ms) // Mainly for testing if buffer too short for the IR to clear this will clear it and prevent another ir trigger
{
    unsigned long start = millis();
    while (millis() - start < ms)
    {
        oIR.GetKeyPressed();
    }
}

void clearDisp() // Just clears the display
{
    oLCD.setBackColor(RCB_PURPLE);
    oLCD.fillRect(0, 0, 200, 200, RCB_PURPLE);
    oLCD.setTextColor(RCB_WHITE, RCB_PURPLE);
}

void UpdateSCR(int type, double a, double b, double c, double d) // type 1:option1 2:option2 moving 3:option2 found target, Inclused many doubles to pass as strings for data
{
    if (type == 1) // option1
    {
        String a1 = String(a);
        String b1 = String(b);
        String c1 = String(c);
        String d1;

        if (d > 0)
        {
            d1 = "+" + String(d);
        }

        if (d < 0)
        {
            d1 = String(d);
        }

        clearDisp();
        oLCD.setTextColor(RCB_WHITE, RCB_PURPLE);
        oLCD.print("RIGHT DUTY CYCLE", CENTER, 8);
        oLCD.print(a1, CENTER, 28, 3);
        oLCD.print("LEFT COUNT", CENTER, 48);
        oLCD.print(b1, CENTER, 68, 5);
        oLCD.print("RIGHT COUNT", CENTER, 88);
        oLCD.print(c1, CENTER, 108, 5);
        oLCD.print("ERROR", CENTER, 128);
        oLCD.print(d1, CENTER, 148, 3);
    }
    if (type == 2) // option 2 moving
    {

        String a1 = String(a) + " CM";
        String b1 = String(b) + " DEGREES";
        String c1;
        String d1;

        clearDisp();
        oLCD.setTextColor(RCB_WHITE, RCB_PURPLE);
        oLCD.print("TARGET DISTANCE", CENTER, 48);
        oLCD.print(a1, CENTER, 62, 3);
        oLCD.print("TARGET ANGLE", CENTER, 76);
        oLCD.print(b1, CENTER, 90, 5);
    }
    if (type == 3) // option 2 found target
    {
        clearDisp();
        oLCD.setTextColor(RCB_WHITE, RCB_PURPLE);
        oLCD.print("TARGET FOUND", CENTER, 48);
    }
}

int targetN(double value, int move) // Distance and CountsPerRevolution No Delay
{
    if (move <= 2)
    {
        value = value / ((6.5 * 3.14159) / (cpr)); // b = dist/count
        excessN = value - (int)value;
        return value;
    }
    else
    {
        double radians = value * (3.14159 / 180); // convert deg to radians
        value = (radians * 5.8097) / ((6.5 * 3.14159) / (cpr)); // b = dist/count
        excessN = value - (int)value;
        return value;
    }
}

// Testing stuff

void Move(int type, int err, double adj, int duty) // type: 1 = Backwards, 2 = Forward, 3 = CCW, 4 = CW No Delay
{
    if (type == 1) // Backwards
    {
        digitalWrite(LeftMotorPPin, LOW);
        digitalWrite(LeftMotorEPin, HIGH);
        ledcWrite(PWMLeft, (duty)*maxDutyCycle / 100.0);
        digitalWrite(RightMotorPPin, HIGH);
        digitalWrite(RightMotorEPin, HIGH);
        ledcWrite(PWMRight, (duty + (adj * err)) * maxDutyCycle / 100.0);
    }
    if (type == 2) // Forward
    {
        digitalWrite(LeftMotorPPin, HIGH);
        digitalWrite(LeftMotorEPin, HIGH);
        ledcWrite(PWMLeft, (duty)*maxDutyCycle / 100.0);
        digitalWrite(RightMotorPPin, LOW);
        digitalWrite(RightMotorEPin, HIGH);
        ledcWrite(PWMRight, (duty + (adj * err)) * maxDutyCycle / 100.0);
    }
    if (type == 3) // CCW
    {
        digitalWrite(LeftMotorPPin, LOW);
        digitalWrite(LeftMotorEPin, HIGH);
        ledcWrite(PWMLeft, (duty)*maxDutyCycle / 100.0);
        digitalWrite(RightMotorPPin, LOW);
        digitalWrite(RightMotorEPin, HIGH);
        ledcWrite(PWMRight, (duty + (adj * err)) * maxDutyCycle / 100.0);
    }
    if (type == 4) // CW
    {
        digitalWrite(LeftMotorPPin, HIGH);
        digitalWrite(LeftMotorEPin, HIGH);
        ledcWrite(PWMLeft, (duty)*maxDutyCycle / 100.0);
        digitalWrite(RightMotorPPin, HIGH);
        digitalWrite(RightMotorEPin, HIGH);
        ledcWrite(PWMRight, (duty + (adj * err)) * maxDutyCycle / 100.0);
    }
}

void Stop() // Stops all motors and PWM signals No Delay
{
    digitalWrite(LeftMotorPPin, LOW);
    digitalWrite(LeftMotorEPin, LOW);
    ledcWrite(PWMLeft, 0);
    digitalWrite(RightMotorPPin, LOW);
    digitalWrite(RightMotorEPin, LOW);
    ledcWrite(PWMRight, 0);
}
void DutyAdj(double &adj, double &error, int count0, int count1) // No Delay
{
    error = count0 - count1;
    adj = 2 * error;
    if (adj > 10 || adj < -10)
    {
        if (adj > 0)
            adj = 10;
        if (adj < 0)
            adj = -10;
    }
}

void Odomitry(int move, double exactTarg)
{
    double Travel = ((countL + countR) / 2.0) * ((6.5 * 3.14159) / (cpr));
    delta = Travel - lastTravel;
    lastTravel = Travel;
    if (move == 2)
        CurrentDis += delta;
    if (move == 1)
        CurrentDis -= delta;
    if (move == 3)
        CurrentDir += (delta / 5.8097) / (3.14159 / 180);
    if (move == 4)
        CurrentDir -= (delta / 5.8097) / (3.14159 / 180);
}

void TranslateRobot(double dist, double duty, int move, int SCR, int debug) // Some Delay move = 5 determine direction of turning
{
    ResetTurnRPM();
    if (move == 5)
    {
        if (dist >= 0)
            move = 3;
        if (dist < 0)
        {
            dist = -1 * dist;
            move = 4;
        }
    }
    bool turning = (move == 3 || move == 4); // predictive stop is only used for turns

    int targ = targetN(dist, move);

    double exactTarg = targ + excessN;
    double adj = 0;
    
    double boost = 65;
    if(turning) boost = 40;

    bool b = true;
    doneL = false;
    doneR = false;

    error = 0;
    lastTravel = 0;
    countL = 0;
    countR = 0;
    count0 = 0;
    count1 = 0;
    count2 = 0;
    count3 = 0;
    rpm_L = 0;
    rpm_R = 0;

    c = millis();
    while (b == true)
    {

        double ramp = (millis() - c) / 150;
        if (ramp > 1.0)
        {
            ramp = 1.0;
        }
        double kick = boost * (1.0 - ramp);

        Move(move, 1, adj, (duty * ramp) + kick);

        if (doneL)
            ledcWrite(PWMLeft, 0);
        if (doneR)
            ledcWrite(PWMRight, 0);

        ReadRPM(rpm_L, 0);
        ReadRPM(rpm_R, 1);
        if (turning)
        {
            ReadTurnRPM(trpm_L, 0);
            ReadTurnRPM(trpm_R, 1);

            if (!doneL && PredictStop(0, exactTarg))
            {
                doneL = true;
                ledcWrite(PWMLeft, 0);
            }
            if (!doneR && PredictStop(1, exactTarg))
            {
                doneR = true;
                ledcWrite(PWMRight, 0);
            }
        }
        else // straight drives stop when both wheels reach the target count
        {
            if (countL >= targ && countR >= targ)
            {
                doneL = true;
                doneR = true;
            }
        }

        double gap_0 = countL - countR * wheelRatio;
        countoff = countR - countL;
        error = rpm_L - rpm_R * wheelRatio;

        adj = 2 * gap_0 + 0.1 * error;

        if (adj > 10 || adj < -10)
        {
            if (adj > 0)
                adj = 10;
            if (adj < 0)
                adj = -10;
        }

        t = micros();

        if (t - dt_1 > 1000000 && SCR >= 1)
        {
            UpdateSCR(SCR, (duty + adj), countL, countR, countoff);
            if (debug == 1)
            {
                oLCD.setBackColor(0x0000);
                oLCD.print(String(rpm_L), 5, 20);
                oLCD.print(String(rpm_R), 5, 40);
                oLCD.setBackColor(RCB_PURPLE);
            }
            dt_1 = t;
        }

        Odomitry(move,0);

        if ((doneL && doneR)) // CHANGED: was countR >= targ && countL >= targ
        {
            Stop();
            delay(200); // wait for counts to stop
            Odomitry(move,0);
            b = false;
        }
        Break(b);
    }
    if (SCR >= 1)
    {
        UpdateSCR(SCR, duty + adj, countL, countR, countoff);
    }
}
// *************************************************************************************
// END: OPTION1 SUPPORT
// *************************************************************************************

// *************************************************************************************
// START: OPTION2 SUPPORT
// *************************************************************************************
void acuScan(double &ang, double &dist, bool debug) // Needs configuring
{
    Tdist_min_0 = 10000;
    Tdist_min_1 = 10000;
    TargetFound = false;
    while (TargetFound == false)
    {
        for (int i = 0; i < 90 && TargetFound == false; i++)
        {

            servowrite(35, i + angle_offset - 45, 1);

            if (i == 0)
            {
                delay(1000);
            }
            else
            {
                delay(1);
            }

            Tdist = calculateDistance(38, 37);

            if (Tdist < Tdist_min_0)
            {
                if (Tdist >= 2)
                {
                    Tdist_min_0 = Tdist;
                    Tangle_0 = i;
                }
            }
            if(debug==true)
            {
            String text = String(i);
            String text1 = String(Tdist_min_0);
            String text2 = String(Tdist);
            oLCD.setBackColor(0x0000);
            oLCD.print(text, 20, 20);
            oLCD.print(text1, 20, 40);
            oLCD.print(text2, 20, 60);
            oLCD.setBackColor(RCB_PURPLE);
            }
            Break(TargetFound);
        }

        for (int i = 90; i > 0 && TargetFound == false; i--)
        {

            servowrite(35, i + angle_offset - 45, 1);

            if (i == 0)
            {
                delay(1000);
            }
            else
            {
                delay(1);
            }

            Tdist = calculateDistance(38, 37);

            if (Tdist < Tdist_min_1)
            {
                if (Tdist >= 2)
                {
                    Tdist_min_1 = Tdist;
                    Tangle_1 = i;
                }
            }
            if(debug==true)
            {
            String text = String(i);
            String text1 = String(Tdist_min_1);
            String text2 = String(Tdist);
            oLCD.setBackColor(0x0000);
            oLCD.print(text, 20, 20);
            oLCD.print(text1, 20, 40);
            oLCD.print(text2, 20, 60);
            oLCD.setBackColor(RCB_PURPLE);
            }
            Break(TargetFound);
            
        }
        TargetFound = true;
    }

    double AngleA = (Tangle_1 + Tangle_0) / 2;
    UpdateSCR(2, (Tdist_min_1 + Tdist_min_0) / 2, AngleA, 0, 0);

    ang = (Tangle_1 + Tangle_0) / 2;
    dist = (Tdist_min_1 + Tdist_min_0) / 2;

    if(debug==true)
    {
    String text = String((Tdist_min_1 + Tdist_min_0) / 2);
    String text1 = String(AngleA);
    oLCD.setBackColor(0x0000);
    oLCD.print(text1, 20, 20);
    oLCD.print(text, 20, 40);
    }
}

double relAngle(double ang, double exof, int reltype)
{
    if (reltype == 0) // Regular offset to find real angle
    {
        return ang - angle_offset + exof;
    }
    if (reltype == 1) // Use for currentDir vs Actual target direction
    {
        return (exof - angle_offset) - ang;
    }
}

void FollowUltra()
{
    double dist;
    double Target_Angle = 0;
    while (FoundTarget == false)
    {

        acuScan(Target_Angle, dist,false); // Scaning for target and getting distance and angle

        // Turn logic
        if (dist <= 2) // State to stop if target found
        {
            FoundTarget = true;
        }
        else if (dist > 150) // State to rotate if no target found
        {
            TranslateRobot(75, 30, 3, 0, 0); // turn left 75 degrees
        }
        else if (Target_Angle >= 80) // edge case on left side
        {
            TranslateRobot(45, 30, 3, 0, 0); // turn left 45 degrees
        }
        else if (Target_Angle <= 10) // edge case on right side
        {
            TranslateRobot(45, 30, 4, 0, 0); // turn right 45 degrees
        }
        else if (Target_Angle < 80 && Target_Angle > 10)
        {
            if (Target_Angle >= 55)
            {
                TranslateRobot(relAngle(Target_Angle, 0, 0), 30, 3, 0, 0); // turn left x degrees
            }
            else if (Target_Angle <= 35)
            {
                TranslateRobot(relAngle(Target_Angle, 0, 0), 30, 4, 0, 0); // turn right x degrees
            }
            else if (Target_Angle < 55 && Target_Angle > 35)
            {
                if (dist > 20)
                {
                    TranslateRobot(dist / 2, 30, 2, 0, 0); // go forward
                }
                else
                {
                    TranslateRobot(dist, 30, 2, 0, 0); // go forward
                }
            }
        }
        Break(FoundTarget);
    }
}
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

void PrintRadarScreen(bool initdisplay)
{
    // oLCD.LCDInitialize(LANDSCAPE, initdisplay);
    DrawBox(0, 0, 250, 140, RCB_PURPLE);
    int R = 75;
    int r = 35;
    oLCD.print("ANGLE ", 15, 10); //
    oLCD.setBackColor(RCB_BLACK);
    oLCD.printNumI(TargetDir, 65, 10, 3); //
    oLCD.setBackColor(RCB_PURPLE);
    oLCD.print(" DEGREES", 90, 10);
    oLCD.print("DISTANCE ", 25, 30); //
    oLCD.setBackColor(RCB_BLACK);
    oLCD.printNumI(TargetDis, 90, 30, 3); //
    oLCD.setBackColor(RCB_PURPLE);
    oLCD.print(" CM", 115, 30);

    printLine(120, 0, true);
    printLine(120, 30, true);
    printLine(120, 60, true);
    printLine(120, 90, true);
    printLine(120, 120, true);
    printLine(120, 150, true);
    printLine(120, 180, true);

    oLCD.drawCircle(oLCD.width() / 2, oLCD.height(), r, RCB_WHITE);
    oLCD.drawCircle(oLCD.width() / 2, oLCD.height(), R, RCB_WHITE);
}

void printLine(double distance, double angle, bool initial)
{
    double radians = angle * (PI / 180);
    distance = distance * .625;
    double x = distance * cos(radians);
    double y = distance * sin(radians);
    double x2 = (oLCD.width() / 2) + x;
    double y2 = (oLCD.height()) - y;
    if (initial == true)
    {
        oLCD.drawLine(oLCD.width() / 2, oLCD.height(), x2, y2, RCB_WHITE);
    }
    else
    {
        double xred = 75 * cos(radians);
        double yred = 75 * sin(radians);
        double x2red = (oLCD.width() / 2) + xred;
        double y2red = (oLCD.height()) - yred;
        oLCD.drawLine(x2, y2, x2red, y2red, RCB_RED);
        oLCD.drawLine(oLCD.width() / 2, oLCD.height(), x2, y2, RCB_BLUE);
    }
}

void ActualRadar(int radius, int i)
{
    int angle = i;
    if (radius > 120)
    {
        radius = 120;
    }
    printLine(radius, angle, false);
    oLCD.drawCircle(oLCD.width() / 2, oLCD.height(), 35, RCB_WHITE);
    oLCD.drawCircle(oLCD.width() / 2, oLCD.height(), 75, RCB_WHITE);
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