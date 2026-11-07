#include "lcdhelperv2.h"
#include "irhelper.h"
#include <math.h>
// B allows us to use colors
#define RCB_RED 0x001F
#define RCB_WHITE 0xFFFF
#define RCB_BLACK 0x0000
#define RCB_PURPLE 0x780F
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
bool TargetFound = false;
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
const int PWMLeft  = 1;
const int PWMRight = 2;
// A
const int pwmFrequency = 496;
const int pwmResolution = 8;
const int maxDutyCycle = (int)pow(2, pwmResolution) - 1;
// B
double rightEncoderCount;
double leftEncoderCount;
double wheelDiameter = 65.31; // mm
double countsPerRevolution = 20;
double wheelCircumference = (wheelDiameter) * (3.14159);
double revolutionstotarget = 100 / (wheelCircumference / 10);
double counttotarget = countsPerRevolution * revolutionstotarget;
double error = 0;

// duty[motor][move]: motor 0 = left, 1 = right
// move: 0 Left CW     1 Left CCW     2 Right CW     3 Right CCW
//       4 Both Fwd    5 Both Back    6 Robot CW     7 Robot CCW
double duty[2][8] = {0};
double stab[2];

double timerL;
double timerR;

double Tdist = 10000;
double Tdist_min_0 = 10000;
double Tdist_min_1 = 10000;
int Tangle_0;
int Tangle_1;

int DutyCycle;

unsigned long tStart = 0;   // when the current window began
double lo = 0;              // lowest duty in this window
double hi = 0;              // highest duty in this window
double sum = 0;             // for the average duty over the window
long n = 0;

double f;
double g;

//
//  Define protypes for functions
//
void ShowDisplay(screen val, char optionstate, char keypressed, bool initdisplay = false);
// *************************************************************************************
// BEGIN: USER FUNCTION
// *************************************************************************************

//*************************************************************************************
// BEGIN: Basic Quality of Life Functions
//*************************************************************************************

void flushIR(unsigned long ms)
{
    unsigned long start = millis();
    while(millis() - start < ms)
    {
        oIR.GetKeyPressed();
    }
}

//Testing stuff

double findStuff(double hilo, double val)
{
    if(f==0) f = val; // Set state
    if(g==0) g = val; // Set state

    if(hilo==0) // Compare and find the lowest value
    {
        if(val<f) // If val < f then f = val
        {
            f = val;
        }
    }

    if(hilo==1) // Compare and find the lowest value
    {
        if(val>f) // If val < f then f = val
        {
            f = val;
        }
    }
}

unsigned long StableTime(double dutyNow, double rpmNow, int Trpm)
{
    const double DutyTotal = 3.0;    // allowed spread (max - min) of duty in the 5 sec window, %

    unsigned long now = millis();

    double newLo = findStuff(0, dutyNow);
    double newHi = findStuff(1, dutyNow);

    if(n == 0 || newHi - newLo > DutyTotal)
    {
        tStart = now;             // restart the 5 s window from here
        newLo = dutyNow;
        newHi = dutyNow;
        sum = 0;
        n = 0;
    }

    lo = newLo;
    hi = newHi;
    sum += dutyNow;
    n++;

    return now - tStart;
}

bool DutyValidate(double dutyc[], int side, int direction, int rot, unsigned long t)
{
    if(t<5000) return false;

    // direction 1 = CW / Forward,  direction 0 = CCW / Backward
    if(side==0 || side==1)
    {
        int opt = side*2 + (1 - direction);    // 0 L CW, 1 L CCW, 2 R CW, 3 R CCW
        duty[side][opt]   = dutyc[side];
        duty[1-side][opt] = 0;                 // other motor is off for this move
    }
    if(side==2)
    {
        int opt = 4 + rot*2 + (1 - direction); // 4 Fwd, 5 Back, 6 Robot CW, 7 Robot CCW
        duty[0][opt] = dutyc[0];
        duty[1][opt] = dutyc[1];
    }
    return true;
}

//Joeys Version of Dutycycle Tuner

void DutyCycleTune(int side,int d,int direction,int Trpm,int rot) // side: Left Motor,Right Motor, Both // d: DutyCycle
{                                                                 // direction: Forward 1 / Backward 0 / CW 1 / CCW 1
    bool up = true;                                               // Trpm: Target Rotations Per Min / rot: Rotating 1 / Non-Rotating 0
    bool up1 = true;

    double t_0 = 0;
    double t_1 = 0;
    double dt = 0;
    double error = 0;
    double rpm_0 = 0;
    double tave[10];
    double tsum = 0;

    double t1_0 = 0;    // 2nd Count for if we need 2 at once
    double t1_1 = 0;
    double dt1 = 0;
    double error1 = 0;
    double rpm1_0 = 0;
    double tave1[10];
    double tsum1 = 0;

    double sumarray = 0;

    int count = 0;
    int count1 = 0;
    
    int op1;
    int op2;

    if(side == 0 || side == 1)  //Left side or Right side one motor rotate on other wheel aka: Slow and accurate turn
    {
        if(side==0)
        {
            op1 = LeftOutputPin;
        }
        if(side==1)
        {
            op1 = RightOutputPin;
        }
        bool b = true;
        while(b==true)
        {
            if(side==0)
            {
                if(direction==1) // Forward
                {
                    digitalWrite(LeftMotorPPin, LOW);
                    digitalWrite(LeftMotorEPin, HIGH);
                    ledcWrite(PWMLeft, (d + error) * maxDutyCycle / 100.0);
                }
                if(direction==0)  //Backward
                {
                    digitalWrite(LeftMotorPPin, HIGH);
                    digitalWrite(LeftMotorEPin, HIGH);
                    ledcWrite(PWMLeft, (d + error) * maxDutyCycle / 100.0);
                }
            }
            else if(side==1)
            {
                if(direction==1) // Forward
                {
                    digitalWrite(RightMotorPPin, HIGH);
                    digitalWrite(RightMotorEPin, HIGH);
                    ledcWrite(PWMRight, (d + error) * maxDutyCycle / 100.0);
                }
                if(direction==0) // Backward
                {
                    digitalWrite(RightMotorEPin, LOW);
                    digitalWrite(RightMotorEPin, HIGH);
                    ledcWrite(PWMRight, (d + error) * maxDutyCycle / 100.0);
                }
            }

            unsigned long t_0 = millis();
            unsigned long t = millis();

            dt = (t_0 - t_1);

            if(digitalRead(op1) == LOW && up == true)
            {
                up = false;
                if(dt > 10)
                {
                    t_1 = t_0;
                    tave[count] = dt;
                    count++;
                }
            }

            if(digitalRead(op1) == HIGH && up == false)
            {
                up = true;
            }

            if(count>=10)
            {   
                tsum = 0;

                for(int i = 0;i < 10;i++)
                {
                    tsum += tave[i];
                }

                tsum = tsum/10;

                rpm_0 = (60000) / (tsum * 20);

                for(int i = 0;i < 10;i++)
                {
                    tave[i] = tave[i+1];
                }

                tave[9] = 0;

                error += (Trpm - rpm_0) * 2 / 100;
                if(d + error > 100) error = 100 - d;
                if(d + error < 0)   error = -d;

                count--;
            }

            String rpm = String(rpm_0);
            String err = String(error);
            oLCD.setBackColor(0x0000);
            oLCD.print(rpm,5,20);
            oLCD.print(err,5,40);

            if (oIR.GetKeyPressed() == KEY_POWER)
            {
                digitalWrite(LeftMotorPPin, LOW);
                digitalWrite(LeftMotorEPin, LOW);
                ledcWrite(PWMLeft, 0);
                digitalWrite(RightMotorPPin, LOW);
                digitalWrite(RightMotorEPin, LOW);
                ledcWrite(PWMRight, 0);
                b = false;
            }

            unsigned long t_v = StableTime(stab[side], rpm_0, Trpm);
            double dutyc[2] = {0, 0};
            sumarray += stab[side];
            dutyc[side] = sumarray/n;   // average duty over the window

            if(DutyValidate(dutyc, side, direction, rot, t))
            {
                b = false;
            }
        }
    }

    if(side==2)
    {
        bool b = true;
        op1 = LeftOutputPin;
        op2 = RightOutputPin;
        while(b==true)
        {
            if(direction==1 && rot==0) // Forward
            {
                digitalWrite(LeftMotorPPin, LOW);
                digitalWrite(LeftMotorEPin, HIGH);
                ledcWrite(PWMLeft, (d + error) * maxDutyCycle / 100.0);

                digitalWrite(RightMotorPPin, HIGH);
                digitalWrite(RightMotorEPin, HIGH);
                ledcWrite(PWMRight, (d + error1) * maxDutyCycle / 100.0);
            }
            if(direction==0 && rot==0)  //Backward
            {
                digitalWrite(LeftMotorPPin, HIGH);
                digitalWrite(LeftMotorEPin, HIGH);
                ledcWrite(PWMLeft, (d + error) * maxDutyCycle / 100.0);

                digitalWrite(RightMotorPPin, LOW);
                digitalWrite(RightMotorEPin, HIGH);
                ledcWrite(PWMRight, (d + error1) * maxDutyCycle / 100.0);
            }
            if(direction==1 && rot==1) // CW
            {
                digitalWrite(LeftMotorPPin, HIGH);
                digitalWrite(LeftMotorEPin, HIGH);
                ledcWrite(PWMLeft, (d + error) * maxDutyCycle / 100.0);

                digitalWrite(RightMotorPPin, HIGH);
                digitalWrite(RightMotorEPin, HIGH);
                ledcWrite(PWMRight, (d + error1) * maxDutyCycle / 100.0);
            }
            if(direction==0 && rot==1)  //CCW
            {
                digitalWrite(LeftMotorPPin, LOW);
                digitalWrite(LeftMotorEPin, HIGH);
                ledcWrite(PWMLeft, (d + error) * maxDutyCycle / 100.0);

                digitalWrite(RightMotorPPin, LOW);
                digitalWrite(RightMotorEPin, HIGH);
                ledcWrite(PWMRight, (d + error1) * maxDutyCycle / 100.0);
            }
            unsigned long t_0 = millis();
            unsigned long t1_0 = millis();
            unsigned long t = millis();

            dt = (t_0 - t_1);
            dt1 = (t1_0 - t1_1);

            if(digitalRead(op1) == LOW && up == true)
            {
                up = false;
                if(dt > 10)
                {
                    t_1 = t_0;
                    tave[count] = dt;
                    count++;
                }
            }

            if(digitalRead(op1) == HIGH && up == false)
            {
                up = true;
            }

            if(count>=10)
            {   
                tsum = 0;

                for(int i = 0;i < 10;i++)
                {
                    tsum += tave[i];
                }

                tsum = tsum/10;

                rpm_0 = (60000) / (tsum * 20);

                for(int i = 0;i < 10;i++)
                {
                    tave[i] = tave[i+1];
                }

                tave[9] = 0;

                error += (Trpm - rpm_0) * 2 / 100;
                if(d + error > 100) error = 100 - d;
                if(d + error < 0)   error = -d;

                count--;
            }

            if(digitalRead(op2) == LOW && up1 == true)
            {
                up1 = false;
                if(dt1 > 10)
                {
                    t1_1 = t1_0;
                    tave1[count1] = dt1;
                    count1++;
                }
            }

            if(digitalRead(op2) == HIGH && up1 == false)
            {
                up1 = true;
            }

            if(count1>=10)
            {   
                tsum1 = 0;

                for(int i = 0;i < 10;i++)
                {
                    tsum1 += tave1[i];
                }

                tsum1 = tsum1/10;

                rpm1_0 = (60000) / (tsum1 * 20);

                for(int i = 0;i < 10;i++)
                {
                    tave1[i] = tave1[i+1];
                }

                tave1[9] = 0;

                error1 += (Trpm - rpm1_0) * 2 / 100;
                if(d + error1 > 100) error1 = 100 - d;
                if(d + error1 < 0)   error1 = -d;

                count1--;
            }

            String rpm = String(rpm_0);
            String err = String(error);
            oLCD.setBackColor(0x0000);
            oLCD.print(rpm,5,20);
            oLCD.print(err,5,40);

            String rpm1 = String(rpm1_0);
            String err1 = String(error1);
            oLCD.setBackColor(0x0000);
            oLCD.print(rpm1,5,60);
            oLCD.print(err1,5,80);

            if (oIR.GetKeyPressed() == KEY_POWER)
            {
                digitalWrite(LeftMotorPPin, LOW);
                digitalWrite(LeftMotorEPin, LOW);
                ledcWrite(PWMLeft, 0);
                digitalWrite(RightMotorPPin, LOW);
                digitalWrite(RightMotorEPin, LOW);
                ledcWrite(PWMRight, 0);
                b = false;
            }
        }
    }
    return;
}

void clearDisp()
{
    oLCD.fillRect(0, 0, 200, 200, RCB_PURPLE);
}

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
    }
    else if (optionstate == 'B' && aval.toInt() <= 40 || optionstate == 'B' && aval.toInt() >= 70)
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
        DutyCycleTune(0,aval.toInt(),0,50,0);
        flushIR(1000);
        DutyCycleTune(0,aval.toInt(),1,50,0);
        flushIR(1000);
        DutyCycleTune(1,aval.toInt(),0,50,0);
        flushIR(1000);
        DutyCycleTune(1,aval.toInt(),1,50,0);
        flushIR(1000);
        DutyCycleTune(2,aval.toInt(),0,50,0);
        flushIR(1000);
        DutyCycleTune(2,aval.toInt(),1,50,0);
        flushIR(1000);
        DutyCycleTune(2,aval.toInt(),0,50,1);
        flushIR(1000);
        DutyCycleTune(2,aval.toInt(),1,50,1);
        //PWMRampTest(0);
         /*
        leftEncoderCount = 0;
        rightEncoderCount = 0;
        digitalWrite(LeftMotorPPin, LOW);
        digitalWrite(LeftMotorEPin, HIGH);
        ledcWrite(PWMLeft, (float)aval.toInt() * maxDutyCycle / 100.0);
        digitalWrite(RightMotorPPin, HIGH);
        digitalWrite(RightMotorEPin, HIGH);
        ledcWrite(PWMRight, (float)aval.toInt() * maxDutyCycle / 100.0);
        int duty = aval.toInt();
        while (leftEncoderCount < counttotarget || rightEncoderCount < counttotarget)
        {
            oLCD.printNumI(67, CENTER, 54, 3, ' ');
            ledcWrite(PWMRight, duty * maxDutyCycle / 100.0);
            if (digitalRead(LeftOutputPin) == LOW)
            {
                oLCD.printNumI(420, CENTER, 32, 3, ' ');
                leftEncoderCount++;
                timerL = millis();
            }
            if (digitalRead(RightOutputPin) == LOW)
            {
                rightEncoderCount++;
                timerR = millis();
            }
            error = leftEncoderCount - rightEncoderCount;
            if (duty < 80 && duty > 20)
            {
                if (error < 0)
                {
                    duty++;
                }
                else if (error > 0)
                {
                    duty--;
                }
            }
            if (oIR.GetKeyPressed() == KEY_POWER)
            {
                oLCD.printNumI(676967, CENTER, 32, 3, ' ');
                break;
            }
            oLCD.setTextColor(RCB_WHITE, RCB_PURPLE);
            oLCD.print("RIGHT DUTY CYCLE", CENTER, 8);
            oLCD.printNumI(duty, CENTER, 28, 3, ' ');
            oLCD.print("LEFT COUNT", CENTER, 48);
            oLCD.printNumI(leftEncoderCount, CENTER, 68, 5, ' ');
            oLCD.print("RIGHT COUNT", CENTER, 88);
            oLCD.printNumI(rightEncoderCount, CENTER, 108, 5, ' ');
            oLCD.print("ERROR", CENTER, 128);
            oLCD.printNumI(error, CENTER, 148, 3, ' ');
            delay(1000);
        }
        digitalWrite(LeftMotorPPin, LOW);
        digitalWrite(LeftMotorEPin, LOW);
        digitalWrite(RightMotorPPin, LOW);
        digitalWrite(RightMotorEPin, LOW);
    }
    */
    }
}
//
//  Option2 code
//
void Option2(char optionstate, char keypressed, bool initdisplay)
{
    Tdist = 10000;
    Tdist_min_0 = 10000;
    TargetFound = false;
    if (TargetFound == false)
    {
        for (int i = 0; i < 90; i++)
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
            String text = String(i);
            String text1 = String(Tdist_min_0);
            String text2 = String(Tdist);
            oLCD.setBackColor(0x0000);
            oLCD.print(text, 20, 20);
            oLCD.print(text1, 20, 40);
            oLCD.print(text2, 20, 60);
        }

        Tdist = 10000;
        for (int i = 90; i > 0; i--)
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
            String text = String(i);
            String text1 = String(Tdist_min_1);
            String text2 = String(Tdist);
            oLCD.setBackColor(0x0000);
            oLCD.print(text, 20, 20);
            oLCD.print(text1, 20, 40);
            oLCD.print(text2, 20, 60);
        }
        if (Tdist_min_1 <= 150 && Tdist_min_1 >= 5)
        {
            TargetFound = true;
        }
    }
    String text = String((Tdist_min_1 + Tdist_min_0) / 2);
    String text1 = String((Tangle_1 + Tangle_0) / 2);
    oLCD.setBackColor(0x0000);
    oLCD.print(text1, 20, 20);
    oLCD.print(text, 20, 40);
    servowrite(35, (Tangle_1 + Tangle_0) / 2, 10);
    delay(3000);
}
//
//  Option3 code
//
void Option3(char optionstate, char keypressed, bool initdisplay)
{
    firstcomp = false;
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
    ledcSetup(PWMLeft,  pwmFrequency, pwmResolution);
    ledcSetup(PWMRight, pwmFrequency, pwmResolution);
    ledcAttachPin(LeftMotorEPin,  PWMLeft);
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
        clearDisp();
        oLCD.print("PREPARING", CENTER, 10);
        oLCD.print("TO", CENTER, 24);
        oLCD.setBackColor(RCB_PURPLE);
        oLCD.print("ACQUIRE", CENTER, 38);
        oLCD.print("TARGET", CENTER, 52);
        firstcomp = false;
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
// *************************************************************************************
// START: OPTION1 SUPPORT
// *************************************************************************************

// *************************************************************************************
// END: OPTION1 SUPPORT
// *************************************************************************************

// *************************************************************************************
// START: OPTION2 SUPPORT
// *************************************************************************************

double findDist()
{
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
// *************************************************************************************
// END: OPTION3 SUPPORT
// *************************************************************************************
// *************************************************************************************
// START: OPTION4 SUPPORT
// *************************************************************************************

// *************************************************************************************
// END: OPTION4 SUPPORT
// *************************************************************************************
