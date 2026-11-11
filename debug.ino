// ============================================================================
//  debug.ino (v2) - background debug monitor for project_2
//
//  Prints what the robot is doing to the Serial Monitor WITHOUT changing any
//  of your code. A separate task on CPU core 0 (your loop runs on core 1)
//  watches your globals, the encoder counts and the motor outputs every 2 ms.
//
//  ADD:     put this file in the project_2 folder (replace the old debug.ino)
//           and add   DebugBegin();   as the last line of setup().
//  REMOVE:  delete this file and that one line.
//
//  It reads these from your code. If you delete or rename one, change it here:
//    countL, countR, LLUs, LRUs, c, cpr, maxDutyCycle, PWMLeft, PWMRight,
//    LeftMotorPPin, RightMotorPPin, rpm_L, rpm_R, trpm_L, trpm_R,
//    doneL, doneR, CoastTime, excessN, CurrentDir, CurrentDis,
//    firstcomp, thirdcomp, nav, ScanDone, FoundTarget, TargetFound,
//    CascadeExit, TargetDir, TargetDis, DutyCycle, oLCD.SCREEN_STATE
//
//  Output (one block per TranslateRobot call):
//    [START]  values just before the move
//    [RUN]    one line every time either wheel gets a new count
//    [CUT]    a wheel was switched off (doneL / doneR), with the prediction it used
//    [STOP]   both motors are off
//    [COAST]  counts that arrive after the motors are off
//    [REST]   where each wheel settled, how far it coasted, and a CoastTime that
//             would have fit this turn
//    [FLAG]   any state variable that changes, at any time
//    [IDLE]   counts while no move is running (turn a wheel by hand to test an encoder)
// ============================================================================

const int DBG_SAMPLE_MS = 2; // how often the monitor looks
const int DBG_REST_MS = 300; // counts unchanged this long after the motors stop = at rest

// Last flag values seen, so [FLAG] only prints what changed
int dbgScreen;
bool dbgFirst, dbgThird, dbgNav, dbgScanDone, dbgFound2, dbgFound3, dbgCascade;
double dbgTDir, dbgTDis;
int dbgDuty;

// What each wheel was doing when it was cut (for the [REST] summary)
long dbgCutCntL = -1;  // count when the left wheel was cut (-1 = not cut)
long dbgCutCntR = -1;  // count when the right wheel was cut
double dbgCutPosL = 0; // estimated position at the cut, in counts
double dbgCutPosR = 0;
double dbgCutGapL = 0; // us per count at the cut (0 = count-based cut, no speed)
double dbgCutGapR = 0;

// ---------------------------------------------------------------------------
//  Small helpers
// ---------------------------------------------------------------------------
double DbgCmPerCount()
{
    return (6.5 * 3.14159) / cpr;
}

double DbgDegPerCount()
{
    return DbgCmPerCount() / 5.8097 * (180 / 3.14159);
}

// Turns: degrees of spin. Drives: cm.
double DbgUnits(int move, double counts)
{
    if (move >= 3)
        return counts * DbgDegPerCount();
    return counts * DbgCmPerCount();
}

const char *DbgUnitName(int move)
{
    if (move >= 3)
        return "deg";
    return "cm";
}

const char *DbgMoveName(int move)
{
    if (move == 1)
        return "BACK";
    if (move == 2)
        return "FWD";
    if (move == 3)
        return "CCW";
    if (move == 4)
        return "CW";
    return "?";
}

// Works out the move from the phase pins, using the same table as your Move()
int DbgReadMove()
{
    int l = digitalRead(LeftMotorPPin);
    int r = digitalRead(RightMotorPPin);
    if (l == HIGH && r == LOW)
        return 2; // Forward
    if (l == LOW && r == HIGH)
        return 1; // Backwards
    if (l == LOW && r == LOW)
        return 3; // CCW
    return 4;     // CW
}

// Duty (%) the PWM hardware is actually outputting on a channel
double DbgDutyPct(int chan)
{
    return ledcRead(chan) * 100.0 / maxDutyCycle;
}

// ---------------------------------------------------------------------------
//  [FLAG] watcher: prints any state variable that changes
// ---------------------------------------------------------------------------
void DbgWatchB(const char *name, bool &last, bool now)
{
    if (now != last)
    {
        Serial.printf("[FLAG]  %s: %d -> %d\n", name, last, now);
        last = now;
    }
}

void DbgWatchI(const char *name, int &last, int now)
{
    if (now != last)
    {
        Serial.printf("[FLAG]  %s: %d -> %d\n", name, last, now);
        last = now;
    }
}

void DbgWatchD(const char *name, double &last, double now)
{
    if (now != last)
    {
        Serial.printf("[FLAG]  %s: %.2f -> %.2f\n", name, last, now);
        last = now;
    }
}

void DbgWatchFlags()
{
    DbgWatchI("SCREEN_STATE", dbgScreen, (int)oLCD.SCREEN_STATE);
    DbgWatchB("firstcomp", dbgFirst, firstcomp);
    DbgWatchB("thirdcomp", dbgThird, thirdcomp);
    DbgWatchB("nav", dbgNav, nav);
    DbgWatchB("ScanDone", dbgScanDone, ScanDone);
    DbgWatchB("FoundTarget", dbgFound2, FoundTarget);
    DbgWatchB("TargetFound", dbgFound3, TargetFound);
    DbgWatchB("CascadeExit", dbgCascade, CascadeExit);
    DbgWatchD("TargetDir", dbgTDir, TargetDir);
    DbgWatchD("TargetDis", dbgTDis, TargetDis);
    DbgWatchI("DutyCycle", dbgDuty, DutyCycle);
}

// ---------------------------------------------------------------------------
//  [CUT]: a wheel was just switched off. Redo the same sum PredictStop does
//  so you can see what it was thinking, and remember it for [REST].
// ---------------------------------------------------------------------------
void DbgCut(int side, unsigned long ms)
{
    long cnt;
    unsigned long lastEdge;
    double trpm;
    if (side == 0)
    {
        cnt = countL;
        lastEdge = LLUs;
        trpm = trpm_L;
    }
    else
    {
        cnt = countR;
        lastEdge = LRUs;
        trpm = trpm_R;
    }
    char name = (side == 0) ? 'L' : 'R';

    double pos = 0;
    double gap = 0;

    if (trpm > 0) // predictive cut (turns)
    {
        gap = 60000000.0 / (trpm * cpr);
        pos = (cnt - 0.5) + (micros() - lastEdge) / gap;
        Serial.printf("[CUT]   %c at %lu ms  count=%ld  trpm=%.1f (%.0f us/count)  position=%.2f + coast %.2f = predicted final %.2f\n",
                      name, ms, cnt, trpm, gap, pos, CoastTime / gap, pos + CoastTime / gap);
    }
    else // count-based cut (straight drives, or not enough counts for a speed yet)
    {
        Serial.printf("[CUT]   %c at %lu ms  count=%ld  (count-based, no speed estimate)\n", name, ms, cnt);
    }

    if (side == 0)
    {
        dbgCutCntL = cnt;
        dbgCutPosL = pos;
        dbgCutGapL = gap;
    }
    else
    {
        dbgCutCntR = cnt;
        dbgCutPosR = pos;
        dbgCutGapR = gap;
    }
}

// ---------------------------------------------------------------------------
//  [REST]: where the wheels settled
// ---------------------------------------------------------------------------
void DbgRestWheel(char name, long cutCnt, double cutPos, double cutGap, long rest)
{
    if (cutCnt < 0)
    {
        Serial.printf("[REST]  %c  rest at %ld  (never cut by done%c - stopped by Stop())\n", name, rest, name);
    }
    else if (cutGap > 0)
    {
        double coast = rest - cutPos; // counts it actually rolled after the cut
        Serial.printf("[REST]  %c  cut at %ld, rest at %ld  coasted %.2f counts (CoastTime allowed %.2f)  -> CoastTime that fits: %.0f us\n",
                      name, cutCnt, rest, coast, CoastTime / cutGap, coast * cutGap);
    }
    else
    {
        Serial.printf("[REST]  %c  cut at %ld, rest at %ld  coasted %ld counts\n", name, cutCnt, rest, rest - cutCnt);
    }
}

void DbgPrintRest(int move, long restL, long restR, double dir, double dis, bool cutShort)
{
    Serial.printf("[REST]  %s%s  cL=%ld cR=%ld  counts say %.1f %s   CurrentDir=%.2f CurrentDis=%.2f\n",
                  DbgMoveName(move), cutShort ? " (next move started before wheels settled)" : "",
                  restL, restR, DbgUnits(move, (restL + restR) / 2.0), DbgUnitName(move), dir, dis);
    DbgRestWheel('L', dbgCutCntL, dbgCutPosL, dbgCutGapL, restL);
    DbgRestWheel('R', dbgCutCntR, dbgCutPosR, dbgCutGapR, restR);
}

// ---------------------------------------------------------------------------
//  The monitor task
// ---------------------------------------------------------------------------
void DebugTask(void *arg)
{
    unsigned long seenC = c; // TranslateRobot sets c = millis() at the start of every move
    int state = 0;           // 0 = idle, 1 = moving, 2 = coasting after both motors are off
    int move = 0;
    bool powered = false;
    bool seenDoneL = false;
    bool seenDoneR = false;
    long lastL = -1;
    long lastR = -1;
    unsigned long stopMs = 0;
    unsigned long changeMs = 0;
    double lastDir = CurrentDir;
    double lastDis = CurrentDis;

    TickType_t wait = pdMS_TO_TICKS(DBG_SAMPLE_MS);
    if (wait < 1)
        wait = 1;

    for (;;)
    {
        DbgWatchFlags();

        long cl = countL;
        long cr = countR;
        double dL = DbgDutyPct(PWMLeft);
        double dR = DbgDutyPct(PWMRight);
        unsigned long now = millis();

        // A new TranslateRobot call has started
        if (c != seenC)
        {
            if (state == 2) // the previous move was still coasting
                DbgPrintRest(move, lastL, lastR, lastDir, lastDis, true);
            seenC = c;
            state = 1;
            powered = false;
            move = 0;
            lastL = -1;
            lastR = -1;
            seenDoneL = false;
            seenDoneR = false;
            dbgCutCntL = -1;
            dbgCutCntR = -1;
            dbgCutGapL = 0;
            dbgCutGapR = 0;
            Serial.println("------------------------------------------------------------");
            Serial.printf("[START] before: CurrentDir=%.2f CurrentDis=%.2f   excessN=%.3f   CoastTime=%.0f us\n",
                          lastDir, lastDis, excessN, CoastTime);
            Serial.println("[RUN]   ms, move, cL, cR, dutyL%, dutyR%, R-L, rpmL, rpmR, trpmL, trpmR, CurrentDir, CurrentDis");
        }

        if (state == 1) // moving
        {
            if (!powered && (dL > 0 || dR > 0))
            {
                powered = true;
                move = DbgReadMove();
            }

            // A wheel was just switched off
            if (doneL && !seenDoneL)
            {
                seenDoneL = true;
                DbgCut(0, now - seenC);
            }
            if (doneR && !seenDoneR)
            {
                seenDoneR = true;
                DbgCut(1, now - seenC);
            }

            if (cl != lastL || cr != lastR)
            {
                lastL = cl;
                lastR = cr;
                Serial.printf("[RUN]   %lu, %s, %ld, %ld, %.1f, %.1f, %+.1f, %.1f, %.1f, %.1f, %.1f, %.2f, %.2f\n",
                              now - seenC, DbgMoveName(move), cl, cr, dL, dR, dR - dL,
                              rpm_L, rpm_R, trpm_L, trpm_R, CurrentDir, CurrentDis);
            }

            if (powered && dL == 0 && dR == 0) // both motors are off
            {
                state = 2;
                stopMs = now;
                changeMs = now;
                Serial.printf("[STOP]  %s at %lu ms  cL=%ld cR=%ld  CurrentDir=%.2f CurrentDis=%.2f\n",
                              DbgMoveName(move), now - seenC, cl, cr, CurrentDir, CurrentDis);
            }
        }
        else if (state == 2) // coasting after the motors are off
        {
            if (cl != lastL || cr != lastR)
            {
                lastL = cl;
                lastR = cr;
                changeMs = now;
                Serial.printf("[COAST] +%lu ms  cL=%ld cR=%ld\n", now - stopMs, cl, cr);
            }
            if (now - changeMs >= DBG_REST_MS)
            {
                DbgPrintRest(move, cl, cr, CurrentDir, CurrentDis, false);
                state = 0;
            }
        }
        else // idle: show counts when a wheel is turned by hand
        {
            if (cl != lastL || cr != lastR)
            {
                lastL = cl;
                lastR = cr;
                Serial.printf("[IDLE]  cL=%ld cR=%ld\n", cl, cr);
            }
        }

        if (state != 1) // remember where things stood before the next move starts
        {
            lastDir = CurrentDir;
            lastDis = CurrentDis;
        }

        vTaskDelay(wait);
    }
}

// ---------------------------------------------------------------------------
//  Call once, as the last line of setup()
// ---------------------------------------------------------------------------
void DebugBegin()
{
    dbgScreen = (int)oLCD.SCREEN_STATE;
    dbgFirst = firstcomp;
    dbgThird = thirdcomp;
    dbgNav = nav;
    dbgScanDone = ScanDone;
    dbgFound2 = FoundTarget;
    dbgFound3 = TargetFound;
    dbgCascade = CascadeExit;
    dbgTDir = TargetDir;
    dbgTDis = TargetDis;
    dbgDuty = DutyCycle;

    Serial.println();
    Serial.println("=== DEBUG MONITOR v2 ON ===");
    Serial.printf("1 count = %.3f cm of wheel travel = %.2f deg of spin turn  (cpr=%d)\n",
                  DbgCmPerCount(), DbgDegPerCount(), cpr);
    Serial.printf("[STATE] screen=%d firstcomp=%d thirdcomp=%d nav=%d ScanDone=%d FoundTarget=%d TargetFound=%d CascadeExit=%d\n",
                  dbgScreen, dbgFirst, dbgThird, dbgNav, dbgScanDone, dbgFound2, dbgFound3, dbgCascade);
    Serial.printf("[STATE] TargetDir=%.2f TargetDis=%.2f DutyCycle=%d CurrentDir=%.2f CurrentDis=%.2f CoastTime=%.0f us\n",
                  TargetDir, TargetDis, DutyCycle, CurrentDir, CurrentDis, CoastTime);

    xTaskCreatePinnedToCore(DebugTask, "debug", 6144, NULL, 1, NULL, 0);
}
