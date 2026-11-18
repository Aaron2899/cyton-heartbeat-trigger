/*
  cyton_rwave_trigger.ino
  -----------------------
  On-board R-wave detection for the OpenBCI Cyton, generating a TTL trigger
  for a Digitimer DS7A so each stimulus lands in the ECG "safe zone"
  (after the T-wave, before the next QRS).

  Everything runs on the Cyton's PIC32; the PC/Bluetooth link is NOT in the
  timing path. Streaming to the PC keeps working for monitoring.

  Per sample (250 Hz) on the chosen channel:
    1. DC-blocking high-pass (~5 Hz)  -> removes electrode offset & drift,
                                          shrinks P/T waves relative to QRS
    2. 4-point moving average          -> low-pass, ~-27 dB at 60 Hz
    3. Rising/falling edge detection with hysteresis (TH_HIGH / TH_LOW)
    4. On the falling edge (QRS finished and validated): place a 300 ms
       block (100 ms baseline + 200 ms response) right after the estimated
       T-wave end, and only if it ends before the predicted next QRS minus
       a margin that grows with the measured beat-to-beat RR variability.
       The trigger is scheduled relative to the rising edge.
    5. Any new rising edge cancels a trigger that hasn't fired yet.
    6. After each trigger the input is blanked (held) so the stimulus
       artifact is never mistaken for a heartbeat.

  Outputs
    - TRIG_PIN (D18): 0.2 ms, 3.3 V positive pulse -> DS7A trigger input
      (use an opto-isolator / buffer, see notes that come with this file)
    - Aux data in the radio stream (BrainFlow "analog" channels):
        aux0 = 1 on the sample where a beat (rising edge) was detected
        aux1 = 1 on the first sample after a trigger fired
        aux2 = filtered detector signal in uV (for tuning thresholds;
               flat during artifact blanking)

  Build: Arduino IDE 1.8.19 + chipKIT core, board "OpenBCI 32", with the
  OpenBCI_32bit_Library and OpenBCI_Wifi_Master libraries installed
  (same setup as the DefaultBoard firmware).
*/

#include <DSPI.h>
#include <math.h>
#include <OpenBCI_Wifi_Master_Definitions.h>
#include <OpenBCI_Wifi_Master.h>
#include <OpenBCI_32bit_Library.h>
#include <OpenBCI_32bit_Library_Definitions.h>

// =========================== USER SETTINGS ===========================
#define ECG_CHANNEL          1      // Cyton channel with the ECG/ESG signal (1-8)
#define POLARITY             (+1)   // +1 if the filtered R-wave points up, -1 if down
#define AUTO_START_STREAM    0      // 1 = start detecting at power-up (standalone, no PC)
                                    // 0 = start with the PC ('b' from BrainFlow/GUI)
#define ADS_GAIN             24.0f  // must match the channel gain (default 24)

// ---- thresholds (uV, on the filtered signal) ----
#define USE_ADAPTIVE_THRESHOLD 1    // 0 = fixed thresholds below
#define FIXED_TH_HIGH_UV     300.0f
#define FIXED_TH_LOW_UV      150.0f
#define ADAPT_HIGH_FRAC      0.50f  // TH_HIGH = 50 % of running R amplitude
#define ADAPT_LOW_FRAC       0.25f  // TH_LOW  = 25 %  (hysteresis band)
#define MIN_TH_HIGH_UV       80.0f  // floor so noise alone can't trigger

// ---- safe-zone timing (ms, relative to the R-wave rising edge) ----
#define QTC_MS               0.0f // Bazett QT = QTc * sqrt(RR[s])  (T-wave end)
// Nader
#define T_MARGIN_MS          350     // Delay from heartbeat to begining of BASELINE
//#define T_MARGIN_MS          100     // extra time after estimated T-wave end (Bazett is approximate)
// Margin before the predicted next rising edge = QRS lead + K x beat-to-beat RR change.
// The T-wave end is tied to the beat just detected (predictable); the next beat is not,
// so the variable margin goes on that side.

// Nader
#define SAFE_ZONE            0      // use this flag to enable/disable the use of safe zone

#define QRS_ONSET_LEAD_MS    100    // QRS starts this long before the detected rising edge
#define RR_VAR_K             2.5f   // x mean |successive RR difference| of the last beats
#define PLACE_CENTERED       0      // 0 = block right after T-wave (recommended), 1 = centre
#define BASELINE_MS          100    // pre-stim baseline inside the safe zone
#define RESPONSE_MS          200    // post-stim analysis window inside the safe zone
#define LATENCY_COMP_MS      0      // measured front-end delay to subtract (calibrate)
#define STIM_EVERY_N_BEATS   1      // 1 = every eligible beat, 2 = every other, ...

// ---- beat validation (ms) ----

// Nader
#define REFRACTORY_MS        600    // (T_MARGIN_MS+BASELINE_MS+RESPONSE_MS) no new beat within this interval (100 bpm)
//#define REFRACTORY_MS        250    // no new beat within this time of a rising edge

#define QRS_MIN_MS           12     // rising->falling shorter than this = spike/noise
#define QRS_MAX_MS           130    // longer than this = artifact/saturation
#define RR_MIN_MS            400    // accept 40-150 bpm
#define RR_MAX_MS            1500
// Nader
#define RR_TOLERANCE         0.50f  // skip stim if last RR deviates >50 % from mean
#define RR_AVG_BEATS         3      // beats used for next-beat prediction and variability
//#define RR_TOLERANCE         0.20f  // skip stim if last RR deviates >20 % from mean
//#define RR_AVG_BEATS         8      // beats used for next-beat prediction and variability


// ---- trigger output ----
#define TRIG_PIN             18     // D18 header pin (free in default board mode)
#define TRIG_PULSE_US        200   // DS7A needs >= 5 us

// ---- stimulus-artifact blanking ----
// After each trigger the detector input is held at its last pre-stimulus value so
// the DS7A artifact can't reach the filters or be counted as a heartbeat.
// Blanking lasts at least BLANK_MIN_MS, then continues until the raw signal is back
// within BLANK_RETURN_FRAC x R-amplitude of the held value, but never past BLANK_MAX_MS.
#define BLANK_MIN_MS         20
#define BLANK_MAX_MS         100
#define BLANK_RETURN_FRAC    0.20f
// =====================================================================

const float FS = 250.0f;
const float MS_PER_SAMPLE = 1000.0f / FS;
const float UV_PER_COUNT = 4.5f / ADS_GAIN / 8388607.0f * 1.0e6f;
const float HP_A = 0.88f;           // 1st-order DC blocker: fc ~ (1-a)*fs/(2*pi) ~ 4.8 Hz

inline uint32_t msToSamples(float ms) { return (uint32_t)(ms / MS_PER_SAMPLE + 0.5f); }

// ---- filter state ----
bool  filtInit = false;
float xPrev = 0.0f, hp = 0.0f;
float maBuf[4] = {0, 0, 0, 0};
float maSum = 0.0f;
uint8_t maIdx = 0;
float yLast = 0.0f;

// ---- detector state ----
enum DetState { LEARN, ARMED, IN_QRS, REFRACTORY };
DetState st = LEARN;
uint32_t n = 0;                     // samples since (re)start
uint32_t riseN = 0, lastBeatN = 0;
uint32_t riseUs = 0;
bool     haveLastBeat = false;
float    qrsPeak = 0.0f;
float    ampEst = 0.0f;             // running R amplitude (uV, filtered)
uint32_t rrBuf[RR_AVG_BEATS];
uint8_t  rrCount = 0, rrIdx = 0;
uint32_t beatCount = 0;
uint32_t lastSampleMs = 0;

// ---- settings the PC can change while running (start at the #define values) ----
// Commands look like "(T350)" or "(N2)". '(' and ')' aren't used by the OpenBCI
// library, so every other character still goes to the library as before.
uint16_t tMarginMs  = T_MARGIN_MS;
uint16_t stimEveryN = STIM_EVERY_N_BEATS;
char     cmdBuf[12];
uint8_t  cmdLen = 0;
bool     inCmd  = false;

// ---- trigger state ----
bool     stimPending = false;
uint32_t stimAtUs = 0;
bool     pulseHigh = false;
uint32_t pulseStartUs = 0;
bool     markBeat = false, markStim = false;

// ---- blanking state ----
bool     blanking = false;
uint32_t blankStartUs = 0;
float    lastGoodUv = 0.0f;

inline bool timeReached(uint32_t t) { return (int32_t)(micros() - t) >= 0; }

void fireTrigger() {
  digitalWrite(TRIG_PIN, HIGH);
  pulseStartUs = micros();
  pulseHigh = true;
  stimPending = false;
  markStim = true;
  blanking = true;                         // hide the stimulus artifact from the detector
  blankStartUs = pulseStartUs;
}

// Returns the value to feed the detector: the raw sample normally, or the held
// pre-stimulus value while blanking.
float blankInput(float uv) {
  if (blanking) {
    uint32_t elapsedUs = micros() - blankStartUs;
    float returnBand = BLANK_RETURN_FRAC * ampEst;
    if (returnBand < MIN_TH_HIGH_UV) returnBand = MIN_TH_HIGH_UV;
    bool minDone  = elapsedUs >= (uint32_t)BLANK_MIN_MS * 1000UL;
    bool maxDone  = elapsedUs >= (uint32_t)BLANK_MAX_MS * 1000UL;
    bool returned = fabsf(uv - lastGoodUv) < returnBand;
    if (maxDone || (minDone && returned)) {
      blanking = false;
      // Timed out on a long artifact tail: restart the high-pass from the current
      // level instead of feeding it a step it would turn into a fake QRS.
      if (!returned) xPrev = uv;
    } else {
      return lastGoodUv;
    }
  }
  lastGoodUv = uv;
  return uv;
}

// Called often from loop(). Fire the trigger if it;s due and end the pulse after TRIG_PULSE_us
void serviceTrigger() {
  if (stimPending && (int32_t)(stimAtUs - micros()) <= 0) fireTrigger();
  if (pulseHigh && (uint32_t)(micros() - pulseStartUs) >= TRIG_PULSE_US){
    digitalWrite(TRIG_PIN, LOW);
    pulseHigh = false; 
    }
  }

void resetDetector() {
  filtInit = false; hp = 0.0f; maSum = 0.0f; maIdx = 0; yLast = 0.0f;
  for (int i = 0; i < 4; i++) maBuf[i] = 0.0f;
  st = LEARN; n = 0; ampEst = 0.0f; haveLastBeat = false;
  rrCount = 0; rrIdx = 0; stimPending = false;
  blanking = false;
}

void onRisingEdge() {
  markBeat = true;
  if (stimPending) stimPending = false;   // beat came early: drop the pending stim
}

void onFallingEdge() {
  uint32_t width = n - riseN;
  if (width < msToSamples(QRS_MIN_MS)) return;      // too narrow: noise spike

  // update running amplitude (clamp so one artifact can't blow it up)
  float pk = qrsPeak;
  // Nader
  if (ampEst > 0.0f && pk > 2.0f * ampEst) pk = 2.0f * ampEst;
  ampEst = 0.75f * ampEst + 0.25f * pk;
//  if (ampEst > 0.0f && pk > 3.0f * ampEst) pk = 3.0f * ampEst;
//  ampEst = 0.875f * ampEst + 0.125f * pk;

  beatCount++;
  bool rrOk = false;
  float rrLastMs = 0.0f;
  if (haveLastBeat) {
    rrLastMs = (riseN - lastBeatN) * MS_PER_SAMPLE;
    if (rrLastMs >= RR_MIN_MS && rrLastMs <= RR_MAX_MS) {
      rrBuf[rrIdx] = riseN - lastBeatN;
      rrIdx = (rrIdx + 1) % RR_AVG_BEATS;
      if (rrCount < RR_AVG_BEATS) rrCount++;
      rrOk = true;
    }
  }
  lastBeatN = riseN;
  haveLastBeat = true;

  if (!rrOk || rrCount < RR_AVG_BEATS) return;       // still learning heart rate

  // mean RR and mean |successive difference| over the buffer (oldest -> newest)
  float rrMeanMs = 0.0f, msdMs = 0.0f;
  for (int j = 0; j < RR_AVG_BEATS; j++) {
    uint32_t cur = rrBuf[(rrIdx + j) % RR_AVG_BEATS];
    rrMeanMs += cur;
    if (j > 0) {
      int32_t prev = rrBuf[(rrIdx + j - 1) % RR_AVG_BEATS];
      msdMs += fabsf((float)((int32_t)cur - prev));
    }
  }
  rrMeanMs = rrMeanMs / RR_AVG_BEATS * MS_PER_SAMPLE;
  msdMs    = msdMs / (RR_AVG_BEATS - 1) * MS_PER_SAMPLE;

  if (fabsf(rrLastMs - rrMeanMs) > RR_TOLERANCE * rrMeanMs) return;  // irregular beat

  // safe zone relative to this rising edge
  
// Nader
#if SAFE_ZONE
  float qtMs      = QTC_MS * sqrtf(rrMeanMs / 1000.0f);
  float safeStart = qtMs + tMarginMs;
  float safeEnd   = rrMeanMs - (QRS_ONSET_LEAD_MS + RR_VAR_K * msdMs);
  float slack     = (safeEnd - safeStart) - (BASELINE_MS + RESPONSE_MS);
  if (slack < 0.0f) return;   // heart rate too high for 300 ms
#else
  float stimMs = tMarginMs + BASELINE_MS;
#endif 

  if (beatCount % stimEveryN != 0) return;

// Nader
// #if PLACE_CENTERED
//  float stimMs = safeStart + BASELINE_MS + 0.5f * slack;
// #else
//  float stimMs = safeStart + BASELINE_MS;             // spare time goes before the next beat
// #endif

  uint32_t t = riseUs + (uint32_t)((stimMs - LATENCY_COMP_MS) * 1000.0f);
  if ((int32_t)(t - micros()) > 0) {
    stimAtUs = t;
    stimPending = true;
  }
}

void processSample(float uv) {
  // 1) high-pass (DC blocker); seed with first sample to avoid a startup step
  if (!filtInit) { xPrev = uv; filtInit = true; }
  hp = uv - xPrev + HP_A * hp;
  xPrev = uv;

  // 2) 4-point moving average
  maSum += hp - maBuf[maIdx];
  maBuf[maIdx] = hp;
  maIdx = (maIdx + 1) & 3;
  float y = POLARITY * 0.25f * maSum;
  yLast = y;
  n++;

  // 3) thresholds
#if USE_ADAPTIVE_THRESHOLD
  float thHigh = ADAPT_HIGH_FRAC * ampEst;
  if (thHigh < MIN_TH_HIGH_UV) thHigh = MIN_TH_HIGH_UV;
  float thLow = thHigh * (ADAPT_LOW_FRAC / ADAPT_HIGH_FRAC);
  // slowly relax if beats stop being found (e.g. amplitude dropped)
// Nader
  if (st != LEARN && (n - lastBeatN) > msToSamples(RR_MAX_MS)) ampEst *= 0.687f;
//  if (st != LEARN && (n - lastBeatN) > msToSamples(2000)) ampEst *= 0.999f;
#else
  float thHigh = FIXED_TH_HIGH_UV;
  float thLow  = FIXED_TH_LOW_UV;
#endif

  // 4) edge state machine
  switch (st) {
    case LEARN:    // 1 s settle, then 2 s of peak tracking to seed the threshold
      if (n > msToSamples(1000) && y > ampEst) ampEst = y;
      if (n >= msToSamples(3000)) { st = ARMED; lastBeatN = n; }
      break;

    case ARMED:
      if (y > thHigh) {                    // RISING EDGE
        st = IN_QRS;
        riseN = n;
        riseUs = micros();
        qrsPeak = y;
        onRisingEdge();
      }
      break;

    case IN_QRS:
      if (y > qrsPeak) qrsPeak = y;
      if (y < thLow) {                     // FALLING EDGE
        onFallingEdge();
        st = REFRACTORY;
      } else if (n - riseN > msToSamples(QRS_MAX_MS)) {
        st = REFRACTORY;                   // too wide: artifact, ignore
      }
      break;

    case REFRACTORY:
      if (n - riseN >= msToSamples(REFRACTORY_MS) && y < thLow) st = ARMED;
      break;
  }
}

// Apply one "(...)" command from the PC and reply, ending with "$$$" like the library does.
void runCommand() {
  cmdBuf[cmdLen] = '\0';
  long v = atol(cmdBuf + 1);
  bool ok = true;
  if (cmdBuf[0] == 'T' && v >= 0 && v <= 1400) {
    tMarginMs = v;
  } else if (cmdBuf[0] == 'N' && v >= 1 && v <= 65535) {
    stimEveryN = v;
  } else {
    ok = false;
  }
  if (board.streaming) return;            // never print text into the data stream
  board.printSerial(ok ? "OK " : "Rejected ");
  board.printSerial(cmdBuf);
  board.printSerial(" | T_MARGIN ");
  board.printSerial((int)tMarginMs);
  board.printSerial(" ms, every ");
  board.printSerial((int)stimEveryN);
  board.printSerial(" beat(s)");
  board.sendEOT();
}

// Characters from the PC: collect our "(...)" commands, pass everything else to the library.
void handleChar(char c) {
  if (c == '(') { inCmd = true; cmdLen = 0; return; }
  if (inCmd) {
    if (c == ')') { inCmd = false; runCommand(); }
    else if (cmdLen < sizeof(cmdBuf) - 1) cmdBuf[cmdLen++] = c;
    else inCmd = false; 
    return;
  }
  board.processChar(c);
}


void setup() {
  board.begin();
  board.useAccel(false);                   // send aux data (our markers) instead of accel
  wifi.begin(true, true);

  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

#if AUTO_START_STREAM
  board.streamStart();
#endif
}

void loop() {
  // a soft reset from the PC turns accel mode back on; keep our markers
  if (board.curAccelMode == board.ACCEL_MODE_ON) board.useAccel(false);

  serviceTrigger();

  if (board.streaming) {
    if (board.channelDataAvailable) {
      board.updateChannelData();

      // restart the detector after any gap in the data (stream stopped/restarted)
      uint32_t nowMs = millis();
      if (nowMs - lastSampleMs > 100) resetDetector();
      lastSampleMs = nowMs;

      processSample(blankInput(board.boardChannelDataInt[ECG_CHANNEL - 1] * UV_PER_COUNT));

      board.auxData[0] = markBeat ? 1 : 0;
      board.auxData[1] = markStim ? 1 : 0;
      float y = yLast;
      if (y > 32767.0f) y = 32767.0f;
      if (y < -32767.0f) y = -32767.0f;
      board.auxData[2] = (short)y;
      markBeat = false;
      markStim = false;

      serviceTrigger();                
      if (stimPending && (int32_t)(stimAtUs - micros())<4000){ //if the trigger is due in the next 4 ms, dont send the data packet
        while (!timeReached(stimAtUs)) { }
        fireTrigger();
        board.sampleCounter++; //make sure a gap is recorded for the dropped packet
      } else{
        board.sendChannelData();
      }
    }
  }

  serviceTrigger();

  if (board.hasDataSerial0()) handleChar(board.getCharSerial0());
  if (board.hasDataSerial1()) handleChar(board.getCharSerial1());

  board.loop();
  wifi.loop();
  if (wifi.hasData()) board.processCharWifi(wifi.getChar());
  if (!wifi.sentGains && wifi.present && wifi.tx) {
    wifi.sendGains(board.numChannels, board.getGains());
  }
}
