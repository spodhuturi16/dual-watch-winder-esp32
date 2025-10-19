#pragma once

/********** WIFI **********/
static const char* WIFI_SSID = "YOUR_WIFI";
static const char* WIFI_PASS = "YOUR_PASS";
static const char* AP_SSID   = "Winder-Setup";
static const char* AP_PASS   = "";  // open AP

/********** HARDWARE **********/
// 28BYJ-48 math
static const long STEPS_PER_REV = 4096;

/*  ULN2003 IN1..IN4  →  ESP32 GPIOs (safe choices for common DevKit/NodeMCU-32S)
    Avoid input-only (34–39) and tricky boot strap pins (0, 2, 12, 15).  */

/// Motor 1 (left)
static const int M1_IN1 = 13;
static const int M1_IN2 = 12;
static const int M1_IN3 = 14;
static const int M1_IN4 = 27;

/// Motor 2 (right)
static const int M2_IN1 = 26;
static const int M2_IN2 = 25;
static const int M2_IN3 = 33;
static const int M2_IN4 = 32;

/// 3-position DPDT selector (to GND; INPUT_PULLUP on pins)
static const int MODE_PIN_A = 16;
static const int MODE_PIN_B = 17;

/// Status LED (optional). GPIO4 avoids boot issues on some boards.
static const int LED_PIN = 4;   // set to -1 to disable

/********** BEHAVIOR (UI controls only TPD + direction) **********/
static int STEP_RPM = 15;   // internal motor speed; tweak if chatter

static int TPD_M1 = 650;    // per-motor TPD default
static int TPD_M2 = 650;

enum DirectionPlan : int { DIR_CW = +1, DIR_CCW = -1, DIR_ALT = 0 };
static int DIRPLAN_M1 = DIR_ALT;  // CW / CCW / Alternate
static int DIRPLAN_M2 = DIR_ALT;

static const unsigned long MODE_DEBOUNCE_MS = 40;
