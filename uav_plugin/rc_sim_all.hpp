#pragma once
// rc_sim_all.hpp - RC joystick control for simulator using SDL2

#include <SDL2/SDL.h>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>

namespace rc_sim {

// ---------- Utilities ----------
static inline int16_t map_i16(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max) {
    // avoid div by zero
    if (in_max == in_min) return out_min;
    const int32_t num = int32_t(x - in_min) * int32_t(out_max - out_min);
    const int32_t den = int32_t(in_max - in_min);
    return int16_t(out_min + num / den);
}

static inline float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    if (!(in_max > in_min)) return out_min;
    return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
}

static inline void logInfo(const char* s)  { std::cout << s << std::flush; }
static inline void logWarn(const char* s)  { std::cerr << s << std::flush; }

static inline float NaNf() {
    return std::numeric_limits<float>::quiet_NaN();
}

// ---------- Joystick layer ----------
inline SDL_Joystick* g_joystick = nullptr;

inline bool joystickInit() {
    static bool joystickInitialized = false;
    static bool warnShown = false;
    if (joystickInitialized) return true;

    if (SDL_WasInit(SDL_INIT_JOYSTICK) == 0) {
        if (SDL_Init(SDL_INIT_JOYSTICK) != 0) {
            logWarn(("SDL_Init(SDL_INIT_JOYSTICK) failed: " + std::string(SDL_GetError()) + "\n").c_str());
            return false;
        }
    }

    if (SDL_NumJoysticks() <= 0) {
        if (!warnShown) {
            logWarn("Joystick not found, begin waiting for joystick...\n");
            warnShown = true;
        }
        return false;
    }

    g_joystick = SDL_JoystickOpen(0);
    if (g_joystick != nullptr) {
        joystickInitialized = true;
        const char* name = SDL_JoystickName(g_joystick);
        std::string msg = "Joystick initialized: ";
        msg += (name ? name : "(unknown)");
        msg += "\n";
        logInfo(msg.c_str());
        return true;
    }

    if (!warnShown) {
        logWarn("Joystick open failed, begin waiting for joystick...\n");
        warnShown = true;
    }
    return false;
}

inline bool joystickGet(int16_t ch[16]) {
    if (!g_joystick) return false;
    SDL_JoystickUpdate();

    const int axes = SDL_JoystickNumAxes(g_joystick);
    for (int i = 0; i < 16; i++) {
        if (i < axes) ch[i] = SDL_JoystickGetAxis(g_joystick, i);
        else          ch[i] = 0; // missing axis -> center
    }
    return true;
}

// ---------- SBUS mock layer ----------
struct SBUSData {
    int16_t ch[16];
};

class SBUS {
public:
    SBUS() = default;

    void begin() {}

    bool read() { return joystickInit(); }
    SBUSData data() {
        SBUSData d{};
        joystickGet(d.ch);

        for (int i = 0; i < 16; i++) {
            d.ch[i] = map_i16(d.ch[i], (int16_t)-32768, (int16_t)32767, (int16_t)1000, (int16_t)2000);
        }
        return d;
    }
};

// ---------- RC logic ----------
inline SBUS rc;
inline uint16_t channels[16]{};     // raw RC channels [1000..2000]

// calibration arrays
inline float channelZero[16]{};
inline float channelMax[16]{};

// mapping channels
inline float rollChannel     = NaNf();
inline float pitchChannel    = NaNf();
inline float throttleChannel = NaNf();
inline float yawChannel      = NaNf();
inline float armChannel      = NaNf();

// reverse flags
inline bool rollReverse     = false;
inline bool pitchReverse     = true;   // Default: reversed
inline bool throttleReverse  = true;   // Default: reversed
inline bool yawReverse       = true;

// normalized control outputs [0..1] or NAN if unmapped
inline float controlRoll     = NaNf();
inline float controlPitch    = NaNf();
inline float controlYaw      = NaNf();
inline float controlThrottle = NaNf();
inline float controlArm      = NaNf();

inline void setupRC(bool setDefaultMapping = true) {
    logInfo("Setup RC\n");
    rc.begin();

    for (int i = 0; i < 16; i++) {
        channelZero[i] = 1000.0f;
        channelMax[i]  = 2000.0f;
    }

    if (setDefaultMapping) {
        rollChannel     = 3.0f;
        pitchChannel    = 4.0f;
        throttleChannel = 1.0f;
        yawChannel      = 0.0f;
        armChannel      = 2.0f;  // Channel 2 for arm/disarm toggle
    }
}

inline void normalizeRC() {
    float controls[16];
    for (int i = 0; i < 16; i++) {
        controls[i] = mapf((float)channels[i], channelZero[i], channelMax[i], 0.0f, 1.0f);
    }

    controlRoll     = (rollChannel     >= 0.0f) ? controls[(int)rollChannel]     : NaNf();
    controlPitch    = (pitchChannel    >= 0.0f) ? controls[(int)pitchChannel]    : NaNf();
    controlYaw      = (yawChannel      >= 0.0f) ? controls[(int)yawChannel]      : NaNf();
    controlThrottle = (throttleChannel >= 0.0f) ? controls[(int)throttleChannel] : NaNf();
    controlArm      = (armChannel      >= 0.0f) ? controls[(int)armChannel]      : NaNf();
}

inline bool readRC(float t_sec) {
    (void)t_sec;  // unused parameter
    if (rc.read()) {
        SBUSData d = rc.data();
        for (int i = 0; i < 16; i++) channels[i] = (uint16_t)d.ch[i];
        normalizeRC();
        return true;
    }
    return false;
}

} // namespace rc_sim

