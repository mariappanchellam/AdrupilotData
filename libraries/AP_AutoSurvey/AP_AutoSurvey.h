#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Mission/AP_Mission.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>

// AP_AutoSurvey: autonomous timed lawnmower survey mission
//
// Parameters (prefix SURV_):
//   SURV_ENABLE   : 0=idle  1=run survey cycle (set from QGC to start)
//   SURV_LAT      : Centre latitude  (decimal degrees)
//   SURV_LON      : Centre longitude (decimal degrees)
//   SURV_ALT      : Survey altitude  (m AGL)
//   SURV_WIDTH    : Area width  E-W  (m)
//   SURV_LEN      : Area length N-S  (m)
//   SURV_SPACING  : Lane spacing     (m)
//   SURV_INTERVAL : Minutes between repeat surveys

class AP_AutoSurvey {
public:
    AP_AutoSurvey();

    static AP_AutoSurvey *get_singleton() { return _singleton; }

    void update();

    // called by vehicle main loop when this library requests a mode change
    bool wants_auto_mode() const  { return _want_auto; }
    bool wants_rtl_mode()  const  { return _want_rtl; }
    void on_mode_changed(uint8_t new_mode);
    void on_mission_complete();

    static const struct AP_Param::GroupInfo var_info[];

    CLASS_NO_COPY(AP_AutoSurvey);

private:
    static AP_AutoSurvey *_singleton;

    // ── parameters ────────────────────────────────────────────
    AP_Int8  _enable;
    AP_Float _lat;
    AP_Float _lon;
    AP_Float _alt;
    AP_Float _width;
    AP_Float _len;
    AP_Float _spacing;
    AP_Float _interval;

    // ── state machine ─────────────────────────────────────────
    enum class State : uint8_t { IDLE, BUILD, FLY, WAIT };
    State    _state       = State::IDLE;
    uint32_t _next_run_ms = 0;
    int8_t   _prev_enable = 0;
    bool     _want_auto   = false;
    bool     _want_rtl    = false;

    // ── copter mode numbers ───────────────────────────────────
    static constexpr uint8_t COPTER_MODE_AUTO = 3;
    static constexpr uint8_t COPTER_MODE_RTL  = 6;

    // ── helpers ───────────────────────────────────────────────
    bool build_mission();
    void send_text(MAV_SEVERITY sev, const char *fmt, ...) const;
    static float meters_to_lat(float m);
    static float meters_to_lon(float m, float lat_deg);
};
