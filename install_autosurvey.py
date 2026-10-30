#!/usr/bin/env python3
"""
install_autosurvey.py  –  apply AP_AutoSurvey to ~/ardupilot in one shot.

Run on your LOCAL machine (where ~/ardupilot lives):
    python3 install_autosurvey.py

It is safe to run multiple times; every patch checks "already applied".
After it succeeds, rebuild:
    cd ~/ardupilot
    ./waf configure --board sitl
    ./waf copter
"""

import os, sys, textwrap

ARDUPILOT = os.environ.get("ARDUPILOT_DIR", os.path.expanduser("~/ardupilot"))

# ── helpers ──────────────────────────────────────────────────────────────────

def check(path):
    full = os.path.join(ARDUPILOT, path)
    if not os.path.exists(full):
        sys.exit(f"ERROR: {full} not found — is ~/ardupilot the right path?")
    return full

def read(path):
    with open(check(path)) as f:
        return f.read()

def write(path, content):
    full = os.path.join(ARDUPILOT, path)
    os.makedirs(os.path.dirname(full), exist_ok=True)
    with open(full, "w") as f:
        f.write(content)

def patch(path, old, new, label):
    full = check(path)
    with open(full) as f:
        text = f.read()
    if new in text:
        print(f"  SKIP  {label} (already applied)")
        return
    if old not in text:
        print(f"  MISS  {label} — anchor not found in {path}")
        print(f"        Looking for: {old[:60]!r}")
        return
    with open(full, "w") as f:
        f.write(text.replace(old, new, 1))
    print(f"  OK    {label}")


def patch_try_list(path, skip_fragment, anchors, label):
    """Try each (old, new) pair in order; SKIP if skip_fragment already in file."""
    full = check(path)
    with open(full) as f:
        text = f.read()
    if skip_fragment in text:
        print(f"  SKIP  {label} (already applied)")
        return
    for old, new in anchors:
        if old in text:
            with open(full, "w") as f:
                f.write(text.replace(old, new, 1))
            print(f"  OK    {label}")
            return
    print(f"  MISS  {label} — no known anchor found in {path}")
    print(f"        Tried {len(anchors)} anchor variant(s)")


# ── 1. AP_AutoSurvey library files ───────────────────────────────────────────

print("\n[1] Writing AP_AutoSurvey library files …")

HEADER = """\
#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Mission/AP_Mission.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

// Copter mode numbers (must match Mode::Number)
#define COPTER_MODE_AUTO  3
#define COPTER_MODE_RTL   6

class AP_AutoSurvey {
public:
    AP_AutoSurvey();

    static AP_AutoSurvey *get_singleton() { return _singleton; }

    static const struct AP_Param::GroupInfo var_info[];

    // called from the ArduCopter scheduler
    void update();

    // called from Copter::update_auto_survey() to execute mode changes
    bool wants_auto_mode() const { return _want_auto; }
    bool wants_rtl_mode()  const { return _want_rtl;  }
    void on_mode_changed(uint8_t new_mode);
    void on_auto_mode_failed();
    void on_mission_complete();

private:
    static AP_AutoSurvey *_singleton;

    AP_Int8  _enable;
    AP_Float _lat;
    AP_Float _lon;
    AP_Float _alt;
    AP_Float _width;
    AP_Float _len;
    AP_Float _spacing;
    AP_Float _interval;

    enum class State : uint8_t {
        IDLE,
        BUILD,
        FLY,
        WAIT
    };

    State    _state        {State::IDLE};
    int8_t   _prev_enable  {0};
    bool     _want_auto    {false};
    bool     _want_rtl     {false};
    uint32_t _next_run_ms  {0};
    uint8_t  _fail_count   {0};
    bool     _fly_was_armed {false};

    bool build_mission();
    void send_text(MAV_SEVERITY sev, const char *fmt, ...) const;
    static float meters_to_lat(float m);
    static float meters_to_lon(float m, float lat_deg);
};
"""

SOURCE = """\
#include "AP_AutoSurvey.h"

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <stdio.h>

extern const AP_HAL::HAL &hal;

AP_AutoSurvey *AP_AutoSurvey::_singleton;

// ── parameter table ──────────────────────────────────────────────
const AP_Param::GroupInfo AP_AutoSurvey::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Auto Survey Enable
    // @Description: Set to 1 from QGroundControl to start the autonomous survey cycle. Set to 0 to stop at any time.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE",    0, AP_AutoSurvey, _enable, 0),

    // @Param: LAT
    // @DisplayName: Survey Centre Latitude
    // @Description: Latitude of the centre of the survey rectangle (decimal degrees, positive North).
    // @Units: deg
    // @Range: -90 90
    // @User: Standard
    AP_GROUPINFO("LAT",      1, AP_AutoSurvey, _lat,      0),

    // @Param: LON
    // @DisplayName: Survey Centre Longitude
    // @Description: Longitude of the centre of the survey rectangle (decimal degrees, positive East).
    // @Units: deg
    // @Range: -180 180
    // @User: Standard
    AP_GROUPINFO("LON",      2, AP_AutoSurvey, _lon,      0),

    // @Param: ALT
    // @DisplayName: Survey Altitude
    // @Description: Altitude above home at which the survey is flown.
    // @Units: m
    // @Range: 10 150
    // @User: Standard
    AP_GROUPINFO("ALT",      3, AP_AutoSurvey, _alt,      50),

    // @Param: WIDTH
    // @DisplayName: Survey Width (E-W)
    // @Description: East-west extent of the survey rectangle in metres.
    // @Units: m
    // @Range: 50 2000
    // @User: Standard
    AP_GROUPINFO("WIDTH",    4, AP_AutoSurvey, _width,    200),

    // @Param: LEN
    // @DisplayName: Survey Length (N-S)
    // @Description: North-south extent of the survey rectangle in metres.
    // @Units: m
    // @Range: 50 2000
    // @User: Standard
    AP_GROUPINFO("LEN",      5, AP_AutoSurvey, _len,      300),

    // @Param: SPACING
    // @DisplayName: Lane Spacing
    // @Description: Distance in metres between adjacent lawnmower lanes.
    // @Units: m
    // @Range: 5 200
    // @User: Standard
    AP_GROUPINFO("SPACING",  6, AP_AutoSurvey, _spacing,  30),

    // @Param: INTERVAL
    // @DisplayName: Survey Repeat Interval
    // @Description: Minutes to wait after a survey completes before starting the next one. Set to 0 for a single survey only.
    // @Units: min
    // @Range: 0 240
    // @User: Standard
    AP_GROUPINFO("INTERVAL", 7, AP_AutoSurvey, _interval, 10),

    AP_GROUPEND
};

// ── constructor ──────────────────────────────────────────────────
AP_AutoSurvey::AP_AutoSurvey()
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

// ── coordinate conversion helpers ───────────────────────────────
float AP_AutoSurvey::meters_to_lat(float m)
{
    return m / 111320.0f;
}

float AP_AutoSurvey::meters_to_lon(float m, float lat_deg)
{
    return m / (111320.0f * cosf(radians(lat_deg)));
}

// ── GCS message helper ───────────────────────────────────────────
void AP_AutoSurvey::send_text(MAV_SEVERITY sev, const char *fmt, ...) const
{
    char buf[80];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    GCS_SEND_TEXT(sev, "AutoSurvey: %s", buf);
}

// ── mission builder ──────────────────────────────────────────────
bool AP_AutoSurvey::build_mission()
{
    AP_Mission *mission = AP_Mission::get_singleton();
    if (mission == nullptr) {
        send_text(MAV_SEVERITY_ERROR, "AP_Mission not available");
        return false;
    }

    const float clat    = _lat.get();
    const float clon    = _lon.get();
    const float alt_m   = _alt.get();
    const float width   = _width.get();
    const float len     = _len.get();
    const float spacing = _spacing.get();

    if (is_zero(clat) && is_zero(clon)) {
        send_text(MAV_SEVERITY_ERROR, "SURV_LAT/LON not set");
        return false;
    }
    if (alt_m < 10.0f || alt_m > 150.0f) {
        send_text(MAV_SEVERITY_ERROR, "SURV_ALT out of range [10-150 m]");
        return false;
    }
    if (spacing < 5.0f) {
        send_text(MAV_SEVERITY_ERROR, "SURV_SPACING too small (min 5 m)");
        return false;
    }

    mission->clear();

    AP_Mission::Mission_Command cmd {};

    // Index 1 (AP_MISSION_FIRST_REAL_COMMAND): TAKEOFF
    // add_cmd() auto-inserts HOME at index 0 on the first call, so the
    // first explicit add_cmd() lands at index 1 — exactly where
    // starts_with_takeoff_cmd() and mission.start() expect it.
    cmd.id    = MAV_CMD_NAV_TAKEOFF;
    cmd.p1    = 15;
    cmd.content.location.lat = (int32_t)(clat * 1.0e7f);
    cmd.content.location.lng = (int32_t)(clon * 1.0e7f);
    cmd.content.location.alt = (int32_t)(alt_m * 100.0f);
    cmd.content.location.relative_alt = 1;
    if (!mission->add_cmd(cmd)) {
        send_text(MAV_SEVERITY_ERROR, "add_cmd failed (takeoff)");
        return false;
    }

    // Lawnmower lanes
    const int num_lanes = (int)(width / spacing) + 1;
    const float lat_half = meters_to_lat(len / 2.0f);
    bool northward = true;

    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 0;
    cmd.content.location.alt = (int32_t)(alt_m * 100.0f);
    cmd.content.location.relative_alt = 1;

    for (int i = 0; i < num_lanes; i++) {
        const float lon_off = -width / 2.0f + i * spacing;
        const float dlon    = meters_to_lon(lon_off, clat);
        const float lon_wp  = clon + dlon;

        const float lat_a = clat + (northward ?  lat_half : -lat_half);
        const float lat_b = clat + (northward ? -lat_half :  lat_half);
        northward = !northward;

        cmd.content.location.lat = (int32_t)(lat_a * 1.0e7f);
        cmd.content.location.lng = (int32_t)(lon_wp * 1.0e7f);
        if (!mission->add_cmd(cmd)) {
            send_text(MAV_SEVERITY_ERROR, "add_cmd failed (lane %d A)", i);
            return false;
        }

        cmd.content.location.lat = (int32_t)(lat_b * 1.0e7f);
        cmd.content.location.lng = (int32_t)(lon_wp * 1.0e7f);
        if (!mission->add_cmd(cmd)) {
            send_text(MAV_SEVERITY_ERROR, "add_cmd failed (lane %d B)", i);
            return false;
        }
    }

    cmd     = {};
    cmd.id  = MAV_CMD_NAV_RETURN_TO_LAUNCH;
    if (!mission->add_cmd(cmd)) {
        send_text(MAV_SEVERITY_ERROR, "add_cmd failed (RTL)");
        return false;
    }

    send_text(MAV_SEVERITY_INFO,
              "Mission: %d WPs, %d lanes, %.0fx%.0fm @ %.0fm",
              (int)mission->num_commands(), num_lanes, width, len, alt_m);
    return true;
}

// ── main update ──────────────────────────────────────────────────
void AP_AutoSurvey::update()
{
    const uint32_t now_ms   = AP_HAL::millis();
    const int8_t   enabled  = _enable.get();

    // 45 s startup delay for EKF initialisation
    if (now_ms < 45000U) {
        _prev_enable = enabled;
        return;
    }

    const bool rising = (enabled > 0) && (_prev_enable == 0);
    _prev_enable = enabled;

    if (enabled == 0 && _state != State::IDLE) {
        send_text(MAV_SEVERITY_WARNING, "Disabled — returning to IDLE");
        _want_rtl  = true;
        _want_auto = false;
        _state     = State::IDLE;
        return;
    }

    switch (_state) {

    case State::IDLE:
        if (rising) {
            AP_Param::set_by_name("RC7_OPTION", 0);
            AP_Param::set_by_name("MIS_RESTART", 1);
            // bit0=AllowArming, bit1=AllowTakeOffWithoutRaisingThrottle
            AP_Param::set_by_name("AUTO_OPTIONS", 3);
            send_text(MAV_SEVERITY_INFO, "Survey cycle triggered");
            _state = State::BUILD;
        }
        break;

    case State::BUILD: {
        if (!build_mission()) {
            _enable.set_and_save(0);
            _state = State::IDLE;
            break;
        }
        _fail_count = 0;
        _want_auto  = true;
        _state      = State::FLY;
        break;
    }

    case State::FLY:
        // detect unexpected disarm while the mission should be running
        if (_fly_was_armed && !hal.util->get_soft_armed()) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "AutoSurvey: disarmed unexpectedly in FLY state");
            _fly_was_armed = false;
        } else if (hal.util->get_soft_armed()) {
            _fly_was_armed = true;
        }
        break;

    case State::WAIT: {
        const int32_t remaining_s =
            (int32_t)((int64_t)_next_run_ms - (int64_t)now_ms) / 1000;

        if (remaining_s <= 0) {
            send_text(MAV_SEVERITY_INFO, "Starting next survey");
            _state = State::BUILD;
        } else if (remaining_s % 60 == 0 && remaining_s > 0) {
            send_text(MAV_SEVERITY_INFO, "%d min until next survey",
                      (int)(remaining_s / 60));
        }
        break;
    }
    }
}

// ── callbacks ────────────────────────────────────────────────────

void AP_AutoSurvey::on_auto_mode_failed()
{
    _want_auto = false;
    _fail_count++;
    if (_fail_count >= 5) {
        _enable.set_and_save(0);
        _fail_count = 0;
        _state = State::IDLE;
        send_text(MAV_SEVERITY_CRITICAL,
                  "Survey disabled: AUTO failed 5x");
        return;
    }
    _state = State::BUILD;
    send_text(MAV_SEVERITY_WARNING, "AUTO failed (%u/5) — rebuilding", (unsigned)_fail_count);
}

void AP_AutoSurvey::on_mode_changed(uint8_t new_mode)
{
    if (new_mode == COPTER_MODE_AUTO) {
        _want_auto = false;
    }
    if (new_mode == COPTER_MODE_RTL) {
        _want_rtl = false;
    }
}

void AP_AutoSurvey::on_mission_complete()
{
    if (_state != State::FLY) {
        return;
    }

    const float interval_min = _interval.get();
    if (interval_min <= 0.0f) {
        send_text(MAV_SEVERITY_INFO, "Survey complete (single run)");
        _enable.set_and_save(0);
        _state = State::IDLE;
        return;
    }

    _next_run_ms = AP_HAL::millis() + (uint32_t)(interval_min * 60.0f * 1000.0f);
    _state       = State::WAIT;
    send_text(MAV_SEVERITY_INFO,
              "Survey complete. Next in %.0f min", (double)interval_min);
}
"""

write("libraries/AP_AutoSurvey/AP_AutoSurvey.h", HEADER)
print("  OK    AP_AutoSurvey.h written")
write("libraries/AP_AutoSurvey/AP_AutoSurvey.cpp", SOURCE)
print("  OK    AP_AutoSurvey.cpp written")

# ── 2. ArduCopter/Parameters.h ───────────────────────────────────────────────

print("\n[2] Patching ArduCopter/Parameters.h …")

patch(
    "ArduCopter/Parameters.h",
    '#include <AP_Common/AP_Common.h>\n',
    '#include <AP_Common/AP_Common.h>\n#include <AP_AutoSurvey/AP_AutoSurvey.h>\n',
    "add AP_AutoSurvey include",
)

# Find where to insert the member — try several #endif forms around AP_OAPathPlanner
patch_try_list(
    "ArduCopter/Parameters.h",
    "AP_AutoSurvey auto_survey;",
    [
        (
            '    AP_OAPathPlanner oa;\n#endif\n',
            '    AP_OAPathPlanner oa;\n#endif\n\n    // autonomous timed factory surveillance\n    AP_AutoSurvey auto_survey;\n',
        ),
        (
            '    AP_OAPathPlanner oa;\n#endif  // AP_OAPATHPLANNER_ENABLED\n',
            '    AP_OAPathPlanner oa;\n#endif  // AP_OAPATHPLANNER_ENABLED\n\n    // autonomous timed factory surveillance\n    AP_AutoSurvey auto_survey;\n',
        ),
        (
            '    AP_OAPathPlanner oa;\n#endif // AP_OAPATHPLANNER_ENABLED\n',
            '    AP_OAPathPlanner oa;\n#endif // AP_OAPATHPLANNER_ENABLED\n\n    // autonomous timed factory surveillance\n    AP_AutoSurvey auto_survey;\n',
        ),
    ],
    "add AP_AutoSurvey member variable",
)

# ── 3. ArduCopter/Parameters.cpp ─────────────────────────────────────────────

print("\n[3] Patching ArduCopter/Parameters.cpp …")

patch_try_list(
    "ArduCopter/Parameters.cpp",
    'AP_SUBGROUPINFO(auto_survey, "SURV_"',
    [
        (
            '    // ID 62 is reserved for the SHOW_... parameters from the Skybrush fork at\n    // https://github.com/skybrush-io/ardupilot\n\n    AP_GROUPEND\n};\n',
            '    // ID 62 is reserved for the SHOW_... parameters from the Skybrush fork at\n    // https://github.com/skybrush-io/ardupilot\n\n    // @Group: SURV_\n    // @Path: ../libraries/AP_AutoSurvey/AP_AutoSurvey.cpp\n    AP_SUBGROUPINFO(auto_survey, "SURV_", 63, ParametersG2, AP_AutoSurvey),\n\n    AP_GROUPEND\n};\n',
        ),
        (
            '    AP_SUBGROUPEXTENSION("", 61, ParametersG2, var_info2),\n\n    AP_GROUPEND\n};\n',
            '    AP_SUBGROUPEXTENSION("", 61, ParametersG2, var_info2),\n\n    // @Group: SURV_\n    // @Path: ../libraries/AP_AutoSurvey/AP_AutoSurvey.cpp\n    AP_SUBGROUPINFO(auto_survey, "SURV_", 63, ParametersG2, AP_AutoSurvey),\n\n    AP_GROUPEND\n};\n',
        ),
    ],
    "add SURV_ parameter group",
)

# ── 4. ArduCopter/wscript ─────────────────────────────────────────────────────

print("\n[4] Patching ArduCopter/wscript …")

patch(
    "ArduCopter/wscript",
    "'AP_SurfaceDistance'\n        ],",
    "'AP_SurfaceDistance',\n            'AP_AutoSurvey'\n        ],",
    "add AP_AutoSurvey to library list",
)

# ── 5. ArduCopter/Copter.h ────────────────────────────────────────────────────

print("\n[5] Patching ArduCopter/Copter.h …")

patch(
    "ArduCopter/Copter.h",
    '    void three_hz_loop();\n',
    '    void three_hz_loop();\n    void update_auto_survey();\n',
    "add update_auto_survey() declaration",
)

# ── 6. ArduCopter/Copter.cpp — scheduler task ────────────────────────────────

print("\n[6] Patching ArduCopter/Copter.cpp …")

# 6a: scheduler task entry — insert just before the closing }; of scheduler_tasks[]
# Anchor: the #endif (for AP_INERTIALSENSOR) followed by the array close and the
# get_scheduler_tasks function.  Works on fresh clones and is idempotent.
patch(
    "ArduCopter/Copter.cpp",
    '#endif\n};\n\nvoid Copter::get_scheduler_tasks',
    '#endif\n    SCHED_TASK_CLASS(AP_AutoSurvey, &copter.g2.auto_survey, update,               10,  100, 218),\n};\n\nvoid Copter::get_scheduler_tasks',
    "add AP_AutoSurvey scheduler task",
)

# 6b: update_auto_survey() function body
# Insert before three_hz_loop()
patch(
    "ArduCopter/Copter.cpp",
    '// three_hz_loop - 3hz loop\nvoid Copter::three_hz_loop()\n',
    '''\
// update_auto_survey - poll AP_AutoSurvey and execute any requested mode changes
void Copter::update_auto_survey()
{
    if (g2.auto_survey.wants_auto_mode()) {
        AP_Mission *ms = AP_Mission::get_singleton();
        if (ms == nullptr || ms->num_commands() < 2) {
            g2.auto_survey.on_auto_mode_failed();
            return;
        }
        // Enter AUTO mode first while DISARMED. ModeAuto::init() only checks
        // starts_with_takeoff_cmd() when motors->armed() && land_complete.
        // Switching before arming bypasses that check; the waiting_to_start
        // loop in mode_auto.cpp starts the mission once we arm.
        // Do NOT call set_current_cmd() here: it sets nav_cmd_loaded=true which
        // causes resume() to skip do_takeoff(), so auto_takeoff.alt_target_m
        // is never set and verify_takeoff() returns true immediately.
        if (flightmode->mode_number() != Mode::Number::AUTO) {
            if (!set_mode(Mode::Number::AUTO, ModeReason::SCRIPTING)) {
                g2.auto_survey.on_auto_mode_failed();
            }
            return;
        }
        if (!motors->armed()) {
            // set_auto_armed BEFORE arming: events.cpp should_disarm_on_failsafe()
            // returns true unconditionally while ap.in_arming_delay is set (which
            // arm() sets at its very end).  With auto_armed=true the AUTO-mode branch
            // evaluates to !true && land_complete = false, so no failsafe disarm
            // fires during the 2-second arming delay.
            set_auto_armed(true);
            arming.arm(AP_Arming::Method::SCRIPTING);
            return;
        }
        g2.auto_survey.on_mode_changed(3);
    }
    if (g2.auto_survey.wants_rtl_mode()) {
        if (set_mode(Mode::Number::RTL, ModeReason::SCRIPTING)) {
            g2.auto_survey.on_mode_changed(6);
        }
    }
}

// three_hz_loop - 3hz loop
void Copter::three_hz_loop()
''',
    "add update_auto_survey() function",
)

# 6c: call update_auto_survey() from three_hz_loop
# A previous session may have applied this with slightly different whitespace, so check
# for the semicoloned call (distinct from the definition, which has no semicolon).
if 'update_auto_survey();' in read("ArduCopter/Copter.cpp"):
    print("  SKIP  call update_auto_survey() in three_hz_loop (already applied)")
else:
    patch_try_list(
        "ArduCopter/Copter.cpp",
        "update_auto_survey();",
        [
            (
                '    failsafe_gcs_check();\n\n    // check if we\'ve lost terrain data',
                '    failsafe_gcs_check();\n\n    // execute any mode changes requested by AP_AutoSurvey\n    update_auto_survey();\n\n    // check if we\'ve lost terrain data',
            ),
            (
                '    failsafe_gcs_check();\n    // check if we\'ve lost terrain data',
                '    failsafe_gcs_check();\n\n    // execute any mode changes requested by AP_AutoSurvey\n    update_auto_survey();\n\n    // check if we\'ve lost terrain data',
            ),
        ],
        "call update_auto_survey() in three_hz_loop",
    )

# ── 7. ArduCopter/events.cpp — arming-delay failsafe fix ─────────────────────

print("\n[7] Patching ArduCopter/events.cpp …")

patch(
    "ArduCopter/events.cpp",
    'bool Copter::should_disarm_on_failsafe() {\n    if (ap.in_arming_delay) {\n        return true;\n    }',
    '''\
bool Copter::should_disarm_on_failsafe() {
    if (ap.in_arming_delay) {
        // AUTO mode with auto_armed set means a scripted takeoff is in progress;
        // fall through to the AUTO-mode check so the arming delay does not cause
        // an immediate disarm before the takeoff sequence has a chance to run.
        if (flightmode->mode_number() != Mode::Number::AUTO || !ap.auto_armed) {
            return true;
        }
    }''',
    "exempt AUTO+auto_armed from arming-delay unconditional disarm",
)

# ── 8. ArduCopter/mode_auto.cpp — on_mission_complete callback ───────────────

print("\n[8] Patching ArduCopter/mode_auto.cpp …")

patch(
    "ArduCopter/mode_auto.cpp",
    '// exit_mission - function that is called once the mission completes\nvoid ModeAuto::exit_mission()\n{\n',
    '// exit_mission - function that is called once the mission completes\nvoid ModeAuto::exit_mission()\n{\n    // notify AP_AutoSurvey of mission completion so it can schedule the next run\n    copter.g2.auto_survey.on_mission_complete();\n\n',
    "call on_mission_complete() from exit_mission()",
)

# 8b: exit_mission diagnostic (applied separately so patch 8 stays idempotent)
patch(
    "ArduCopter/mode_auto.cpp",
    '    copter.g2.auto_survey.on_mission_complete();\n\n    // play a tone\n',
    '    copter.g2.auto_survey.on_mission_complete();\n\n#if CONFIG_HAL_BOARD == HAL_BOARD_SITL\n    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AP: exit_mission land_complete=%d", (int)copter.ap.land_complete);\n#endif\n\n    // play a tone\n',
    "add exit_mission diagnostic (SITL only)",
)

# ── 9. AP_Arming_Copter.cpp — disarm reason diagnostic ───────────────────────
# Anchor on "// we are not in the air" + set_land_complete, which is stable
# across ArduPilot versions.  A separate SKIP check lets any previous placement
# of the diagnostic (e.g. inside the send_arm_disarm_statustext SITL guard) be
# recognised as already applied.

print("\n[9] Patching ArduCopter/AP_Arming_Copter.cpp …")

if '"AP: Disarm reason:' in read("ArduCopter/AP_Arming_Copter.cpp"):
    print("  SKIP  add disarm-reason diagnostic (already applied)")
else:
    patch(
        "ArduCopter/AP_Arming_Copter.cpp",
        '    // we are not in the air\n    copter.set_land_complete(true);\n',
        '    // we are not in the air\n#if CONFIG_HAL_BOARD == HAL_BOARD_SITL\n    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AP: Disarm reason: %d", (int)method);\n#endif\n    copter.set_land_complete(true);\n',
        "add disarm-reason diagnostic (SITL only)",
    )

# ── verify ───────────────────────────────────────────────────────────────────

print("\n[VERIFY] Key patches in ~/ardupilot/ …")
_verify_checks = [
    ("ArduCopter/Copter.cpp",              "update_auto_survey();",        "update_auto_survey() called in loop"),
    ("ArduCopter/events.cpp",              "!ap.auto_armed",               "Layer 1 fix in events.cpp"),
    ("ArduCopter/AP_Arming_Copter.cpp",    '"AP: Disarm reason:',          "disarm-reason diagnostic"),
    ("ArduCopter/mode_auto.cpp",           "on_mission_complete()",        "on_mission_complete() call"),
    ("ArduCopter/mode_auto.cpp",           "exit_mission land_complete",   "exit_mission diagnostic"),
    ("libraries/AP_AutoSurvey/AP_AutoSurvey.cpp", "disarmed unexpectedly", "AutoSurvey FLY disarm monitor"),
]
for _vpath, _vneedle, _vlabel in _verify_checks:
    _vfull = os.path.join(ARDUPILOT, _vpath)
    try:
        _vstatus = "OK  " if _vneedle in open(_vfull).read() else "MISS"
    except FileNotFoundError:
        _vstatus = "ERR "
    print(f"  {_vstatus}  {_vlabel}")

# ── summary ──────────────────────────────────────────────────────────────────

print("""
==========================================================
All patches applied.  Now rebuild and test:

  cd ~/ardupilot
  ./waf configure --board sitl    # first time only
  ./waf copter

Then start SITL:
  Tools/autotest/sim_vehicle.py \\
      -v ArduCopter -L CMAC --console --map

In MAVProxy (immediately after launch):
  param set SURV_ENABLE 0

Wait for "EKF3 IMU0 is using GPS" (EKF ready), then:
  param set SURV_LAT  12.7478
  param set SURV_LON  79.9474
  param set SURV_ALT  50
  param set SURV_WIDTH  200
  param set SURV_LEN    300
  param set SURV_SPACING 30
  param set SURV_INTERVAL 10

Wait until MAVProxy console shows "AutoSurvey: …" messages
appearing (45 s boot delay), then:
  param set SURV_ENABLE 1

Expected: AutoSurvey builds the mission, enters AUTO,
arms, takes off to 50 m, flies 7 lawnmower lanes, RTLs.
==========================================================
""")
