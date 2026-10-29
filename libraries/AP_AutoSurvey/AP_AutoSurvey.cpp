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

    // ── sanity checks ─────────────────────────────────────────
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

    // ── Index 0: home placeholder ─────────────────────────────
    // starts_with_takeoff_cmd() checks from AP_MISSION_FIRST_REAL_COMMAND (1).
    // Occupy index 0 so TAKEOFF lands at index 1.
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.content.location.lat = (int32_t)(clat * 1.0e7f);
    cmd.content.location.lng = (int32_t)(clon * 1.0e7f);
    cmd.content.location.alt = (int32_t)(alt_m * 100.0f);
    cmd.content.location.relative_alt = 1;
    if (!mission->add_cmd(cmd)) {
        send_text(MAV_SEVERITY_ERROR, "add_cmd failed (home)");
        return false;
    }
    cmd = {};

    // ── Index 1: takeoff ──────────────────────────────────────
    cmd.id    = MAV_CMD_NAV_TAKEOFF;
    cmd.p1    = 15;               // minimum pitch (deg)
    cmd.content.location.lat = (int32_t)(clat * 1.0e7f);
    cmd.content.location.lng = (int32_t)(clon * 1.0e7f);
    cmd.content.location.alt = (int32_t)(alt_m * 100.0f);
    cmd.content.location.relative_alt = 1;
    if (!mission->add_cmd(cmd)) {
        send_text(MAV_SEVERITY_ERROR, "add_cmd failed (takeoff)");
        return false;
    }

    // ── lawnmower lanes ───────────────────────────────────────
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

        // waypoint A (start of lane)
        cmd.content.location.lat = (int32_t)(lat_a * 1.0e7f);
        cmd.content.location.lng = (int32_t)(lon_wp * 1.0e7f);
        if (!mission->add_cmd(cmd)) {
            send_text(MAV_SEVERITY_ERROR, "add_cmd failed (lane %d A)", i);
            return false;
        }

        // waypoint B (end of lane)
        cmd.content.location.lat = (int32_t)(lat_b * 1.0e7f);
        cmd.content.location.lng = (int32_t)(lon_wp * 1.0e7f);
        if (!mission->add_cmd(cmd)) {
            send_text(MAV_SEVERITY_ERROR, "add_cmd failed (lane %d B)", i);
            return false;
        }
    }

    // ── RTL ───────────────────────────────────────────────────
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

// ── main update — called from ArduCopter scheduler ───────────────
void AP_AutoSurvey::update()
{
    const uint32_t now_ms   = AP_HAL::millis();
    const int8_t   enabled  = _enable.get();

    // Suppress rising-edge detection for 45 s after boot so the EKF can
    // initialise before we attempt to arm and enter AUTO mode.
    if (now_ms < 45000U) {
        _prev_enable = enabled;
        return;
    }

    // detect rising edge 0→1
    const bool rising = (enabled > 0) && (_prev_enable == 0);
    _prev_enable = enabled;

    // user disabled at any time → abort
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
            // Prevent RC7 SaveWaypoint from clearing the survey mission.
            AP_Param::set_by_name("RC7_OPTION", 0);
            // Force starts_with_takeoff_cmd() to always check from index 1.
            AP_Param::set_by_name("MIS_RESTART", 1);
            send_text(MAV_SEVERITY_INFO, "Survey cycle triggered");
            _state = State::BUILD;
        }
        break;

    case State::BUILD: {
        if (!build_mission()) {
            // build failed — disable and return to idle
            _enable.set_and_save(0);
            _state = State::IDLE;
            break;
        }
        _fail_count = 0;   // mission successfully built; reset failure counter
        _want_auto  = true;
        _state      = State::FLY;
        break;
    }

    case State::FLY: {
        // _want_auto is cleared by on_mode_changed() once Copter switches
        // mission complete is notified by on_mission_complete()
        break;
    }

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

// ── callbacks from ArduCopter ────────────────────────────────────

void AP_AutoSurvey::on_auto_mode_failed()
{
    // AUTO mode entry failed (e.g. mission was cleared by an RC switch).
    _want_auto = false;
    _fail_count++;
    if (_fail_count >= 5) {
        // Repeated failures — likely an RC switch keeps clearing the mission.
        _enable.set_and_save(0);
        _fail_count = 0;
        _state = State::IDLE;
        send_text(MAV_SEVERITY_CRITICAL,
                  "Survey disabled: mission cleared 5x. "
                  "Run: param set RC7_OPTION 0 then re-enable");
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
        // single survey — disable
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
