-- =============================================================
-- auto_survey.lua  –  Autonomous Timed Factory Surveillance
-- ArduPilot Lua Script  |  Requires SCR_ENABLE = 1
-- =============================================================
-- INSTALLATION:
--   SITL  : copy to ~/ardupilot/scripts/
--   Real FC: copy to SD card /APM/scripts/
--
-- PARAMETERS (prefix SURV_):
--   SURV_ENABLE   : 0=idle  1=run survey cycle
--   SURV_LAT      : Centre latitude  (decimal degrees, +N)
--   SURV_LON      : Centre longitude (decimal degrees, +E)
--   SURV_ALT      : Survey altitude  (m AGL)
--   SURV_WIDTH    : Area width  E-W  (m)
--   SURV_LEN      : Area length N-S  (m)
--   SURV_SPACING  : Lane spacing     (m)  – lawnmower step
--   SURV_INTERVAL : Minutes between repeat surveys
-- =============================================================

local PARAM_KEY  = 72          -- unique key (1-200, must not clash)
local SCRIPT_VER = "1.0"

-- ── register custom parameters ────────────────────────────────
assert(param:add_table(PARAM_KEY, "SURV_", 8), "auto_survey: param table failed")

local function bind_param(idx, name, default)
    assert(param:add_param(PARAM_KEY, idx, name, default),
           "auto_survey: failed to add SURV_" .. name)
end

bind_param(1, "ENABLE",    0)
bind_param(2, "LAT",       12.7478)   -- default: Sriperumbudur area
bind_param(3, "LON",       79.9474)
bind_param(4, "ALT",       50)
bind_param(5, "WIDTH",     200)
bind_param(6, "LEN",       300)
bind_param(7, "SPACING",   30)
bind_param(8, "INTERVAL",  10)

-- ── MAVLink / ArduPilot constants ─────────────────────────────
local MAV_FRAME_GLOBAL          = 0
local MAV_FRAME_GLOBAL_REL_ALT  = 3
local MAV_CMD_NAV_WAYPOINT      = 16
local MAV_CMD_NAV_TAKEOFF       = 22
local MAV_CMD_NAV_RTL           = 20
local COPTER_MODE_AUTO          = 3
local COPTER_MODE_RTL           = 6
local SEV_INFO                  = 6
local SEV_WARNING               = 4
local SEV_ERROR                 = 3

-- ── state machine ─────────────────────────────────────────────
local ST_IDLE    = 0
local ST_BUILD   = 1
local ST_FLY     = 2
local ST_WAIT    = 3

local state       = ST_IDLE
local next_run_ms = 0
local prev_enable = 0

-- ── helpers ───────────────────────────────────────────────────
local function log(sev, msg)
    gcs:send_text(sev, "AutoSurvey: " .. msg)
end

local function meters_to_lat(m)
    return m / 111320.0
end

local function meters_to_lon(m, lat_deg)
    return m / (111320.0 * math.cos(math.rad(lat_deg)))
end

-- ── mission builder ───────────────────────────────────────────
-- Builds a lawnmower grid survey over a rectangle centred on
-- (clat, clon).  Lanes run N-S; each lane is `spacing` metres
-- apart in the E-W direction.
local function build_mission(clat, clon, alt, width, len, spacing)
    assert(mission:clear(), "auto_survey: mission:clear() failed")

    local item = mavlink_mission_item_int_t()
    local idx  = 0

    -- ── WP 0: home placeholder (required by ArduPilot) ────────
    item:seq(idx); item:frame(MAV_FRAME_GLOBAL); item:command(MAV_CMD_NAV_WAYPOINT)
    item:current(0); item:autocontinue(1)
    item:param1(0); item:param2(0); item:param3(0); item:param4(0)
    item:x(math.floor(clat * 1e7))
    item:y(math.floor(clon * 1e7))
    item:z(0)
    assert(mission:set_item(idx, item), "set_item failed at WP0")
    idx = idx + 1

    -- ── WP 1: takeoff ─────────────────────────────────────────
    item:seq(idx); item:frame(MAV_FRAME_GLOBAL_REL_ALT); item:command(MAV_CMD_NAV_TAKEOFF)
    item:current(1); item:autocontinue(1)
    item:param1(15); item:param2(0); item:param3(0); item:param4(0)
    item:x(math.floor(clat * 1e7))
    item:y(math.floor(clon * 1e7))
    item:z(alt)
    assert(mission:set_item(idx, item), "set_item failed at TAKEOFF")
    idx = idx + 1

    -- ── lawnmower lanes ───────────────────────────────────────
    local num_lanes = math.floor(width / spacing) + 1
    local lat_half  = meters_to_lat(len / 2)
    local southward = true   -- alternate direction each lane

    for lane = 0, num_lanes - 1 do
        local lon_off = -width / 2 + lane * spacing
        local dlon    = meters_to_lon(lon_off, clat)
        local lon_wp  = clon + dlon

        -- start and end lat for this lane (alternate N→S / S→N)
        local lat_a = clat + (southward and  lat_half or -lat_half)
        local lat_b = clat + (southward and -lat_half or  lat_half)
        southward   = not southward

        for _, lat_pt in ipairs({lat_a, lat_b}) do
            item:seq(idx); item:frame(MAV_FRAME_GLOBAL_REL_ALT)
            item:command(MAV_CMD_NAV_WAYPOINT)
            item:current(0); item:autocontinue(1)
            item:param1(0); item:param2(2); item:param3(0); item:param4(0)
            item:x(math.floor(lat_pt * 1e7))
            item:y(math.floor(lon_wp * 1e7))
            item:z(alt)
            assert(mission:set_item(idx, item),
                   string.format("set_item failed at lane %d", lane))
            idx = idx + 1
        end
    end

    -- ── final RTL ─────────────────────────────────────────────
    item:seq(idx); item:frame(MAV_FRAME_GLOBAL_REL_ALT); item:command(MAV_CMD_NAV_RTL)
    item:current(0); item:autocontinue(1)
    item:param1(0); item:param2(0); item:param3(0); item:param4(0)
    item:x(0); item:y(0); item:z(0)
    assert(mission:set_item(idx, item), "set_item failed at RTL")
    idx = idx + 1

    log(SEV_INFO, string.format(
        "Mission built: %d WPs, %d lanes, %.0fx%.0fm @ %.0fm AGL",
        idx, num_lanes, width, len, alt))
    return idx
end

-- ── main update ───────────────────────────────────────────────
local function update()
    local now_ms  = millis()
    local enabled = param:get("SURV_ENABLE") or 0

    -- detect fresh enable edge (0→1)
    local rising_edge = (enabled > 0) and (prev_enable == 0)
    prev_enable = enabled

    -- user disabled at any point → return to idle
    if enabled == 0 and state ~= ST_IDLE then
        log(SEV_WARNING, "Disabled — returning to IDLE")
        vehicle:set_mode(COPTER_MODE_RTL)
        state = ST_IDLE
        return update, 1000
    end

    -- ── IDLE ──────────────────────────────────────────────────
    if state == ST_IDLE then
        if rising_edge then
            log(SEV_INFO, "Survey cycle triggered")
            state = ST_BUILD
        end
        return update, 500
    end

    -- ── BUILD: upload mission, switch to AUTO ─────────────────
    if state == ST_BUILD then
        local clat    = param:get("SURV_LAT")
        local clon    = param:get("SURV_LON")
        local alt     = param:get("SURV_ALT")
        local width   = param:get("SURV_WIDTH")
        local len     = param:get("SURV_LEN")
        local spacing = param:get("SURV_SPACING")

        if not (clat and clon and alt and width and len and spacing) then
            log(SEV_WARNING, "Parameters not ready, retrying in 2s")
            return update, 2000
        end

        -- safety checks
        if alt < 10 or alt > 150 then
            log(SEV_ERROR, "SURV_ALT out of range [10-150m] — aborting")
            param:set_and_save("SURV_ENABLE", 0)
            state = ST_IDLE
            return update, 1000
        end
        if spacing < 5 then
            log(SEV_ERROR, "SURV_SPACING too small (min 5m) — aborting")
            param:set_and_save("SURV_ENABLE", 0)
            state = ST_IDLE
            return update, 1000
        end

        local ok, err = pcall(build_mission, clat, clon, alt, width, len, spacing)
        if not ok then
            log(SEV_ERROR, "Mission build failed: " .. tostring(err))
            param:set_and_save("SURV_ENABLE", 0)
            state = ST_IDLE
            return update, 1000
        end

        if not vehicle:set_mode(COPTER_MODE_AUTO) then
            log(SEV_WARNING, "set_mode AUTO failed, retrying...")
            return update, 2000
        end

        log(SEV_INFO, "AUTO mode set — survey flying")
        state = ST_FLY
        return update, 1000
    end

    -- ── FLY: monitor until mission completes ──────────────────
    if state == ST_FLY then
        local ms = mission:state()
        local mode = vehicle:get_mode()

        -- mission finished or vehicle left AUTO
        local done = (ms == mission.MISSION_COMPLETE) or
                     (ms == mission.MISSION_STOPPED)  or
                     (mode ~= COPTER_MODE_AUTO and mode ~= COPTER_MODE_RTL)

        if done then
            local interval_ms = (param:get("SURV_INTERVAL") or 10) * 60 * 1000
            next_run_ms = now_ms + interval_ms
            state = ST_WAIT
            log(SEV_INFO, string.format(
                "Survey complete. Next in %.0f min",
                interval_ms / 60000))
        end
        return update, 2000
    end

    -- ── WAIT: countdown to next survey ────────────────────────
    if state == ST_WAIT then
        local remaining_s = math.max(0, math.floor((next_run_ms - now_ms) / 1000))

        if remaining_s <= 0 then
            log(SEV_INFO, "Starting next survey")
            state = ST_BUILD
        elseif remaining_s % 60 == 0 and remaining_s > 0 then
            log(SEV_INFO, string.format("%d min until next survey",
                math.floor(remaining_s / 60)))
        end
        return update, 1000
    end

    return update, 500
end

-- ── entry point ───────────────────────────────────────────────
log(SEV_INFO, string.format(
    "Loaded v%s — set SURV_ENABLE=1 from GCS or Auto Survey button to start",
    SCRIPT_VER))

return update()
