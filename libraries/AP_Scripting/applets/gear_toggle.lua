-- Pitch-based landing gear control script
-- Toggles landing gear when aircraft is pitched up >45 degrees for 1+ seconds
-- SAFETY: Only works when disarmed and safety switch active (SAFE mode)

-- USER CONFIGURATION
local DEBUG_MESSAGES = false  -- Set to true to enable GCS debug messages
local LANDING_GEAR_CHANNEL = 9  -- Servo 10 = channel 9 (0-indexed)
local PITCH_THRESHOLD = 45  -- degrees
local PITCH_HOLD_TIME = 1000  -- milliseconds (1 second)
local COOLDOWN_PERIOD = 5000  -- 5 seconds between toggles

-- State tracking
local gear_deployed = false
local pitch_high_start_time = 0
local pitch_triggered = false
local last_toggle_time = 0
local gear_action_preference = "DEPLOY"
local first_trigger_done = false
local cycling_gear = false
local cycle_step = 0
local cycle_step_time = 0

-- Helper function for debug messages
function debug_msg(message)
    if DEBUG_MESSAGES then
        gcs:send_text(0, message)
    end
end

function safety_checks_passed()
    return not arming:is_armed() and SRV_Channels:get_safety_state() and not vehicle:get_likely_flying()
end

function deploy_gear()
    SRV_Channels:set_output_pwm_chan_timeout(LANDING_GEAR_CHANNEL, 1000, 1000)
    gear_deployed = true
    debug_msg("LUA: Landing Gear DEPLOYED")
end

function retract_gear()
    SRV_Channels:set_output_pwm_chan_timeout(LANDING_GEAR_CHANNEL, 2000, 1000)
    gear_deployed = false
    debug_msg("LUA: Landing Gear RETRACTED")
end

function start_gear_cycle()
    cycling_gear = true
    cycle_step = 1
    cycle_step_time = millis()
    debug_msg("LUA: Starting gear cycle...")
    deploy_gear()
end

function handle_gear_cycle()
    local current_time = millis()
    if current_time:tofloat() - cycle_step_time:tofloat() > 1500 then
        cycle_step = cycle_step + 1
        cycle_step_time = current_time

        if cycle_step == 2 then
            retract_gear()
        elseif cycle_step == 3 then
            deploy_gear()
        elseif cycle_step == 4 then
            cycling_gear = false
            first_trigger_done = true
            gear_action_preference = "RETRACT"
            debug_msg("LUA: Gear cycle complete")
        end
    end
end

function monitor_pitch()
    local current_time = millis()

    -- Safety check
    if not safety_checks_passed() then
        if pitch_high_start_time > 0 then
            pitch_high_start_time = 0
            pitch_triggered = false
            debug_msg("LUA: Safety check failed - reset")
        end
        return
    end

    -- Get pitch
    local pitch_rad = ahrs:get_pitch()
    if not pitch_rad then return end
    local current_pitch = math.deg(pitch_rad)

    -- Debug pitch display
    if DEBUG_MESSAGES and current_time % 3000 < 100 then
        debug_msg(string.format("LUA: Pitch: %.1f°", current_pitch))
    end

    -- Check pitch threshold
    if current_pitch > PITCH_THRESHOLD then
        if pitch_high_start_time == 0 then
            pitch_high_start_time = current_time
            debug_msg(string.format("LUA: Pitch %.1f° detected", current_pitch))
        elseif (current_time - pitch_high_start_time) > PITCH_HOLD_TIME and not pitch_triggered then
            pitch_triggered = true

            if current_time - last_toggle_time > COOLDOWN_PERIOD then
                if not first_trigger_done then
                    start_gear_cycle()
                else
                    if gear_action_preference == "DEPLOY" then
                        deploy_gear()
                        gear_action_preference = "RETRACT"
                    else
                        retract_gear()
                        gear_action_preference = "DEPLOY"
                    end
                    debug_msg(string.format("LUA: Gear %s", gear_deployed and "DEPLOYED" or "RETRACTED"))
                end
                last_toggle_time = current_time
            else
                debug_msg("LUA: In cooldown")
            end
        end
    else
        if pitch_high_start_time > 0 then
            debug_msg("LUA: Pitch reset")
        end
        pitch_high_start_time = 0
        pitch_triggered = false
    end
end

function update()
    if cycling_gear then
        handle_gear_cycle()
    else
        monitor_pitch()
    end
    return update, 100
end

return update()
