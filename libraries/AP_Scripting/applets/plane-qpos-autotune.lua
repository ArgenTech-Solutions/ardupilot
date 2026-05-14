--[[

QuadPlane XY position controller autotune for QLOITER

This script tunes the Plane 4.5 style QuadPlane parameters:
  Q_P_VELXY_P
  Q_P_VELXY_I
  Q_P_POSXY_P

It follows the safety and workflow style of the existing QuickTune applets:
  - controlled by an RCx_OPTION scripting switch
  - low switch position disables and restores unsaved gains
  - middle switch position runs tuning
  - high switch position saves final gains

The tuning method is deliberately simple and conservative for Lua:
  - it only runs in QLOITER
  - it uses vehicle:update_target_location() to inject repeatable XY target steps
  - it scores each candidate using integrated position error, horizontal velocity,
    horizontal acceleration estimate, and settling time
  - it finalizes one parameter at a time and announces the selected value to the GCS

--]]

local MAV_SEVERITY = {
   EMERGENCY = 0,
   ALERT = 1,
   CRITICAL = 2,
   ERROR = 3,
   WARNING = 4,
   NOTICE = 5,
   INFO = 6,
   DEBUG = 7,
}

local MODE_QLOITER = 19

local PARAM_TABLE_KEY = 19
local PARAM_TABLE_PREFIX = "PTUN_"
local PARAM_TABLE_SIZE = 17

local UPDATE_RATE_HZ = 20
local UPDATE_PERIOD_MS = math.floor(1000 / UPDATE_RATE_HZ)
local SWITCH_POS_LOW = 0
local SWITCH_POS_MIDDLE = 1
local SWITCH_POS_HIGH = 2

local STEP_SEQUENCE = {
   {forward = 1, right = 0, name = "FWD"},
   {forward = -1, right = 0, name = "BACK"},
   {forward = 0, right = 1, name = "RIGHT"},
   {forward = 0, right = -1, name = "LEFT"},
}

local STAGES = {
   "Q_P_VELXY_P",
   "Q_P_POSXY_P",
}

-- Bind a firmware parameter for scripted access.
local function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format("PTun: could not find %s", name))
   return p
end

-- Create a PTUN script parameter and bind to it.
local function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format("PTun: could not add %s", name))
   return bind_param(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, PARAM_TABLE_SIZE), "PTun: could not add param table")

--[[
  // @Param: PTUN_ENABLE
  // @DisplayName: Position autotune enable
  // @Description: Enable QuadPlane XY position autotune
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local PTUN_ENABLE = bind_add_param('ENABLE', 1, 0)

--[[
  // @Param: PTUN_RC_FUNC
  // @DisplayName: Position autotune RC function
  // @Description: RCn_OPTION number to use to control start, stop and save
  // @Values: 300:Scripting1,301:Scripting2,302:Scripting3,303:Scripting4,304:Scripting5,305:Scripting6,306:Scripting7,307:Scripting8
  // @User: Standard
--]]
local PTUN_RC_FUNC = bind_add_param('RC_FUNC', 2, 300)

--[[
  // @Param: PTUN_STEP_M
  // @DisplayName: Position autotune step size
  // @Description: Horizontal target step distance used for each test move
  // @Range: 1 15
  // @Units: m
  // @User: Standard
--]]
local PTUN_STEP_M = bind_add_param('STEP_M', 3, 4)

--[[
  // @Param: PTUN_SETTLE_M
  // @DisplayName: Position autotune settle error
  // @Description: Position error threshold used for settling detection
  // @Range: 0.05 2.0
  // @Units: m
  // @User: Standard
--]]
local PTUN_SETTLE_M = bind_add_param('SETTLE_M', 4, 0.10)

--[[
  // @Param: PTUN_SETTLE_V
  // @DisplayName: Position autotune settle velocity
  // @Description: Horizontal speed threshold used for settling detection
  // @Range: 0.05 3.0
  // @Units: m/s
  // @User: Standard
--]]
local PTUN_SETTLE_V = bind_add_param('SETTLE_V', 5, 0.10)

--[[
  // @Param: PTUN_SETTLE_T
  // @DisplayName: Position autotune settle time
  // @Description: Time required within settle thresholds before the response is considered settled
  // @Range: 0.2 3.0
  // @Units: s
  // @User: Standard
--]]
local PTUN_SETTLE_T = bind_add_param('SETTLE_T', 6, 1.0)

--[[
  // @Param: PTUN_STEP_T
  // @DisplayName: Position autotune max step time
  // @Description: Maximum measurement time for each target step response
  // @Range: 2 12
  // @Units: s
  // @User: Standard
--]]
local PTUN_STEP_T = bind_add_param('STEP_T', 7, 4)

--[[
  // @Param: PTUN_POSW
  // @DisplayName: Position autotune position weight
  // @Description: Weight applied to integrated position error in candidate scoring
  // @Range: 0.1 20
  // @User: Standard
--]]
local PTUN_POSW = bind_add_param('POSW', 8, 8)

--[[
  // @Param: PTUN_VELW
  // @DisplayName: Position autotune velocity weight
  // @Description: Weight applied to integrated horizontal speed in candidate scoring
  // @Range: 0.1 20
  // @User: Standard
--]]
local PTUN_VELW = bind_add_param('VELW', 9, 2)

--[[
  // @Param: PTUN_ACCW
  // @DisplayName: Position autotune accel weight
  // @Description: Weight applied to integrated horizontal acceleration estimate in candidate scoring
  // @Range: 0.1 20
  // @User: Standard
--]]
local PTUN_ACCW = bind_add_param('ACCW', 10, 1)

--[[
  // @Param: PTUN_AUTOSAVE
  // @DisplayName: Position autotune auto save
  // @Description: Number of seconds after completion to auto-save the final gains. Set to zero to require a high switch position.
  // @Range: 0 60
  // @Units: s
  // @User: Standard
--]]
local PTUN_AUTOSAVE = bind_add_param('AUTOSAVE', 11, 0)

--[[
  // @Param: PTUN_MINMUL
  // @DisplayName: Position autotune minimum multiplier
  // @Description: Lower clamp applied to candidates as a multiple of the starting gain
  // @Range: 0.1 1.0
  // @User: Standard
--]]
local PTUN_MINMUL = bind_add_param('MINMUL', 12, 0.5)

--[[
  // @Param: PTUN_MAXMUL
  // @DisplayName: Position autotune maximum multiplier
  // @Description: Upper clamp applied to candidates as a multiple of the starting gain
  // @Range: 1.0 3.0
  // @User: Standard
--]]
local PTUN_MAXMUL = bind_add_param('MAXMUL', 13, 2.0)

--[[
   // @Param: PTUN_STEPS
   // @DisplayName: Position autotune sweep steps
   // @Description: Number of evenly spaced gain multipliers tested between the minimum and maximum multipliers for each sweep
   // @Range: 2 20
   // @User: Standard
--]]
local PTUN_STEPS = bind_add_param('STEPS', 14, 8)

--[[
   // @Param: PTUN_IPRAT
   // @DisplayName: Position autotune I to P ratio
   // @Description: Maximum allowed Q_P_VELXY_I to Q_P_VELXY_P ratio applied during the VELXY_I sweep and final selection
   // @Range: 0.1 2.0
   // @User: Standard
--]]
local PTUN_IPRAT = bind_add_param('IPRAT', 16, 0.5)

local tuned_params = {
   Q_P_VELXY_P = bind_param("Q_P_VELXY_P"),
   Q_P_VELXY_I = bind_param("Q_P_VELXY_I"),
   Q_P_POSXY_P = bind_param("Q_P_POSXY_P"),
}

local saved_values = {}
local changed_values = {}

local stage_index = 1
local candidate_values = nil
local candidate_index = 1
local candidate_results = {}
local measure_phase = "idle"
local sequence_index = 1
local sequence_target = nil
local reference_target = nil
local center_target = nil
local recovery_pending = false
local step_started_s = 0
local settle_started_s = nil
local tune_done_time = nil
local last_status_time = 0
local last_switch_state = nil
local last_start_block_reason = nil
local start_requested = false
local last_sample_time_s = nil
local last_vel_n = nil
local last_vel_e = nil
local last_roll_deg = nil
local last_pitch_deg = nil
local active_metrics = nil
local tune_running = false
local need_restore = false

-- Return current time in seconds.
local function now_s()
   return millis():tofloat() * 0.001
end

-- Round a number to three decimal places.
local function round3(value)
   return math.floor(value * 1000 + 0.5) / 1000
end

-- Wrap an angle into the [-pi, pi] range.
local function wrap_pi(angle)
   while angle > math.pi do
      angle = angle - 2.0 * math.pi
   end
   while angle < -math.pi do
      angle = angle + 2.0 * math.pi
   end
   return angle
end

-- Return horizontal speed and an acceleration estimate from AHRS velocity.
local function horizontal_speed_and_accel(dt)
   local velocity_ned = ahrs:get_velocity_NED()
   if velocity_ned == nil then
      return nil, nil, nil, nil
   end
   local vel_n = velocity_ned:x()
   local vel_e = velocity_ned:y()
   local speed = math.sqrt(vel_n * vel_n + vel_e * vel_e)
   local accel = 0
   if last_vel_n ~= nil and last_vel_e ~= nil and dt > 0 then
      local acc_n = (vel_n - last_vel_n) / dt
      local acc_e = (vel_e - last_vel_e) / dt
      accel = math.sqrt(acc_n * acc_n + acc_e * acc_e)
   end
   last_vel_n = vel_n
   last_vel_e = vel_e
   return speed, accel, vel_n, vel_e
end

-- Return horizontal speed without computing acceleration.
local function horizontal_speed_only()
   local velocity_ned = ahrs:get_velocity_NED()
   if velocity_ned == nil then
      return nil
   end
   local vel_n = velocity_ned:x()
   local vel_e = velocity_ned:y()
   return math.sqrt(vel_n * vel_n + vel_e * vel_e)
end

-- Read the configured three-position switch state.
local function get_switch_position()
   local sw = rc:find_channel_for_option(PTUN_RC_FUNC:get())
   if sw == nil then
      return 0
   end
   local switch_pos = sw:get_aux_switch_pos()
   if switch_pos == nil then
      return 0
   end
   if switch_pos == SWITCH_POS_LOW then
      return -1
   end
   if switch_pos == SWITCH_POS_HIGH then
      return 1
   end
   return 0
end

-- Convert a switch state number into a readable label.
local function switch_state_name(state)
   if state < 0 then
      return "LOW"
   end
   if state > 0 then
      return "HIGH"
   end
   return "MIDDLE"
end

-- Announce switch changes once per transition.
local function report_switch_transition(state)
   if last_switch_state == state then
      return
   end
   last_switch_state = state
   gcs:send_text(MAV_SEVERITY.INFO, string.format("PTun: switch %s", switch_state_name(state)))
end

-- Restore any unsaved tuned parameters to their cached starting values.
local function restore_all_params()
   for name, param_obj in pairs(tuned_params) do
      if changed_values[name] then
         param_obj:set(saved_values[name])
         changed_values[name] = false
      end
   end
   need_restore = false
end

-- Save any changed tuned parameters permanently.
local function save_all_params()
   for name, param_obj in pairs(tuned_params) do
      if changed_values[name] then
         param_obj:set_and_save(param_obj:get())
         saved_values[name] = param_obj:get()
         changed_values[name] = false
      end
   end
   need_restore = false
   gcs:send_text(MAV_SEVERITY.NOTICE, "PTun: final gains saved")
end

-- Cache the pre-tune parameter values for restore and candidate generation.
local function cache_saved_values()
   for name, param_obj in pairs(tuned_params) do
      saved_values[name] = param_obj:get()
      changed_values[name] = false
   end
end

-- Return the label shown in GCS messages for each stage.
local function display_param_name(name)
   if name == "Q_P_VELXY_P" then
      return "Q_P_VELXY_P/Q_P_VELXY_I"
   end
   return name
end

-- Apply a tuned parameter value and keep the coupled velocity-loop ratio enforced.
local function set_param(name, value)
   local p = tuned_params[name]
   if name == "Q_P_VELXY_P" then
      local coupled_i = math.max(0.01, value * PTUN_IPRAT:get())
      tuned_params.Q_P_VELXY_I:set(coupled_i)
      changed_values.Q_P_VELXY_I = true
   end
   if name == "Q_P_VELXY_I" then
      local max_i = math.max(0.01, tuned_params.Q_P_VELXY_P:get() * PTUN_IPRAT:get())
      if value > max_i then
         value = max_i
      end
   end
   local old_value = p:get()
   p:set(value)
   changed_values[name] = true
   need_restore = true
   logger:write("PTUN", "Stage,Value,Score", "fff", stage_index, value, 0)
   gcs:send_text(MAV_SEVERITY.INFO, string.format("PTun: %s %.3f -> %.3f", display_param_name(name), old_value, value))
end

-- Clamp a candidate gain to the configured multiplier limits.
local function clamp_candidate(base_value, factor)
   local min_value = base_value * PTUN_MINMUL:get()
   local max_value = base_value * PTUN_MAXMUL:get()
   local candidate = base_value * factor
   if candidate < min_value then
      candidate = min_value
   end
   if candidate > max_value then
      candidate = max_value
   end
   if candidate <= 0.01 then
      candidate = 0.01
   end
   return candidate
end

-- Build the sorted candidate list for one parameter sweep.
local function build_candidates(name)
   local base_value = saved_values[name]
   local candidates = {}
   local seen = {}
   local step_count = math.floor(PTUN_STEPS:get() + 0.5)
   if step_count < 2 then
      step_count = 2
   end
   local min_mul = PTUN_MINMUL:get()
   local max_mul = PTUN_MAXMUL:get()
   for i = 0, step_count - 1 do
      local factor
      if step_count == 1 then
         factor = 1.0
      else
         factor = min_mul + (max_mul - min_mul) * i / (step_count - 1)
      end
      local candidate = round3(clamp_candidate(base_value, factor))
      if name == "Q_P_VELXY_I" then
         local max_i = math.max(0.01, tuned_params.Q_P_VELXY_P:get() * PTUN_IPRAT:get())
         if candidate > max_i then
            candidate = round3(max_i)
         end
      end
      local key = string.format("%.3f", candidate)
      if not seen[key] then
         table.insert(candidates, candidate)
         seen[key] = true
      end
   end
   table.sort(candidates)
   return candidates
end

-- Return target-relative horizontal error magnitude and NE components.
local function get_target_error(target_loc)
   local current_loc = ahrs:get_location()
   if current_loc == nil or target_loc == nil then
      return nil, nil, nil
   end
   local dist_ne = target_loc:get_distance_NE(current_loc)
   local err_n = dist_ne:x()
   local err_e = dist_ne:y()
   return math.sqrt(err_n * err_n + err_e * err_e), err_n, err_e
end

-- Command the active QLOITER target to a supplied location.
local function command_target_location(target_loc)
   local next_wp = vehicle:get_target_location()
   if next_wp == nil or target_loc == nil then
      return false
   end
   return vehicle:update_target_location(next_wp, target_loc)
end

-- Request a garbage collection cycle when the API exists.
local function maybe_collect_garbage()
   if type(collectgarbage) == "function" then
      collectgarbage("collect")
   end
end

-- Drop temporary sweep state so memory can be reclaimed between tests.
local function collect_sweep_garbage()
   active_metrics = nil
   sequence_target = nil
   last_sample_time_s = nil
   last_vel_n = nil
   last_vel_e = nil
   last_roll_deg = nil
   last_pitch_deg = nil
   maybe_collect_garbage()
end

-- Convert a body-frame step definition into an earth-frame axis from current yaw.
local function step_axis_from_yaw(step_def)
   local yaw_rad = ahrs:get_yaw()
   if yaw_rad == nil then
      return nil, nil
   end
   local cos_yaw = math.cos(yaw_rad)
   local sin_yaw = math.sin(yaw_rad)
   local axis_n = step_def.forward * cos_yaw - step_def.right * sin_yaw
   local axis_e = step_def.forward * sin_yaw + step_def.right * cos_yaw
   return axis_n, axis_e
end

-- Initialize per-step measurement state for one commanded target move.
local function begin_measurement(target_loc, center_loc, axis_n, axis_e)
   local step_def = STEP_SEQUENCE[sequence_index]
   active_metrics = {
      score = 0,
      pos_sum = 0,
      vel_sum = 0,
      acc_sum = 0,
      cross_sum = 0,
      lean_sum = 0,
      lean_rate_sum = 0,
      peak_pos = 0,
      peak_vel = 0,
      peak_acc = 0,
      peak_cross = 0,
      peak_lean = 0,
      settle_time = PTUN_STEP_T:get(),
      samples = 0,
      target = target_loc,
      center = center_loc,
      segment_name = step_def.name,
      step_axis_n = axis_n,
      step_axis_e = axis_e,
      prev_along_err = nil,
      half_cycle_peak = 0,
      zero_crossings = 0,
      prev_roll_sign = 0,
      prev_pitch_sign = 0,
      roll_half_peak = 0,
      pitch_half_peak = 0,
      roll_crossings = 0,
      pitch_crossings = 0,
      last_roll_peak = 0,
      prev_roll_peak = 0,
      last_pitch_peak = 0,
      prev_pitch_peak = 0,
      err_angle = nil,
      angle_travel = 0,
      toilet_bowl = false,
      abort_reason = nil,
   }
   step_started_s = now_s()
   settle_started_s = nil
   last_sample_time_s = nil
   last_vel_n = nil
   last_vel_e = nil
   last_roll_deg = nil
   last_pitch_deg = nil
end

-- Sample the current step response and decide whether it should stop.
local function update_measurement()
   if active_metrics == nil then
      return false
   end

   local now = now_s()
   local dt = 1.0 / UPDATE_RATE_HZ
   if last_sample_time_s ~= nil then
      dt = math.max(0.001, now - last_sample_time_s)
   end
   last_sample_time_s = now

   local pos_err, err_n, err_e = get_target_error(active_metrics.target)
   local speed, accel = horizontal_speed_and_accel(dt)
   if pos_err == nil or speed == nil or accel == nil then
      return false
   end

   local along_err = err_n * active_metrics.step_axis_n + err_e * active_metrics.step_axis_e
   local cross_err = -err_n * active_metrics.step_axis_e + err_e * active_metrics.step_axis_n
   local zero_cross_threshold = math.max(PTUN_SETTLE_M:get(), PTUN_STEP_M:get() * 0.03)
   active_metrics.half_cycle_peak = math.max(active_metrics.half_cycle_peak, math.abs(along_err))
   if active_metrics.prev_along_err ~= nil and
      math.abs(active_metrics.prev_along_err) > zero_cross_threshold and
      math.abs(along_err) > zero_cross_threshold and
      active_metrics.prev_along_err * along_err < 0 then
      active_metrics.zero_crossings = active_metrics.zero_crossings + 1
      active_metrics.half_cycle_peak = math.abs(along_err)
   end
   active_metrics.prev_along_err = along_err

   local roll_deg = math.deg(ahrs:get_roll())
   local pitch_deg = math.deg(ahrs:get_pitch())
   local lean_deg = math.sqrt(roll_deg * roll_deg + pitch_deg * pitch_deg)
   local lean_rate = 0
   if last_roll_deg ~= nil and last_pitch_deg ~= nil then
      local d_roll = roll_deg - last_roll_deg
      local d_pitch = pitch_deg - last_pitch_deg
      lean_rate = math.sqrt(d_roll * d_roll + d_pitch * d_pitch) / dt
   end
   last_roll_deg = roll_deg
   last_pitch_deg = pitch_deg

   local att_deadband_deg = math.max(8.0, PTUN_STEP_M:get() * 2.0)
   local roll_sign = 0
   if roll_deg > att_deadband_deg then
      roll_sign = 1
   elseif roll_deg < -att_deadband_deg then
      roll_sign = -1
   end
   local pitch_sign = 0
   if pitch_deg > att_deadband_deg then
      pitch_sign = 1
   elseif pitch_deg < -att_deadband_deg then
      pitch_sign = -1
   end

   active_metrics.roll_half_peak = math.max(active_metrics.roll_half_peak, math.abs(roll_deg))
   active_metrics.pitch_half_peak = math.max(active_metrics.pitch_half_peak, math.abs(pitch_deg))

   if active_metrics.prev_roll_sign ~= 0 and roll_sign ~= 0 and roll_sign ~= active_metrics.prev_roll_sign then
      active_metrics.roll_crossings = active_metrics.roll_crossings + 1
      active_metrics.prev_roll_peak = active_metrics.last_roll_peak
      active_metrics.last_roll_peak = active_metrics.roll_half_peak
      active_metrics.roll_half_peak = math.abs(roll_deg)
   end
   if active_metrics.prev_pitch_sign ~= 0 and pitch_sign ~= 0 and pitch_sign ~= active_metrics.prev_pitch_sign then
      active_metrics.pitch_crossings = active_metrics.pitch_crossings + 1
      active_metrics.prev_pitch_peak = active_metrics.last_pitch_peak
      active_metrics.last_pitch_peak = active_metrics.pitch_half_peak
      active_metrics.pitch_half_peak = math.abs(pitch_deg)
   end
   if roll_sign ~= 0 then
      active_metrics.prev_roll_sign = roll_sign
   end
   if pitch_sign ~= 0 then
      active_metrics.prev_pitch_sign = pitch_sign
   end

   local err_angle = math.atan(err_e, err_n)
   if active_metrics.err_angle ~= nil then
      active_metrics.angle_travel = active_metrics.angle_travel + math.abs(wrap_pi(err_angle - active_metrics.err_angle))
   end
   active_metrics.err_angle = err_angle

   active_metrics.samples = active_metrics.samples + 1
   active_metrics.pos_sum = active_metrics.pos_sum + pos_err * dt
   active_metrics.vel_sum = active_metrics.vel_sum + speed * dt
   active_metrics.acc_sum = active_metrics.acc_sum + accel * dt
   active_metrics.cross_sum = active_metrics.cross_sum + math.abs(cross_err) * dt
   active_metrics.lean_sum = active_metrics.lean_sum + lean_deg * dt
   active_metrics.lean_rate_sum = active_metrics.lean_rate_sum + lean_rate * dt
   active_metrics.peak_pos = math.max(active_metrics.peak_pos, pos_err)
   active_metrics.peak_vel = math.max(active_metrics.peak_vel, speed)
   active_metrics.peak_acc = math.max(active_metrics.peak_acc, accel)
   active_metrics.peak_cross = math.max(active_metrics.peak_cross, math.abs(cross_err))
   active_metrics.peak_lean = math.max(active_metrics.peak_lean, lean_deg)

   active_metrics.score =
      PTUN_POSW:get() * active_metrics.pos_sum +
      PTUN_VELW:get() * active_metrics.vel_sum +
      PTUN_ACCW:get() * active_metrics.acc_sum +
      PTUN_POSW:get() * (0.75 * active_metrics.cross_sum) +
      0.35 * active_metrics.lean_sum +
      0.02 * active_metrics.lean_rate_sum

   if pos_err <= PTUN_SETTLE_M:get() and speed <= PTUN_SETTLE_V:get() then
      if settle_started_s == nil then
         settle_started_s = now
      elseif active_metrics.settle_time >= PTUN_STEP_T:get() then
         if now - settle_started_s >= PTUN_SETTLE_T:get() then
            active_metrics.settle_time = now - step_started_s
         end
      end
   else
      settle_started_s = nil
   end

   local bowl_radius_threshold = math.max(PTUN_STEP_M:get() * 1.5, PTUN_SETTLE_M:get() * 6.0)
   local bowl_speed_threshold = math.max(PTUN_SETTLE_V:get() * 2.5, 0.8)
   if now - step_started_s > 1.5 and
      settle_started_s == nil and
      pos_err > bowl_radius_threshold and
      speed > bowl_speed_threshold and
      active_metrics.angle_travel > math.rad(160) then
      active_metrics.toilet_bowl = true
      active_metrics.abort_reason = "toilet bowl"
      return true
   end

   local osc_peak_threshold = math.max(PTUN_SETTLE_M:get() * 4.0, PTUN_STEP_M:get() * 0.15)
   local slow_settle_time = math.max(2.0, PTUN_STEP_T:get() * 0.75)
   local att_peak_threshold = math.max(15.0, PTUN_STEP_M:get() * 4.0)
   local roll_large = active_metrics.prev_roll_peak > att_peak_threshold and active_metrics.last_roll_peak > att_peak_threshold
   local pitch_large = active_metrics.prev_pitch_peak > att_peak_threshold and active_metrics.last_pitch_peak > att_peak_threshold
   local roll_nondecay = roll_large and active_metrics.last_roll_peak > active_metrics.prev_roll_peak * 0.90
   local pitch_nondecay = pitch_large and active_metrics.last_pitch_peak > active_metrics.prev_pitch_peak * 0.90
   local roll_growing = roll_large and active_metrics.last_roll_peak > active_metrics.prev_roll_peak * 1.08
   local pitch_growing = pitch_large and active_metrics.last_pitch_peak > active_metrics.prev_pitch_peak * 1.08
   local severe_attitude = active_metrics.peak_lean > math.max(28.0, att_peak_threshold * 1.5)

   if settle_started_s == nil and now - step_started_s > 1.2 then
      local along_oscillation = active_metrics.zero_crossings >= 2 and active_metrics.half_cycle_peak > osc_peak_threshold
      local attitude_oscillation =
         (active_metrics.roll_crossings >= 2 and (roll_growing or (roll_nondecay and severe_attitude))) or
         (active_metrics.pitch_crossings >= 2 and (pitch_growing or (pitch_nondecay and severe_attitude)))
      if attitude_oscillation and (along_oscillation or severe_attitude) then
         active_metrics.toilet_bowl = true
         active_metrics.abort_reason = "oscillation"
         return true
      end
   end

   if settle_started_s == nil and
      now - step_started_s > slow_settle_time and
      ((active_metrics.zero_crossings >= 3 and active_metrics.half_cycle_peak > osc_peak_threshold) or
      (active_metrics.roll_crossings >= 3 and active_metrics.last_roll_peak > att_peak_threshold) or
      (active_metrics.pitch_crossings >= 3 and active_metrics.last_pitch_peak > att_peak_threshold)) then
      active_metrics.toilet_bowl = true
      active_metrics.abort_reason = "slow settling oscillation"
      return true
   end

   if now - step_started_s >= PTUN_STEP_T:get() then
      return true
   end

   if active_metrics.settle_time < PTUN_STEP_T:get() then
      return true
   end

   return false
end

-- Finalize one step measurement into averaged metrics for scoring.
local function finalize_measurement()
   if active_metrics == nil then
      return nil
   end
   local duration = math.max(0.001, now_s() - step_started_s)
   local result = {
      score = active_metrics.score + active_metrics.settle_time + (active_metrics.toilet_bowl and 5000 or 0),
      pos_mean = active_metrics.pos_sum / duration,
      vel_mean = active_metrics.vel_sum / duration,
      acc_mean = active_metrics.acc_sum / duration,
      cross_mean = active_metrics.cross_sum / duration,
      lean_mean = active_metrics.lean_sum / duration,
      peak_pos = active_metrics.peak_pos,
      peak_vel = active_metrics.peak_vel,
      peak_acc = active_metrics.peak_acc,
      peak_cross = active_metrics.peak_cross,
      peak_lean = active_metrics.peak_lean,
      settle_time = active_metrics.settle_time,
      segment_name = active_metrics.segment_name,
      toilet_bowl = active_metrics.toilet_bowl,
      abort_reason = active_metrics.abort_reason,
   }
   logger:write(
      "PTSC",
      "Stage,Cand,Score,PosE,VelE,AccE,Settle",
      "fffffff",
      stage_index,
      tuned_params[STAGES[stage_index]]:get(),
      result.score,
      result.pos_mean,
      result.vel_mean,
      result.acc_mean,
      result.settle_time
   )
   active_metrics = nil
   return result
end

-- Build the next target location for one body-frame step.
local function make_step_target(base_target, step_def)
   local axis_n, axis_e = step_axis_from_yaw(step_def)
   if axis_n == nil or axis_e == nil then
      return nil, nil, nil
   end
   local new_target = base_target:copy()
   new_target:offset(axis_n * PTUN_STEP_M:get(), axis_e * PTUN_STEP_M:get())
   return new_target, axis_n, axis_e
end

-- Reset per-candidate sequencing around the fixed tuning center.
local function reset_candidate_state()
   sequence_index = 1
   measure_phase = "command"
   if center_target == nil then
      return false
   end
   reference_target = center_target:copy()
   if reference_target == nil then
      return false
   end
   return true
end

-- Accumulate one finished step result into the current candidate totals.
local function record_candidate_result(candidate_result, result)
   candidate_result.run_count = candidate_result.run_count + 1
   candidate_result.score_sum = candidate_result.score_sum + result.score
   candidate_result.pos_sum = candidate_result.pos_sum + result.pos_mean
   candidate_result.vel_sum = candidate_result.vel_sum + result.vel_mean
   candidate_result.acc_sum = candidate_result.acc_sum + result.acc_mean
   candidate_result.cross_sum = candidate_result.cross_sum + result.cross_mean
   candidate_result.lean_sum = candidate_result.lean_sum + result.lean_mean
   candidate_result.settle_sum = candidate_result.settle_sum + result.settle_time
end

-- Pull the vehicle back to the fixed tune center between candidates.
local function handle_center_recovery()
   if not recovery_pending then
      return false
   end
   if center_target == nil then
      recovery_pending = false
      return false
   end
   local pos_err = get_target_error(center_target)
   if pos_err == nil then
      return true
   end
   command_target_location(center_target)
   local speed = horizontal_speed_only()
   local recovery_radius = math.max(PTUN_SETTLE_M:get() * 2.0, PTUN_STEP_M:get() * 0.25)
   local recovery_speed = math.max(PTUN_SETTLE_V:get() * 1.5, 0.5)
   if pos_err <= recovery_radius and (speed == nil or speed <= recovery_speed) then
      recovery_pending = false
      return false
   end
   return true
end

-- Average all recorded step results for the just-finished candidate.
local function finalize_candidate(param_name, candidate_value)
   local result_acc = candidate_results[candidate_index]
   local total_score = result_acc.score_sum
   local pos_mean = result_acc.pos_sum
   local vel_mean = result_acc.vel_sum
   local acc_mean = result_acc.acc_sum
   local cross_mean = result_acc.cross_sum
   local lean_mean = result_acc.lean_sum
   local settle_mean = result_acc.settle_sum
   local count = result_acc.run_count
   if count == 0 then
      total_score = 1e9
      count = 1
   end
   candidate_results[candidate_index].score = total_score / count
   candidate_results[candidate_index].pos_mean = pos_mean / count
   candidate_results[candidate_index].vel_mean = vel_mean / count
   candidate_results[candidate_index].acc_mean = acc_mean / count
   candidate_results[candidate_index].cross_mean = cross_mean / count
   candidate_results[candidate_index].lean_mean = lean_mean / count
   candidate_results[candidate_index].settle_mean = settle_mean / count

   gcs:send_text(
      MAV_SEVERITY.INFO,
      string.format(
         "PTun: %s=%.3f score=%.2f",
         display_param_name(param_name),
         candidate_value,
         candidate_results[candidate_index].score
      )
   )

   collect_sweep_garbage()
end

-- Pick the best tested candidate for a stage and apply it.
local function choose_best_candidate(param_name)
   local best = nil
   for i = 1, #candidate_results do
      if candidate_results[i].run_count > 0 then
         best = candidate_results[i]
         break
      end
   end
   if best == nil then
      return
   end

   for i = 1, #candidate_results do
      if candidate_results[i].run_count > 0 and candidate_results[i].score < best.score then
         best = candidate_results[i]
      end
   end
   set_param(param_name, best.value)
   gcs:send_text(
      MAV_SEVERITY.NOTICE,
      string.format(
         "PTun: finalised %s=%.3f",
         display_param_name(param_name),
         best.value
      )
   )
end

-- Advance to the next stage or finish the tune when all stages are complete.
local function advance_stage()
   stage_index = stage_index + 1
   candidate_index = 1
   candidate_values = nil
   candidate_results = {}
   reference_target = nil
   measure_phase = "idle"

   collect_sweep_garbage()
   maybe_collect_garbage()

   if stage_index > #STAGES then
      if center_target ~= nil then
         command_target_location(center_target)
      end
      tune_running = false
      tune_done_time = now_s()
      start_requested = false
      gcs:send_text(MAV_SEVERITY.NOTICE, "PTun: tuning complete")
   end
end

-- Clear all tuning state so the next run starts cleanly.
local function reset_tune_state()
   tune_running = false
   stage_index = 1
   candidate_index = 1
   candidate_values = nil
   candidate_results = {}
   measure_phase = "idle"
   sequence_index = 1
   sequence_target = nil
   reference_target = nil
   center_target = nil
   recovery_pending = false
   step_started_s = 0
   settle_started_s = nil
   tune_done_time = nil
   active_metrics = nil
   last_start_block_reason = nil
   start_requested = false
   last_sample_time_s = nil
   last_vel_n = nil
   last_vel_e = nil
   last_roll_deg = nil
   last_pitch_deg = nil
   maybe_collect_garbage()
end

-- Verify the vehicle is in QLOITER with an active target.
local function ensure_mode_and_target()
   if vehicle:get_mode() ~= MODE_QLOITER then
      return false, "requires QLOITER"
   end
   if vehicle:get_target_location() == nil then
      return false, "no active QLOITER target"
   end
   return true, nil
end

-- Report why tuning cannot start yet without spamming the GCS.
local function report_start_block(reason)
   if reason == nil then
      last_start_block_reason = nil
      return
   end
   local now = now_s()
   if last_start_block_reason ~= reason or now - last_status_time > 2.0 then
      gcs:send_text(MAV_SEVERITY.INFO, string.format("PTun: waiting, %s", reason))
      last_status_time = now
      last_start_block_reason = reason
   end
end

-- Start a new tune when the switch request and vehicle state allow it.
local function start_tune_if_ready()
   if tune_running or not start_requested then
      return
   end
   if not arming:is_armed() then
      report_start_block("vehicle not armed")
      return
   end
   local ok, reason = ensure_mode_and_target()
   if not ok then
      report_start_block(reason)
      return
   end
   local current_loc = ahrs:get_location()
   if current_loc == nil then
      report_start_block("no current location")
      return
   end
   cache_saved_values()
   reset_tune_state()
   center_target = current_loc:copy()
   command_target_location(center_target)
   tune_running = true
   start_requested = false
   report_start_block(nil)
   gcs:send_text(MAV_SEVERITY.NOTICE, "PTun: starting XY autotune")
end

-- Run the state machine for the current candidate and stage.
local function handle_candidate_state(param_name)
   if candidate_values == nil then
      candidate_values = build_candidates(param_name)
      candidate_results = {}
      candidate_index = 1
      for i = 1, #candidate_values do
         candidate_results[i] = {
            value = candidate_values[i],
            score = 1e9,
            run_count = 0,
            score_sum = 0,
            pos_sum = 0,
            vel_sum = 0,
            acc_sum = 0,
            cross_sum = 0,
            lean_sum = 0,
            settle_sum = 0,
         }
      end
      gcs:send_text(MAV_SEVERITY.INFO, string.format("PTun: sweeping %s", display_param_name(param_name)))
   end

   if candidate_index > #candidate_values then
      recovery_pending = true
      choose_best_candidate(param_name)
      advance_stage()
      return
   end

   local candidate_value = candidate_values[candidate_index]
   local candidate_result = candidate_results[candidate_index]

   if measure_phase == "idle" then
      set_param(param_name, candidate_value)
      if not reset_candidate_state() then
         return
      end
      return
   end

   if measure_phase == "command" then
      local next_wp = vehicle:get_target_location()
      if next_wp == nil then
         return
      end
      local step_def = STEP_SEQUENCE[sequence_index]
      local axis_n, axis_e
      sequence_target, axis_n, axis_e = make_step_target(reference_target, step_def)
      if sequence_target ~= nil and vehicle:update_target_location(next_wp, sequence_target) then
         begin_measurement(sequence_target, reference_target, axis_n, axis_e)
         measure_phase = "measure"
      end
      return
   end

   if measure_phase == "measure" then
      if update_measurement() then
         local result = finalize_measurement()
         if result ~= nil then
            record_candidate_result(candidate_result, result)
         end
         if result ~= nil and result.toilet_bowl then
            candidate_results[candidate_index].score = result.score
            candidate_results[candidate_index].pos_mean = result.pos_mean
            candidate_results[candidate_index].vel_mean = result.vel_mean
            candidate_results[candidate_index].acc_mean = result.acc_mean
            candidate_results[candidate_index].cross_mean = result.cross_mean
            candidate_results[candidate_index].lean_mean = result.lean_mean
            candidate_results[candidate_index].settle_mean = result.settle_time
            gcs:send_text(
               MAV_SEVERITY.WARNING,
               string.format("PTun: %s detected on %s=%.3f, stopping sweep", result.abort_reason or "instability", display_param_name(param_name), candidate_value)
            )
            recovery_pending = true
            choose_best_candidate(param_name)
            advance_stage()
            return
         end
         sequence_index = sequence_index + 1
         if sequence_index > #STEP_SEQUENCE then
            finalize_candidate(param_name, candidate_value)
            candidate_index = candidate_index + 1
            sequence_index = 1
            recovery_pending = true
            measure_phase = "idle"
         else
            measure_phase = "command"
         end
      end
   end
end

-- Apply tune state updates while the autotune run is active.
local function update_tune()
   if not tune_running then
      return
   end

   if not arming:is_armed() then
      gcs:send_text(MAV_SEVERITY.WARNING, "PTun: disarmed, stopping")
      restore_all_params()
      reset_tune_state()
      return
   end

   local ok, reason = ensure_mode_and_target()
   if not ok then
      gcs:send_text(MAV_SEVERITY.WARNING, string.format("PTun: stopping, %s", reason))
      restore_all_params()
      reset_tune_state()
      return
   end

   if handle_center_recovery() then
      return
   end

   if stage_index <= #STAGES then
      handle_candidate_state(STAGES[stage_index])
   end
end

-- Save tuned values automatically after the configured completion delay.
local function maybe_auto_save()
   if tune_done_time == nil then
      return
   end
   if PTUN_AUTOSAVE:get() <= 0 then
      return
   end
   if now_s() - tune_done_time >= PTUN_AUTOSAVE:get() then
      save_all_params()
      tune_done_time = nil
   end
end

-- Main script entry point called by the ArduPilot scheduler.
local function update()
   if PTUN_ENABLE:get() <= 0 then
      return update, 1000
   end

   local switch_pos = get_switch_position()
   local previous_switch_state = last_switch_state
   report_switch_transition(switch_pos)

   if switch_pos < 0 then
      start_requested = false
      if tune_running or need_restore then
         restore_all_params()
         reset_tune_state()
         gcs:send_text(MAV_SEVERITY.NOTICE, "PTun: stopped and restored")
      end
      return update, UPDATE_PERIOD_MS
   end

   if switch_pos > 0 then
      start_requested = false
      if tune_done_time ~= nil then
         save_all_params()
         tune_done_time = nil
      end
      return update, UPDATE_PERIOD_MS
   end

    if previous_switch_state ~= nil and previous_switch_state < 0 and switch_pos == 0 then
      start_requested = true
      report_start_block(nil)
      gcs:send_text(MAV_SEVERITY.INFO, "PTun: start requested")
   end

   start_tune_if_ready()
   update_tune()
   maybe_auto_save()

   return update, UPDATE_PERIOD_MS
end

cache_saved_values()
gcs:send_text(MAV_SEVERITY.NOTICE, "PTun: QuadPlane XY autotune loaded")

return update()