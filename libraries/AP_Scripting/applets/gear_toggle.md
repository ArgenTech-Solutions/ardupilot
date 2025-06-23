# Gear_Toggle Lua Script

This script provides automatic landing gear control using aircraft pitch angle as a trigger. It is designed for safe ground operations only and includes multiple safety checks to prevent in-flight activation.

## How it works

The script monitors the aircraft's pitch angle and automatically toggles the landing gear when the nose is raised above 45 degrees and held for approximately 1 second, then lowered back down. The script only operates when the aircraft is safely on the ground (disarmed with safety switch active). On the first activation, the script automatically cycles the gear through its full range (deploy→retract→deploy) to ensure proper servo positioning, then operates normally with alternating deploy/retract actions on subsequent triggers.

## Safety Features

The script includes multiple safety checks that must ALL be satisfied for gear operation:

- **Aircraft DISARMED**: Vehicle must not be armed
- **Safety Switch ACTIVE**: Hardware safety switch must be in SAFE position
- **Not Flying**: Aircraft must be detected as on the ground

If any safety condition fails, the script immediately stops monitoring pitch and resets its state.

## Setup and Use

- Ensure your landing gear servo is connected to **Servo 10** (or modify `LANDING_GEAR_CHANNEL` in the script)

- Verify your servo operates correctly with:
  - **PWM 1000**: Gear deployed (down)
  - **PWM 2000**: Gear retracted (up)

- If your servo operates in reverse, swap the PWM values in the `deploy_gear()` and `retract_gear()` functions

- Load the gear_toggle.lua script to the `/APM/scripts/` folder on your flight controller's SD card

- Ensure scripting is enabled: `SCR_ENABLE = 1` and reboot

- Test the script by manually tilting the aircraft nose up >45° for 1+ seconds while disarmed and safety switch active

## Configuration

### Landing Gear Channel
```lua
local LANDING_GEAR_CHANNEL = 9  -- Servo 10 = channel 9 (0-indexed)
```

### Trigger Settings
```lua
local PITCH_THRESHOLD = 45      -- degrees
local PITCH_HOLD_TIME = 1000    -- milliseconds (1 second)
local COOLDOWN_PERIOD = 5000    -- 5 seconds between toggles
```

### Debug Messages
```lua
local DEBUG_MESSAGES = false    -- Set to true to enable GCS debug messages
```

By default, the script operates silently. To enable debug messages that show pitch detection, gear actions, and status information, change `DEBUG_MESSAGES` to `true`. This is useful for troubleshooting or verifying the script is working correctly.

## Output Messages (Debug Mode Only)

### "LUA: Pitch X.X° detected"
The script has detected pitch above the 45° threshold and is monitoring the hold time.

### "LUA: Landing Gear DEPLOYED" / "LUA: Landing Gear RETRACTED"
Gear action has been completed successfully.

### "LUA: Starting gear cycle..."
First-time activation - the script is cycling the gear through its full range.

### "LUA: Gear cycle complete"
First-time gear cycling sequence has finished, gear is properly deployed.

### "LUA: Safety check failed - reset"
One or more safety conditions are not met (armed, safety switch off, or flying detected).

### "LUA: In cooldown"
Pitch trigger detected but within the 5-second cooldown period.

### "LUA: Pitch reset"
Pitch has returned below threshold, monitoring reset for next trigger.

## Known Issues

- The script assumes the gear starts in a retracted state. If your gear is already deployed when the script starts, the first trigger will still deploy (due to the cycling sequence).

- Servo position feedback is not available, so the script tracks gear state internally. If the servo is manually moved or fails, the script's state tracking may become incorrect.

- The script requires a stable AHRS system for pitch detection. If AHRS is not functioning properly, pitch monitoring will fail.