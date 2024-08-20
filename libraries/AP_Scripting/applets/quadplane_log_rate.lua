--[[
    Quadplane log rate control

    This script gives additional control over the logging parameters depending
    on if the aircraft is in forward flight or in hover. This allows high rate
    logging for control analysis in hover, and a much lower rate for forward
    flight to save space on the SD card.

    Parameters:

    // @Param: Q_LOG_MASK_FF
    // @DisplayName: Forward-flight log bitmask
    // @Description: This value gets loaded into LOG_BITMASK when in forward flight.
    // @Bitmask: 0:Fast Attitude,1:Medium Attitude,2:GPS,3:Performance,4:Control Tuning,5:Navigation Tuning,7:IMU,8:Mission Commands,9:Battery Monitor,10:Compass,11:TECS,12:Camera,13:RC Input-Output,14:Rangefinder,19:Raw IMU,20:Fullrate Attitude,21:Video Stabilization,22:Fullrate Notch
    // @User: Advanced

    // @Param: Q_LOG_MASK_HV
    // @DisplayName: Hover log bitmask
    // @Description: This value gets loaded into LOG_BITMASK when in hover.
    // @Bitmask: 0:Fast Attitude,1:Medium Attitude,2:GPS,3:Performance,4:Control Tuning,5:Navigation Tuning,7:IMU,8:Mission Commands,9:Battery Monitor,10:Compass,11:TECS,12:Camera,13:RC Input-Output,14:Rangefinder,19:Raw IMU,20:Fullrate Attitude,21:Video Stabilization,22:Fullrate Notch
    // @User: Advanced

    // @Param: Q_LOG_RATEMAX_FF
    // @DisplayName: Forward-flight log rate
    // @Description: This value gets loaded into LOG_FILE_RATEMAX when in forward flight.
    // @Units: Hz
    // @Range: 0 1000
    // @Increment: 0.1
    // @User: Standard

    // @Param: Q_LOG_RATEMAX_HV
    // @DisplayName: Hover log rate
    // @Description: This value gets loaded into LOG_FILE_RATEMAX when in hover.
    // @Units: Hz
    // @Range: 0 1000
    // @Increment: 0.1
    // @User: Standard
  ]]

-- Script initialization and parameter binding
local SCRIPT_NAME = "Quadplane log rate control"
local PARAM_TABLE_KEY = 63 -- Arbitrary, but must be unique among all scripts loaded
local PARAM_TABLE_PREFIX = "Q_LOG_"
local utilities = require("utilities")
utilities.param_add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 6)

-- New parameters
local MASK_FF    = utilities.bind_add_param("MASK_FF",    0xFFFF)
local MASK_HV    = utilities.bind_add_param("MASK_HV",    0xFFFF)
local RATEMAX_FF = utilities.bind_add_param("RATEMAX_FF", 0)
local RATEMAX_HV = utilities.bind_add_param("RATEMAX_HV", 0)

-- Existing parameters
local LOG_BITMASK      = utilities.bind_param("LOG_BITMASK")
local LOG_FILE_RATEMAX = utilities.bind_param("LOG_FILE_RATEMAX")

-- update function, called periodically
local function update()
    --[[
    Update function, called periodically. Checks whether the aircraft is in
    forward flight or in hover, and sets the max-rate and bitmask parameters
    accordingly.
    ]]

    -- Get the current mode
    local in_hover = quadplane:in_assisted_flight() or quadplane:in_vtol_mode()

    -- Set the logging rate and bitmask according to the current mode
    if in_hover then
        local mask, ratemax = MASK_HV:get(), RATEMAX_HV:get()
        if mask then
            LOG_BITMASK:set(mask)
        end
        if ratemax then
            LOG_FILE_RATEMAX:set(ratemax)
        end
    else
        local mask, ratemax = MASK_FF:get(), RATEMAX_FF:get()
        if mask then
            LOG_BITMASK:set(mask)
        end
        if ratemax then
            LOG_FILE_RATEMAX:set(ratemax)
        end
    end
end

-- Script is now loaded, print success message and start the protected loop
gcs:send_text(6, SCRIPT_NAME .. string.format(" loaded"))
local function protected_wrapper()
    --[[
    Wrapper around the update function to catch and handle errors, preventing
    script termination.
    --]]
    local success, err = pcall(update)
    if not success then
        gcs:send_text(0, "Internal Error: " .. err)
        return protected_wrapper, 10000 -- Retry after 10 seconds on error
    end
    return protected_wrapper, 200       -- Normal update rate at 5Hz
end

return protected_wrapper() -- Start the update loop
