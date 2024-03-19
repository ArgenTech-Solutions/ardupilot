--[[ 
Name: EFI Scripting backend driver for simulated engine
--]]

-- Check Script uses a miniumum firmware version
local SCRIPT_AP_VERSION = 4.5
local SCRIPT_NAME       = "EFI: SITL"

local VERSION = FWVersion:major() + (FWVersion:minor() * 0.1)

assert(VERSION >= SCRIPT_AP_VERSION, string.format('%s Requires: %s:%.1f. Found Version: %s', SCRIPT_NAME, FWVersion:type(), SCRIPT_AP_VERSION, VERSION))

-- Check if parameter SIM_OPOS_LAT exists (prevents from accidentally loading on real aircraft)
assert(param:get('SIM_OPOS_LAT') ~= nil, string.format('%s was designed for SITL', SCRIPT_NAME))

local MAV_SEVERITY_ERROR = 3
local MAV_SEVERITY_INFO = 6

PARAM_TABLE_KEY = 36
PARAM_TABLE_PREFIX = "SIM_EFI_"

local UPDATE_HZ = 4

-- Engine simulation constants
local CHT_HDT_FF = 140
local CHT_CDT_FF = 40
local CHT_CDT_ST = 90
local CHT_MIN = 90
local CHT_CRUISE_SPEED = 23
local CHT_TCONST = 0.03
local CHT_RPM_MIN = 2750
local CHT_RPM_MAX = 8500
local BURN_RPM = 7200
local BURN_RATE = 0.77
local FUEL_PRESS = 320
local FP_VARI = 20
local FP_RATE = 1.5

-- bind a parameter to a variable given
local function BIND_PARAM(name)
    local p = Parameter()
    assert(p:init(name), string.format('could not find %s parameter', name))
    return p
end

-- add a parameter and bind it to a variable
local function BIND_ADD_PARAM(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return BIND_PARAM(PARAM_TABLE_PREFIX .. name)
end

local function CONSTRAIN(v, vmin, vmax)
    if v < vmin then
        v = vmin
    end
    if v > vmax then
        v = vmax
    end
    return v
end

-- Setup EFI Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 2), 'could not add SIM_EFI param table')
local CHT_INCREASE = BIND_ADD_PARAM('CHT_INC', 1, 0)
local IDLE_PWM = BIND_ADD_PARAM('IDL_PWM', 2, 1100)

-- Bind servo limit parameters for ignition control relay emulation
local SERVO3_MIN = BIND_PARAM('SERVO3_MIN')
local SERVO3_MAX = BIND_PARAM('SERVO3_MAX')
local SERVO3_MIN_BACKUP = SERVO3_MIN:get()
local SERVO3_MAX_BACKUP = SERVO3_MAX:get()
local SERVO3_TRIM = BIND_PARAM('SERVO3_TRIM')
local SIM_PIN_MASK = BIND_PARAM('SIM_PIN_MASK')

local efi_backend = nil

local function C_TO_KELVIN(temp)
   return temp + 273.15
end

--[[
   EFI Engine Object
--]]
local function engine_control()
    local self = {}

    -- Build up the EFI_State that is passed into the EFI Scripting backend
    local efi_state = EFI_State()
    local cylinder_state = Cylinder_Status()

    -- private fields as locals
    local rpm = 0
    local air_pressure = 0
    self.fuel_press = FUEL_PRESS
    local fuel_consumption_lph = 0
    local fuel_total_l = 0

    -- Temperature Data Structure
    local temps = {}
    temps.egt = 0.0                             -- Exhaust Gas Temperature
    temps.cht = baro:get_external_temperature() -- Cylinder Head Temperature
    temps.imt = baro:get_external_temperature() -- intake manifold temperature

    -- Build and set the EFI_State that is passed into the EFI Scripting backend
    function self.set_EFI_State()
       -- Cylinder_Status
       cylinder_state:cylinder_head_temperature(C_TO_KELVIN(temps.cht))

       efi_state:engine_speed_rpm(uint32_t(rpm))

       efi_state:fuel_consumption_rate_cm3pm(fuel_consumption_lph * 1000.0 / 60.0)
       efi_state:fuel_pressure(self.fuel_press)
       efi_state:estimated_consumed_fuel_volume_cm3(fuel_total_l * 1000.0)
       efi_state:atmospheric_pressure_kpa(air_pressure)
       efi_state:intake_manifold_temperature(C_TO_KELVIN(temps.imt))

       -- copy cylinder_state to efi_state
       efi_state:cylinder_status(cylinder_state)


       efi_state:last_updated_ms(millis())

        -- Set the EFI_State into the EFI scripting driver
        efi_backend:handle_scripting(efi_state)
    end

    -- Simulate various engine parameters
    function self.simulate_engine()
        -- Get RPM, map nil to 0 and constrain to prevent error on -1
        rpm = CONSTRAIN(RPM:get_rpm(1) or 0, 0, 50000)

        -- Get air temperature and pressure
        air_pressure = baro:get_pressure()/100 or 0

        -- Currently, the external temperature function returns 25C too high
        -- (planning to fix this with a PR to ArduPilot)
        temps.imt = baro:get_external_temperature()-25 or 0

        -- ========== CHT ==========
        -- These constants change depending on airspeed. Interpolate the steady state temperature change using King's Law
        -- [scales with 1/(1+Y*sqrt(velocity))]
        local Y = (CHT_CDT_ST/CHT_CDT_FF - 1)/math.sqrt(CHT_CRUISE_SPEED)
        -- The steady state temperature difference at full throttle while stationary
        -- (inferred from cht_hdt_ff, cht_cdt_ff, and cht_cdt_st)
        local cht_hdt_st = (CHT_HDT_FF + CHT_INCREASE:get()) * (1 + Y*math.sqrt(CHT_CRUISE_SPEED))

        local airspeed = ahrs:airspeed_estimate() or 0
        local cht_hdt = cht_hdt_st/(1 + Y*math.sqrt(airspeed))
        local cht_cdt = CHT_CDT_ST/(1 + Y*math.sqrt(airspeed))

        -- The time constant is defined in the parameters as per second,
        -- transform into per-loop.
        local cht_tconst = CHT_TCONST / UPDATE_HZ;

        -- Steady state delta-T scales linearly by power output of engine
        -- (P_idle + C*RPM^3)
        local C1 = (cht_hdt-cht_cdt)/(CHT_RPM_MAX^3 - CHT_RPM_MIN^3)
        local C2 = cht_hdt - C1*CHT_RPM_MAX^3

        local cht_steady = C2 + C1*rpm^3 + temps.imt

        -- Handle the min temp regulation for the cowling flap
        -- (Really this should be modelled as an adjustment to the airspeed, since the flap works by adjusting air flow,
        -- but that is hard and won't provide an appreciable difference to the operator)
        if(temps.cht < CHT_MIN and cht_steady < CHT_MIN) then
            cht_steady = CHT_MIN;
        end

        -- If the engine is off, the steady-state temperature is ambient
        if(rpm == 0) then
            cht_steady = temps.imt
        end

        -- Move the CHT towards the steady state
        temps.cht = (1-cht_tconst)*temps.cht + cht_tconst*cht_steady;

        -- ========== Fuel Consumption ==========
        -- Fuel consumption will be modeled as burning a certain amount at a certain
        -- RPM. More will be consumed at higher RPM, and less at lower RPM. We'll
        -- assume fuel consumption scales with RPM^3.
        local x = rpm/BURN_RPM;
        fuel_consumption_lph = BURN_RATE*x^3
        fuel_total_l = fuel_total_l + fuel_consumption_lph/3600.0/UPDATE_HZ

        -- ========== Fuel Pressure ==========
        -- Pressure falls until it hits a certain value, then jumps back up
        self.fuel_press = self.fuel_press - FP_RATE / UPDATE_HZ;
        if(self.fuel_press < (FUEL_PRESS - FP_VARI)) then
            self.fuel_press = FUEL_PRESS;
        end
    end
    -- return the instance
    return self
end -- end function engine_control(_driver, _idx)

local engine = engine_control()

local function update()
   if not efi_backend then
      efi_backend = efi:get_backend(0)
      if not efi_backend then
         return
      end
   end

   engine.simulate_engine()
   engine.set_EFI_State()

    -- Emulate ignition control on relay2
    if (SIM_PIN_MASK:get() & 0x2) == 0 then
        SERVO3_TRIM:set(SERVO3_MIN_BACKUP)
        SERVO3_MIN:set(SERVO3_MIN_BACKUP)
        SERVO3_MAX:set(SERVO3_MIN_BACKUP + 1)
    else
        SERVO3_TRIM:set(IDLE_PWM:get() or 1100)
        SERVO3_MIN:set(IDLE_PWM:get() or 1100)
        SERVO3_MAX:set(SERVO3_MAX_BACKUP)
    end

end

gcs:send_text(MAV_SEVERITY_INFO, SCRIPT_NAME .. string.format(" loaded"))

-- wrapper around update(). This calls update() and if update faults
-- then an error is displayed, but the script is not stopped
local function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs:send_text(MAV_SEVERITY_ERROR, "Internal Error: " .. err)
        -- when we fault we run the update function again after 1s, slowing it
        -- down a bit so we don't flood the console with errors
        return protected_wrapper, 1000
    end
    return protected_wrapper, 1000 / UPDATE_HZ
end

-- start running update loop
return protected_wrapper()
