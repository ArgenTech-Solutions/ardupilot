# EFI SITL Simulator

This script pretends to be a simple EFI engine for the SITL simulator. It supports the following features:
- Engine heat model, which takes into account air temperature (including the
  effect of altitude) and airspeed. Also very crudely models the effect of a
  cowling flap to prevent over cooling.
- Fuel flow model.
- Crude fuel pressure fluctuations.
- Ignition enable/disable relay, hacked into the throttle servo channel. The
  script looks at the GPIO outputs in the `SIM_PIN_MASK` parameter. If the
  engine ignition relay is off, the script locks the throttle to 1000us. If the
  ignition relay is on, the script uses the throttle input as normal, and has a
  minimum of 1100us to simulate a low idle.

## Parameters

### `SIM_EFI_CHT_INC`
The reported cylinder head temperature will increase by this many degrees
celcius. This is here so an instructor can simulate an overheating engine for a 
student.

### `SIM_EFI_IDL_PWM`
Tweak this value "to taste" to get the desired idle RPM.
