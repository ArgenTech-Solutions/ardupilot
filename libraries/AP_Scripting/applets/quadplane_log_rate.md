# Quadplane Log Rate Control

This Lua script for ArduPilot provides enhanced control over logging parameters
based on whether the quadplane is in forward flight or hover mode. This allows
for high-rate logging for detailed control analysis during hover and a lower
rate during forward flight to conserve space on the SD card.

This breaks out the existing `LOG_BITMASK` and `LOG_FILE_RATEMAX` parameters
into two pairs: one for forward flight and one for hover. The script then
automatically switches between the two pairs based on the current flight mode.

## Parameters

### Q_LOG_MASK_FF
Loaded into `LOG_BITMASK` when in forward flight.

### Q_LOG_MASK_HV
Loaded into `LOG_BITMASK` when in hover.

### Q_LOG_RATEMAX_FF
Loaded into `LOG_FILE_RATEMAX` when in forward flight.

### Q_LOG_RATEMAX_HV
Loaded into `LOG_FILE_RATEMAX` when in hover.
