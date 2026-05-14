# QuadPlane XY Position AutoTune

This applet tunes the QuadPlane XY controller in QLOITER with two sweeps:

- Q_P_VELXY_P with Q_P_VELXY_I coupled to it by PTUN_IPRAT
- Q_P_POSXY_P

For each candidate, the script commands repeatable body-frame target steps around a fixed center point captured when tuning starts. It scores each candidate from measured horizontal position error, horizontal speed, horizontal acceleration estimate, cross-track error, lean, and settling time. If the response shows toilet-bowling or oscillation, the current sweep stops and the best tested candidate is selected.

## Intended Use

Use this script only after:

- the VTOL attitude and rate loops are already tuned
- GPS and EKF position hold are healthy
- the vehicle can already hold a sort of stable hover in QLOITER

This is a QuadPlane applet. It is not intended for fixed-wing GUIDED tuning and it does not tune vertical position control.

## Switch Behaviour

The script uses an RC switch assigned to the PTUN_RC_FUNC scripting option.

- Low: stop tuning and restore unsaved values
- Middle: start or continue tuning
- High: save the current tuned values

## Parameters

- PTUN_ENABLE: set to 1 to enable the applet
- PTUN_RC_FUNC: scripting RC function number, default 300
- PTUN_STEP_M: commanded horizontal target step size in meters
- PTUN_SETTLE_M: position threshold used for settle detection, default 0.1
- PTUN_SETTLE_V: velocity threshold used for settle detection, default 0.1
- PTUN_SETTLE_T: time inside the settle thresholds before a step is counted as settled, default 1.0
- PTUN_STEP_T: maximum time allowed for one response measurement
- PTUN_POSW: position-error weighting in the candidate score
- PTUN_VELW: velocity weighting in the candidate score
- PTUN_ACCW: acceleration weighting in the candidate score
- PTUN_AUTOSAVE: auto-save delay in seconds, 0 disables auto-save
- PTUN_MINMUL: minimum candidate multiplier relative to the starting gain
- PTUN_MAXMUL: maximum candidate multiplier relative to the starting gain
- PTUN_STEPS: number of evenly spaced candidates tested between PTUN_MINMUL and PTUN_MAXMUL
- PTUN_IPRAT: maximum allowed Q_P_VELXY_I : Q_P_VELXY_P ratio during the coupled velocity sweep and in the final selection

## Recommended Workflow

1. Enter QLOITER and establish a steady hover.
2. Put the RC switch in the middle position.
3. Let the applet sweep Q_P_VELXY_P/Q_P_VELXY_I, then Q_P_POSXY_P.
4. Watch the GCS messages. Each tested value reports its score, and each completed sweep reports the selected final value.
5. Move the switch high to save, or low to discard and restore.

## Notes

- The applet stops tuning if the vehicle leaves QLOITER.
- The tuning center is the vehicle's current location when the tune starts, and the script always pulls the aircraft back to that center between candidates.
- The test directions are body-frame forward, back, right, and left so the excitation stays aligned with the aircraft heading.
- Q_P_POSXY_P is chosen by lowest score from the tested candidates. There is no extra reduction or overshoot-limit post-processing.

## Advanced 

### Score Equation

The running score accumulated during the measurement is:

$$
S_{run} =
PTUN\_POSW \cdot \int e_{pos}\,dt +
PTUN\_VELW \cdot \int v_{xy}\,dt +
PTUN\_ACCW \cdot \int a_{xy}\,dt +
PTUN\_POSW \cdot 0.75 \cdot \int |e_{cross}|\,dt +
0.35 \cdot \int \theta_{lean}\,dt +
0.02 \cdot \int \dot{\theta}_{lean}\,dt
$$


- $e_{pos}$ is horizontal position error to the commanded step target
- $v_{xy}$ is horizontal speed
- $a_{xy}$ is estimated horizontal acceleration
- $e_{cross}$ is cross-track error relative to the commanded step axis
- $\theta_{lean}$ is total lean angle magnitude from roll and pitch
- $\dot{\theta}_{lean}$ is lean-rate magnitude

The finalized per-step score used by the tuner is then:

$$
S_{final} = S_{run} + t_{settle} +
\begin{cases}
5000, & \text{if toilet-bowling or oscillation is detected} \\
0, & \text{otherwise}
\end{cases}
$$

Each particular step's value is scored by averaging $S_{final}$ across the four commanded body-frame steps.
