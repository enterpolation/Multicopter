# Multicopter framework
Simulation and stabilization of a multicoper written with Python.
## Quick start
Run multicopter package from the inside of repository directory.
```
python -m multicopter\
    --path_to_config <PATH TOCONFIG>\
    --simulation_time <TIME IN SECONDS>\
    --number_of_points <NUMBER OF TIME POINTS>\
```

If you want to plot phase vector and rotor speeds, use flags `--plot_phase` and
`--plot_speeds`.

## Config example
```yaml
G: 9.81

NUM_ROTORS: 4
ROTOR_ANGELS: [0, 90, 180, 270]

M: 0.3
L: 0.225
K: 2.980e-6
B: 1.140e-7

J_X: 4.856e-3
J_Y: 8.801e-3
J_Z: 4.856e-3
```