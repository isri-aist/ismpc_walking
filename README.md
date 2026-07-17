# ismpc_walking

Walking Controller using IS-MPC Gait Generation, tested with HRP4, HRP4CR, HRP-2Kai

## Dependencies

- [mc_rtc](https://github.com/jrl-umi3218/mc_rtc)
- [Footsteps planner](https://github.com/antodld/FootSteps_Planner) 
- [pendulum feasibility solver](https://github.com/antodld/pendulum_feasibility_solver) 

## Installation

### Nix
#### Development shell

A default `mc_rtc.yaml` will be used. If you wish to change the robot, simply create a minimal `mc_rtc.yaml` file with

```yaml
MainRobot: <robot module name>
Timestep: <robot timestep> 
```

##### Minimal environment (run)

No dynamics simulation, only the default JVRC1 robot

```
nix develop .#mc-rtc-superbuild-ismpc-minimal
mc_rtc_ticker [-f mc_rtc.yaml]
```

##### Full environment (run)

All robots, `mc_mujoco` simulation

```
nix develop .#mc-rtc-superbuild-ismpc-full
mc_mujoco -s [-f mc_rtc.yaml]
```

##### Local development

If you wish to work on the controller's code, use

```
nix develop .#mc-rtc-superbuild-ismpc-full-devel
```

and follow the displayed instructions.


### From source

```
cd ismpc_walking
mkdir build && cd build
cmake ..
make 
sudo make install
```

## Run the controller
```
Enabled: [ismpc_walking]
MainRobot: HRP2DRC
```

