# ismpc_walking

Walking Controller Using IS-MPC Gait Generation, tested with HRP4, HRP4CR, HRP2

## Dependencie

- [mc_rtc](https://github.com/jrl-umi3218/mc_rtc)
- [Footsteps planner](https://github.com/antodld/FootSteps_Planner) 
- [pendulum feasibility solver](https://github.com/antodld/pendulum_feasibility_solver) 

## Installation
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

