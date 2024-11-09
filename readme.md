# MPC-DS: A Safe Path Tracking Method for UGVs in Dynamic Environments with Dense Obstacles


## Introduction
This repository contains the source code for the paper **"MPC-DS: A Safe Path Tracking Method for UGVs in Dynamic Environments with Dense Obstacles"**. The approach integrates Model Predictive Control (MPC) to enhance the safety and path tracking of Unmanned Ground Vehicles (UGVs) in complex, dynamic settings with dense obstacles.

## Current Status
Currently, only partial code and demo materials are available. The full code will be released once the paper is officially accepted. Stay tuned for updates.




## Abstract
In narrow environments with dynamic dense obstacles, it is difficult to find feasible paths for unmanned ground vehicles (UGVs) by using existing methods due to the strict security constraints on UGV movements.
To overcome such difficulty, this work proposes an improved local path tracking algorithm based on model predictive control and a dynamic control barrier function with slack variable (MPC-DS).
In this algorithm, slack variable is integrated with the control barrier function to convert the strict constraints to soft ones.
To regulate the values of slack variable, a suitable penalty coefficient selected by using a series of comparative simulations is incorporated into MPC's cost function.
To test effectiveness of the proposed method, it is compared with three mainstream methods in environments with dense and sparse dynamic obstacles.
Results of simulations and physical experiments show that UGV controlled by the proposed algorithm can avoid obstacles safely and efficiently in  complex environments.
It is worth noting that its use reduces energy consumption  by 54.9\% in comparison with the existing ones.

## Features
- Safe path tracking with dynamic obstacle avoidance.
- Real-time path optimization for UGVs.
- Compatible with dense, cluttered environments.


## Demo Videos
To see MPC-DS in action, check out the following demo videos:
- [Experiment and paper videos](https://www.youtube.com/watch?v=PIQQxaxqipc)

## License
[MIT License](LICENSE)

