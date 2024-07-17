# Modern Robotics

## Capstone Project
The Capstone project includes implementing software intended to execute a pic and place operation. The robot for the the same is a YouBot with a universal mobile base and a 5-DOF robot arm. Descriptions for the same can be found on: [Modern Robotics Capstone Project](https://hades.mech.northwestern.edu/index.php/Mobile_Manipulation_Capstone). The material referred is found [here](https://hades.mech.northwestern.edu/index.php/Modern_Robotics). \
Simulation results towards completing the course are displayed below. Primary approaches dealt with implementing a PI controller with Feed-forward correcting towards the computed *error-twist*.

<details>
<summary> Best Simulation</summary>

The task entails moving the cube from (x,y)=(1,0) to (x,y)=(0,-1).\
**Error-Twist over time.**
![bestSim](https://github.com/adi25wagh/Modern_Robotics/blob/main/results/best/bestSimFig.png?raw=true)

**Simulation**
![bestSim](https://github.com/adi25wagh/Modern_Robotics/blob/main/results/best/bestSim.gif)

|Parameter|Proportional Gain|Integral Gain|Feed-Forwards enabled|
|--------:|-----------------|-------------|---------------------|
|		 Value|				 |            3|                 True|
</details>

<details>
<summary> Overshoot </summary>

The task entails moving the cube from (x,y)=(1,0) to (x,y)=(0,-1).\
**Error-Twist over time.**
![overshootSim](https://github.com/adi25wagh/Modern_Robotics/blob/main/results/overshoot/overshootSimFig.png?raw=true)

**Simulation**
![overshootSim](https://github.com/adi25wagh/Modern_Robotics/blob/main/results/overshoot/overshootSim.gif)

|Parameter|Proportional Gain|Integral Gain|Feed-Forwards enabled|
|--------:|-----------------|-------------|---------------------|
|		 Value|				 5.5|            3|                False|
</details>

<details>
<summary> New Task </summary>

The task entails moving the cube from (x,y)=(1,-1) to (x,y)=(-1,1).\
**Error-Twist over time.**
![newTaskSim](https://github.com/adi25wagh/Modern_Robotics/blob/main/results/newTask/newTaskFig.png?raw=true)

**Simulation**
![newTaskSim](https://github.com/adi25wagh/Modern_Robotics/blob/main/results/newTask/newTaskSim.gif)

|Parameter|Proportional Gain|Integral Gain|Feed-Forwards enabled|
|--------:|-----------------|-------------|---------------------|
|     Value|				 5.5|            3|                 True|

![image]()
</details>