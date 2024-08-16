# Modern Robotics

## Capstone Project
The Capstone project includes implementing software to execute a pic and place operation. The robot for the same is a YouBot with a universal mobile base and a 5-DOF robot arm. The same can be described on: [Modern Robotics Capstone Project](https://hades.mech.northwestern.edu/index.php/Mobile_Manipulation_Capstone). The material referred to is found [here](https://hades.mech.northwestern.edu/index.php/Modern_Robotics). <br>

Simulation results towards completing the course are displayed below. Primary approaches dealt with implementing a PI controller with Feed-forward correcting towards the computed *error-twist*.

<details>
<summary> Best Simulation</summary>

The task entails moving the cube from (x,y)=(1,0) to (x,y)=(0,-1).\
**Error-Twist over time.**<br>
![bestSim](https://github.com/adi25wagh/Modern_Robotics/blob/main/results/best/bestSimFig.png?raw=true)
<br>
**Simulation**<br>
![bestSim](https://github.com/adi25wagh/Modern_Robotics/blob/main/results/best/bestSim.gif?raw=true)
<br>

|Parameter|Proportional Gain|Integral Gain|Feed-Forwards enabled|
|--------:|-----------------|-------------|---------------------|
|		 Value|				 5.5|            3|                 True|
</details>

<details>
<summary> Overshoot </summary>

The task entails moving the cube from (x,y)=(1,0) to (x,y)=(0,-1).\
**Error-Twist over time.**<br>
![overshootSim](https://github.com/adi25wagh/Modern_Robotics/blob/main/results/overshoot/overshootSimFig.png?raw=true)
<br>
**Simulation**<br>
![overshootSim](https://github.com/adi25wagh/Modern_Robotics/blob/main/results/overshoot/overshootSim.gif?raw=true)
<br>
|Parameter|Proportional Gain|Integral Gain|Feed-Forwards enabled|
|--------:|-----------------|-------------|---------------------|
|		 Value|				 5.5|            3|                False|
</details>

<details>
<summary> New Task </summary>

The task entails moving the cube from (x,y)=(1,-1) to (x,y)=(-1,1).\
**Error-Twist over time.**<br>
![newTaskSim](https://github.com/adi25wagh/Modern_Robotics/blob/main/results/newTask/newTaskFig.png?raw=true)
<br>
**Simulation**<br>
![newTaskSim](https://github.com/adi25wagh/Modern_Robotics/blob/main/results/newTask/newTaskSim.gif?raw=true)
<br>
|Parameter|Proportional Gain|Integral Gain|Feed-Forwards enabled|
|--------:|-----------------|-------------|---------------------|
|     Value|				 5.5|            3|                 True|

</details>
