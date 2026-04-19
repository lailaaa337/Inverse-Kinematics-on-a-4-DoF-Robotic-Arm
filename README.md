
# Robotic Arm Kinematics & Simulation (Forward + Inverse Kinematics)

##  Overview
This project focuses on the **modeling, analysis, and simulation of a multi-link robotic arm**, including both **forward and inverse kinematics**.

The system is designed to:
- Compute the end-effector position using joint angles  
- Model transformations using Denavit-Hartenberg (DH) parameters  
- Simulate robotic movement and gripper control in a virtual environment  

This project combines **robotics theory, mathematical modeling, and simulation**, making it a strong foundation for real-world robotic systems.

---

##  Project Objectives

- Model a robotic arm using **DH parameters**
- Derive **forward kinematics equations**
- Compute transformation matrices for each link
- Implement **inverse kinematics**
- Simulate robot behavior using **CoppeliaSim**
- Control and test a robotic **gripper mechanism**

---

##  System Description

The robotic system consists of a **4-DOF robotic arm** with a gripper.

Each joint contributes to the overall motion of the end-effector:
- θ1 → Base rotation  
- θ2 → Shoulder movement  
- θ3 → Elbow movement  
- θ4 → Wrist/gripper orientation  

> As shown in the project documentation (page 2), the system is modeled using DH parameters defining links and transformations :contentReference[oaicite:0]{index=0}.

---

##  Kinematics Modeling

### 🔹 Denavit-Hartenberg Parameters

| Link | θ (theta) | d | a | α (alpha) |
|------|----------|---|---|-----------|
| 1    | θ1       | 0 | 0 | +90°      |
| 2    | θ2       | 0 | L1| 0         |
| 3    | θ3       | 0 | L2| 0         |
| 4    | θ4       | 0 | L3| 0         |

---

###  Forward Kinematics

The position of the end-effector (X, Y, Z) is computed as:

```math
X = cos(θ1) [L2 cos(θ2) + L3 cos(θ2 + θ3) + L4 cos(θ2 + θ3 + θ4)]
````

```math
Y = sin(θ1) [L2 cos(θ2) + L3 cos(θ2 + θ3) + L4 cos(θ2 + θ3 + θ4)]
```

```math
Z = L1 + L2 sin(θ2) + L3 sin(θ2 + θ3) + L4 sin(θ2 + θ3 + θ4)
```

> These equations define how joint angles affect the spatial position of the robot.

---

###  Transformation Matrices

Each link transformation is represented using homogeneous matrices:

* A₁ (Base rotation)
* A₂ (Link 2)
* A₃ (Link 3)
* A₄ (Link 4)

Overall transformation:

```math
T_0^4 = A_1 \cdot A_2 \cdot A_3 \cdot A_4
```

> As illustrated in the report (page 3), each matrix contributes to the final pose of the end-effector.

---

##  Inverse Kinematics

* Implemented using **numerical and matrix-based approaches**
* Allows computing joint angles from a desired end-effector position

> The implementation (page 4) demonstrates matrix multiplication and rotation composition for solving inverse kinematics .

---

##  Gripper System

* Simulated grasping mechanism
* Supports:

  * Open / Close actions
  * Object interaction
  * Stability testing

> Page 5 highlights how the gripper was tested in simulation to ensure reliable object handling.

---

##  Simulation Environment

The system was implemented and tested using:

###  CoppeliaSim

* Simulated robotic arm movement
* Visualized kinematics in real-time
* Tested gripper interaction with objects
* Debugged control logic safely

> This allowed validation of the system before any real hardware implementation.

---

##  Technologies Used

* **MATLAB (Robotics Toolbox)**
* **Python (NumPy for matrix operations)**
* **CoppeliaSim**
* **Robotics Kinematics (DH Modeling)**

---

##  Project Demo

<p align="center">
  <img src="./ProjectDemo.gif" width="700"/>
</p>
---

##  How It Works

1. Define robot structure using DH parameters
2. Compute transformation matrices
3. Calculate forward kinematics
4. Apply inverse kinematics for positioning
5. Simulate robot in CoppeliaSim
6. Control gripper for object manipulation

---

##  What I Learned

* Modeling robotic systems mathematically
* Understanding forward vs inverse kinematics
* Working with transformation matrices
* Simulation-based testing
* Debugging robotic motion without hardware
* Combining theory with practical implementation

---

##  Future Improvements

* Add real hardware implementation
* Implement trajectory planning
* Add PID control for smoother motion
* Integrate computer vision
* Upgrade to 6-DOF robotic arm

---

##  Disclaimer

This project is for **educational and simulation purposes only**.

---

##  Team Members

* Laila Tarek
* Miran Samer
* Mariam Eladawy
* Abdullah Nizar

---

##  Project Report

 Full documentation available here:
[View Report](Document%2029.pdf)

```



