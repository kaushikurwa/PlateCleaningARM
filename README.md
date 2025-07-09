# Plate Cleaning Robot using ABB IRB 120

##  📌Overview

This project showcases an automated plate-cleaning system developed using the ABB IRB 120 industrial robotic arm. Leveraging numerical inverse kinematics and a PD control strategy, the robot performs precise spiral cleaning motions over randomly positioned plates in a defined workspace.

The system emphasizes:
- **Numerical Inverse Kinematics (IK)** for precise joint angle computation.
- **Proportional-Derivative (PD) Control** for smooth trajectory following.
- **Spiral Trajectory Planning** to replicate manual scrubbing.
- **Cubic Polynomial Transitions** for safe, efficient movement between plates.

---

##  🛠️Features

- **Task Space Definition:** Ensures safe operation by excluding robot base area.
- **Random Plate Position Generation:** Places multiple plates with no overlap and safe spacing.
- **Spiral Cleaning Path:** Cleans each plate using a widening spiral.
- **Shortest Path Optimization:** Chooses the next nearest plate to minimize time.
- **Collision-Free Transitions:** Uses smooth, collision-avoiding paths.
- **Real-Time Simulation:** Visualizes cleaning and transition motions in MATLAB.

---

##  🧩System Flow

1. **Initialize Robot:** ABB IRB 120 starts at origin with zero joint configuration.
2. **Generate Plates:** Random positions within workspace.
3. **Clean Plates:** Spiral trajectory cleans each plate thoroughly.
4. **Transition:** Cubic polynomial path moves to next nearest plate.
5. **Loop:** Repeat until all plates are cleaned.

---

##  📊Tech Stack

- **Robot:** ABB IRB 120
- **Language:** MATLAB
- **Methods:** Numerical IK, PD Control, Cubic Polynomial Trajectories
- **Libraries:** Robotics Toolbox for MATLAB

---

##  Visuals
<p align="center">
  <img src="/images/ABB ARM.gif" width="500">
</p>

<img src="/images/1.png" width="500"><img src="/images/2.png" width="500">
<img src="/images/3.png" width="500"><img src="/images/4.png" width="500">


---

##  🚀How to Run

1. Clone this repository
   ```bash
   git clone https://github.com/kaushikurwa/PlateCleaningARM.git
   ```
2. Open MATLAB and run:
   ```matlab
   PlateCleaningRobot.mlx
   ```
3. Watch the robot simulate cleaning multiple plates with smooth transitions.

---

##  ✅Future Work

- Integrate real-time vision for plate detection.
- Implement physical hardware control with ABB RobotStudio.
- Add soap dispensing and water spray modules.

---

##  📄License

This project is open-source for educational purposes.

---
