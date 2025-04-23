# 🐑 AI Sheepdog Bot Simulation (MDP + A*)

### Author: Hench Wu  
Summer 2023  
**Files:**  
- `SheepRobot3.java` – Main program  
- `NodeDL.java` – Doubly-linked node class representing each map cell  

How to Run
Compile the Java files:
javac SheepRobot3.java NodeDL.java

Run the simulation:
java SheepRobot3

---

## 📌 Overview

This project simulates an intelligent **robot sheepdog** guiding a **sheep** into a central **pen** using Markov Decision Processes (MDP) and A* search.

- The robot adapts its movement using value iteration and dynamic reward mapping.
- The sheep has **passive** (wandering) and **active** (reactive) behaviors depending on the robot’s proximity.
- The simulation tests **robot path optimization**, **obstacle avoidance**, and **decision-making under uncertainty**.

---

## 🧠 AI Features

### 🤖 Robot
- Uses **MDP** for decision-making with adaptive rewards.
- Switches between **goal-directed behavior** and **sheep-leading behavior** based on sheep's position.
- Occasionally uses **A\*** for shortest path computation.

### 🐑 Sheep
- Passively wanders unless the robot is within a scanning radius.
- Actively charges at the robot when detected.
- Adjusts its directional probability to evade or pursue based on MDP-style weights.

---

## 🔧 How It Works

1. A grid (default 31x31) is initialized.
2. The **pen (goal)** is placed in the center.
3. The **robot** starts in the bottom-right corner.
4. The **sheep** starts in the upper-left corner.
5. The robot leads the sheep into the pen while avoiding being caught.

---

## 🎮 Visual Mode

By default, `visual = true`:
- Simulation prints the grid in the console with live updates:
  - `R`: Robot
  - `S`: Sheep
  - `G`: Goal (pen)
  - `X`: Wall or blocked tile
  - `*`: Visited cell (A* path)
  
To turn off visualization and run batch trials:
```java
visual = false;
int trial = 40;

