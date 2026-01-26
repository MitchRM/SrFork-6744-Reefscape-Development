# PathPlanner in This Robot Code

This robot uses **PathPlanner** to create and run autonomous routines.

Instead of writing long autonomous Java code by hand, we:

1. Design paths and autos visually in PathPlanner
2. Save them as files
3. Load and execute them from Java

This makes autonomous easier to build, debug, and change.

---

## 📁 Where PathPlanner Files Live

All PathPlanner files are stored in: src/main/deploy/pathplanner

Inside that folder you will see:

### `paths/`
Contains `.path` files.

A **path** is a single trajectory:
- Where the robot drives
- How fast it moves
- How it turns

Paths are building blocks.


### `autos/`
Contains `.auto` files.

An **auto** is a full autonomous routine made from:
- One or more paths
- Event markers
- Named commands

Autos are what the driver actually selects on the dashboard.

---

## 🔗 How Java Connects to PathPlanner

The connection happens in two places:


### 1. DriveSubsystem

In `DriveSubsystem`, we call:

```java
AutoBuilder.configure(...
```

This tells PathPlanner:
	•	How to get the robot’s current position
	•	How to reset odometry at the start of auto
	•	How to drive the robot using ChassisSpeeds

You usually do NOT need to change this unless drivetrain hardware changes.

Think of this as “plugging PathPlanner into the drivetrain.”

### 2. RobotContainer

In RobotContainer, we do two important things.

**A. Register Named Commands**
Example:
```java
NamedCommands.registerCommand("L2", ele_GoL2);
```
This means:

If PathPlanner sees an event marker named "L2",
it will run the ele_GoL2 command.

These string names MUST match exactly.

Named commands allow autos to:
	•	Move the elevator
	•	Intake coral
	•	Shoot
	•	etc.
without writing special Java auto code.


**B. Load Autos by Name**
Example:
 ```java
 m_chooser.addOption("L2_JI_C2", new PathPlannerAuto("L2_JI_C2"));
```
This loads the auto named L2_JI_C2.auto
from the deploy folder.

The chooser appears on Shuffleboard as “Auto Chooser.”

Drivers select which auto to run before the match.

⸻

🧠 **Mental Model**

Think of it like this:

PathPlanner GUI
⬇
Creates .path and .auto files
⬇
Robot deploys those files
⬇
RobotContainer loads autos by name
⬇
DriveSubsystem follows the paths
⬇
NamedCommands trigger elevator / shooter actions

Java does NOT describe the autonomous paths.
Java only executes what PathPlanner creates.

⸻

⚠ Important Notes
	•	Paths and autos must be redeployed after changes
	•	Named command strings must match exactly
	•	Odometry must be correct for autos to work
	•	Autos depend heavily on DriveSubsystem

⸻

✅ For New Programmers

If you are learning this code:
	1.	Read RobotContainer
	2.	Then DriveSubsystem
	3.	Then come back here

PathPlanner makes much more sense once you understand those files.
