# ForNewProgrammers
Welcome to the robot code!

This project uses WPILib’s **command-based framework**.
It may feel overwhelming at first — that’s normal.

Here’s how to explore the code without getting lost.

---

## ✅ Step 1: Start with RobotContainer

Open:
`src/main/java/frc/robot/RobotContainer.java`

This is the MOST IMPORTANT file.
It shows:
- Which subsystems exist
- How controllers are mapped to actions
- How autonomous is selected
- How commands are wired together
Read this first.

Don’t worry about understanding every line — focus on the structure.

---

## ✅ Step 2: DriveSubsystem

Next:
`subsystems/DriveSubsystem.java`

This explains:
- Swerve drive
- Field-relative driving
- Odometry
- PathPlanner integration

You do NOT need to understand all the math.
Focus on:
- What the subsystem owns
- Which methods commands call

---

## ✅ Step 3: ElevatorSubsystem

Then:
`subsystems/ElevatorSubsystem.java`

This teaches:
- Closed-loop control
- Target positions
- How two motors work together
- How subsystems can provide commands

This is a great example of a mechanism subsystem.

---

## ✅ Step 4: ShooterSubsystem

Finally:
`subsystems/ShooterSubsystem.java`
This is simpler:

- One motor

- Intake / shoot / stop

- Color sensor

Good example of open-loop control.

---

## ⚠ Files You Can Mostly Ignore at First

- MAXSwerveModule (vendor code)

- Configs.java (hardware tuning)

- Constants.java (numbers and IDs)

These are important later — not on day one.

---

## 🧠 Important Concepts
### Subsystems own hardware

Motors and sensors live in subsystems.

Commands never touch hardware directly.

---

### Commands request behavior

Commands tell subsystems what to do.

Subsystems decide HOW to do it.

---

### Lambdas do NOT run immediately

Code like:
```java
() -> controller.getLeftY()
```
is stored and called later by the scheduler.

This is critical to understand.
---

## **🎯 Final Advice**

Don’t try to understand everything at once.

Learn in layers:

RobotContainer
→ DriveSubsystem
→ ElevatorSubsystem
→ ShooterSubsystem

Ask questions.

Experiment.

Break things (safely).

That’s how you learn robotics.