package frc.robot;

/*
 * ============================ RobotContainer ============================
 *
 * RobotContainer is the "wiring hub" of a command-based robot.
 *
 * This class is responsible for:
 *   - Creating ONE instance of each subsystem
 *   - Creating controllers (joysticks, gamepads)
 *   - Defining how buttons trigger commands
 *   - Defining default commands (what runs when nothing else is scheduled)
 *   - Selecting which autonomous command to run
 *
 * This class is NOT responsible for:
 *   - Low-level motor control
 *   - Reading sensors directly
 *   - Implementing robot behavior logic
 *
 * Those responsibilities belong in:
 *   - Subsystems (hardware ownership + basic actions)
 *   - Commands (requesting actions from subsystems)
 *
 * Think of RobotContainer as a wiring diagram:
 * it connects inputs (controllers, autos) to behaviors (commands),
 * but it does not DO the work itself.
 */

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

  /* ===================================================================== */
  /*                           SUBSYSTEM CREATION                           */
  /* ===================================================================== */
  /*
   * Subsystems are created ONCE and live for the entire life of the robot.
   *
   * Each subsystem:
   *   - Owns its hardware (motors, sensors)
   *   - Provides methods or commands to control that hardware
   *
   * Commands and RobotContainer NEVER directly control motors.
   */

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();

 

  /* ===================================================================== */
  /*                   COMMANDS SHARED WITH AUTONOMOUS                      */
  /* ===================================================================== */
  /*
   * These commands are created as fields so they can be:
   *   - Triggered by controller buttons
   *   - Triggered by PathPlanner autonomous routines
   *
   * PathPlanner refers to commands by STRING NAME, so these must exist
   * ahead of time and remain alive.
   */

  public final Command ele_GoLoad =
      new InstantCommand(
          () -> m_elevator.setTargetPosition(ElevatorConstants.kStageLoad),
          m_elevator);

  public final Command ele_GoL1 =
      new InstantCommand(
          () -> m_elevator.setTargetPosition(ElevatorConstants.kStageL1),
          m_elevator);

  public final Command ele_GoL2 =
      new InstantCommand(
          () -> m_elevator.setTargetPosition(ElevatorConstants.kStageL2),
          m_elevator);

  public final Command ele_GoL3 =
      new InstantCommand(
          () -> m_elevator.setTargetPosition(ElevatorConstants.kStageL3),
          m_elevator);

  /* ===================================================================== */
  /*                            DRIVER CONTROLLERS                          */
  /* ===================================================================== */
  /*
   * RobotContainer is responsible for reading controllers.
   *
   * Subsystems should NEVER read joysticks directly.
   * This keeps subsystems reusable and testable.
   */

  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  private final CommandXboxController m_driverController2 =
      new CommandXboxController(OIConstants.kDriverController2Port);

  /* ===================================================================== */
  /*                           AUTONOMOUS SELECTION                         */
  /* ===================================================================== */
  /*
   * The SendableChooser allows drivers to select which autonomous routine
   * to run using Shuffleboard / SmartDashboard.
   *
   * Each option corresponds to a PathPlanner ".auto" file.
   */

  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  /* ===================================================================== */
  /*                               CONSTRUCTOR                              */
  /* ===================================================================== */

  public RobotContainer() {

    /* ----------------------------------------------------------------- */
    /*                    PATHPLANNER NAMED COMMANDS                      */
    /* ----------------------------------------------------------------- */
    /*
     * PathPlanner autos can reference "Named Commands" by name.
     *
     * Example:
     *   In PathPlanner, an event marker named "L2"
     *   will trigger the command registered as "L2" here.
     *
     * IMPORTANT:
     *   The string names MUST match exactly.
     */

    NamedCommands.registerCommand("Load", ele_GoLoad);
    NamedCommands.registerCommand("L1", ele_GoL1);
    NamedCommands.registerCommand("L2", ele_GoL2);
    NamedCommands.registerCommand("L3", ele_GoL3);

    NamedCommands.registerCommand("Intake", m_shooter.olIntakeCommand());
    NamedCommands.registerCommand("Reverse", m_shooter.reverseIntakeCommand());
    NamedCommands.registerCommand("Shoot", m_shooter.releaseCommand());
    NamedCommands.registerCommand("Stop", m_shooter.stopMotor());

    /* ----------------------------------------------------------------- */
    /*                         AUTONOMOUS OPTIONS                         */
    /* ----------------------------------------------------------------- */

    m_chooser.addOption("preload_Auto", new PathPlannerAuto("preload_Auto"));
    m_chooser.addOption("L2_JI_C2", new PathPlannerAuto("L2_JI_C2"));
    m_chooser.addOption("L2_FE_C5", new PathPlannerAuto("L2_FE_C5"));
    m_chooser.addOption("Move_Forward_Short", new PathPlannerAuto("Move_Forward_Short"));
    m_chooser.addOption("Do Nothing", new Command() {});

    SmartDashboard.putData("Auto Chooser", m_chooser);

    /* ----------------------------------------------------------------- */
    /*                        BUTTON CONFIGURATION                         */
    /* ----------------------------------------------------------------- */

    configureButtonBindings();

    /* ----------------------------------------------------------------- */
    /*                         DEFAULT COMMANDS                            */
    /* ----------------------------------------------------------------- */
    /*
     * Default commands run whenever no other command is using a subsystem.
     *
     * For the drivetrain, this means:
     *   - When the driver is not running an auto or special command,
     *     joystick input controls the robot.
     *
     * LAMBDA EXPLANATION:
     *   The "() ->" code below does NOT run immediately.
     *   It is stored and executed repeatedly (~50 times per second)
     *   by the WPILib command scheduler.
     */

    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(
                    m_driverController.getLeftY(),
                    OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(
                    m_driverController.getLeftX(),
                    OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(
                    m_driverController.getRightX(),
                    OIConstants.kDriveDeadband)),
            m_robotDrive));
  }

  /* ===================================================================== */
  /*                           BUTTON BINDINGS                              */
  /* ===================================================================== */

  private void configureButtonBindings() {

    /* ---------------------- Driver Controller ------------------------ */

    // Left trigger: force swerve modules into X configuration
    m_driverController.leftTrigger()
        .whileTrue(
            new RunCommand(
                () -> m_robotDrive.setX(),
                m_robotDrive));

    /*
     * Right bumper controls field-relative mode.
     *
     * When held:
     *   - Robot drives in ROBOT-relative mode (useful for testing)
     * When released:
     *   - Return to FIELD-relative driving
     */
    m_driverController.rightBumper()
        .whileFalse(new RunCommand(() -> m_robotDrive.setFieldRelative(true)))
        .whileTrue(new RunCommand(() -> m_robotDrive.setFieldRelative(false)));

    /* ---------------------- Copilot Controller ------------------------ */

    // Elevator preset positions
    m_driverController2.a().toggleOnTrue(ele_GoL1);
    m_driverController2.b().toggleOnTrue(ele_GoL2);
    m_driverController2.x().toggleOnTrue(ele_GoL3);
    m_driverController2.y().toggleOnTrue(ele_GoLoad);
    m_driverController2.rightBumper().toggleOnTrue(ele_GoLoad);

    // Intake / shooter controls
    m_driverController2.leftBumper()
        .whileTrue(m_shooter.reverseIntakeCommand());

    m_driverController.rightTrigger()
        .whileTrue(m_shooter.releaseCommand());

    m_driverController2.rightTrigger()
        .onTrue(m_shooter.olIntakeCommand())
        .onFalse(m_shooter.stopMotor());

    // Elevator homing / reset
    m_driverController2.povDown()
        .onTrue(m_elevator.slowBottom())
        .toggleOnFalse(m_elevator.resetElevator());
  }

  /* ===================================================================== */
  /*                              AUTONOMOUS                                */
  /* ===================================================================== */

  /*
   * This method is called by Robot.java at the start of autonomous mode.
   *
   * Whatever command is returned here will be scheduled and run.
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

}