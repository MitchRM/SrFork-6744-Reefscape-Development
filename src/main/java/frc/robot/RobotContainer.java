// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.auto.AutonomousCommand;
import frc.robot.commands.auto.AutonomousCommand2;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
//import frc.robot.BuildConstants;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  public void updateshuffleboard(){
    SmartDashboard.updateValues();
  }

  public void setRelativeCommandFalse(){
    fieldrelative = false;
  }
  public void setRelativeCommandTrue(){
    fieldrelative = true;
  }
  public void toggleFieldRelative(){ // is this unused?
    fieldrelative = !fieldrelative;
  }                          

  public boolean fieldrelative = true;





// The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  public final Command ele_GoLoad = new InstantCommand(() -> m_elevator.setTargetPosition(ElevatorConstants.kStageLoad), m_elevator);
  public final Command ele_GoL1 = new InstantCommand(() -> m_elevator.setTargetPosition(ElevatorConstants.kStageL1), m_elevator);
  public final Command ele_GoL2 = new InstantCommand(() -> m_elevator.setTargetPosition(ElevatorConstants.kStageL2), m_elevator);
  public final Command ele_GoL3 = new InstantCommand(() -> m_elevator.setTargetPosition(ElevatorConstants.kStageL3), m_elevator);
  public final AutonomousCommand autoCommand = new AutonomousCommand(m_robotDrive);
  public final AutonomousCommand2 autoCommand2 = new AutonomousCommand2(m_robotDrive);




  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_driverController2 = new CommandXboxController(OIConstants.kDriverController2Port);

  //m_chooser
  SendableChooser<Command> m_chooser = new SendableChooser<>();


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    //m_chooser

    // Shuffleboard.getTab("Autonomous").add(m_chooser);

    NamedCommands.registerCommand("Load", ele_GoLoad);
    NamedCommands.registerCommand("L1", ele_GoL1);
    NamedCommands.registerCommand("L2", ele_GoL2);
    NamedCommands.registerCommand("L3", ele_GoL3);

    NamedCommands.registerCommand("Intake", m_shooter.olIntakeCommand());
    NamedCommands.registerCommand("Reverse", m_shooter.reverseIntakeCommand());
    NamedCommands.registerCommand("Shoot", m_shooter.releaseCommand());
    NamedCommands.registerCommand("Stop", m_shooter.stopMotor());

    m_chooser.addOption("preload_Auto", new PathPlannerAuto("preload_Auto"));
    m_chooser.addOption("L2_JI_C2", new PathPlannerAuto("L2_JI_C2"));
    m_chooser.addOption("L2_FE_C5", new PathPlannerAuto("L2_FE_C5"));

    m_chooser.addOption("Move_Forward_Short", new PathPlannerAuto("Move_Forward_Short"));
    m_chooser.addOption("Do Nothing", new Command(){});
    SmartDashboard.putData("Auto Chooser", m_chooser);





    // Configure the button bindings
    configureButtonBindings();
      

    

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                fieldrelative),
            m_robotDrive));
    /*
    m_elevator.setDefaultCommand(
      new RunCommand(
        () -> m_elevator.stickControl(-MathUtil.applyDeadband(m_driverController2.getLeftY(), OIConstants.kDriveDeadband)), 
        m_elevator
      )
    );
    */
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

  // Driver controller - mdriverController
    // Right trigger sets swerve in X configuration
    m_driverController.leftTrigger()
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive
        ));
    // Right bumper controls field reletive - button relesed set to robot relative for swerve testing
    m_driverController.rightBumper()
        .whileFalse(new RunCommand(
          () -> setRelativeCommandTrue()))
        .whileTrue(new RunCommand(
          () -> setRelativeCommandFalse()));      

  //Copilot controller - mdriverController2
    // A button elevator stage L1
    m_driverController2.a().toggleOnTrue(ele_GoL1);
    // B button elevator stage L2
    m_driverController2.b().toggleOnTrue(ele_GoL2);
    // X button elevator stage L3
    m_driverController2.x().toggleOnTrue(ele_GoL3);
    // Left bumper elevator stage Load
    m_driverController2.y().toggleOnTrue(ele_GoLoad);
    // Right bumper elevator stage Load
    m_driverController2.rightBumper().toggleOnTrue(ele_GoLoad);
    // Left Bumper
    m_driverController2.leftBumper().whileTrue(m_shooter.reverseIntakeCommand());
    // Right trigger triggers release command to shoot
    m_driverController.rightTrigger().whileTrue(m_shooter.releaseCommand());
    // Left trigger intakes coral
    m_driverController2.rightTrigger().onTrue(m_shooter.olIntakeCommand()).onFalse(m_shooter.stopMotor());
    // Pilot D-Pad Down to Reset the elevator
    m_driverController2.povDown().onTrue(m_elevator.slowBottom()).toggleOnFalse(m_elevator.resetElevator());     
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }


    // Print Git Data (maybe we will try this later)
    //public void printGitData() {
    //  System.out.println("Repo:" + BuildConstants.MAVEN_NAME);
    //  System.out.println("Branch:" + BuildConstants.GIT_BRANCH);
    //  System.out.println("Git Date:" + BuildConstants.GIT_DATE);
    //  System.out.println("Build Date:" + BuildConstants.BUILD_DATE);
    //};
  
}
