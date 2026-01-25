package frc.robot.subsystems;

/*
 * ========================= OVERVIEW =========================
 *
 * The DriveSubsystem is responsible for EVERYTHING related to
 * moving the robot around the field.
 *
 * It owns:
 *   - The four swerve modules (motors + encoders)
 *   - The gyro (robot heading)
 *   - Odometry (tracking where the robot is on the field)
 *   - The interface used by PathPlanner for autonomous paths
 *
 * Other code (commands) is NOT allowed to directly control motors.
 * Commands must call methods on this subsystem instead.
 *
 * This separation is a key idea in command-based robot design.
 */

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  /* ==========================================================
   *                     HARDWARE OBJECTS
   * ==========================================================
   *
   * Each MAXSwerveModule represents ONE wheel:
   *   - A drive motor (forward/backward)
   *   - A turning motor (wheel angle)
   *   - Encoders to measure position and speed
   *
   * These objects come from REV’s MAXSwerve example code.
   * We treat them as a black box.
   */

  private final MAXSwerveModule m_frontLeft =
      new MAXSwerveModule(
          DriveConstants.kFrontLeftDrivingCanId,
          DriveConstants.kFrontLeftTurningCanId,
          DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight =
      new MAXSwerveModule(
          DriveConstants.kFrontRightDrivingCanId,
          DriveConstants.kFrontRightTurningCanId,
          DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft =
      new MAXSwerveModule(
          DriveConstants.kRearLeftDrivingCanId,
          DriveConstants.kRearLeftTurningCanId,
          DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight =
      new MAXSwerveModule(
          DriveConstants.kRearRightDrivingCanId,
          DriveConstants.kRearRightTurningCanId,
          DriveConstants.kBackRightChassisAngularOffset);

  /*
   * Gyro sensor:
   * - Measures robot rotation
   * - Used for field-relative driving
   * - Used for odometry
   */
  private final Pigeon2 m_pigeon =
      new Pigeon2(
          DriveConstants.kGyroCanId,
          DriveConstants.kCanBus);

  /* ==========================================================
   *                  DASHBOARD VISUALIZATION
   * ==========================================================
   *
   * This Sendable publishes swerve data so tools like
   * Elastic / SmartDashboard can draw the robot.
   *
   * This is debugging / visualization only.
   * It does NOT affect robot behavior.
   */

  private final Sendable m_swerveSendable =
      new Sendable() {
        @Override
        public void initSendable(SendableBuilder builder) {

          builder.setSmartDashboardType("SwerveDrive");

          builder.addDoubleProperty(
              "Front Left Angle",
              () -> m_frontLeft.getPosition().angle.getRadians(),
              null);

          builder.addDoubleProperty(
              "Front Left Velocity",
              () -> m_frontLeft.getState().speedMetersPerSecond,
              null);

          builder.addDoubleProperty(
              "Front Right Angle",
              () -> m_frontRight.getPosition().angle.getRadians(),
              null);

          builder.addDoubleProperty(
              "Front Right Velocity",
              () -> m_frontRight.getState().speedMetersPerSecond,
              null);

          builder.addDoubleProperty(
              "Back Left Angle",
              () -> m_rearLeft.getPosition().angle.getRadians(),
              null);

          builder.addDoubleProperty(
              "Back Left Velocity",
              () -> m_rearLeft.getState().speedMetersPerSecond,
              null);

          builder.addDoubleProperty(
              "Back Right Angle",
              () -> m_rearRight.getPosition().angle.getRadians(),
              null);

          builder.addDoubleProperty(
              "Back Right Velocity",
              () -> m_rearRight.getState().speedMetersPerSecond,
              null);

          builder.addDoubleProperty(
              "Robot Angle",
              () -> m_pigeon.getRotation2d().getRadians(),
              null);
        }
      };

  /* ==========================================================
   *                         ODOMETRY
   * ==========================================================
   *
   * Odometry estimates where the robot is on the FIELD.
   *
   * It combines:
   *   - Wheel positions
   *   - Gyro angle
   *
   * This pose is critical for autonomous paths.
   */

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          m_pigeon.getRotation2d(),
          new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_rearLeft.getPosition(),
              m_rearRight.getPosition()
          });

  /* ==========================================================
   *                       CONSTRUCTOR
   * ==========================================================
   */

  public DriveSubsystem() {

    // Report drivetrain type to WPILib diagnostics
    HAL.report(
        tResourceType.kResourceType_RobotDrive,
        tInstances.kRobotDriveSwerve_MaxSwerve);

    /*
     * ---------------- PathPlanner Setup ----------------
     *
     * This block tells PathPlanner:
     *   - How to get the robot's current pose
     *   - How to reset odometry at the start of auto
     *   - How to drive the robot using ChassisSpeeds
     *
     * This is REQUIRED for PathPlanner autos to work.
     *
     * NOTE:
     *  - Speeds supplied to PathPlanner MUST be ROBOT RELATIVE
     *  - Field-relative math is handled internally by PathPlanner
     */

    try {
      RobotConfig config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
          this::getPose,                    // Where am I?
          this::resetOdometry,              // Reset pose at auto start
          this::getRobotRelativeSpeeds,     // Current robot-relative speed
          (speeds, feedforwards) ->         // How to drive using those speeds
              driveRobotRelative(speeds),

          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0), // Translation PID
              new PIDConstants(5.0, 0.0, 0.0)  // Rotation PID
          ),

          config,

          // Should the path be mirrored for red alliance?
          () -> {
            var alliance = DriverStation.getAlliance();
            return alliance.isPresent()
                && alliance.get() == DriverStation.Alliance.Red;
          },

          this
      );

    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  /* ==========================================================
   *                        PERIODIC
   * ==========================================================
   *
   * This runs every 20ms.
   */

  @Override
  public void periodic() {

    // Update robot pose estimate
    m_odometry.update(
        m_pigeon.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    // Debug info
    SmartDashboard.putNumber("Gyro Rate", getTurnRate());
    SmartDashboard.putData("Pigeon Gyro", m_pigeon);
    SmartDashboard.putData("Swerve Drive", m_swerveSendable);
  }

  /* ==========================================================
   *                    POSE / ODOMETRY API
   * ==========================================================
   */

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_pigeon.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /* ==========================================================
   *                     TELEOP DRIVING
   * ==========================================================
   *
   * This method is usually called by a default command
   * using joystick inputs.
   *
   * xSpeed, ySpeed, and rot are expected to be in the range [-1, 1].
   */

  public void drive(
      double xSpeed,
      double ySpeed,
      double rot,
      boolean fieldRelative) {

    double xSpeedDelivered =
        xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered =
        ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered =
        rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotDelivered,
                    m_pigeon.getRotation2d())
                : new ChassisSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotDelivered));

    setModuleStates(swerveModuleStates);
  }

  /* ==========================================================
   *                  AUTONOMOUS DRIVING
   * ==========================================================
   *
   * PathPlanner calls this path automatically using lambdas
   * provided in the constructor.
   *
   * Students do NOT need to call this directly.
   */

  private void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds, false);
  }

  private void drive(ChassisSpeeds speeds, boolean fieldRelative) {

    if (fieldRelative) {
      speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              speeds,
              getPose().getRotation());
    }

    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates,
        DriveConstants.kMaxSpeedMetersPerSecond);

    setModuleStates(swerveModuleStates);
  }

  /* ==========================================================
   *                     MODULE HELPERS
   * ==========================================================
   */

  private SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    };
  }

  private ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        getModuleStates());
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {

    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates,
        DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /* ==========================================================
   *                       UTILITIES
   * ==========================================================
   */

  public void setX() {
    m_frontLeft.setDesiredState(
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearLeft.resetEncoders();
    m_rearRight.resetEncoders();
  }

  public void zeroHeading() {
    m_pigeon.reset();
  }

  public double getHeading() {
    return m_pigeon.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return m_pigeon.getAngularVelocityZWorld().getValueAsDouble()
        * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}