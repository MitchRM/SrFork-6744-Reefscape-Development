package frc.robot;

/*
 * ============================ Constants ==============================
 *
 * Robot-wide constants. This class should not be used for any other purpose. All constants
 * should be declared globally (i.e. public static). Do not put anything functional in this class.
 */

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;


public final class Constants {

  /* ==============================================================================
   *                       DRIVE SUBSYSTEM CONSTANTS
   * ==============================================================================
   */
  public static final class DriveConstants {
    
    // Maximum driving speed commands - These are the maximum speeds that can be requested by 
    // the driver or autonomous, they are not the maximum speed cababiity of the robot.
    public static final double kMaxSpeedMetersPerSecond = 3; // originally 4.8    TUNING
    public static final double kMaxAngularSpeed = 1.5*Math.PI ; // radians per second    originally 2*Pi   TUNING

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 17;

    public static final int kFrontLeftTurningCanId = 10;
    public static final int kRearLeftTurningCanId = 12;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearRightTurningCanId = 16;

    public static final int kGyroCanId = 9;
    public static final String kCanBus = "rio";

    public static final boolean kGyroReversed = false;
  }

  public static final class ElevatorConstants {
    /** The Holy Ratio is the ratio that converts the internal rotation count of
     * the elevator motor to the position of the bottom of the elevator in inches
     * @apiNote The holy ratio is {@value}
     * @apiNote NEVER EVER CHANGE THIS CONSTANT EVER OR I'M GONNA MURDER YOU WITH HAMMERS 
     */
    public static final double kHolyRatio = 24.625/66.25;

    public static final int kShepherdCanId = 7;
    public static final int kSheepCanId = 8;

    public static final int kStartingPosition = 0;
    public static final int kCurrentLimit = 50;

    public static final double kZeroOffest = 0.420;

    public static final double kPositionConversionFactor = kHolyRatio;
    public static final double kVelocityConversionFactor = kHolyRatio;


    /** Sets the Idle mode of the motors.
     * @apiNote This should remain as {@link IdleMode#kBrake}
     * unless you want to manually rotate the motors
     */
    public static final IdleMode kIdleMode = IdleMode.kBrake;

    // Deprecated, but still nice:
    // In Desmos,
    // y=0.37037x+19.5\left\{0<\ x<27\right\}
    // y=0.6842x+11.0265\left\{27<x<65\right\}
    // is the approximate position curve for the elevator
    // where x is in Degrees of PCF1 and y is in Inches
    // Measured from the top of the shooter

    public static final double kMaxVelocity = 3072;
    public static final double kMaxAcceleration = 1536;

    public static final double kP = 0.50000000;
    public static final double kI = 0.00000000;
    public static final double kD = 0.00000000;
    
    /** The soft limit for the elevator going forward.
     * @apiNote This soft limit should NEVER go above 24
     */
    public static final double kFwdSoftLimit = 23;
    /** The soft limit for the elevator going backward.
     * @apiNote This soft limit should NEVER go below 1
     */
    public static final double kRevSoftLimit = -23;

    /** The allowed tolerance for the elevator
     * @apiNote This value is in inches
     * @apiNote This really shouldn't ever go above an inch.
     */
    public static final double kPositionTolerance = 0.75;

    public static final double kStageLoad = 7.86;
    public static final double kStageL1 = 6.61;
    public static final double kStageL2 = 13.65;
    public static final double kStageL3 = 22;
    public static final double kStageAlgae = 6.61;
  }

  public static final class ModuleConstants {
    /** The MAXSwerve module can be configured with one of three pinion gears: 12T,
    * 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    * more teeth will result in a robot that drives faster).
    */
    public static final int kDrivingMotorPinionTeeth = 12;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kDriverController2Port = 1;
    public static final double kDriveDeadband = 0.05;   //  TUNING
    // An additional driver control TUNING option to try 
    // would be to square controller inputs that vary from 0 t0 1
  }

  public static final class ColorSensorConstants {
    public static final I2C.Port kSensorPort = I2C.Port.kOnboard;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1;  // originally 3  TUNING
    public static final double kMaxAccelerationMetersPerSecondSquared = 1; // originally 3  TUNING
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI/2; // originally Pi  TUNING
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI; // originally Pi  TUNING

    public static final double kPXController = 1;  //  TUNING
    public static final double kPYController = 1;  //  TUNING
    public static final double kPThetaController = 1; //   TUNING

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  // Coral shooter command constants
  public static final class ShooterConstants {
    public static final int kShooterCanId = 6; 
    public static final double k_shooterSpeed = 0.6; // percent
    public static final double k_slowShooter = 0.1; // percent
    //  public static final double k_shootDistance = 4.0; //inches (for later)
    public static final double k_shooterintakeSpeed = 0.3;
  }
}
