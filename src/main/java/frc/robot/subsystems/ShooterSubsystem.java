package frc.robot.subsystems;

/*
 * ========================= ShooterSubsystem =========================
 *
 * This subsystem controls the shooter / intake mechanism.
 *
 * It owns:
 *   - One Spark MAX motor (shooter/intake wheels)
 *   - An encoder on that motor (currently unused)
 *   - A color sensor used to detect game pieces
 *
 * The shooter currently uses OPEN-LOOP control:
 *   - We directly set motor speed
 *   - There is no PID or velocity control yet
 *
 * Commands request actions like:
 *   - Intake
 *   - Shoot
 *   - Reverse
 *   - Stop
 *
 * The subsystem handles how those actions affect hardware.
 */

import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ColorSensorConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  /* ==========================================================
   *                         HARDWARE
   * ==========================================================
   */

  /*
   * Single motor that both intakes and shoots game pieces.
   *
   * Positive vs negative direction determines:
   *   - Intake
   *   - Reverse
   *   - Shooting
   */
  private final SparkMax m_shooterMotor =
      new SparkMax(
          ShooterConstants.kShooterCanId,
          MotorType.kBrushless);

  /*
   * Encoder on the shooter motor.
   *
   * Currently not used for control.
   * Kept for future upgrades such as:
   *   - Velocity-based shooting
   *   - Distance-based feeding
   */
  private final RelativeEncoder m_shootEncoder =
      m_shooterMotor.getEncoder();

  /*
   * Color sensor used to detect game pieces.
   *
   * This allows the robot to know when it has collected coral.
   */
  private final ColorSensorV3 m_colorSensor =
      new ColorSensorV3(ColorSensorConstants.kSensorPort);

  /* ==========================================================
   *                       CONSTRUCTOR
   * ==========================================================
   */

  public ShooterSubsystem() {

    SparkMaxConfig shootMotorConfig = new SparkMaxConfig();

    // Limit current and brake when stopped
    shootMotorConfig
        .smartCurrentLimit(30)
        .idleMode(IdleMode.kBrake);

    // Apply configuration to motor controller
    m_shooterMotor.configure(
        shootMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Zero encoder at startup
    m_shootEncoder.setPosition(0);
  }

  /* ==========================================================
   *                          SENSORS
   * ==========================================================
   */

  /*
   * Returns true if the color sensor detects "white".
   *
   * The threshold values were determined experimentally.
   * These magic numbers could be moved to Constants later.
   */
  public boolean isWhite() {

    int blue = m_colorSensor.getBlue();
    int red = m_colorSensor.getRed();
    int green = m_colorSensor.getGreen();

    return (red > 7800 && green > 14600 && blue > 7800);
  }

  /* ==========================================================
   *                SUBSYSTEM-PROVIDED COMMANDS
   * ==========================================================
   *
   * These commands directly control the shooter motor.
   *
   * They use startEnd(), meaning:
   *   - Motor starts when command begins
   *   - Motor stops automatically when command ends
   */

  public Command stopMotor() {
    return run(() -> m_shooterMotor.set(0));
  }

  /*
   * Shoot game piece out of robot.
   */
  public Command releaseCommand() {
    return startEnd(
        () -> m_shooterMotor.set(-ShooterConstants.k_shooterSpeed),
        () -> m_shooterMotor.set(0));
  }

  /*
   * Intake game piece into robot.
   *
   * "ol" stands for open-loop.
   */
  public Command olIntakeCommand() {
    return startEnd(
        () -> m_shooterMotor.set(-ShooterConstants.k_shooterintakeSpeed),
        () -> m_shooterMotor.set(0));
  }

  /*
   * Reverse intake to eject or unjam.
   */
  public Command reverseIntakeCommand() {
    return startEnd(
        () -> m_shooterMotor.set(ShooterConstants.k_shooterintakeSpeed),
        () -> m_shooterMotor.set(0));
  }

  /* ==========================================================
   *                         PERIODIC
   * ==========================================================
   *
   * Currently unused.
   * Could be used later for dashboard telemetry.
   */

  @Override
  public void periodic() {
  }
}