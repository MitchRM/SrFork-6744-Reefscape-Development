package frc.robot.subsystems;

/*
 * ========================= ElevatorSubsystem =========================
 *
 * This subsystem controls the robot's elevator mechanism.
 *
 * It owns:
 *   - Two Spark MAX motor controllers
 *   - Encoders on those motors
 *   - Closed-loop (MAXMotion) position control
 *
 * Other code NEVER directly controls elevator motors.
 * Commands request actions from this subsystem instead.
 *
 * The elevator is controlled using a TARGET POSITION model:
 *   - Commands ask for a height (L1, L2, L3, Load, etc.)
 *   - The subsystem handles moving motors to that position
 *
 * This makes robot behavior predictable and repeatable.
 */

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Configs;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  /* ==========================================================
   *                         MOTORS
   * ==========================================================
   *
   * This elevator uses two motors:
   *
   *  - shepherd: primary motor (leader)
   *  - sheep:    secondary motor (follower)
   *
   * The follower relationship is configured in Configs,
   * not in this file.
   *
   * Only the shepherd motor receives motion commands.
   */

  private final SparkMax m_shepherd;
  private final SparkMax m_sheep;

  /* ==========================================================
   *                   CLOSED-LOOP CONTROLLERS
   * ==========================================================
   *
   * Spark MAX provides a closed-loop controller object
   * that handles PID + motion profiling internally.
   *
   * We only command the shepherd controller.
   */

  private final SparkClosedLoopController p_shepherd;
  
  /* ==========================================================
   *                         ENCODERS
   * ==========================================================
   *
   * Each motor has a relative encoder.
   * These measure position since power-on.
   *
   * There is also an absolute encoder ("calibrator").
   * Absolute encoders know their position even after power loss.
   *
   * Currently the absolute encoder is used only for debugging.
   * In future seasons it could be used for automatic zeroing.
   */

  private final RelativeEncoder e_shepherd;
  private final RelativeEncoder e_sheep;
  private final SparkAbsoluteEncoder e_cal;

  /* ==========================================================
   *                         CONFIGS
   * ==========================================================
   *
   * Motor configuration (PID values, current limits,
   * inversion, followers, etc.) lives in Configs.java.
   *
   * This keeps tuning separate from logic.
   */

  private final SparkMaxConfig c_shepherd;
  private final SparkMaxConfig c_sheep;

  /* ==========================================================
   *                           STATE
   * ==========================================================
   *
   * m_setpoint represents the desired elevator position.
   * It is compared against encoder values to determine
   * when the elevator has reached its goal.
   */

  private double m_setpoint;

  /* ==========================================================
   *                        CONSTRUCTOR
   * ==========================================================
   */

  public ElevatorSubsystem() {

    m_shepherd =
        new SparkMax(
            ElevatorConstants.kShepherdCanId,
            SparkMax.MotorType.kBrushless);

    m_sheep =
        new SparkMax(
            ElevatorConstants.kSheepCanId,
            SparkMax.MotorType.kBrushless);

    c_shepherd = Configs.ElevatorSubsystem.shepherdConfig;
    c_sheep = Configs.ElevatorSubsystem.sheepConfig;

    // Apply configuration to both motors
    m_shepherd.configure(
        c_shepherd,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_sheep.configure(
        c_sheep,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Get closed-loop controllers
    p_shepherd = m_shepherd.getClosedLoopController();
    
    // Get encoders
    e_shepherd = m_shepherd.getEncoder();
    e_sheep = m_sheep.getEncoder();
    e_cal = m_shepherd.getAbsoluteEncoder();

    // Zero relative encoders on startup
    e_shepherd.setPosition(0);
    e_sheep.setPosition(0);

    m_setpoint = ElevatorConstants.kStartingPosition;
  }

  /* ==========================================================
   *                   POSITION CONTROL API
   * ==========================================================
   */

  /*
   * Set a new target position for the elevator.
   * MAXMotion will automatically move the motor to this point.
   */
  public void setTargetPosition(double setpoint) {
    m_setpoint = setpoint;
    moveToSetpoint();
  }

  /*
   * Returns true when the elevator is close enough to
   * its target position.
   */
  public boolean atTargetPosition() {
    return Math.abs(avgEncoderPos() - m_setpoint)
        < ElevatorConstants.kPositionTolerance;
  }

  /*
   * Average the two encoder readings.
   * This gives a more stable estimate of elevator height.
   */
  public double avgEncoderPos() {
    return (e_shepherd.getPosition() + e_sheep.getPosition()) / 2.0;
  }

  /*
   * Internal helper that sends the target position
   * to the Spark MAX motion controller.
   */
  private void moveToSetpoint() {
    p_shepherd.setReference(
        m_setpoint,
        ControlType.kMAXMotionPositionControl);
  }

  /* ==========================================================
   *                   MANUAL OVERRIDE CONTROL
   * ==========================================================
   *
   * These methods bypass closed-loop control.
   * They are used for calibration or emergency movement.
   */

  public void stickControl(double stick) {
    m_shepherd.set(stick);
  }

  /* ==========================================================
   *                SUBSYSTEM-PROVIDED COMMANDS
   * ==========================================================
   *
   * Subsystems can return small commands for actions
   * tightly coupled to their hardware.
   */

  public Command resetElevator() {
    return run(() -> e_shepherd.setPosition(0));
  }

  public Command slowBottom() {
    return startEnd(
        () -> m_shepherd.set(-0.1),
        () -> m_shepherd.set(0));
  }

  /* ==========================================================
   *                       UTILITY MODES
   * ==========================================================
   */

  public void setArmCoastMode() {

    SparkMaxConfig c_mod = new SparkMaxConfig();
    c_mod.idleMode(IdleMode.kCoast);

    m_shepherd.configure(
        c_mod,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

    m_sheep.configure(
        c_mod,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setArmBrakeMode() {

    SparkMaxConfig c_mod = new SparkMaxConfig();
    c_mod.idleMode(IdleMode.kBrake);

    m_shepherd.configure(
        c_mod,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

    m_sheep.configure(
        c_mod,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  /* ==========================================================
   *                         PERIODIC
   * ==========================================================
   *
   * Runs every 20ms.
   * Used here for dashboard telemetry.
   */

  @Override
  public void periodic() {

    SmartDashboard.putNumber(
        "Calibrator Position",
        e_cal.getPosition());

    SmartDashboard.putNumber(
        "Calibrator Velocity",
        e_cal.getVelocity());

    SmartDashboard.putNumber(
        "Sheep Position",
        e_sheep.getPosition());

    SmartDashboard.putNumber(
        "Sheep Velocity",
        e_sheep.getVelocity());

    SmartDashboard.putNumber(
        "Shepherd Position",
        e_shepherd.getPosition());

    SmartDashboard.putNumber(
        "Shepherd Velocity",
        e_shepherd.getVelocity());

    SmartDashboard.putNumber(
        "Setpoint",
        m_setpoint);

    SmartDashboard.putBoolean(
        "At Target",
        atTargetPosition());
  }
}