package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodSubsystem extends SubsystemBase {

  private final TalonFX motor = new TalonFX(Constants.CAN.HOOD_MOTOR, "rio");
  private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);

  // hood ratio is 28:1  =>  28 motor rotations = 1 hood rotation (360 deg)
  // motor rotations per hood degree = 28 / 360
  private static final double MOTOR_ROTATIONS_PER_DEGREE = 29.0 / 360.0;

  // LIMITS (now in degrees)
  public static final double MIN_DEG = 0.0;
  public static final double MAX_DEG = 35.0;

  // Step size in degrees
  private static final double STEP_DEG = 3.0;

  // tune for PID
  private static final double kP = 15.0;
  private static final double kI = 0.0;
  private static final double kD = 0.2;

  // Motion Magic (these are in motor rotations/sec and motor rotations/sec^2)
  private static final double CRUISE_VEL_RPS = 40.0;
  private static final double ACCEL_RPS2 = 30.0;

  private double targetDeg = 0.0;

  public HoodSubsystem() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;

    cfg.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VEL_RPS;
    cfg.MotionMagic.MotionMagicAcceleration = ACCEL_RPS2;

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motor.getConfigurator().apply(cfg);

    targetDeg = getPositionDegrees(); // initialize safely
  }

  // =============================
  // Core setter in DEGREES
  // =============================

  public void setTargetDegrees(double degrees) {
    targetDeg = MathUtil.clamp(degrees, MIN_DEG, MAX_DEG);

    double motorRotations = degreesToMotorRotations(targetDeg);
    motor.setControl(mmRequest.withPosition(motorRotations));
  }

  // =============================
  // Manual stepping
  // =============================

  public void incrementUp() {
    setTargetDegrees(targetDeg + STEP_DEG);
  }

  public void incrementDown() {
    setTargetDegrees(targetDeg - STEP_DEG);
  }

  // =============================
  // Getters
  // =============================

  public double getPositionDegrees() {
    double motorRot = motor.getPosition().getValueAsDouble();
    return motorRotationsToDegrees(motorRot);
  }

  public double getTargetDegrees() {
    return targetDeg;
  }

  // If you still need this for old code, you can delete it later
  public double getTargetRotations() {
    return degreesToMotorRotations(targetDeg);
  }

  public void stop() {
    motor.set(0.0);
  }

  // =============================
  // Conversions
  // =============================

  private static double degreesToMotorRotations(double hoodDeg) {
    return hoodDeg * MOTOR_ROTATIONS_PER_DEGREE;
  }

  private static double motorRotationsToDegrees(double motorRot) {
    return motorRot / MOTOR_ROTATIONS_PER_DEGREE;
  }
}