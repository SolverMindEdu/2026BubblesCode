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

  private final TalonFX motor = new TalonFX(Constants.Hood.MOTOR_ID, "rio");
  private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);

  private static final double MOTOR_ROTATIONS_PER_DEGREE = 3.125 / 360.0;

  private static final double kP = 10.0;
  private static final double kI = 0.0;
  private static final double kD = 0.2;

  private double targetDeg = 0.0;

  public HoodSubsystem() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;

    cfg.MotionMagic.MotionMagicCruiseVelocity = Constants.Hood.CRUISE_VEL_RPS;
    cfg.MotionMagic.MotionMagicAcceleration = Constants.Hood.ACCEL_RPS2;

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motor.getConfigurator().apply(cfg);

    targetDeg = getPositionDegrees();
  }

  public void setTargetDegrees(double degrees) {
    targetDeg = MathUtil.clamp(
        degrees,
        Constants.Hood.MIN_DEG,
        Constants.Hood.MAX_DEG
    );

    double motorRotations = degreesToMotorRotations(targetDeg);
    motor.setControl(mmRequest.withPosition(motorRotations));
  }

  public void incrementUp() {
    setTargetDegrees(targetDeg + Constants.Hood.STEP_DEG);
  }

  public void incrementDown() {
    setTargetDegrees(targetDeg - Constants.Hood.STEP_DEG);
  }

  public double getPositionDegrees() {
    double motorRot = motor.getPosition().getValueAsDouble();
    return motorRotationsToDegrees(motorRot);
  }

  public double getTargetDegrees() {
    return targetDeg;
  }

  public double getTargetRotations() {
    return degreesToMotorRotations(targetDeg);
  }

  public void stop() {
    motor.set(0.0);
  }

  private static double degreesToMotorRotations(double hoodDeg) {
    return hoodDeg * MOTOR_ROTATIONS_PER_DEGREE;
  }

  private static double motorRotationsToDegrees(double motorRot) {
    return motorRot / MOTOR_ROTATIONS_PER_DEGREE;
  }

  public boolean isConnected() {
    return motor.isConnected();
  }
}