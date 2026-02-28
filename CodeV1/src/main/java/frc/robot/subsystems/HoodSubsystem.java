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

  // LIMITS
  public static final double MIN_ROT = 0.0;
  public static final double MAX_ROT = 3.0;

  // Step size
  private static final double STEP = 0.5;

  // tune for PID
  private static final double kP = 9.5;
  private static final double kI = 0.0;
  private static final double kD = 0.1;

  // Motion Magic
  private static final double CRUISE_VEL_RPS = 10.0;
  private static final double ACCEL_RPS2 = 10.0;

  private double targetRot = 0.0;

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

    targetRot = getPositionRotations(); // initialize safely
  }

  private void setTarget(double rotations) {
    targetRot = MathUtil.clamp(rotations, MIN_ROT, MAX_ROT);
    motor.setControl(mmRequest.withPosition(targetRot));
  }

  public void incrementUp() {
    setTarget(targetRot + STEP);
  }

  public void incrementDown() {
    setTarget(targetRot - STEP);
  }

  public double getPositionRotations() {
    return motor.getPosition().getValueAsDouble();
  }

  public double getTargetRotations() {
  return targetRot;
  }

  public void stop() {
    motor.set(0.0);
  }
}