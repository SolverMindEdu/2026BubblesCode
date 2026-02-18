package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSlapdown extends SubsystemBase {
  private final TalonFX motor = new TalonFX(13, "rio");

  private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);

  // Tune these on-robot
  private static final double kP = 20.0;
  private static final double kI = 0.0;
  private static final double kD = 0.3;

  // "Very slow" Motion Magic profile (rotations/sec, rotations/sec^2)
  // Start conservative, then speed up if needed.
  private static final double CRUISE_VEL_RPS = 30.0;
  private static final double ACCEL_RPS2 = 50.0;

  // Finish tolerance
  private static final double TOLERANCE_ROT = 0.05;

  public IntakeSlapdown() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;

    cfg.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VEL_RPS;
    cfg.MotionMagic.MotionMagicAcceleration = ACCEL_RPS2;
    cfg.MotionMagic.MotionMagicJerk = 0.0;

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // If direction is backwards, flip this (or swap to CounterClockwise_Positive)
    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motor.getConfigurator().apply(cfg);
  }

  public void goToRotations(double rotations) {
    motor.setControl(mmRequest.withPosition(rotations));
  }

  public double getPositionRotations() {
    return motor.getPosition().getValueAsDouble();
  }

  public boolean atSetpoint(double targetRotations) {
    return Math.abs(getPositionRotations() - targetRotations) <= TOLERANCE_ROT;
  }

  public void stop() {
    motor.set(0.0);
  }
}
