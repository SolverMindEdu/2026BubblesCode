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

public class Climber extends SubsystemBase {
  private final TalonFX motor = new TalonFX(Constants.CAN.CLIMBER_MOTOR, "rio");
  private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);

  // LIMITS!!!!
  public static final double UP_LIMIT_ROT = 120.0;     // highest it can go
  public static final double DOWN_LIMIT_ROT = 0.0;  // lowest it can go

  // Setpoints
  public static final double UP_ROT = 120.0;
  public static final double DOWN_ROT = 10.0;
  public static final double HOME = 0.0;


  // tune for PID
  private static final double kP = 11.0;
  private static final double kI = 0.0;
  private static final double kD = 0.1;

  // tune speed for motion magic
  private static final double CRUISE_VEL_RPS = 90.0;
  private static final double ACCEL_RPS2 = 70.0;

  // tolerance
  private static final double TOLERANCE_ROT = 0.05;

  public Climber() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;

    cfg.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VEL_RPS;
    cfg.MotionMagic.MotionMagicAcceleration = ACCEL_RPS2;
    cfg.MotionMagic.MotionMagicJerk = 0.0;

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Down is positive up is negative
    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motor.getConfigurator().apply(cfg);
  }

  public void goToRotations(double requestedRotations) {
    double clamped = MathUtil.clamp(requestedRotations, DOWN_LIMIT_ROT, UP_LIMIT_ROT);
    motor.setControl(mmRequest.withPosition(clamped));
  }

  public boolean atSetpoint(double targetRotations) {
    double clamped = MathUtil.clamp(targetRotations, DOWN_LIMIT_ROT, UP_LIMIT_ROT);
    return Math.abs(getPositionRotations() - clamped) <= TOLERANCE_ROT;
  }

  public void up() {
    goToRotations(UP_ROT);
  }

  public void down() {
    goToRotations(DOWN_ROT);
  }

  public void home() {
    goToRotations(HOME);
  }
  public double getPositionRotations() {
    return motor.getPosition().getValueAsDouble();
  }

  public boolean isUp() {
    return atSetpoint(UP_ROT);
  }

  public boolean isDown() {
    return atSetpoint(DOWN_ROT);
  }

  public void stop() {
    motor.set(0.0);
  }
}