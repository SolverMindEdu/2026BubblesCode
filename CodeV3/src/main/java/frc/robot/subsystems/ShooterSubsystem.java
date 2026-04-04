package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX shooterLeft = new TalonFX(Constants.CAN.SHOOTER_LEFT, "rio");
  private final TalonFX shooterMiddle = new TalonFX(Constants.CAN.SHOOTER_MIDDLE, "rio");
  private final TalonFX shooterRight = new TalonFX(Constants.CAN.SHOOTER_RIGHT, "rio");

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  private double targetRps = 0.0;
  private static final double SPEED_TOLERANCE_RPS = 3.0;

  private final Debouncer atSpeedDebounce =
      new Debouncer(0.10, Debouncer.DebounceType.kRising);

  private static final double kP = 0.25;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kV = 0.12;

  public ShooterSubsystem() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.kV = kV;

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterLeft.getConfigurator().apply(cfg);

    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    shooterMiddle.getConfigurator().apply(cfg);

    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    shooterRight.getConfigurator().apply(cfg);
  }

  public void setTargetRps(double rps) {
    targetRps = Math.max(0.0, rps);

    shooterLeft.setControl(velocityRequest.withVelocity(targetRps));
    shooterMiddle.setControl(velocityRequest.withVelocity(targetRps));
    shooterRight.setControl(velocityRequest.withVelocity(targetRps));
  }

  public void stop() {
    shooterLeft.stopMotor();
    shooterMiddle.stopMotor();
    shooterRight.stopMotor();
    targetRps = 0.0;
  }

  public double getLeftRps() {
    return shooterLeft.getVelocity().getValueAsDouble();
  }

  public double getMiddleRps() {
    return shooterMiddle.getVelocity().getValueAsDouble();
  }

  public double getRightRps() {
    return shooterRight.getVelocity().getValueAsDouble();
  }

  public double getAvgRps() {
    return (getLeftRps() + getMiddleRps() + getRightRps()) / 3.0;
  }

  public boolean isAtTargetSpeed() {
    if (targetRps <= 0.0) return false;

    boolean within =
        Math.abs(getLeftRps() - targetRps) <= SPEED_TOLERANCE_RPS &&
        Math.abs(getMiddleRps() - targetRps) <= SPEED_TOLERANCE_RPS &&
        Math.abs(getRightRps() - targetRps) <= SPEED_TOLERANCE_RPS;

    return atSpeedDebounce.calculate(within);
  }
}