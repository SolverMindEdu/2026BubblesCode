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

public class IntakeSlapdownSubsystem extends SubsystemBase {

  private final TalonFX motor =
      new TalonFX(Constants.Intake.SLAPDOWN_ID, "rio");

  private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);

  private double appliedCruiseVel = Double.NaN;
  private double appliedAccel = Double.NaN;

  public IntakeSlapdownSubsystem() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = Constants.Intake.kP;
    slot0.kI = Constants.Intake.kI;
    slot0.kD = Constants.Intake.kD;

    cfg.MotionMagic.MotionMagicCruiseVelocity = Constants.Intake.CRUISE_VEL_RPS;
    cfg.MotionMagic.MotionMagicAcceleration = Constants.Intake.ACCEL_RPS2;
    cfg.MotionMagic.MotionMagicJerk = 0.0;

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motor.getConfigurator().apply(cfg);

    appliedCruiseVel = Constants.Intake.CRUISE_VEL_RPS;
    appliedAccel = Constants.Intake.ACCEL_RPS2;
  }

  private void applyMotionMagicProfile(double cruiseVel, double accel) {
    if (appliedCruiseVel == cruiseVel && appliedAccel == accel) {
      return;
    }

    TalonFXConfiguration cfg = new TalonFXConfiguration();

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = Constants.Intake.kP;
    slot0.kI = Constants.Intake.kI;
    slot0.kD = Constants.Intake.kD;

    cfg.MotionMagic.MotionMagicCruiseVelocity = cruiseVel;
    cfg.MotionMagic.MotionMagicAcceleration = accel;
    cfg.MotionMagic.MotionMagicJerk = 0.0;

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motor.getConfigurator().apply(cfg);

    appliedCruiseVel = cruiseVel;
    appliedAccel = accel;
  }

  public void goToRotations(double requestedRotations) {
    applyMotionMagicProfile(
        Constants.Intake.CRUISE_VEL_RPS,
        Constants.Intake.ACCEL_RPS2
    );

    double clamped = MathUtil.clamp(
        requestedRotations,
        Constants.Intake.UP_LIMIT_ROT,
        Constants.Intake.DOWN_LIMIT_ROT
    );

    motor.setControl(mmRequest.withPosition(clamped));
  }

  public void goToRotationsSlow(double requestedRotations) {
    applyMotionMagicProfile(
        Constants.Intake.SLOW_UP_CRUISE_VEL_RPS,
        Constants.Intake.SLOW_UP_ACCEL_RPS2
    );

    double clamped = MathUtil.clamp(
        requestedRotations,
        Constants.Intake.UP_LIMIT_ROT,
        Constants.Intake.DOWN_LIMIT_ROT
    );

    motor.setControl(mmRequest.withPosition(clamped));
  }

  public void up() {
    goToRotations(Constants.Intake.UP_ROT);
  }

  public void upSlow() {
    goToRotationsSlow(Constants.Intake.UP_ROT);
  }

  public void travel() {
    goToRotations(Constants.Intake.TRAVEL_ROT);
  }

  public void shoot() {
    goToRotations(Constants.Intake.SHOOT_ROT);
  }

  public void down() {
    goToRotations(Constants.Intake.DOWN_ROT);
  }

  public double getPositionRotations() {
    return motor.getPosition().getValueAsDouble();
  }

  public boolean atSetpoint(double targetRotations) {
    double clamped = MathUtil.clamp(
        targetRotations,
        Constants.Intake.UP_LIMIT_ROT,
        Constants.Intake.DOWN_LIMIT_ROT
    );

    return Math.abs(getPositionRotations() - clamped)
        <= Constants.Intake.TOLERANCE_ROT;
  }

  public boolean isUp() {
    return atSetpoint(Constants.Intake.UP_ROT);
  }

  public boolean isTravel() {
    return atSetpoint(Constants.Intake.TRAVEL_ROT);
  }

  public boolean isShoot() {
    return atSetpoint(Constants.Intake.SHOOT_ROT);
  }

  public boolean isDown() {
    return atSetpoint(Constants.Intake.DOWN_ROT);
  }

  public void stop() {
    motor.set(0.0);
  }
}