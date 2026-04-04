package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeRollerSubsystem extends SubsystemBase {

  private final TalonFX roller = new TalonFX(Constants.CAN.INTAKE_ROLLERS, "rio");

  private final DutyCycleOut percentRequest = new DutyCycleOut(0.0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

  private static final double kP = 0.12;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kV = 0.12;

  public IntakeRollerSubsystem() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.kV = kV;

    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();

    currentLimits.StatorCurrentLimitEnable = true;
    currentLimits.StatorCurrentLimit = 85.0; 
    currentLimits.SupplyCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit = 45.0;
    cfg.CurrentLimits = currentLimits;

    roller.getConfigurator().apply(cfg);
  }

  public void runIntakeRps(double rps) {
    roller.setControl(velocityRequest.withVelocity(rps));
  }

  public void runIntakePercent(double percent) {
    roller.setControl(percentRequest.withOutput(percent));
  }

  public void stop() {
    roller.setControl(percentRequest.withOutput(0.0));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("IntakeRoller/StatorCurrent", roller.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("IntakeRoller/SupplyCurrent", roller.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("IntakeRoller/VelocityRps", roller.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("IntakeRoller/AppliedVolts", roller.getMotorVoltage().getValueAsDouble());
  }
}