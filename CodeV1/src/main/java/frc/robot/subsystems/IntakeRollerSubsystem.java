package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeRollerSubsystem extends SubsystemBase {

    private final TalonFX roller = new TalonFX(Constants.CAN.INTAKE_ROLLERS, "rio");
    private final DutyCycleOut percentRequest = new DutyCycleOut(0);

    public IntakeRollerSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // flip
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        roller.getConfigurator().apply(cfg);
    }

    // run rollers (percent output)
    public void runIntake(double percent) {
        roller.setControl(percentRequest.withOutput(percent));
    }

    public void stop() {
        roller.setControl(percentRequest.withOutput(0.0));
    }
}
