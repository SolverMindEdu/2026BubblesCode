package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSubsystem extends SubsystemBase {

    private final TalonFX roller = new TalonFX(11, "rio");

    // Percent output control
    private final DutyCycleOut percentRequest = new DutyCycleOut(0);

    // Velocity control (rotations per second)
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    public RollerSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast; // rollers should coast

        roller.getConfigurator().apply(cfg);
    }

    /* ========================
       Percent Output Mode
       ======================== */

    public void runPercent(double percent) {
        roller.setControl(percentRequest.withOutput(percent));
    }

    /* ========================
       Velocity Mode (RPS)
       ======================== */

    public void runVelocity(double rps) {
        roller.setControl(velocityRequest.withVelocity(rps));
    }

    public void stop() {
        roller.setControl(percentRequest.withOutput(0));
    }
}
