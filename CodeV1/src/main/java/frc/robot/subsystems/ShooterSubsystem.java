package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX shooterLeft = new TalonFX(Constants.CAN.SHOOTER_LEFT, "rio");
    private final TalonFX shooterMiddle = new TalonFX(Constants.CAN.SHOOTER_MIDDLE, "rio");
    private final TalonFX shooterRight = new TalonFX(Constants.CAN.SHOOTER_RIGHT, "rio");

    private final DutyCycleOut percentRequest = new DutyCycleOut(0);

    public ShooterSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast; // shooters usually coast

        // Configure each motor separately (same cfg object is fine)
        // Change these 3 lines to match your wiring/direction needs
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterLeft.getConfigurator().apply(cfg);

        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shooterMiddle.getConfigurator().apply(cfg);

        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shooterRight.getConfigurator().apply(cfg);
    }

    // run all shooter motors at the same percent output
    public void run(double percent) {
        shooterLeft.setControl(percentRequest.withOutput(percent));
        shooterMiddle.setControl(percentRequest.withOutput(percent));
        shooterRight.setControl(percentRequest.withOutput(percent));
    }

    public void stop() {
        run(0.0);
    }
}