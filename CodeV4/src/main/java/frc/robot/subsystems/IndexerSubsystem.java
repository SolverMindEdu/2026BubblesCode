package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {

    private final TalonFX indexermotor = new TalonFX(Constants.Indexer.MOTOR_ID, "rio");

    private final DutyCycleOut percentRequest = new DutyCycleOut(0);

    public IndexerSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        indexermotor.getConfigurator().apply(cfg);
    }
    
    public void run(double percent) {
        indexermotor.setControl(percentRequest.withOutput(percent));
    }

    public void stop() {
        run(0.0);
    }

    public boolean isConnected() {
        return indexermotor.isConnected();
    }
}