package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class KickerSubsystem extends SubsystemBase {

    private final TalonFX kickerMotor = new TalonFX(Constants.CAN.KICKER_MOTOR, "rio");

    private final DutyCycleOut percentRequest = new DutyCycleOut(0);

    public KickerSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        kickerMotor.getConfigurator().apply(cfg);
    }

    // run kicker motor
    public void run(double percent) {
        kickerMotor.setControl(percentRequest.withOutput(percent));
    }

    public void stop() {
        run(0.0);
    }
}