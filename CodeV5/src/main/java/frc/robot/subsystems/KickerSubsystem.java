package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class KickerSubsystem extends SubsystemBase {

    private final TalonFX leftKickerMotor =
        new TalonFX(Constants.Kickers.LEFT_ID, "rio");
    private final TalonFX rightKickerMotor =
        new TalonFX(Constants.Kickers.RIGHT_ID, "rio");

    private final DutyCycleOut percentRequest = new DutyCycleOut(0);

    public KickerSubsystem() {
        TalonFXConfiguration leftCfg = new TalonFXConfiguration();
        leftCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftKickerMotor.getConfigurator().apply(leftCfg);

        TalonFXConfiguration rightCfg = new TalonFXConfiguration();
        rightCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightKickerMotor.getConfigurator().apply(rightCfg);
    }

    public void run(double percent) {
        leftKickerMotor.setControl(percentRequest.withOutput(percent));
        rightKickerMotor.setControl(percentRequest.withOutput(percent));
    }

    public void reverse(double percent) {
        run(-Math.abs(percent));
    }

    public void stop() {
        run(0.0);
    }

    public boolean isConnected() {
    return leftKickerMotor.isConnected() &&
           rightKickerMotor.isConnected();
    }
}