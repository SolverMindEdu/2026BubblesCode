package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
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

    private final DutyCycleOut percentRequest = new DutyCycleOut(0);
    private double targetRps = 0.0;
    private static final double SPEED_TOLERANCE_RPS = 3.0;

    //speed for this long
    private final Debouncer atSpeedDebounce = new Debouncer(0.10, Debouncer.DebounceType.kRising);

    public ShooterSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterLeft.getConfigurator().apply(cfg);
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shooterMiddle.getConfigurator().apply(cfg);
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shooterRight.getConfigurator().apply(cfg);
    }

    public void run(double percent) {
        shooterLeft.setControl(percentRequest.withOutput(percent));
        shooterMiddle.setControl(percentRequest.withOutput(percent));
        shooterRight.setControl(percentRequest.withOutput(percent));
    }

    public void stop() {
        run(0.0);
        targetRps = 0.0;
    }

    public void setTargetRps(double rps) {
        targetRps = Math.max(0.0, rps);
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

        double avg = getAvgRps();
        boolean within = Math.abs(avg - targetRps) <= SPEED_TOLERANCE_RPS;
        return atSpeedDebounce.calculate(within);
    }
}