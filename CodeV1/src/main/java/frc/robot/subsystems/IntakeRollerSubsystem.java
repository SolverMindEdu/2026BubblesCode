package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeRollerSubsystem extends SubsystemBase {

    private final TalonFX roller = new TalonFX(Constants.CAN.INTAKE_ROLLERS, "rio");
    private final DutyCycleOut percentRequest = new DutyCycleOut(0);
    private double requestedPercent = 0.0;
    private boolean enabled = false;

    // Anti-Jam system
    // Jam = high current + low speed for a short time
    private static final double JAM_STATOR_CURRENT_A = 55.0;  // tune
    private static final double JAM_VELOCITY_RPS = 5.0;       // tune
    private static final double JAM_DEBOUNCE_SEC = 0.10;      // must be jammed for this long

    // Clear jam
    private static final double UNJAM_REVERSE_PERCENT = 0.55; // tune
    private static final double UNJAM_MIN_SEC = 0.1;         // min
    private static final double UNJAM_MAX_SEC = 0.4;         // max

    // Cooldown
    private static final double COOLDOWN_SEC = 0.10;

    private enum Mode { IDLE, RUNNING, UNJAMMING, COOLDOWN }
    private Mode mode = Mode.IDLE;

    private final Timer jamTimer = new Timer();
    private final Timer unjamTimer = new Timer();
    private final Timer cooldownTimer = new Timer();

    public IntakeRollerSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // flip
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        roller.getConfigurator().apply(cfg);

        jamTimer.stop();
        unjamTimer.stop();
        cooldownTimer.stop();
    }

    // start roller and activate anti jam
    public void runIntake(double percent) {
        requestedPercent = percent;
        enabled = true;

        if (mode == Mode.IDLE) {
            mode = Mode.RUNNING;
        }
    }

    // stop roller and disable anti jam
    public void stop() {
        enabled = false;
        requestedPercent = 0.0;
        mode = Mode.IDLE;

        jamTimer.stop(); jamTimer.reset();
        unjamTimer.stop(); unjamTimer.reset();
        cooldownTimer.stop(); cooldownTimer.reset();

        setPercent(0.0);
    }

    // no antijam
    public void runPercentRaw(double percent) {
        enabled = false;
        requestedPercent = percent;
        mode = Mode.IDLE;
        setPercent(percent);
    }

    @Override
    public void periodic() {
        // If not enabled do nothing
        if (!enabled) return;

        double statorCurrent = roller.getStatorCurrent().getValueAsDouble(); // amps
        double velocityRps = Math.abs(roller.getVelocity().getValueAsDouble()); // motor rps

        boolean tryingToMove = Math.abs(requestedPercent) > 0.05;

        // only detect jams while intake not when stopped
        boolean jamCondition = tryingToMove
                && (statorCurrent >= JAM_STATOR_CURRENT_A)
                && (velocityRps <= JAM_VELOCITY_RPS);

        switch (mode) {
            case RUNNING -> {
                setPercent(requestedPercent);
                if (jamCondition) {
                    if (!jamTimer.isRunning()) {
                        jamTimer.reset();
                        jamTimer.start();
                    }
                    if (jamTimer.hasElapsed(JAM_DEBOUNCE_SEC)) {
                        jamTimer.stop();
                        jamTimer.reset();

                        mode = Mode.UNJAMMING;
                        unjamTimer.reset();
                        unjamTimer.start();

                        setPercent(-Math.signum(requestedPercent) * UNJAM_REVERSE_PERCENT);
                    }
                } else {
                    if (jamTimer.isRunning()) {
                        jamTimer.stop();
                        jamTimer.reset();
                    }
                }
            }

            case UNJAMMING -> {
                setPercent(-Math.signum(requestedPercent) * UNJAM_REVERSE_PERCENT);

                boolean minDone = unjamTimer.hasElapsed(UNJAM_MIN_SEC);
                boolean maxDone = unjamTimer.hasElapsed(UNJAM_MAX_SEC);

                boolean jamGone = !jamCondition;

                if ((minDone && jamGone) || maxDone) {
                    unjamTimer.stop();
                    unjamTimer.reset();

                    mode = Mode.COOLDOWN;
                    cooldownTimer.reset();
                    cooldownTimer.start();
                    setPercent(requestedPercent);
                }
            }

            case COOLDOWN -> {
                setPercent(requestedPercent);

                if (cooldownTimer.hasElapsed(COOLDOWN_SEC)) {
                    cooldownTimer.stop();
                    cooldownTimer.reset();
                    mode = Mode.RUNNING;
                }
            }

            case IDLE -> {
                setPercent(0.0);
            }
        }
    }

    private void setPercent(double percent) {
        roller.setControl(percentRequest.withOutput(percent));
    }
}
