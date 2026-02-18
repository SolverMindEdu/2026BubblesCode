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
    private double requestedPercent = 0.0;   // what your code wants (example: -0.7)
    private boolean enabled = false;

    // Anti-Jam system
    // Jam = high current + low speed for a short time
    private static final double JAM_STATOR_CURRENT_A = 55.0;  // tune
    private static final double JAM_VELOCITY_RPS = 5.0;       // tune
    private static final double JAM_DEBOUNCE_SEC = 0.10;      // must be jammed for this long

    // Clear jam
    private static final double UNJAM_REVERSE_PERCENT = 0.55; // tune, reverse power magnitude
    private static final double UNJAM_MIN_SEC = 0.1;         // minimum reverse burst
    private static final double UNJAM_MAX_SEC = 0.4;         // max reverse burst

    // Cooldown to prevent rapid flip-flop
    private static final double COOLDOWN_SEC = 0.10;

    private enum Mode { IDLE, RUNNING, UNJAMMING, COOLDOWN }
    private Mode mode = Mode.IDLE;

    private final Timer jamTimer = new Timer();
    private final Timer unjamTimer = new Timer();
    private final Timer cooldownTimer = new Timer();

    public IntakeRollerSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Keep or flip depending on your wiring. You already set this.
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        roller.getConfigurator().apply(cfg);

        jamTimer.stop();
        unjamTimer.stop();
        cooldownTimer.stop();
    }

    // =============================
    // Public API
    // =============================

    /** Start rollers with anti-jam enabled. Example: runIntake(-0.7). */
    public void runIntake(double percent) {
        requestedPercent = percent;
        enabled = true;

        if (mode == Mode.IDLE) {
            mode = Mode.RUNNING;
        }
    }

    /** Stops roller and disables anti-jam. */
    public void stop() {
        enabled = false;
        requestedPercent = 0.0;
        mode = Mode.IDLE;

        jamTimer.stop(); jamTimer.reset();
        unjamTimer.stop(); unjamTimer.reset();
        cooldownTimer.stop(); cooldownTimer.reset();

        setPercent(0.0);
    }

    /** Optional: use this if you want to run without anti-jam logic. */
    public void runPercentRaw(double percent) {
        enabled = false; // bypass state machine
        requestedPercent = percent;
        mode = Mode.IDLE;
        setPercent(percent);
    }

    // =============================
    // Core logic
    // =============================

    @Override
    public void periodic() {
        // If not enabled, do nothing (unless raw caller is driving it)
        if (!enabled) return;

        double statorCurrent = roller.getStatorCurrent().getValueAsDouble(); // amps
        double velocityRps = Math.abs(roller.getVelocity().getValueAsDouble()); // motor rps

        boolean tryingToMove = Math.abs(requestedPercent) > 0.05;

        // Only detect jams while trying to intake/outtake (not when stopped)
        boolean jamCondition = tryingToMove
                && (statorCurrent >= JAM_STATOR_CURRENT_A)
                && (velocityRps <= JAM_VELOCITY_RPS);

        switch (mode) {
            case RUNNING -> {
                // Apply requested direction continuously
                setPercent(requestedPercent);

                // Debounce jam detection
                if (jamCondition) {
                    if (!jamTimer.isRunning()) {
                        jamTimer.reset();
                        jamTimer.start();
                    }
                    if (jamTimer.hasElapsed(JAM_DEBOUNCE_SEC)) {
                        // Start unjam
                        jamTimer.stop();
                        jamTimer.reset();

                        mode = Mode.UNJAMMING;
                        unjamTimer.reset();
                        unjamTimer.start();

                        // Reverse opposite of requested direction
                        setPercent(-Math.signum(requestedPercent) * UNJAM_REVERSE_PERCENT);
                    }
                } else {
                    // clear timer if not jammy
                    if (jamTimer.isRunning()) {
                        jamTimer.stop();
                        jamTimer.reset();
                    }
                }
            }

            case UNJAMMING -> {
                // Keep reversing
                setPercent(-Math.signum(requestedPercent) * UNJAM_REVERSE_PERCENT);

                // Decide when to stop unjamming:
                // 1) at least UNJAM_MIN_SEC
                // 2) either jam condition is gone OR we hit UNJAM_MAX_SEC
                boolean minDone = unjamTimer.hasElapsed(UNJAM_MIN_SEC);
                boolean maxDone = unjamTimer.hasElapsed(UNJAM_MAX_SEC);

                boolean jamGone = !jamCondition;

                if ((minDone && jamGone) || maxDone) {
                    unjamTimer.stop();
                    unjamTimer.reset();

                    mode = Mode.COOLDOWN;
                    cooldownTimer.reset();
                    cooldownTimer.start();

                    // Resume requested direction immediately after cooldown begins
                    // (or you can resume right away, but cooldown helps avoid flip-flop)
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
