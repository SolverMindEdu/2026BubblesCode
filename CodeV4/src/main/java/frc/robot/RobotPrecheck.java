package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IntakeSlapdownSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotPrecheck {

    private final IntakeSlapdownSubsystem intakeSlapdown;
    private final IntakeRollerSubsystem intakeRollers;
    private final IndexerSubsystem indexer;
    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;
    private final LEDs leds;
    private final CommandSwerveDrivetrain drivetrain;
    private final KickerSubsystem kicker;

    public RobotPrecheck(
            IntakeSlapdownSubsystem intakeSlapdown,
            IntakeRollerSubsystem intakeRollers,
            IndexerSubsystem indexer,
            ShooterSubsystem shooter,
            HoodSubsystem hood,
            LEDs leds,
            CommandSwerveDrivetrain drivetrain,
            KickerSubsystem kicker) {

        this.intakeSlapdown = intakeSlapdown;
        this.intakeRollers = intakeRollers;
        this.indexer = indexer;
        this.shooter = shooter;
        this.hood = hood;
        this.leds = leds;
        this.drivetrain = drivetrain;
        this.kicker = kicker;
    }

    public void update() {

        // === CONNECTIVITY ===
        boolean intakeSlapdownOK = intakeSlapdown.isConnected();
        boolean intakeRollerOK = intakeRollers.isConnected();
        boolean indexerOK = indexer.isConnected();
        boolean shooterOK =
            shooter.isConnectedLeft() &&
            shooter.isConnectedMiddle() &&
            shooter.isConnectedRight();
        boolean hoodOK = hood.isConnected();
        boolean ledsOK = leds.isConnected();
        boolean kickerOK = kicker.isConnected();

        // === POSE CHECK ===
        boolean swervePoseOK = isPoseValid(drivetrain.getPose());

        // === SYSTEM CHECKS ===
        boolean batteryOK = RobotController.getBatteryVoltage() > 10.0;
        boolean dsAttached = DriverStation.isDSAttached();

        // === FINAL RESULT ===
        boolean allPassed =
            intakeSlapdownOK &&
            intakeRollerOK &&
            indexerOK &&
            shooterOK &&
            hoodOK &&
            kickerOK &&
            ledsOK &&
            swervePoseOK &&
            batteryOK &&
            dsAttached;

        String failures = buildFailures(
                intakeSlapdownOK,
                intakeRollerOK,
                indexerOK,
                shooterOK,
                hoodOK,
                ledsOK,
                swervePoseOK,
                batteryOK,
                kickerOK,
                dsAttached
        );

        // === DASHBOARD OUTPUT ===
        SmartDashboard.putBoolean("Precheck/AllPassed", allPassed);

        SmartDashboard.putBoolean("Precheck/IntakeSlapdown", intakeSlapdownOK);
        SmartDashboard.putBoolean("Precheck/IntakeRoller", intakeRollerOK);
        SmartDashboard.putBoolean("Precheck/Indexer", indexerOK);
        SmartDashboard.putBoolean("Precheck/Shooter", shooterOK);
        SmartDashboard.putBoolean("Precheck/Hood", hoodOK);
        SmartDashboard.putBoolean("Precheck/LEDs", ledsOK);
        SmartDashboard.putBoolean("Precheck/SwervePose", swervePoseOK);
        SmartDashboard.putBoolean("Precheck/Kicker", kickerOK);

        SmartDashboard.putBoolean("Precheck/BatteryOK", batteryOK);
        SmartDashboard.putBoolean("Precheck/DSAttached", dsAttached);

        SmartDashboard.putNumber("Precheck/BatteryVoltage", RobotController.getBatteryVoltage());

        SmartDashboard.putString("Precheck/Failures", failures);
    }

    private boolean isPoseValid(Pose2d pose) {
        return pose != null &&
                Double.isFinite(pose.getX()) &&
                Double.isFinite(pose.getY()) &&
                Double.isFinite(pose.getRotation().getRadians());
    }

    private String buildFailures(
            boolean intakeSlapdownOK,
            boolean intakeRollerOK,
            boolean indexerOK,
            boolean shooterOK,
            boolean hoodOK,
            boolean ledsOK,
            boolean swervePoseOK,
            boolean batteryOK,
            boolean kickerOK,
            boolean dsAttached) {

        StringBuilder failures = new StringBuilder();

        if (!intakeSlapdownOK) failures.append("Slapdown ");
        if (!intakeRollerOK) failures.append("Roller ");
        if (!indexerOK) failures.append("Indexer ");
        if (!shooterOK) failures.append("Shooter ");
        if (!hoodOK) failures.append("Hood ");
        if (!ledsOK) failures.append("LEDs ");
        if (!swervePoseOK) failures.append("Pose ");
        if (!batteryOK) failures.append("Battery ");
        if (!dsAttached) failures.append("DS ");
        if (!kickerOK) failures.append("Kicker ");

        return failures.length() == 0 ? "OK" : failures.toString();
    }
}