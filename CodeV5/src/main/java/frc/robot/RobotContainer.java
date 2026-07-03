package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IntakeSlapdownSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
    private final double MaxSpeed =
        1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    private final double MaxAngularRate =
        RotationsPerSecond.of(1.4).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain =
        TunerConstants.createDrivetrain();

    private double driveScale = 0.2;

    private final IntakeSlapdownSubsystem intakeSlapdown = new IntakeSlapdownSubsystem();
    private final IntakeRollerSubsystem intakeRollers = new IntakeRollerSubsystem();
    private final IndexerSubsystem indexer = new IndexerSubsystem();
    private final Intake intake = new Intake(intakeSlapdown, intakeRollers, indexer);

    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final KickerSubsystem kicker = new KickerSubsystem();
    private final HoodSubsystem hood = new HoodSubsystem();
    private final Shooter shooter =
        new Shooter(shooterSubsystem, kicker, indexer, hood, intakeSlapdown, intakeRollers);

    private final RobotPrecheck precheck =
    new RobotPrecheck(
        intakeSlapdown,
        intakeRollers,
        indexer,
        shooterSubsystem,
        hood,
        drivetrain,
        kicker
    );

    public RobotContainer() {
        configureBindings();
    }

    public RobotPrecheck getPrecheck() {
        return precheck;
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double xCmd = -joystick.getLeftY() * MaxSpeed * driveScale;
                double yCmd = -joystick.getLeftX() * MaxSpeed * driveScale;
                double rotCmd = -joystick.getRightX() * MaxAngularRate;

                return drive.withVelocityX(xCmd).withVelocityY(yCmd).withRotationalRate(rotCmd);
            })
        );

        joystick.leftBumper().onTrue(
            drivetrain.runOnce(drivetrain::seedFieldCentric)
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        var lt = joystick.leftTrigger();
        var ltHeld = lt.debounce(0.2);

        boolean[] holdStarted = new boolean[1];

        ltHeld.onTrue(
            Commands.sequence(
                Commands.runOnce(() -> holdStarted[0] = true),
                intake.holdDownAndIntake()
            )
        );

        lt.onFalse(
            Commands.sequence(
                Commands.defer(
                    () -> holdStarted[0]
                        ? intake.stopIntakeAndGoTravel()
                        : intake.tapToggleUpTravel(),
                    Set.of(intake)
                ),
                Commands.runOnce(() -> holdStarted[0] = false)
            )
        );

        // Demo shoot: hood goes to the set angle and flywheel spins at the set
        // speed while held; on release the hood returns to 0 and everything stops.
        joystick.rightTrigger().whileTrue(
            shooter.shootWhileHeld()
        );

        joystick.a()
            .whileTrue(intake.holdReverseJamClear())
            .onFalse(intake.stopIntakeAndGoTravel());

        joystick.rightBumper().onTrue(
            Commands.runOnce(() -> {
                hood.setTargetDegrees(Constants.Hood.MIN_DEG);
                intakeSlapdown.up();
                intakeRollers.stop();
                indexer.stop();
                kicker.stop();
                shooterSubsystem.stop();
            }, hood, intakeSlapdown, intakeRollers, indexer, kicker, shooterSubsystem)
        );

        joystick.y().onTrue(
            Commands.runOnce(() -> {
                driveScale = (driveScale == 0.57) ? 0.45
                        : (driveScale == 0.45) ? 0.7
                        : 0.57;
            })
        );

        joystick.povDown().onTrue(
            hood.zeroRoutine()
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Demo version: no autonomous routines.
        return Commands.none();
    }
}
