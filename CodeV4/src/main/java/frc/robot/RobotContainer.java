package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Set;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IntakeSlapdownSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShotCalculator;

public class RobotContainer {
    private final double MaxSpeed =
        1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    private final double MaxAngularRate =
        RotationsPerSecond.of(1.8).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(0);
    private boolean isIdleActive = false;

    public final CommandSwerveDrivetrain drivetrain =
        TunerConstants.createDrivetrain();

    private final ShotCalculator shotCalc = new ShotCalculator(drivetrain);
    private double driveScale = 0.57;
    BooleanSupplier isIdle = () -> joystick.rightTrigger().getAsBoolean() && updateIdleStatus();

    private final IntakeSlapdownSubsystem intakeSlapdown = new IntakeSlapdownSubsystem();
    private final IntakeRollerSubsystem intakeRollers = new IntakeRollerSubsystem();
    private final IndexerSubsystem indexer = new IndexerSubsystem();
    private final LEDs leds = new LEDs();
    private final Intake intake = new Intake(intakeSlapdown, intakeRollers, indexer, leds);

    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final KickerSubsystem kicker = new KickerSubsystem();
    private final HoodSubsystem hood = new HoodSubsystem();
    private final Shooter shooter =
        new Shooter(shooterSubsystem, kicker, indexer, hood, shotCalc, intakeSlapdown, intakeRollers);

    private final SwerveRequest.SwerveDriveBrake xLock = new SwerveRequest.SwerveDriveBrake();
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private final PIDController aimThetaPid = new PIDController(12.0, 0.0, 0.3);
    private final RobotPrecheck precheck =
    new RobotPrecheck(
        intakeSlapdown,
        intakeRollers,
        indexer,
        shooterSubsystem,
        hood,
        leds,
        drivetrain,
        kicker
    );

    private double filteredTargetRad = 0.0;
    private boolean targetInitialized = false;

    public RobotContainer() {
        aimThetaPid.enableContinuousInput(-Math.PI, Math.PI);

        registerNamedCommands();
        configureAutoChooser();
        configureBindings();

        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData("ShotCalculator", shotCalc);
    }

    public RobotPrecheck getPrecheck() {
        return precheck;
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand(
            "IntakeDeploy",
            Commands.runOnce(() -> {
                intakeSlapdown.up();
                intakeRollers.stop();
                indexer.stop();
            }, intakeSlapdown, intakeRollers, indexer)
        );

        NamedCommands.registerCommand(
            "IntakeRun",
            intake.holdDownAndIntake()
        );

        NamedCommands.registerCommand(
            "IntakeStop",
            intake.stopIntakeAndGoTravel()
        );

        NamedCommands.registerCommand(
            "ShootInitialize",
            Commands.parallel(
                shooter.shootWhileHeld()
            ).withTimeout(0.1)
        );

        NamedCommands.registerCommand(
            "ShootSingleSwipe",
            Commands.parallel(
                shooter.shootWhileHeld(),
                autoAimDrive()
            ).withTimeout(6.0)
        );

        NamedCommands.registerCommand(
            "ShootDepot",
            Commands.parallel(
                shooter.shootWhileHeld(),
                autoAimDrive()
            ).withTimeout(4.0)
        );
    }

    private void configureAutoChooser() {
        autoChooser.setDefaultOption("1.5 Swipe Right", new PathPlannerAuto("1.5SwipeRight"));
        autoChooser.addOption("Single Swipe Right", new PathPlannerAuto("SingleSwipeRight"));
        autoChooser.addOption("Far 1.5 Swipe Right", new PathPlannerAuto("Far1.5SwipeRight"));
        autoChooser.addOption("Far Single Swipe Right", new PathPlannerAuto("FarSingleSwipeRight"));
        autoChooser.addOption("1.5 Swipe Left", new PathPlannerAuto("1.5SwipeLeft"));
        autoChooser.addOption("Single Swipe Left", new PathPlannerAuto("SingleSwipeLeft"));
        autoChooser.addOption("1Swipe+Depot Left", new PathPlannerAuto("1Swipe+DepotLeft"));
        autoChooser.addOption("Far 1.5 Swipe Left", new PathPlannerAuto("Far1.5SwipeLeft"));
        autoChooser.addOption("Middle Depot", new PathPlannerAuto("MiddleDepotAuto"));
    }

    public boolean updateIdleStatus() {
        double error = Math.abs(Math.toDegrees(
            MathUtil.angleModulus(filteredTargetRad - drivetrain.getState().Pose.getRotation().getRadians())
        ));
        boolean inputOutsideDeadband =
            Math.abs(joystick.getLeftX()) > 0.12 ||
            Math.abs(joystick.getLeftY()) > 0.12 ||
            Math.abs(joystick.getRightX()) > 0.12;

        if (inputOutsideDeadband || DriverStation.isAutonomous()) {
            isIdleActive = false;
        } else if (error < 1.0) {
            isIdleActive = true;
        } else if (error > 3.5) {
            isIdleActive = false;
        }

        return isIdleActive;
    }

    private boolean xLockActive = false;

    private Command autoAimDrive() {
        return drivetrain.applyRequest(() -> {
            var p = shotCalc.getParameters();

            double targetRad = p.robotHeadingRadians();
            double currentRad = drivetrain.getState().Pose.getRotation().getRadians();

            double leftY = -joystick.getLeftY();
            double leftX = -joystick.getLeftX();
            double rightX = -joystick.getRightX();

            double xCmd = leftY * MaxSpeed * 0.5;
            double yCmd = leftX * MaxSpeed * 0.5;

            boolean driverTransOverride =
                Math.abs(leftY) > 0.12 || Math.abs(leftX) > 0.12;
            boolean driverRotOverride =
                Math.abs(rightX) > 0.12;

            if (!Double.isFinite(targetRad)) {
                targetInitialized = false;
                xLockActive = false;
                double rotCmd = rightX * MaxAngularRate;
                return drive.withVelocityX(xCmd).withVelocityY(yCmd).withRotationalRate(rotCmd);
            }

            if (!targetInitialized) {
                filteredTargetRad = targetRad;
                targetInitialized = true;
            } else {
                double delta = MathUtil.angleModulus(targetRad - filteredTargetRad);
                filteredTargetRad = MathUtil.angleModulus(filteredTargetRad + delta * 0.2);
            }

            double errorDeg = Math.abs(Math.toDegrees(MathUtil.angleModulus(filteredTargetRad - currentRad)));

            SmartDashboard.putNumber("AimErrorDeg", errorDeg);
            SmartDashboard.putBoolean("XLockActive", xLockActive);

            if (driverRotOverride) {
                xLockActive = false;
                double rotCmd = rightX * MaxAngularRate;
                return drive.withVelocityX(xCmd).withVelocityY(yCmd).withRotationalRate(rotCmd);
            }

            if (driverTransOverride) {
                xLockActive = false;
            }

            // Hysteresis: engage at 1°, disengage at 3°
            if (errorDeg < 0.2 && !driverTransOverride) {
                xLockActive = true;
            } else if (errorDeg > 4.0) {
                xLockActive = false;
            }

            if (xLockActive) {
                return xLock;
            }

            double errorRad = MathUtil.angleModulus(filteredTargetRad - currentRad);
            double omegaCmd = aimThetaPid.calculate(currentRad, filteredTargetRad);
            double autoAimMaxOmega = RotationsPerSecond.of(0.5).in(RadiansPerSecond);
            omegaCmd = MathUtil.clamp(omegaCmd, -autoAimMaxOmega, autoAimMaxOmega);

            return drive.withVelocityX(xCmd).withVelocityY(yCmd).withRotationalRate(omegaCmd);
        });
    }

    private Command autoAlignPassDrive() {
        return drivetrain.applyRequest(() -> {
            double currentRad = drivetrain.getState().Pose.getRotation().getRadians();

            double xCmd = -joystick.getLeftY() * MaxSpeed * 0.5;
            double yCmd = -joystick.getLeftX() * MaxSpeed * 0.5;

            var alliance = DriverStation.getAlliance();
            double targetRad;

            if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                targetRad = Math.PI;
            } else {
                targetRad = 0.0;
            }

            double errorRad = MathUtil.angleModulus(targetRad - currentRad);

            if (Math.abs(errorRad) < Math.toRadians(1.0)) {
                return drive.withVelocityX(xCmd).withVelocityY(yCmd).withRotationalRate(0.0);
            }

            double omegaCmd = aimThetaPid.calculate(currentRad, targetRad);
            double autoAimMaxOmega = RotationsPerSecond.of(0.5).in(RadiansPerSecond);
            omegaCmd = MathUtil.clamp(omegaCmd, -autoAimMaxOmega, autoAimMaxOmega);

            return drive.withVelocityX(xCmd).withVelocityY(yCmd).withRotationalRate(omegaCmd);
        });
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

        // BooleanSupplier isIdle = () -> updateIdleStatus();

        // new Trigger(isIdle)
        //     .whileTrue(
        //         new SequentialCommandGroup(
        //             new WaitCommand(0.01),
        //             drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake())
        //         )
        //     );

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

        ltHeld.whileTrue(
            Commands.run(
                () -> shooter.setTargetRps(10.0),
                shooter
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

        joystick.rightTrigger().whileTrue(
            Commands.parallel(
                shooter.shootWhileHeld(),
                autoAimDrive(),
                Commands.run(() -> leds.setState(LEDs.LEDState.INTAKING_GREEN_FLASH), leds)
            )
        );

        joystick.b().whileTrue(
            Commands.parallel(
                shooter.shootWhileHeld(),
                autoAlignPassDrive(),
                Commands.run(() -> leds.setState(LEDs.LEDState.PASSING_YELLOW_FLASH), leds)
            )
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
        return autoChooser.getSelected();
    }
}