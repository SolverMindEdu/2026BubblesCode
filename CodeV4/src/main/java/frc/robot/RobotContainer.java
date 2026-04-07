package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.LEDState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShotCalculator;

public class RobotContainer {
    private double driveScale = 0.5;
    private final double MaxSpeed =
        1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * driveScale;
    private final double MaxAngularRate =
        RotationsPerSecond.of(0.65).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain =
        TunerConstants.createDrivetrain();

    private final ShotCalculator shotCalc = new ShotCalculator(drivetrain);

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

    private final SendableChooser<Command> autoChooser;

    private final PIDController aimThetaPid = new PIDController(11.0, 0.0, 0.3);

    private double filteredTargetRad = 0.0;
    private boolean targetInitialized = false;

    public RobotContainer() {
        aimThetaPid.enableContinuousInput(-Math.PI, Math.PI);
        autoChooser = AutoBuilder.buildAutoChooser();

        configureBindings();

        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData("ShotCalculator", shotCalc);
    }

    private Command autoAimDrive() {
        return drivetrain.applyRequest(() -> {
            var p = shotCalc.getParameters();

            double targetRad = p.robotHeadingRadians();
            double currentRad = drivetrain.getState().Pose.getRotation().getRadians();

            double xCmd = -joystick.getLeftY() * MaxSpeed * 0.5;
            double yCmd = -joystick.getLeftX() * MaxSpeed * 0.5;

            if (!Double.isFinite(targetRad)) {
                targetInitialized = false;
                double rotCmd = -joystick.getRightX() * MaxAngularRate;
                return drive.withVelocityX(xCmd).withVelocityY(yCmd).withRotationalRate(rotCmd);
            }

            if (!targetInitialized) {
                filteredTargetRad = targetRad;
                targetInitialized = true;
            } else {
                double delta = MathUtil.angleModulus(targetRad - filteredTargetRad);
                filteredTargetRad = MathUtil.angleModulus(filteredTargetRad + delta * 0.2);
            }

            double errorRad = MathUtil.angleModulus(filteredTargetRad - currentRad);

            if (Math.abs(errorRad) < Math.toRadians(1.0)) {
                return drive.withVelocityX(xCmd).withVelocityY(yCmd).withRotationalRate(0.0);
            }

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
                targetRad = 0.0;
            } else {
                targetRad = Math.PI;
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

        joystick.rightTrigger().whileTrue(
            Commands.parallel(
                shooter.shootWhileHeld(),
                autoAimDrive()
            )
        );

        joystick.b().whileTrue(
            Commands.parallel(
                shooter.shootWhileHeld(),
                autoAlignPassDrive()
            )
        );

        joystick.a()
            .whileTrue(intake.holdReverseJamClear())
            .onFalse(intake.stopIntakeAndGoTravel());

        joystick.x().onTrue(
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
                driveScale = (driveScale == 0.5) ? 0.65 : 0.5;
            })
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}