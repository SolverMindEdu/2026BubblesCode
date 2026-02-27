// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.io.DriverControlsIO;
import frc.robot.io.DriverControlsIOReal;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IntakeSlapdown;
import frc.robot.subsystems.FullSubsystems.Intake;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import org.littletonrobotics.junction.Logger;
import frc.robot.io.DriverControlsIOInputsAutoLogged;
import frc.robot.subsystems.FullSubsystems.Shooter;

public class RobotContainer {
    private double MaxSpeed = 1.0 * Constants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    
    public final CommandSwerveDrivetrain drivetrain = Constants.createDrivetrain();
    private final IntakeSlapdown slapdown = new IntakeSlapdown();
    private final IntakeRollerSubsystem rollers = new IntakeRollerSubsystem();
    private final Intake intake = new Intake(slapdown, rollers);
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final KickerSubsystem kicker = new KickerSubsystem();
    private final IndexerSubsystem indexer = new IndexerSubsystem();


    private final Shooter shooterFull = new frc.robot.subsystems.FullSubsystems.Shooter(shooter, kicker, indexer);

    private final DriverControlsIO driverIO = RobotBase.isReal() ? new DriverControlsIOReal(0) : (inputs) -> {};
    private final DriverControlsIOInputsAutoLogged driverInputs = new DriverControlsIOInputsAutoLogged();
    private final SendableChooser<Command> autoChooser;


    public RobotContainer() {
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(driverInputs.leftY * MaxSpeed * 0.2)
                    .withVelocityY(-driverInputs.leftX * MaxSpeed * 0.2)
                    .withRotationalRate(-driverInputs.rightX * MaxAngularRate)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        Trigger leftTrigger = joystick.leftTrigger();

        //intake hold
        leftTrigger
            .debounce(0.25)
            .whileTrue(
                Commands.defer(
                    () -> intake.isTravel()
                        ? intake.goDownAndIntake()
                        : Commands.none(),
                    Set.of(intake)
                )
            );

        //intake release
        leftTrigger.onFalse(
            Commands.defer(() -> {
                if (intake.getState() == Intake.State.DOWN
                    || intake.getState() == Intake.State.MOVING) {

                    return intake.stopIntakeAndGoTravel();
                }
                return intake.tapToggleUpTravel();

            }, Set.of(intake))
        );

        joystick.rightTrigger().whileTrue(shooterFull.shootWhileHeld());

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void updateDriverInputs() {
        driverIO.updateInputs(driverInputs);
        Logger.processInputs("Driver", driverInputs);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
