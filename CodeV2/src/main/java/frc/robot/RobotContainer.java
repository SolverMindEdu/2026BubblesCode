package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.io.DriverControlsIO;
import frc.robot.io.DriverControlsIOReal;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IntakeSlapdown;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShotCalculator;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.LEDState;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.HoodSubsystem;

import frc.robot.subsystems.FullSubsystems.Intake;
import frc.robot.subsystems.FullSubsystems.Shooter;

public class RobotContainer {
  private final double MaxSpeed = 1.0 * Constants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.10)
          .withRotationalDeadband(MaxAngularRate * 0.10)
          .withDriveRequestType(DriveRequestType.Velocity);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public final CommandSwerveDrivetrain drivetrain = Constants.createDrivetrain();
  private final ShotCalculator shotCalc = new ShotCalculator(drivetrain);

  // Intake
  private final IntakeSlapdown slapdown = new IntakeSlapdown();
  private final IntakeRollerSubsystem rollers = new IntakeRollerSubsystem();

  // Shooter chain
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final KickerSubsystem kicker = new KickerSubsystem();
  private final IndexerSubsystem indexer = new IndexerSubsystem();
  private final HoodSubsystem hood = new HoodSubsystem();
  private final LEDs leds = new LEDs();
  private double filteredTargetRad = 0.0;
  private boolean targetInitialized = false;

  private final Shooter shooter =
      new Shooter(shooterSubsystem, kicker, indexer, hood, shotCalc, slapdown);

  private final Intake intake = new Intake(slapdown, rollers, indexer);

  private final DriverControlsIO driverIO =
      RobotBase.isReal() ? new DriverControlsIOReal(0) : (inputs) -> {};
  private final DriverControlsIO.DriverControlsIOInputs driverInputs =
      new DriverControlsIO.DriverControlsIOInputs();

  private final SendableChooser<Command> autoChooser;
  private final PIDController aimThetaPid = new PIDController(9.5, 0.0, 0.2);

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    registerNamedCommands();
    aimThetaPid.enableContinuousInput(-Math.PI, Math.PI);
    configureBindings();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("ShotCalculator", shotCalc);
  }

  private boolean isAimedWithinDegrees(double toleranceDeg) {
  var p = shotCalc.getParameters();

  double targetRad = p.robotHeadingRadians();
  double currentRad = drivetrain.getState().Pose.getRotation().getRadians();

  if (!Double.isFinite(targetRad)) {
    return false;
  }

  double errorRad = MathUtil.angleModulus(targetRad - currentRad);
  double errorDeg = Math.toDegrees(Math.abs(errorRad));

  return errorDeg <= toleranceDeg;
  }

  private boolean isAligningForShot() {
    return driverInputs.rightTrigger > 0.5 && !isAimedWithinDegrees(3.0);
  }

  private boolean isReadyToShoot() {
    return driverInputs.rightTrigger > 0.5
        && shooter.isAtSpeed()
        && isAimedWithinDegrees(3.0)
        && !shooter.isFeeding();
  }

  private boolean isIntakingNow() {
    return driverInputs.leftTrigger > 0.5 || intake.getState() == Intake.State.DOWN;
  }

  private boolean isPassingNow() {
    return driverInputs.rightTrigger > 0.5 && shooter.isPassing() && !shooter.isFeeding();
  }

  private void updateLEDState() {
    LEDState desired;

    if (DriverStation.isAutonomousEnabled()) {
      desired = LEDState.AUTO_RAINBOW;
    } else if (shooter.isFeeding()) {
      desired = LEDState.SHOOTING_BLUE_FLASH;
    } else if (isIntakingNow()) {
      desired = LEDState.INTAKING_YELLOW;
    } else if (isPassingNow()) {
      desired = LEDState.PASSING_YELLOW_FLASH;
    } else if (isAligningForShot()) {
      desired = LEDState.ALIGNING_RED_FLASH;
    } else if (isReadyToShoot()) {
      desired = LEDState.READY_GREEN;
    } else {
      desired = LEDState.IDLE_BLUE;
    }

    leds.setState(desired);
  }

  public void updateDriverInputs() {
    driverIO.updateInputs(driverInputs);
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand(
        "IntakeDeploy",
        Commands.runOnce(slapdown::down, slapdown));

    NamedCommands.registerCommand(
        "IntakeTravel",
        Commands.runOnce(slapdown::travel, slapdown));
    NamedCommands.registerCommand(
        "RunIntake",
        Commands.startEnd(
                () -> {
                  rollers.runIntakeRps(65.0); 
                  indexer.run(0.4);
                },
                () -> {
                  rollers.stop();
                  indexer.stop();
                },
                rollers,
                indexer)
            .withTimeout(1.0));
    NamedCommands.registerCommand(
        "StopIntake",
        Commands.runOnce(
            () -> {
              rollers.stop();
              indexer.stop();
            },
            rollers,
            indexer));

    NamedCommands.registerCommand(
        "Shoot",
        Commands.parallel(
                shooter.shootWhileHeld(), 
                autoAimDrive())
            .withTimeout(3.0));
  }

  // private Command autoAimDrive() {
  //   return drivetrain.applyRequest(() -> {
  //     var p = shotCalc.getParameters();

  //     double targetRad = p.robotHeadingRadians();
  //     double currentRad = drivetrain.getState().Pose.getRotation().getRadians();

  //     double xCmd = driverInputs.leftY * MaxSpeed * 0.5;
  //     double yCmd = driverInputs.leftX * MaxSpeed * 0.5;

  //     if (!Double.isFinite(targetRad)) {
  //       double rotCmd = -driverInputs.rightX * MaxAngularRate;
  //       return drive.withVelocityX(xCmd).withVelocityY(yCmd).withRotationalRate(rotCmd);
  //     }

  //     double omegaCmd = aimThetaPid.calculate(currentRad, targetRad);
  //     omegaCmd = MathUtil.clamp(omegaCmd, -MaxAngularRate, MaxAngularRate);

  //     return drive.withVelocityX(xCmd).withVelocityY(yCmd).withRotationalRate(omegaCmd);
  //   });
  // }

private Command autoAimDrive() {
  return drivetrain.applyRequest(() -> {
    var p = shotCalc.getParameters();

    double targetRad = p.robotHeadingRadians();
    double currentRad = drivetrain.getState().Pose.getRotation().getRadians();

    double xCmd = driverInputs.leftY * MaxSpeed * 0.5;
    double yCmd = driverInputs.leftX * MaxSpeed * 0.5;

    if (!Double.isFinite(targetRad)) {
      targetInitialized = false;
      double rotCmd = -driverInputs.rightX * MaxAngularRate;
      return drive.withVelocityX(xCmd).withVelocityY(yCmd).withRotationalRate(rotCmd);
    }

    // smooth target heading
    if (!targetInitialized) {
      filteredTargetRad = targetRad;
      targetInitialized = true;
    } else {
      double delta = MathUtil.angleModulus(targetRad - filteredTargetRad);
      filteredTargetRad = MathUtil.angleModulus(filteredTargetRad + delta * 0.2);
    }

    double errorRad = MathUtil.angleModulus(filteredTargetRad - currentRad);

    // tiny deadband near lock
    if (Math.abs(errorRad) < Math.toRadians(1.0)) {
      return drive.withVelocityX(xCmd).withVelocityY(yCmd).withRotationalRate(0.0);
    }

    double omegaCmd = aimThetaPid.calculate(currentRad, filteredTargetRad);

    double autoAimMaxOmega = RotationsPerSecond.of(0.5).in(RadiansPerSecond);
    omegaCmd = MathUtil.clamp(omegaCmd, -autoAimMaxOmega, autoAimMaxOmega);

    return drive.withVelocityX(xCmd).withVelocityY(yCmd).withRotationalRate(omegaCmd);
    });
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> {
          double xCmd = driverInputs.leftY * MaxSpeed * 0.6;
          double yCmd = driverInputs.leftX * MaxSpeed * 0.6;
          double rotCmd = driverInputs.rightX * MaxAngularRate;

          return drive.withVelocityX(xCmd).withVelocityY(yCmd).withRotationalRate(rotCmd);
        }));

    leds.setDefaultCommand(Commands.run(this::updateLEDState, leds));

    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    Trigger leftTrigger = new Trigger(() -> driverInputs.leftTrigger > 0.5);
    Trigger rightTrigger = new Trigger(() -> driverInputs.rightTrigger > 0.5);
    Trigger buttonA = new Trigger(() -> driverInputs.a);
    Trigger leftBumper = new Trigger(() -> driverInputs.leftBumper);

    leftTrigger
        .debounce(0.25)
        .whileTrue(
            Commands.defer(
                () -> intake.isTravel() ? intake.goDownAndIntake() : Commands.none(),
                Set.of(intake)));

    leftTrigger.onFalse(
        Commands.defer(   
            () -> {
              if (intake.getState() == Intake.State.DOWN || intake.getState() == Intake.State.MOVING) {
                return intake.stopIntakeAndGoTravel();
              }
              return intake.tapToggleUpTravel();
            },
            Set.of(intake)));

    rightTrigger.whileTrue(
        Commands.parallel(
            shooter.shootWhileHeld(),
            autoAimDrive()));

    buttonA.whileTrue(
        Commands.startEnd(
            () -> {
              indexer.run(-0.7);
              rollers.runIntakePercent(-0.5);
            },
            () -> {
              indexer.stop();
              rollers.stop();
            },
            indexer,
            rollers));

    leftBumper.onTrue(
        drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}