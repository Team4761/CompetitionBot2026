// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.baseCommands.DoNothingCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.intake.ExtendCommandCAUGHT;
import frc.robot.subsystems.intake.IntakeCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.OuttakeCommand;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.ElasticManualOverrideCommand;
import frc.robot.subsystems.turret.INTERMITENTShootCommand;
import frc.robot.subsystems.turret.KickerSpinCommand;
import frc.robot.subsystems.turret.ShootAtAngleDRIFTCommand;
import frc.robot.subsystems.turret.ShootAtAngleSTUTTERCommand;
import frc.robot.subsystems.turret.ShootCommand;
import frc.robot.subsystems.turret.ShootWithPowerCommand;
import frc.robot.subsystems.turret.SpindexSpinCommand;
import frc.robot.subsystems.turret.TurretAimChangeCommand;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final VisionSubsystem vision = new VisionSubsystem(drivetrain);
    private final TurretSubsystem turret = new TurretSubsystem();
    private final GyroSubsystem gyro = new GyroSubsystem();
    private final Orchestra orchestra = new Orchestra("output.chrp");

    private final double MaxSpeed = 0.55 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // practice-safe top speed cap
    private final double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond); // reduced max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController controller_drive =
        new CommandXboxController(Constants.Controller.DRIVER_PORT);
    private final CommandXboxController controller_operator =
        new CommandXboxController(Constants.Controller.OPERATOR_PORT);
    private final SlewRateLimiter rotationLimiter =
        new SlewRateLimiter(Constants.Controller.ROTATION_SLEW_RATE_RAD_PER_SEC_SQ);

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();
        configurePathPlannerAutos();
        SmartDashboard.putData("PathPlanner Auto Chooser", autoChooser);
    }

    private void configurePathPlannerAutos() {
        NamedCommands.registerCommand("Shoot8", new ShootCommand(turret).withTimeout(4));
        NamedCommands.registerCommand("shootCommand", new ShootCommand(turret).withTimeout(4));
        NamedCommands.registerCommand(
            "ShootWith[SHORT]PowerCommand",
            new ShootWithPowerCommand(turret, Constants.Turret.ShootConfig.SHORT_SPITTER_SPEED)
        );
        NamedCommands.registerCommand("ExtendIntakeComandCAUGHT", new ExtendCommandCAUGHT(intake));
        NamedCommands.registerCommand("ExtedIntakeCommand", new ExtendCommandCAUGHT(intake));
        NamedCommands.registerCommand("DoNothingCommand", new DoNothingCommand());
        NamedCommands.registerCommand("DoNothingComand", new DoNothingCommand());
        NamedCommands.registerCommand("Intake Command", new IntakeCommand(intake));
        NamedCommands.registerCommand("IntakeCommand", new IntakeCommand(intake));
        NamedCommands.registerCommand(
            "Stop Intake",
            new InstantCommand(() -> {
                intake.intakeMotor.enableCoasting();
                intake.intakeMotor.stopTurning();
            })
        );
        NamedCommands.registerCommand("OuttakeCommand", new OuttakeCommand(intake));

        autoChooser.setDefaultOption("None", Commands.none());
        for (String autoName : AutoBuilder.getAllAutoNames()) {
            try {
                if (autoName.startsWith("LEFT")) {
                    String suffix = autoName.substring(4);
                    autoChooser.addOption("LEFT" + suffix, new PathPlannerAuto(autoName, false));
                    autoChooser.addOption("RIGHT" + suffix, new PathPlannerAuto(autoName, true));
                } else {
                    autoChooser.addOption(autoName, new PathPlannerAuto(autoName));
                }
            } catch (Exception e) {
                DriverStation.reportError(
                    "Skipping invalid PathPlanner auto '" + autoName + "': " + e.getMessage(),
                    e.getStackTrace()
                );
            }
        }
    }

    private void configureBindings() {
        configureDefaultDrive();
        configureDisabledBehavior();
        configureDriverBindings();
        configureOperatorBindings();
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureDefaultDrive() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double xInput = -1 * applyDeadband(
                    controller_drive.getLeftY(),
                    Constants.Controller.TRANSLATION_INPUT_DEADBAND
                );
                double yInput = -1 * applyDeadband(
                    controller_drive.getLeftX(),
                    Constants.Controller.TRANSLATION_INPUT_DEADBAND
                );
                double turnInput = shapeTurnInput(
                    -1 * applyDeadband(
                        controller_drive.getRightX(),
                        Constants.Controller.ROTATION_INPUT_DEADBAND
                    )
                );

                return drive.withVelocityX(xInput * MaxSpeed)
                    .withVelocityY(yInput * MaxSpeed)
                    .withRotationalRate(
                        rotationLimiter.calculate(
                            turnInput * Constants.Controller.ROTATION_MULTIPLIER * MaxAngularRate
                        )
                    );
            })
        );
    }

    private void configureDisabledBehavior() {
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        RobotModeTriggers.disabled().onTrue(
            drivetrain.runOnce(() -> rotationLimiter.reset(0.0)).ignoringDisable(true)
        );
    }

    private void configureDriverBindings() {
        controller_drive.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        controller_drive.rightTrigger().whileTrue(new IntakeCommand(intake));
        controller_drive.leftTrigger().whileTrue(new OuttakeCommand(intake));
    }

    private void configureOperatorBindings() {
        controller_operator.leftTrigger().whileTrue(
            new ShootWithPowerCommand(turret, Constants.Turret.ShootConfig.SHORT_SPITTER_SPEED)
        );
        controller_operator.rightTrigger().whileTrue(
            new INTERMITENTShootCommand(turret, Constants.Turret.ShootConfig.SPITTER_SPEED)
        );

        controller_operator.x().whileTrue(new ShootAtAngleDRIFTCommand(turret, turret.verticalMotor.getAngle()));
        controller_operator.y().whileTrue(new ShootAtAngleSTUTTERCommand(turret, turret.verticalMotor.getAngle()));
        controller_operator.rightBumper().whileTrue(
            new ShootWithPowerCommand(turret, Constants.Turret.ShootConfig.MED_SPITTER_SPEED)
        );

        controller_operator.leftBumper().and(controller_operator.rightTrigger()).whileTrue(new ShootCommand(turret));
        controller_operator.leftBumper().whileTrue(new TurretAimChangeCommand(
            turret,
            () -> applyDeadband(controller_operator.getRightX(), Constants.Controller.TURRET_INPUT_DEADBAND),
            () -> applyDeadband(controller_operator.getLeftY(), Constants.Controller.TURRET_INPUT_DEADBAND)
        ));

        controller_operator.leftBumper().and(controller_operator.b()).whileTrue(
            new SpindexSpinCommand(turret, -1 * Constants.Turret.ShootConfig.SPINDEXER_SPEED)
        );
        controller_operator.leftBumper().and(controller_operator.a()).whileTrue(
            new KickerSpinCommand(turret, -1 * Constants.Turret.ShootConfig.KICKER_SPEED)
        );
        controller_operator.leftBumper().and(controller_operator.y()).whileTrue(
            new SpindexSpinCommand(turret, Constants.Turret.ShootConfig.SPINDEXER_SPEED)
        );
        controller_operator.leftBumper().and(controller_operator.x()).whileTrue(
            new KickerSpinCommand(turret, Constants.Turret.ShootConfig.KICKER_SPEED)
        );
        controller_operator.leftBumper().onTrue(new ElasticManualOverrideCommand(() -> true));
        controller_operator.leftBumper().onFalse(new ElasticManualOverrideCommand(() -> false));
    }

    private double shapeTurnInput(double rawTurn) {
        double deadbanded = MathUtil.applyDeadband(rawTurn, Constants.Controller.ROTATION_INPUT_DEADBAND);
        return Math.copySign(deadbanded * deadbanded, deadbanded);
    }

    private double applyDeadband(double rawInput, double deadband) {
        return MathUtil.applyDeadband(rawInput, deadband);
    }

    public Command getAutonomousCommand() {
        Command selectedAuto = autoChooser.getSelected();
        return selectedAuto != null ? selectedAuto : Commands.none();
    }

    public GyroSubsystem getGyroSubsystem() { return gyro; }
    public IntakeSubsystem getIntakeSubsystem() { return intake; }
    public VisionSubsystem getVisionSubsystem() { return vision; }
    public TurretSubsystem getTurretSubsystem() { return turret; }
    public CommandSwerveDrivetrain getDrivetrain() { return drivetrain; }
}
