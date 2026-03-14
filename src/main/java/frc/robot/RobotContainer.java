// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.autos.competition.DepotCollectAuto;
import frc.robot.autos.competition.PreloadShootAuto;
import frc.robot.autos.competition.PreloadShootThenDepotCollectAuto;
import frc.robot.autos.competition.PreloadShootThenTrenchCollectAuto;
import frc.robot.autos.testing.CloseRangeShotTuningAuto;
import frc.robot.autos.testing.DriveForwardDistanceTestAuto;
import frc.robot.autos.testing.IntakeGatherTestAuto;
import frc.robot.autos.testing.LongRangeShotTuningAuto;
import frc.robot.autos.testing.OutpostLaneShotTestAuto;
import frc.robot.autos.testing.RelativePoseDriveTestAuto;
import frc.robot.autos.testing.ShootDurationTuningAuto;
import frc.robot.autos.testing.TurnInPlaceTestAuto;
import frc.robot.commandGroups.TurretLockCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.intake.IntakeCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.JostleCommand;
import frc.robot.subsystems.intake.OuttakeCommand;
import frc.robot.subsystems.turret.ElasticManualOverrideCommand;
import frc.robot.subsystems.turret.KickerSpinCommand;
import frc.robot.subsystems.turret.ShootAtAngleCommand;
import frc.robot.subsystems.turret.ShootAtAngleDRIFTCommand;
import frc.robot.subsystems.turret.ShootAtAngleSTUTTERCommand;
import frc.robot.subsystems.turret.ShootCommand;
import frc.robot.subsystems.turret.SpindexSpinCommand;
import frc.robot.subsystems.turret.TurretAimChangeCommand;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.DisenableTrackerCommand;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
    private static final double INTAKE_EXTEND_SPEED = 1;

    private static final IntakeSubsystem intake = new IntakeSubsystem();
    private static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private static final VisionSubsystem vision = new VisionSubsystem(drivetrain);
    private static final TurretSubsystem turret = new TurretSubsystem();
    private static final GyroSubsystem gyro = new GyroSubsystem();
    private static final Orchestra orchestra = new Orchestra("output.chrp");

    private double MaxSpeed = 0.55 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.35).in(RadiansPerSecond);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController controller_drive = new CommandXboxController(0);
    private final CommandXboxController controller_operator = new CommandXboxController(1);
    private final SlewRateLimiter rotationLimiter =
        new SlewRateLimiter(Constants.Controller.ROTATION_SLEW_RATE_RAD_PER_SEC_SQ);

    private final SendableChooser<Supplier<Command>> autoChooser = new SendableChooser<>();

    // Tracks which intake state
    private boolean isIntakeExtended = false;

    public RobotContainer() {
        configureBindings();
        configAutos();
        configDefaultCommands();
    }

    private void configureBindings() {
        //#region --- Robot Config ---

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double xInput = -1 * applyDeadband(controller_drive.getLeftY(), Constants.Controller.TRANSLATION_INPUT_DEADBAND);
                double yInput = -1 * applyDeadband(controller_drive.getLeftX(), Constants.Controller.TRANSLATION_INPUT_DEADBAND);
                double turnInput = shapeTurnInput(-1 * applyDeadband(controller_drive.getRightX(), Constants.Controller.ROTATION_INPUT_DEADBAND));
                return drive.withVelocityX(xInput * MaxSpeed)
                    .withVelocityY(yInput * MaxSpeed)
                    .withRotationalRate(rotationLimiter.calculate(turnInput * MaxAngularRate));
            })
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        RobotModeTriggers.disabled().onTrue(
            drivetrain.runOnce(() -> rotationLimiter.reset(0.0)).ignoringDisable(true)
        );
        //#endregion

        //#region --- Driver Controller Bindings ---

        controller_drive.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        controller_drive.rightTrigger().whileTrue(new IntakeCommand(intake));
        controller_drive.leftTrigger().whileTrue(new OuttakeCommand(intake));

        //#endregion

        //#region --- Operator Controller Bindings ---

        controller_operator.leftTrigger().whileTrue(new ShootAtAngleCommand(turret, 31.0));
        controller_operator.rightTrigger().whileTrue(new ShootAtAngleCommand(turret, 0.0));

        controller_operator.x().whileTrue(new ShootAtAngleDRIFTCommand(turret, this.turret.getVerticalAngle()));
        controller_operator.y().whileTrue(new ShootAtAngleSTUTTERCommand(turret, this.turret.getVerticalAngle()));

        controller_operator.rightBumper().whileTrue(new JostleCommand(intake));

        controller_operator.leftBumper().and(controller_operator.rightTrigger()).whileTrue(new ShootCommand(turret));

        controller_operator.leftBumper().whileTrue(new TurretAimChangeCommand(
            turret,
            () -> applyDeadband(controller_operator.getRightX(), Constants.Controller.TURRET_INPUT_DEADBAND),
            () -> applyDeadband(controller_operator.getLeftY(), Constants.Controller.TURRET_INPUT_DEADBAND)
        ));

        controller_operator.leftBumper().and(controller_operator.b()).whileTrue(new SpindexSpinCommand(turret, -1 * Constants.Turret.ShootConfig.SPINDEXER_SPEED));
        controller_operator.leftBumper().and(controller_operator.a()).whileTrue(new KickerSpinCommand(turret, -1 * Constants.Turret.ShootConfig.KICKER_SPEED));
        controller_operator.leftBumper().and(controller_operator.back()).whileTrue(new DisenableTrackerCommand(vision));
        controller_operator.leftBumper().onTrue(new ElasticManualOverrideCommand(() -> true));
        controller_operator.leftBumper().onFalse(new ElasticManualOverrideCommand(() -> false));

        //#endregion

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configDefaultCommands() {
        turret.setDefaultCommand(new TurretLockCommand(turret, vision));
    }

    private double shapeTurnInput(double rawTurn) {
        double deadbanded = MathUtil.applyDeadband(rawTurn, Constants.Controller.ROTATION_INPUT_DEADBAND);
        return Math.copySign(deadbanded * deadbanded, deadbanded);
    }

    private double applyDeadband(double rawInput, double deadband) {
        return MathUtil.applyDeadband(rawInput, deadband);
    }

    private void configAutos() {
        autoChooser.setDefaultOption("COMP - Do Nothing", Commands::none);
        autoChooser.addOption(
            "COMP - Preload Shoot",
            () -> new PreloadShootAuto(turret)
        );
        autoChooser.addOption(
            "COMP - Preload Shoot + Depot Collect",
            () -> new PreloadShootThenDepotCollectAuto(intake, drivetrain, turret)
        );
        autoChooser.addOption(
            "COMP - Depot Collect",
            () -> new DepotCollectAuto(intake, drivetrain)
        );
        autoChooser.addOption(
            "COMP - Preload Shoot + Trench Collect",
            () -> new PreloadShootThenTrenchCollectAuto(intake, drivetrain, turret)
        );
        autoChooser.addOption(
            "TEST - Long range shot tuning",
            () -> new LongRangeShotTuningAuto(turret)
        );
        autoChooser.addOption(
            "TEST - Close range shot tuning",
            () -> new CloseRangeShotTuningAuto(turret)
        );
        autoChooser.addOption(
            "TEST - Shoot duration tuning",
            () -> new ShootDurationTuningAuto(turret)
        );
        autoChooser.addOption(
            "TEST - Drive forward distance",
            () -> new DriveForwardDistanceTestAuto(drivetrain)
        );
        autoChooser.addOption(
            "TEST - Turn in place",
            () -> new TurnInPlaceTestAuto(drivetrain)
        );
        autoChooser.addOption(
            "TEST - Relative pose example",
            () -> new RelativePoseDriveTestAuto(drivetrain)
        );
        autoChooser.addOption(
            "TEST - Intake gather",
            () -> new IntakeGatherTestAuto(intake, drivetrain)
        );
        autoChooser.addOption(
            "TEST - Outpost lane / shot experiment",
            () -> new OutpostLaneShotTestAuto(turret, drivetrain)
        );
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        Supplier<Command> selectedAuto = autoChooser.getSelected();
        return selectedAuto != null ? selectedAuto.get() : Commands.none();
    }

    public static GyroSubsystem getGyroSubsystem() { return gyro; }
    public static IntakeSubsystem getIntakeSubsystem() { return intake; }
    public static VisionSubsystem getVisionSubsystem() { return vision; }
    public static TurretSubsystem getTurretSubsystem() { return turret; }
    public static CommandSwerveDrivetrain getDrivetrain() { return drivetrain; }
}
