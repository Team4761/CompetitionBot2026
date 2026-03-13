// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Set;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.autos.CloseRangeShoot3s;
import frc.robot.autos.DriveFwd2s;
import frc.robot.autos.ExtendDownMoveAndGather;
import frc.robot.autos.LongRangeShoot3s;
import frc.robot.autos.OutpostAuto;
import frc.robot.autos.Shoot3s;
import frc.robot.autos.ShootLongGoUnderTrenchIntakeFromMiddle;
import frc.robot.baseCommands.DoNothingCommand;
import frc.robot.commandGroups.TurretLockCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.JostleCommand;
import frc.robot.subsystems.intake.OuttakeCommand;
import frc.robot.subsystems.intake.JostleCommand;
import frc.robot.subsystems.turret.ElasticManualOverrideCommand;
import frc.robot.subsystems.turret.ElasticRetrieveDataCommand;
import frc.robot.subsystems.turret.KickerSpinCommand;
import frc.robot.subsystems.turret.ShootAtAngleAtSpeedCommand;
import frc.robot.subsystems.turret.ShootCommand;
import frc.robot.subsystems.turret.SpindexSpinCommand;
import frc.robot.subsystems.turret.TurretAimChangeCommand;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.DisenableTrackerCommand;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.intake.ExtendCommand;
import frc.robot.subsystems.intake.IntakeCommand;

public class RobotContainer {
    private static final double INTAKE_EXTEND_SPEED = 1;

    private static final IntakeSubsystem intake = new IntakeSubsystem();
    private static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private static final VisionSubsystem vision = new VisionSubsystem(drivetrain);
    private static final TurretSubsystem turret = new TurretSubsystem();
    private static final GyroSubsystem gyro = new GyroSubsystem();
    private static final Orchestra orchestra = new Orchestra("output.chrp");

    private double MaxSpeed = 0.55 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // practice-safe top speed cap
    private double MaxAngularRate = RotationsPerSecond.of(0.35).in(RadiansPerSecond); // reduced max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity) // Use closed-loop velocity control for smoother low-speed behavior
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController controller_drive = new CommandXboxController(0);
    private final CommandXboxController controller_operator = new CommandXboxController(1);
    private final SlewRateLimiter rotationLimiter =
        new SlewRateLimiter(Constants.Controller.ROTATION_SLEW_RATE_RAD_PER_SEC_SQ);

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // Tracks which intake state
    private boolean isIntakeExtended = false;

    public RobotContainer() {
        configureBindings();
        configAutos();
    }

    private void configureBindings() {
        turret.setDefaultCommand(new TurretLockCommand(turret, vision));

        // Driver controller bindings

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                double xInput = -1 * applyDeadband(controller_drive.getLeftY(), Constants.Controller.TRANSLATION_INPUT_DEADBAND);
                double yInput = -1 * applyDeadband(controller_drive.getLeftX(), Constants.Controller.TRANSLATION_INPUT_DEADBAND);
                double turnInput = shapeTurnInput(-1 * applyDeadband(controller_drive.getRightX(), Constants.Controller.ROTATION_INPUT_DEADBAND));
                return drive.withVelocityX(xInput * MaxSpeed)
                    .withVelocityY(yInput * MaxSpeed)
                    .withRotationalRate(rotationLimiter.calculate(turnInput * MaxAngularRate)); // smoothed turn request
            })
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        RobotModeTriggers.disabled().onTrue(
            drivetrain.runOnce(() -> rotationLimiter.reset(0.0)).ignoringDisable(true)
        );
         // Reset the field-centric heading on left bumper press.
        controller_drive.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        //run intake/outtake
        controller_drive.rightTrigger().whileTrue(new IntakeCommand(intake));
        controller_drive.leftTrigger().whileTrue(new OuttakeCommand(intake));
        
        // Operator controller bindings
        controller_operator.leftTrigger().whileTrue(new ShootAtAngleAtSpeedCommand(turret, 31, 1));

        // Manual Override
        controller_operator.leftBumper().whileTrue(new TurretAimChangeCommand(
            turret,
            () -> applyDeadband(controller_operator.getRightX(), Constants.Controller.TURRET_INPUT_DEADBAND),
            () -> applyDeadband(controller_operator.getLeftY(), Constants.Controller.TURRET_INPUT_DEADBAND)
        ));
        controller_operator.rightTrigger().whileTrue(new ShootAtAngleAtSpeedCommand(turret, 0, 1));
        controller_operator.leftBumper().and(controller_operator.rightTrigger()).whileTrue(new ShootCommand(turret)); // [BEN] please implement IgnoreSafeties here
        controller_operator.leftBumper().and(controller_operator.b()).whileTrue(new SpindexSpinCommand(turret, -Constants.Turret.ShootConfig.SPINDEXER_SPEED));
        controller_operator.leftBumper().and(controller_operator.a()).whileTrue(new KickerSpinCommand(turret, -Constants.Turret.ShootConfig.KICKER_SPEED));
        controller_operator.leftBumper().and(controller_operator.back()).whileTrue(new DisenableTrackerCommand(vision));
        controller_operator.leftBumper().onTrue(new ElasticManualOverrideCommand(() -> true));
        controller_operator.leftBumper().onFalse(new ElasticManualOverrideCommand(() -> false));
        controller_operator.a().whileTrue(new ElasticRetrieveDataCommand(turret));
        // Climber & Intake Extension
        
        //what we had previosly
        /*controller_operator.start().onTrue(new ExtendAtSpeedCommand(intake, -0.1)); // Go up when the start button is held, and go down when it's released
        controller_operator.start().onFalse(new ExtendAtSpeedCommand(intake, 0.1)); // Go up when the start button is held, and go down when it's released*/

        
        controller_operator.rightBumper().whileTrue(new JostleCommand(intake));
        controller_operator.start().onTrue(
            Commands.defer(
                () -> {
                    System.out.println(this.isIntakeExtended);
                    if (!this.isIntakeExtended) {
                        this.isIntakeExtended = true;
                        return new ExtendCommand(intake);
                    } else {
                        return new DoNothingCommand();
                    }
                },
                Set.of(intake)
            )
        );
       
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private double shapeTurnInput(double rawTurn) {
        double deadbanded = MathUtil.applyDeadband(rawTurn, Constants.Controller.ROTATION_INPUT_DEADBAND);
        return Math.copySign(deadbanded * deadbanded, deadbanded);
    }

    private double applyDeadband(double rawInput, double deadband) {
        return MathUtil.applyDeadband(rawInput, deadband);
    }

    private void configAutos() {
        /*
         * Current Autos we want to have:
         * Do Nothing  √
         * Shoot √
        * Shoot Long √  
         * Shoot, Go to Outpost, Shoot [TODO]
         * Go to Depot, Pickup, Shoot [TODO] (kind of done we need to test extend down move and gather)
         * Shoot Long, Go under Trench, Intake From Middle [TODO]
         */
        autoChooser.setDefaultOption("Do Nothing", Commands.none());
        autoChooser.addOption(
            "Long Range Shoot 3s",
            new LongRangeShoot3s(turret)
        );
        autoChooser.addOption(
            "Shoot 3s",
            new Shoot3s(turret)
        );
        autoChooser.addOption(
            "Close Range Shoot 3s",
            new CloseRangeShoot3s(turret)
        );
        autoChooser.addOption(
            "Drive for 2 seconds",
            new DriveFwd2s(drivetrain)
        );
        autoChooser.addOption(
            "Extend Down Move And Gather",
            new ExtendDownMoveAndGather(intake,drivetrain)
        );
        autoChooser.addOption(
            "Shoot Long Go Under Trench Intake From Middle",
            new ShootLongGoUnderTrenchIntakeFromMiddle(intake,drivetrain,turret)
        );
        autoChooser.addOption(
            "Outpost Auto",
            new OutpostAuto(turret, vision, drivetrain)
        );
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public static GyroSubsystem getGyroSubsystem() { return gyro; }
    public static IntakeSubsystem getIntakeSubsystem() { return intake; }
    public static VisionSubsystem getVisionSubsystem() { return vision; }
    public static TurretSubsystem getTurretSubsystem() { return turret; }
    public static CommandSwerveDrivetrain getDrivetrain() { return drivetrain; }
}
