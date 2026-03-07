// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.autos.DriveFwd2s;
import frc.robot.autos.ExtendDownMoveAndGather;
import frc.robot.autos.Shoot4s;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.OuttakeCommand;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.turret.ShootCommand;
import frc.robot.subsystems.turret.TurretAimChangeCommand;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.intake.ExtendAtSpeedCommand;
import frc.robot.subsystems.intake.IntakeCommand;

public class RobotContainer {
    private static final IntakeSubsystem intake = new IntakeSubsystem();
    private static final CommandSwerveDrivetrain swerve = TunerConstants.createDrivetrain();
    private static final VisionSubsystem vision = new VisionSubsystem();
    private static final TurretSubsystem turret = new TurretSubsystem();
    private static final GyroSubsystem gyro = new GyroSubsystem();
    private static final ClimberSubsystem climber = new ClimberSubsystem();

    private double MaxSpeed = 0.55 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // practice-safe top speed cap
    private double MaxAngularRate = RotationsPerSecond.of(0.35).in(RadiansPerSecond); // reduced max angular velocity
    //private double TurretMaxAngularRate = RotationsPerSecond.of

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity) // Use closed-loop velocity control for smoother low-speed behavior
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController controller_drive = new CommandXboxController(0);
    private final CommandXboxController controller_turret = new CommandXboxController(1);
    //private final CommandXboxController controller_operator = new CommandXboxController(1);
    private final SlewRateLimiter rotationLimiter =
        new SlewRateLimiter(Constants.Controller.ROTATION_SLEW_RATE_RAD_PER_SEC_SQ);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();
        configAutos();
    }



    private void configureBindings() {
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

        // Driver controller bindings
        controller_drive.a().whileTrue(drivetrain.applyRequest(() -> brake));
        controller_drive.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(
                -1 * applyDeadband(controller_drive.getLeftY(), Constants.Controller.TRANSLATION_INPUT_DEADBAND),
                -1 * applyDeadband(controller_drive.getLeftX(), Constants.Controller.TRANSLATION_INPUT_DEADBAND)))
        ));
        controller_drive.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        controller_drive.y().whileTrue(new IntakeCommand(intake));
        controller_drive.x().whileTrue(new OuttakeCommand(intake));

        // Operator controller bindings
        controller_turret.rightTrigger().whileTrue(new ShootCommand(turret));
        
        turret.setDefaultCommand(new TurretAimChangeCommand(
            turret,
            () -> applyDeadband(controller_turret.getRightX(), Constants.Controller.TURRET_INPUT_DEADBAND),
            () -> applyDeadband(controller_turret.getLeftY(), Constants.Controller.TURRET_INPUT_DEADBAND)));
        controller_turret.start().whileTrue(new ExtendAtSpeedCommand(intake, -0.5));
        
        // Reset the field-centric heading on left bumper press.
        

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
         * Do Nothing 
         * Shoot
         * Shoot Long
         * Shoot, Go to Corner, Shoot [TODO]
         * Go to Depot, Pickup, Shoot [TODO]
         * Shoot Long, Go under Trench, Intake From Middle [TODO]
         * Climb (maybe) [TODO]
         */
        autoChooser.setDefaultOption("Do Nothing", Commands.none());
        autoChooser.addOption(
            "Test move",
            new DriveFwd2s(swerve)
        );
        autoChooser.addOption(
            "Shoot",
            new ShootCommand(turret)
        );
        autoChooser.addOption(
            "Drive for 2 seconds",
            new DriveFwd2s(swerve)
        );
        autoChooser.addOption(
            "Shoot4s",
            new Shoot4s(turret)
        );
        autoChooser.addOption(
            "Extand Down Move And Gather",
            new ExtendDownMoveAndGather(intake,swerve)
        );

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public static ClimberSubsystem getClimberSubsystem() { return climber; }
    public static GyroSubsystem getGyroSubsystem() { return gyro; }
    public static IntakeSubsystem getIntakeSubsystem() { return intake; }
    public static VisionSubsystem getVisionSubsystem() { return vision; }
    public static TurretSubsystem getTurretSubsystem() { return turret; }
    public static CommandSwerveDrivetrain getDrivetrain() { return swerve; }
}
