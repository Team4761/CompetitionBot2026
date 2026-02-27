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
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.SmartKrakenMotor;
import frc.robot.util.SmartVortexMotor;

public class RobotContainer {
    private static final double ROTATION_INPUT_DEADBAND = 0.12;
    private static final double ROTATION_SLEW_RATE_RAD_PER_SEC_SQ = 3.0;
    private static final double TEST_VORTEX_OUTPUT = 0.20;
    private static final double TEST_KRAKEN_OUTPUT = 0.20;

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

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(ROTATION_SLEW_RATE_RAD_PER_SEC_SQ);
    private final SmartVortexMotor testVortexMotor = SmartVortexMotor.Builder.newInstance()
        .canId(25)
        .build();
    private final SmartKrakenMotor testKrakenMotor = SmartKrakenMotor.Builder.newInstance()
        .port(56)
        .PID(0.0, 0.0, 0.0)
        .outputRange(-1.0, 1.0)
        .angleLimits(-1.0, -1.0)
        .mode(SmartKrakenMotor.MotorMode.CONTINUOUS)
        .build();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                double turnInput = shapeTurnInput(-joystick.getRightX());
                return drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(rotationLimiter.calculate(turnInput * MaxAngularRate)); // smoothed turn request
            })
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        RobotModeTriggers.disabled().onTrue(drivetrain.runOnce(() -> rotationLimiter.reset(0.0)).ignoringDisable(true));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));
        joystick.y()
            .and(joystick.back().negate())
            .and(joystick.start().negate())
            .whileTrue(Commands.startEnd(
                () -> {
                    testVortexMotor.setSpeed(TEST_VORTEX_OUTPUT);
                    testKrakenMotor.setSpeed(TEST_KRAKEN_OUTPUT);
                },
                () -> {
                    testVortexMotor.stopTurning();
                    testKrakenMotor.stopTurning();
                }
            ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private double shapeTurnInput(double rawTurn) {
        double deadbanded = MathUtil.applyDeadband(rawTurn, ROTATION_INPUT_DEADBAND);
        return Math.copySign(deadbanded * deadbanded, deadbanded);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
