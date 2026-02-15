package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.DisenableTrackerCommand;

import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.*;

public class RobotContainer {
    // Subsystems
    private final CommandXboxController controller;

    private final SlewRateLimiter xlimiter;
    private final SlewRateLimiter ylimiter;

    private static final ClimberSubsystem climber = new ClimberSubsystem();
    private static final IntakeSubsystem intake = new IntakeSubsystem();
    private static final SwerveSubsystem swerve = new SwerveSubsystem();
    private static final VisionSubsystem vision = new VisionSubsystem();
    private static final TurretSubsystem turret = new TurretSubsystem();

    public RobotContainer() {
        controller = new CommandXboxController(Constants.CONTROLLER_PORT);
        xlimiter = new SlewRateLimiter(10);
        ylimiter = new SlewRateLimiter(10);

        configBindings();
        configDefaultCmds();
    }

    public void configDefaultCmds() {
        int multiplier = 2;
        swerve.setDefaultCommand(
                swerve.driveCommand(
                        () -> xlimiter.calculate(MathUtil.applyDeadband(controller.getLeftY(), 0.08) * multiplier),
                        () -> ylimiter.calculate(MathUtil.applyDeadband(controller.getLeftX(), 0.08) * multiplier),
                        () -> MathUtil.applyDeadband(controller.getRightX(), 0.08)
                )
        );
    }

    private void configBindings() {
        if (swerve != null) {
            // Reset 'forwards' direction of robot when in operator relative mode
            controller.x().onTrue(swerve.reZeroCommand());
        }

        if (vision != null) {
            // Tracking
            controller.b().onTrue(new DisenableTrackerCommand(vision));
        }
    }

    public static ClimberSubsystem getClimberSubsystem() { return climber; }
    public static IntakeSubsystem getIntakeSubsystem() { return intake; }
    public static SwerveSubsystem getSwerveSubsystem() { return swerve; }
    public static VisionSubsystem getVisionSubsystem() { return vision; }
    public static TurretSubsystem getTurretSubsystem() { return turret; }
}
