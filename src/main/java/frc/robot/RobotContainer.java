package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.DisenableTrackerCommand;
import frc.robot.Constants.Gyro;
import frc.robot.autos.DriveFwd2s;
import frc.robot.commandGroups.FireFromSpindexer;
import frc.robot.commandGroups.TurretTrackCommand;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.turret.SpindexSpinCommand;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.SmartCameraNetwork;

public class RobotContainer {
    private final CommandXboxController controller;
    private final SlewRateLimiter xlimiter;
    private final SlewRateLimiter ylimiter;
    private final SmartCameraNetwork cameraNetwork;
    private final SendableChooser<Command> autoChooser;

    private static final ClimberSubsystem climber = new ClimberSubsystem();
    private static final GyroSubsystem gyro = new GyroSubsystem();
    private static final IntakeSubsystem intake = new IntakeSubsystem();
    private static final SwerveSubsystem swerve = new SwerveSubsystem();
    private static final VisionSubsystem vision = new VisionSubsystem();
    private static final TurretSubsystem turret = new TurretSubsystem();

    public RobotContainer() {
        controller = new CommandXboxController(Constants.CONTROLLER_PORT);
        xlimiter = new SlewRateLimiter(10);
        ylimiter = new SlewRateLimiter(10);
        cameraNetwork = SmartCameraNetwork.Builder.newInstance().build();
        autoChooser = new SendableChooser<>();

        configBindings();
        configDefaultCmds();
        configAutos();
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
            controller.x().onTrue(swerve.reZeroCommand());
        }

        if (vision != null) {
            controller.b().onTrue(new DisenableTrackerCommand(vision));
        }

        if (turret != null) {
            controller.y().whileTrue(new FireFromSpindexer(turret));
            controller.rightBumper()
                .whileTrue(new TurretTrackCommand(turret, cameraNetwork, Constants.Vision.TRACKED_TAG_ID));
        }
    }

    private void configAutos() {
        autoChooser.setDefaultOption("Do Nothing", Commands.none());
        autoChooser.addOption(
            "Drive Forward (2s)",
            new DriveFwd2s(swerve)
        );

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public static ClimberSubsystem getClimberSubsystem() { return climber; }
    public static GyroSubsystem getGyroSubsystem() { return gyro; }
    public static IntakeSubsystem getIntakeSubsystem() { return intake; }
    public static SwerveSubsystem getSwerveSubsystem() { return swerve; }
    public static VisionSubsystem getVisionSubsystem() { return vision; }
    public static TurretSubsystem getTurretSubsystem() { return turret; }
}
