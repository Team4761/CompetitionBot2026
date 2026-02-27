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
import frc.robot.subsystems.turret.TurretManualAimCommand;
import frc.robot.subsystems.turret.ShootCommand;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.SmartCameraNetwork;
import frc.robot.util.SmartKrakenMotor;

public class RobotContainer {
    private final CommandXboxController controller = new CommandXboxController(Constants.CONTROLLER_PORT);
    // NEW: Operator controller for specialized turret/shooter control
    private final CommandXboxController operatorController = new CommandXboxController(Constants.OPERATOR_CONTROLLER_PORT);

    private final SwerveSubsystem swerve = new SwerveSubsystem();
    private final VisionSubsystem vision = new VisionSubsystem();
    private final TurretSubsystem turret = new TurretSubsystem();
    private final SmartCameraNetwork cameraNetwork = new SmartCameraNetwork();
    
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        configDefaultCommands();
        configBindings();
        configDefaultCmds();
        configAutos();
    }

    private void configDefaultCommands() {
        // Set the default command for the turret to allow manual aiming via operator joysticks
        if (turret != null) {
            turret.setDefaultCommand(new TurretManualAimCommand(
                turret,
                () -> MathUtil.applyDeadband(operatorController.getLeftY(), 0.08),  // Left Y: AOA
                () -> MathUtil.applyDeadband(operatorController.getRightX(), 0.08)  // Right X: Turn
            ));
        }

        /* swerve.setDefaultCommand(
                new SwerveDriveCommand(
                        swerve,
                        () -> MathUtil.applyDeadband(controller.getLeftX() * multiplier, 0.08),
                        () -> MathUtil.applyDeadband(controller.getLeftY() * multiplier, 0.08),
                        () -> MathUtil.applyDeadband(controller.getRightX(), 0.08)
                )
        );
        */
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
            
            // NEW: Operator command to shoot (Spitter + Kicker)
            operatorController.rightTrigger()
                .whileTrue(new ShootCommand(turret, Constants.Turret.SPITTER_SPEED, Constants.Turret.KICKER_SPEED));
        }
    }

    private void configAutos() {
        autoChooser.setDefaultOption("Do Nothing", Commands.none());
        autoChooser.addOption(
            "Test move",
            new DriveFwd2s(swerve)
        );

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    //public static ClimberSubsystem getClimberSubsystem() { return climber; }
    //public static GyroSubsystem getGyroSubsystem() { return gyro; }
    //public static IntakeSubsystem getIntakeSubsystem() { return intake; }
    public static SwerveSubsystem getSwerveSubsystem() { return swerve; }
    public static VisionSubsystem getVisionSubsystem() { return vision; }
    public static TurretSubsystem getTurretSubsystem() { return turret; }
    public static SwerveSubsystem getSwerveSubsystem() { return null; /* Add reference */ }

}