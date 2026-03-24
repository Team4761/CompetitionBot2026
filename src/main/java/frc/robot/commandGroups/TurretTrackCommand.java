// package frc.robot.commandGroups;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.subsystems.turret.*;
// import frc.robot.util.SmartCameraNetwork;
// import frc.robot.util.SmartCameraNetwork.TargetObservation;
// import java.util.Optional;

// public class TurretTrackCommand extends Command {
//     private final TurretSubsystem turretSubsystem;
//     private final SmartCameraNetwork cameraNetwork;
//     private final int fiducialId;
//     private final SlewRateLimiter angleStepLimiter;

//     public TurretTrackCommand(TurretSubsystem turretSubsystem, SmartCameraNetwork cameraNetwork, int fiducialId) {
//         this.turretSubsystem = turretSubsystem;
//         this.cameraNetwork = cameraNetwork;
//         this.fiducialId = fiducialId;
//         this.angleStepLimiter = new SlewRateLimiter(Constants.Turret.Horizontal.MAX_TRACK_RATE_DEGREES_PER_SEC);
//         addRequirements(turretSubsystem);
//     }

//     @Override
//     public void initialize() {
//         angleStepLimiter.reset(0.0);
//     }

//     @Override
//     public void execute() {
//         Optional<TargetObservation> observation = cameraNetwork.getBestObservation(fiducialId);

//         if (observation.isEmpty()) {
//             turretSubsystem.setHorizontalMotor(0.0);
//             angleStepLimiter.reset(0.0);
//             return;
//         }

//         double angleError = observation.get().getAngleDegrees();
//         double desiredStep = 0.0;

//         if (Math.abs(angleError) > Constants.Vision.ANGLE_DEADBAND) {
//             desiredStep = MathUtil.clamp(
//                 angleError * Constants.Turret.Horizontal.ANGLE_TURN_PERCENTAGE,
//                 -Constants.Turret.Horizontal.MAX_TRACK_STEP_DEGREES,
//                 Constants.Turret.Horizontal.MAX_TRACK_STEP_DEGREES
//             );
//         }

//         turretSubsystem.turnHorizontalMotor(angleStepLimiter.calculate(desiredStep));
//     }

//     @Override
//     public boolean isFinished() { return false; }

//     @Override
//     public void end(boolean isInterrupted) {
//         turretSubsystem.stopHorizontal();
//     }
// }
