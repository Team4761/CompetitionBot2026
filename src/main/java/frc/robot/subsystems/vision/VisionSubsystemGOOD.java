package frc.robot.subsystems.vision;

import java.util.List;
//make photon vision contstants in constants[TODO][FIXME] [HELP]
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class VisionSubsystemGOOD {
    private CommandSwerveDrivetrain drivetrain;
    private AprilTagFieldLayout layout;

    private Pose2d visionPose;
    private final Field2d visionField = new Field2d();//dont know how to use but is definitly helpful
   private final Field2d generalField = new Field2d();

    PhotonCamera camera = new PhotonCamera("Camera");
    private PhotonPipelineResult result = null;
    private PhotonTrackedTarget trackedTarget = null;
    private PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded), //load the april tags
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,//idk
        Constants.Vision.CAMARA_POS //make camera awarE of rest of robot
    );



    private EstimatedRobotPose leftEstimatedRobotPose = null;
    private List<TargetCorner> leftCorners = null;
    public VisionSubsystemGOOD(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.layout = AprilTagFields.k2026RebuiltWelded.loadAprilTagLayoutField();

    }

    /*
        Things we need :
            constants
                we need the positions of the camera on the robot and offsets
                we need april tag ids to look out for
            commands/things
                change angle of robot/ turret to point at april tag at deisred angle inputs(deiredd angle. april tag)
                we need to know where the camera thinks it is and how that compares to odometry
                we need 

    */

    public void addVisionPose2d(Pose2d pose2d, double timestampSeconds) {
        SmartDashboard.putNumber("aVP2d pose2d X", pose2d.getX());
        SmartDashboard.putNumber("aVP2d pose2d Y", pose2d.getY());
        SmartDashboard.putNumber("aVP2d pose2d Rot", pose2d.getRotation().getDegrees());
        SmartDashboard.putNumber("aVP2d timestampSeconds", timestampSeconds);
        // Sets trust value for vision measurements
        // charizardsSkateboard.setVisionMeasurementStdDevs(curStdDevs);
        // charizardsSkateboard.addVisionMeasurement(pose2d, timestampSeconds);
        //drivetrain.setVisionMeasurementStdDevs(kSingleTagStdDevs);
        /*
        double xUpperLimitOfTrustBox = m_drivetrain.getState().Pose.getX() + RadiusOfToleranceSquare;
        double xLowerLimitOfTrustBox = m_drivetrain.getState().Pose.getX() - RadiusOfToleranceSquare;
        double yUpperLimitOfTrustBox = m_drivetrain.getState().Pose.getY() + RadiusOfToleranceSquare;
        double yLowerLimitOfTrustBox = m_drivetrain.getState().Pose.getY() - RadiusOfToleranceSquare;
     
  
        //  SmartDashboard.putNumber("charizardsSkateboard X", charizardsSkateboard.getState().Pose.getX());
        //  SmartDashboard.putNumber("charizardsSkateboard Y", charizardsSkateboard.getState().Pose.getY());
        //  SmartDashboard.putNumber("charizardsSkateboard Rot", charizardsSkateboard.getState().Pose.getRotation().getDegrees());
        if (MathUtil.isNear(0, m_drivetrain.getState().Speeds.vxMetersPerSecond, 0.1) && MathUtil.isNear(0, m_drivetrain.getState().Speeds.vyMetersPerSecond, 0.1)) {
        //are we moving,, if so then add trust box  
        m_drivetrain.addVisionMeasurement(pose2d, timestampSeconds);
        } else if(pose2d.getX() >= xLowerLimitOfTrustBox && pose2d.getX() <= xUpperLimitOfTrustBox && pose2d.getY() >= yLowerLimitOfTrustBox && pose2d.getY() <= yUpperLimitOfTrustBox) {
        m_drivetrain.addVisionMeasurement(pose2d, timestampSeconds);
        } */

     drivetrain.addVisionMeasurement(pose2d, Utils.getCurrentTimeSeconds());
   }

   
   public void periodic() {
     // Get camera results
     result = camera.getLatestResult();
     
     // backLeftResult = backLeftCamera.getLatestResult();
     // backRightResult = backRightCamera.getLatestResult(); //NEEDS CHANGING BEFORE WE RETIRE FOR MICHALS SAFTEY NEXT YEAR
     // Central Camera
     // Try to update "latestRobotPose" with a new "EstimatedRobotPose" using a "PhotonPoseEstimator"
     // If "latestRobotPose" is updated, call addVisionPose2d() and pass the updated "latestRobotPose" as an argument
     try {
       // Only accepts camera results if they see more than 1 april tag, or if it sees 1 april tag and the poseAmbiguity is low
       // COMMENT OUT THE LINE BELOW THIS AND IT'S CLOSING BRACKETS IF THIS DOESN'T WORK
       if ((result.getTargets().size() == 1 && result.getBestTarget().poseAmbiguity < PhotonVisionConstants.AMBIGUITY_RATIO_CUTOFF) 
    ||   result.getTargets().size() > 1) {
         leftEstimatedRobotPose = poseEstimator.update(result).get();
         //leftEstimatedRobotPose = leftPoseEstimator.estimateCoprocMultiTagPose(leftResult).get();
         //updateEstimationStdDevs(leftPoseEstimator.update(leftResult), cameraLeft.getAllUnreadResults().get(0).getTargets());
         addVisionPose2d(leftEstimatedRobotPose.estimatedPose.toPose2d(), Utils.getCurrentTimeSeconds());
         visionField.setRobotPose(leftEstimatedRobotPose.estimatedPose.toPose2d());
       }
     } catch (Exception e) {
       leftEstimatedRobotPose = null;
     }
     // Same thing but for the left camera
     try {
       // Only accepts camera results if they see more than 1 april tag, or if it sees 1 april tag and the poseAmbiguity is low
       // COMMENT OUT THE LINE BELOW THIS AND IT'S CLOSING BRACKETS IF THIS DOESN'T WORK
     } catch (Exception e) {
       forwardEstimatedRobotPose = null;
       SmartDashboard.putBoolean("forwardLatestRobotPose Update", false);
     }
     SmartDashboard.putData("Vision Pose Estimation", visionField);
     generalField.setRobotPose(drivetrain.getState().Pose);
     SmartDashboard.putData("Robot Pose", generalField);
     
     try {
       SmartDashboard.putNumber("Angle To Hub", Math.toDegrees(calculateAngleToHub()));//use function we already hav instead of this
       SmartDashboard.putNumber("Angle of Robot", getEstimatedPose().getRotation().getDegrees());
       SmartDashboard.putNumber("Pose X", getEstimatedPose().getX());
       SmartDashboard.putNumber("Pose Y", getEstimatedPose().getY());
     } catch (Exception e) {
     }
   }
   
   private double calculateCornersAvgX() {
     return (leftCorners.get(0).x + leftCorners.get(1).x + leftCorners.get(2).x 
     + leftCorners.get(3).x) /4;
   }

   public PhotonTrackedTarget getTarget(){
     return forwardTrackedTarget;
   }

   public double getGenericAngle() {
     return pointAtGeneralController.calculate(drivetrain.getState().Pose.getRotation().getDegrees(), 90);
   }

   public Pose2d getEstimatedPose() {
     return drivetrain.getState().Pose;
   }

    //cahnge to be using odometry instead of viosion to simplify the snap to command move to odometry subsystem
   /*
   public double calculateAngleToHub() {
     try {
       var alliance = DriverStation.getAlliance();
       double xRelativeToHub = 1;
       double yRelativeToHub = 1;
       // double[] turretPosition = calculateTurretFieldPosition();
       if(alliance.get() == Alliance.Blue) {
         xRelativeToHub = getEstimatedPose().getX() - PoseConstants.BLUE_ALLIANCE_HUB_LOCATION[0];
         yRelativeToHub = getEstimatedPose().getY() - PoseConstants.BLUE_ALLIANCE_HUB_LOCATION[1];
       }
       else if(alliance.get() == Alliance.Red) {
         xRelativeToHub = getEstimatedPose().getX() - PoseConstants.RED_ALLIANCE_HUB_LOCATION[0];
         yRelativeToHub = getEstimatedPose().getY() - PoseConstants.RED_ALLIANCE_HUB_LOCATION[1];
       }
       SmartDashboard.putNumber("XRelativeToHub", xRelativeToHub);
       SmartDashboard.putNumber("YRelativeToHub", yRelativeToHub);
       angleToHub = Math.atan(yRelativeToHub/xRelativeToHub);
       return angleToHub;
     } catch (Exception e) {
    
       return angleToHub;
     }
   }
   
   
   public double[] calculateTurretFieldPosition() {
     // Vector A is the robot position
     double[] a = {getEstimatedPose().getX(), getEstimatedPose().getY()};
     // Vector B is the position of the turret in the robot
     // Compensates for the initial angle of the turret relative to the robot
     double[] b = {Math.cos(getEstimatedPose().getRotation().getRadians()) * TurretConstants.ROBOT_RELATIVE_TURRET_MAGNITUDE,
                   Math.sin(getEstimatedPose().getRotation().getRadians()) * TurretConstants.ROBOT_RELATIVE_TURRET_MAGNITUDE};
  
     // Adding the two together gets you the turret's position on the field
     double[] c = {a[0] + b[0], a[1] + b[1]};
     return c;
   }
 }
   */
}

