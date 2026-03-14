package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.VecBuilder;
import java.util.Optional;
import java.util.stream.Collectors;
import java.lang.reflect.Method;

public class Vision extends SubsystemBase {

  private final SwerveSubsystem swerve;
  private final PhotonCamera camera = new PhotonCamera("Limelight 4");
  private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
  private final PhotonPoseEstimator photonPoseEstimator;

  public Vision(SwerveSubsystem swerve) {
    this.swerve = swerve;

    // Construct PhotonPoseEstimator via reflection to prefer the newer
    // (fieldLayout, strategy, camera, robotToCamera) constructor if present.
    PhotonPoseEstimator tmp = null;
    try {
      var ctor4 = PhotonPoseEstimator.class.getConstructor(
          AprilTagFieldLayout.class,
          PoseStrategy.class,
          PhotonCamera.class,
          edu.wpi.first.math.geometry.Transform3d.class);
      tmp = (PhotonPoseEstimator) ctor4.newInstance(fieldLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          camera,
          Constants.kRobotToCam);
    } catch (NoSuchMethodException e) {
      try {
        // Fall back to (AprilTagFieldLayout, PoseStrategy, Transform3d)
        var ctor3 = PhotonPoseEstimator.class.getConstructor(
            AprilTagFieldLayout.class,
            PoseStrategy.class,
            edu.wpi.first.math.geometry.Transform3d.class);
        tmp = (PhotonPoseEstimator) ctor3.newInstance(fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            Constants.kRobotToCam);
      } catch (NoSuchMethodException e2) {
        try {
          // Last fallback: (AprilTagFieldLayout, Transform3d)
          var ctor2 = PhotonPoseEstimator.class.getConstructor(
              AprilTagFieldLayout.class,
              edu.wpi.first.math.geometry.Transform3d.class);
          tmp = (PhotonPoseEstimator) ctor2.newInstance(fieldLayout,
              Constants.kRobotToCam);
        } catch (Exception e3) {
          throw new RuntimeException("Failed to construct PhotonPoseEstimator", e3);
        }
      } catch (Exception ex3) {
        throw new RuntimeException(ex3);
      }
    } catch (Exception ex) {
      throw new RuntimeException(ex);
    }
    this.photonPoseEstimator = tmp;
  }

  @Override
  @SuppressWarnings("unchecked")
  public void periodic() {
    // Correct way to handle multiple results in 2026.2.1
    var allResults = camera.getAllUnreadResults();

    for (var res : allResults) {
      // Use reflection to prefer estimateFieldToRobot(...) and avoid deprecation warnings.
      // This allows the code to work with multiple PhotonVision vendordeps releases.
      // 
      // Note: This currently applies every unread result. For higher performance or 
      // accuracy, you might want to add gating (e.g. min target count, Max Reprojection Error)
      // or only apply the result with the most targets / lowest error.
      Optional<EstimatedRobotPose> maybe = Optional.empty();
      try {
        Method m = photonPoseEstimator.getClass().getMethod("estimateFieldToRobot", res.getClass());
        maybe = (Optional<EstimatedRobotPose>) m.invoke(photonPoseEstimator, res);
      } catch (NoSuchMethodException nsme) {
        try {
          Method m2 = photonPoseEstimator.getClass().getMethod("update", res.getClass());
          maybe = (Optional<EstimatedRobotPose>) m2.invoke(photonPoseEstimator, res);
        } catch (Exception ignored) {}
      } catch (Exception ignored) {}

      maybe.ifPresent(est -> {
        var targets = res.getTargets();
        if (targets.isEmpty()) return;

        // Calculate average distance to targets for scaling trust
        double sumDistance = 0;
        for (var target : targets) {
          sumDistance += target.getBestCameraToTarget().getTranslation().getNorm();
        }
        double avgDistance = sumDistance / targets.size();

        // Hard Gating: Skip if targets are too far away or if we see no tags
        if (targets.isEmpty() || avgDistance > 6.0) return;

        // Gating/Confidence: Trust vision less (higher std dev) as distance increases
        // Or if we only see one tag.
        double confidenceMultiplier = (targets.size() > 1) ? 0.5 : 1.0;
        
        // Standard Deviation formula: Base + (factor * distance^2)
        // Adjust these constants (0.1, 0.4) based on testing
        double stdDevEntry = 0.1 + (0.4 * Math.pow(avgDistance, 2) * confidenceMultiplier);

        Pose2d pose2d = est.estimatedPose.toPose2d();
        
        swerve.getSwerveDrive().addVisionMeasurement(
            pose2d, 
            est.timestampSeconds,
            VecBuilder.fill(stdDevEntry, stdDevEntry, 0.9)
        );

        SmartDashboard.putString("Vision/EstimatedPose", pose2d.toString());
        SmartDashboard.putNumber("Vision/AvgDistance", avgDistance);
        SmartDashboard.putNumber("Vision/StdDev", stdDevEntry);
        
        String ids = targets.stream()
            .map(t -> Integer.toString(t.getFiducialId()))
            .collect(Collectors.joining(","));
        SmartDashboard.putString("Vision/DetectedTagIDs", ids);
      });
    }
  }

}
