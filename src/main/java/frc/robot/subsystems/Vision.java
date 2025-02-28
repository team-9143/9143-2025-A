package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
    private Swerve swerve;

    private boolean trackingEnabled = false;
    private boolean positionTrackingEnabled = true;
    
    // Shuffleboard
    private final ShuffleboardTab visionTab;

    public Vision(Swerve swerve) {
        this.swerve = swerve;
        
        // Initialize Shuffleboard tab
        visionTab = Shuffleboard.getTab("Vision");
        
        // Configure Shuffleboard
        configureShuffleboard();

        // Set all Limelights to AprilTag pipeline (Pipeline 0)
        for (String name : VisionConstants.LIMELIGHT_NAMES) {
            LimelightHelpers.setPipelineIndex("limelight-" + name, 0);
        }
    }
    
    private void configureShuffleboard() {
        // Status layout - general vision status information
        visionTab.addBoolean("Tracking Enabled", this::isTrackingEnabled)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withSize(3, 2)
            .withPosition(0, 0);

        visionTab.addBoolean("Target Detected", () -> getBestTarget().isPresent())
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withSize(3, 2)
            .withPosition(0, 2);

        // Target information layout - main data about the detected target
        visionTab.addNumber("Best Tag ID", () -> getBestTarget().map(target -> (double)target.id).orElse(-1.0))
            .withWidget(BuiltInWidgets.kTextView)
            .withSize(3, 2)
            .withPosition(3, 0);

        visionTab.addNumber("Best Tag Distance", () -> {
                return getBestTarget().map(target -> Math.hypot(target.poseX, target.poseY)).orElse(0.0);
            })
            .withWidget(BuiltInWidgets.kDial)
            .withSize(3, 2)
            .withPosition(3, 2);

        // Target position and orientation layout - detailed positioning data
        visionTab.addNumber("Target Yaw (tx)", () -> getBestTarget().map(target -> target.tx).orElse(0.0))
            .withWidget(BuiltInWidgets.kTextView)
            .withSize(3, 2)
            .withPosition(6, 0);

        visionTab.addNumber("Target Pitch (ty)", () -> getBestTarget().map(target -> target.ty).orElse(0.0))
            .withWidget(BuiltInWidgets.kTextView)
            .withSize(3, 2)
            .withPosition(6, 2);

        visionTab.addNumber("Target Pose X", () -> getBestTarget().map(target -> target.poseX).orElse(0.0))
            .withWidget(BuiltInWidgets.kTextView)
            .withSize(3, 2)
            .withPosition(9, 0);

        visionTab.addNumber("Target Pose Y", () -> getBestTarget().map(target -> target.poseY).orElse(0.0))
            .withWidget(BuiltInWidgets.kTextView)
            .withSize(3, 2)
            .withPosition(9, 2);

        // Offsets and errors layout - tracking performance metrics
        visionTab.addNumber("Target Horizontal Offset", () -> {
                return getBestTarget().map(target -> getTargetHorizontalOffset(target.id)).orElse(0.0);
            })
            .withWidget(BuiltInWidgets.kTextView)
            .withSize(3, 2)
            .withPosition(12, 0);

        // Error tracking for PID feedback visualization
        visionTab.addNumber("Distance Error", () -> {
                return getBestTarget().map(target -> {
                    double distance = Math.hypot(target.poseX, target.poseY);
                    double targetVerticalOffset = getTargetVerticalOffset(target.id);
                    return distance - targetVerticalOffset;
                }).orElse(0.0);
            })
            .withWidget(BuiltInWidgets.kGraph)
            .withSize(5, 2)
            .withPosition(0, 4);

        visionTab.addNumber("Horizontal Error", () -> {
                return getBestTarget().map(target -> {
                    double targetHorizontalOffset = getTargetHorizontalOffset(target.id);
                    return target.poseY - targetHorizontalOffset;
                }).orElse(0.0);
            })
            .withWidget(BuiltInWidgets.kGraph)
            .withSize(5, 2)
            .withPosition(5, 4);

        visionTab.addNumber("Angle Error", () -> getBestTarget().map(target -> target.tx).orElse(0.0))
            .withWidget(BuiltInWidgets.kGraph)
            .withSize(5, 2)
            .withPosition(10, 4);

        // Source information
        visionTab.addString("Limelight Source", () -> getBestTarget().map(target -> target.limelightName).orElse("none"))
            .withWidget(BuiltInWidgets.kTextView)
            .withSize(3, 2)
            .withPosition(12, 2);
    }

    // Toggles AprilTag tracking and controls Limelight LEDs.
    public void toggleTracking(boolean enabled) {
        if (enabled != trackingEnabled) {
            trackingEnabled = enabled;
            for (String name : VisionConstants.LIMELIGHT_NAMES) {
                if (trackingEnabled) {
                    LimelightHelpers.setLEDMode_ForceOn("limelight-" + name);
                } else {
                    LimelightHelpers.setLEDMode_ForceOff("limelight-" + name);
                }
            }
        }
    }

    // Returns whether AprilTag tracking is enabled.
    public boolean isTrackingEnabled() {
        return trackingEnabled;
    }

    // Retrieves the horizontal offset for a specific AprilTag.
    public double getTargetHorizontalOffset(int tagId) {
        if (tagId >= 0 && tagId < VisionConstants.APRILTAG_OFFSETS.length) {
            return VisionConstants.APRILTAG_OFFSETS[tagId][0];
        }
        // Return default value and log warning
        System.out.println("Warning: Tag ID " + tagId + " out of bounds");
        return 0.0;
    }

    // Retrieves the vertical offset for a specific AprilTag.
    public double getTargetVerticalOffset(int tagId) {
        if (tagId >= 0 && tagId < VisionConstants.APRILTAG_OFFSETS.length) {
            return VisionConstants.APRILTAG_OFFSETS[tagId][1];
        }
        // Return default value and log warning
        System.out.println("Warning: Tag ID " + tagId + " out of bounds");
        return 0.0;
    }

    // Retrieves the best AprilTag target from all Limelights.
    public Optional<AprilTagTarget> getBestTarget() {
        AprilTagTarget bestTarget = null;
        double bestArea = 0;

        for (String name : VisionConstants.LIMELIGHT_NAMES) {
            String limelightName = "limelight-" + name;
            if (LimelightHelpers.getTV(limelightName)) {
                double currentArea = LimelightHelpers.getTA(limelightName);
                if (currentArea > bestArea) {
                    bestTarget = new AprilTagTarget(
                        (int) LimelightHelpers.getFiducialID(limelightName),
                        LimelightHelpers.getTX(limelightName),
                        LimelightHelpers.getTY(limelightName),
                        LimelightHelpers.getTargetPose3d_CameraSpace(limelightName).getX(),
                        LimelightHelpers.getTargetPose3d_CameraSpace(limelightName).getY(),
                        name
                    );
                    bestArea = currentArea;
                }
            }
        }

        return Optional.ofNullable(bestTarget);
    }

    // Retrieves the latest pose estimate for vision-assisted localization.
    public Optional<Pose2d> getLatestPoseEstimate() {
        Optional<AprilTagTarget> target = getBestTarget();
        if (target.isPresent()) {
            return Optional.of(LimelightHelpers.getBotPose2d_wpiBlue("limelight-" + target.get().limelightName));
        }
        return Optional.empty();
    }

    // Stops vision-based movement cleanly.
    public void stopVisionMovement() {
        trackingEnabled = false;
        for (String name : VisionConstants.LIMELIGHT_NAMES) {
            LimelightHelpers.setLEDMode_ForceOff("limelight-" + name);
        }
    }

    public void enablePositionTracking(boolean enabled) {
        positionTrackingEnabled = enabled;
    }

    public boolean isPositionTrackingEnabled() {
        return positionTrackingEnabled;
    }

    // Represents an AprilTag target detected by a Limelight.
    public static class AprilTagTarget {
        public final int id;
        public final double tx;
        public final double ty;
        public final double poseX;
        public final double poseY;
        public final String limelightName;

        public AprilTagTarget(int id, double tx, double ty, double poseX, double poseY, String limelightName) {
            this.id = id;
            this.tx = tx;
            this.ty = ty;
            this.poseX = poseX;
            this.poseY = poseY;
            this.limelightName = limelightName;
        }
    }

    private void updateRobotPosition() {
        // Find the Limelight with the most visible tags for best position estimate
        String bestLimelight = null;
        int maxTagCount = 0;

        for (String name : VisionConstants.LIMELIGHT_NAMES) {
            String limelightName = "limelight-" + name;
            if (LimelightHelpers.getTV(limelightName)) {
                int tagCount = LimelightHelpers.getTargetCount(limelightName);
                if (tagCount > maxTagCount) {
                    maxTagCount = tagCount;
                    bestLimelight = limelightName;
                }
            }
        }

        // Only update position if we have at least one tag visible
        if (bestLimelight != null && maxTagCount > 0) {
            // Get robot pose estimate from Limelight on the field
            Pose2d botPose = LimelightHelpers.getBotPose2d_wpiBlue(bestLimelight);

            // Get timestamp of when this measurement was taken
            double timestamp = LimelightHelpers.getLatency_Pipeline(bestLimelight) / 1000.0 +
                              LimelightHelpers.getLatency_Capture(bestLimelight) / 1000.0;
            double timestampSeconds = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - timestamp;

            // Calculate standard deviation based on distance and number of tags
            // This is a simple model - you may want to tune this for your robot
            if (botPose != null) {
                // Calculate standard deviation - more tags and closer distance means more confidence
                double stdDevPoseFactor = 1.0 / Math.sqrt(maxTagCount);
                // Closer tags (larger area) mean better confidence
                double tagArea = LimelightHelpers.getTA(bestLimelight);

                // Basic standard deviation model - this can be tuned
                double xyStdDev = 0.1 + stdDevPoseFactor * (1.0 / Math.max(0.1, tagArea));
                double rotStdDev = 0.05 + stdDevPoseFactor * (1.0 / Math.max(0.1, tagArea));

                // Create matrix for standard deviation
                edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3, edu.wpi.first.math.numbers.N1> visionStdDevs =
                    edu.wpi.first.math.VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev);

                // Add the vision measurement to the Swerve drivetrain
                swerve.addVisionMeasurement(botPose, timestampSeconds, visionStdDevs);
            }
        }
    }

    @Override
    public void periodic() {
        // No need to update widgets here as they're now supplier-based
        
        if (positionTrackingEnabled) {
            updateRobotPosition();
        }
    }
}