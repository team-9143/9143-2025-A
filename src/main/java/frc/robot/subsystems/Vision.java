package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
	private boolean trackingEnabled = false;

	// Shuffleboard
	private final ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");
	private final GenericEntry trackingEnabledWidget;
	private final GenericEntry bestTagIdWidget;
	private final GenericEntry bestTagDistanceWidget;
	private final GenericEntry bestTagHorizontalOffsetWidget;

	public Vision() {
		// Initialize Shuffleboard widgets
		trackingEnabledWidget = visionTab.add("Tracking Enabled", false).getEntry();
		bestTagIdWidget = visionTab.add("Best Tag ID", 0).getEntry();
		bestTagDistanceWidget = visionTab.add("Best Tag Distance", 0.0).getEntry();
		bestTagHorizontalOffsetWidget = visionTab.add("Best Tag Horizontal Offset", 0.0).getEntry();

		// Set all Limelights to AprilTag pipeline (Pipeline 0)
		for (String name : VisionConstants.LIMELIGHT_NAMES) {
			LimelightHelpers.setPipelineIndex("limelight-" + name, 0);
		}
	}

	// Toggles AprilTag tracking and controls Limelight LEDs.
	public void toggleTracking() {
		trackingEnabled = !trackingEnabled;
		for (String name : VisionConstants.LIMELIGHT_NAMES) {
			LimelightHelpers.setLEDMode_ForceOff("limelight-" + name);
			if (trackingEnabled) {
				LimelightHelpers.setLEDMode_ForceOn("limelight-" + name);
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
        return 0.0;
    }

    // Retrieves the vertical offset for a specific AprilTag.
    public double getTargetVerticalOffset(int tagId) {
        if (tagId >= 0 && tagId < VisionConstants.APRILTAG_OFFSETS.length) {
            return VisionConstants.APRILTAG_OFFSETS[tagId][1];
        }
        return 0.0;
    }

	// Retrieves the best AprilTag target from all Limelights.
	public Optional<AprilTagTarget> getBestTarget() {
		AprilTagTarget bestTarget = null;
		double bestArea = 0;

		for (String name : VisionConstants.LIMELIGHT_NAMES) {
			if (LimelightHelpers.getTV("limelight-" + name)) {
				double currentArea = LimelightHelpers.getTA("limelight-" + name);
				if (currentArea > bestArea) {
					bestTarget = new AprilTagTarget(
						(int) LimelightHelpers.getFiducialID("limelight-" + name),
						LimelightHelpers.getTX("limelight-" + name),
						LimelightHelpers.getTargetPose3d_CameraSpace("limelight-" + name).getX(),
						LimelightHelpers.getTargetPose3d_CameraSpace("limelight-" + name).getY(),
						name
					);
					bestArea = currentArea;
				}
			}
		}

		return Optional.ofNullable(bestTarget);
	}

	// Retrieves the latest pose estimate for vision-assisted localization.
	public Pose2d getLatestPoseEstimate() {
		// Use the best Limelight's pose estimate
		Optional<AprilTagTarget> target = getBestTarget();
		if (target.isPresent()) {
			return LimelightHelpers.getBotPose2d_wpiBlue("limelight-" + target.get().limelightName);
		}
		return null;
	}

	// Stops vision-based movement cleanly.
	public void stopVisionMovement() {
		trackingEnabled = false;
		for (String name : VisionConstants.LIMELIGHT_NAMES) {
			LimelightHelpers.setLEDMode_ForceOff("limelight-" + name);
		}
	}

	// Represents an AprilTag target detected by a Limelight.
	public static class AprilTagTarget {
		public final int id;
		public final double tx;
		public final double poseX;
		public final double poseY;
		public final String limelightName;

		public AprilTagTarget(int id, double tx, double poseX, double poseY, String limelightName) {
			this.id = id;
			this.tx = tx;
			this.poseX = poseX;
			this.poseY = poseY;
			this.limelightName = limelightName;
		}
	}

	@Override
	public void periodic() {
		// Update Shuffleboard
		Optional<AprilTagTarget> target = getBestTarget();
		trackingEnabledWidget.setBoolean(trackingEnabled);
		if (target.isPresent()) {
			bestTagIdWidget.setDouble(target.get().id);
			bestTagDistanceWidget.setDouble(Math.hypot(target.get().poseX, target.get().poseY));
			bestTagHorizontalOffsetWidget.setDouble(target.get().poseY);
		}
	}
}