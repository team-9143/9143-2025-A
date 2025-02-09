package frc.robot.subsystems;

import java.util.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
    private final NetworkTable[] limelights;
    private final Map<Integer, Double> tagDistances = new HashMap<>();
    private boolean trackingEnabled = false;
    
    // Network Tables
    private final NetworkTable trackingTable;
    private final DoubleSubscriber targetDistanceSub;
    private final DoubleSubscriber currentDistanceSub;
    private final DoubleSubscriber angleToTargetSub;
    private final DoublePublisher targetDistancePub;
    private final DoublePublisher currentDistancePub;
    private final DoublePublisher angleToTargetPub;
    
    private final ShuffleboardTab visionTab;
    private final List<ShuffleboardLayout> limelightLayouts = new ArrayList<>();
    private final ShuffleboardLayout trackingLayout;
    
    // Network Table subscribers for each Limelight
    private final List<DoubleSubscriber> tv_subs = new ArrayList<>();
    private final List<DoubleSubscriber> tx_subs = new ArrayList<>();
    private final List<DoubleSubscriber> ty_subs = new ArrayList<>();
    private final List<DoubleSubscriber> ta_subs = new ArrayList<>();
    private final List<DoubleSubscriber> tid_subs = new ArrayList<>();
    private final List<DoubleSubscriber> pipeline_subs = new ArrayList<>();
    private final List<DoubleSubscriber> poseX_subs = new ArrayList<>();
    private final List<DoubleSubscriber> poseY_subs = new ArrayList<>();
    private final List<DoubleSubscriber> poseZ_subs = new ArrayList<>();

    public Vision() {
        // Initialize NetworkTables
        limelights = new NetworkTable[VisionConstants.LIMELIGHT_NAMES.length];

        // Initialize tracking NetworkTable and subscribers
        trackingTable = NetworkTableInstance.getDefault().getTable(VisionConstants.NT_APRILTAG_TABLE);
        targetDistanceSub = trackingTable.getDoubleTopic(VisionConstants.NT_TARGET_DISTANCE).subscribe(0.0);
        currentDistanceSub = trackingTable.getDoubleTopic(VisionConstants.NT_CURRENT_DISTANCE).subscribe(0.0);
        angleToTargetSub = trackingTable.getDoubleTopic(VisionConstants.NT_ANGLE_TO_TARGET).subscribe(0.0);
        
        // Initialize publishers
        targetDistancePub = trackingTable.getDoubleTopic(VisionConstants.NT_TARGET_DISTANCE).publish();
        currentDistancePub = trackingTable.getDoubleTopic(VisionConstants.NT_CURRENT_DISTANCE).publish();
        angleToTargetPub = trackingTable.getDoubleTopic(VisionConstants.NT_ANGLE_TO_TARGET).publish();

        // Create Shuffleboard layouts
        visionTab = Shuffleboard.getTab("Vision");
        trackingLayout = visionTab.getLayout("Tracking Status", BuiltInLayouts.kList)
            .withSize(VisionConstants.ShuffleboardLayout.TRACKING_WIDTH, 
                VisionConstants.ShuffleboardLayout.TRACKING_HEIGHT)
            .withPosition(0, 0);
        
        // Initialize each Limelight
        for (int i = 0; i < VisionConstants.LIMELIGHT_NAMES.length; i++) {
            limelights[i] = NetworkTableInstance.getDefault().getTable(VisionConstants.LIMELIGHT_NAMES[i]);

            // Create layout for this Limelight
            ShuffleboardLayout layout = visionTab.getLayout(VisionConstants.LIMELIGHT_NAMES[i], BuiltInLayouts.kList)
                .withSize(VisionConstants.ShuffleboardLayout.VISION_TAB_WIDTH, 
					VisionConstants.ShuffleboardLayout.VISION_TAB_HEIGHT)
                .withPosition(2 + (i * 2), 0);
            limelightLayouts.add(layout);
            
            // Initialize subscribers
            tv_subs.add(limelights[i].getDoubleTopic("tv").subscribe(0));
            tx_subs.add(limelights[i].getDoubleTopic("tx").subscribe(0));
            ty_subs.add(limelights[i].getDoubleTopic("ty").subscribe(0));
            ta_subs.add(limelights[i].getDoubleTopic("ta").subscribe(0));
            tid_subs.add(limelights[i].getDoubleTopic("tid").subscribe(0));
            pipeline_subs.add(limelights[i].getDoubleTopic("getpipe").subscribe(0));
            poseX_subs.add(limelights[i].getDoubleTopic("targetpose_cameraspace").subscribe(0));
            poseY_subs.add(limelights[i].getDoubleTopic("targetpose_cameraspace").subscribe(0));
            poseZ_subs.add(limelights[i].getDoubleTopic("targetpose_cameraspace").subscribe(0));
        }

        initializeTagDistances();
    }
	// Add getters for the tracking values
    public double getTargetDistance() {
        return targetDistanceSub.get();
    }
    
    public double getCurrentDistance() {
        return currentDistanceSub.get();
    }
    
    public double getAngleToTarget() {
        return angleToTargetSub.get();
    }
    
    private void initializeTagDistances() {
        tagDistances.put(1, VisionConstants.TagDistances.TAG_1);
        tagDistances.put(2, VisionConstants.TagDistances.TAG_2);
        tagDistances.put(3, VisionConstants.TagDistances.TAG_3);
        tagDistances.put(4, VisionConstants.TagDistances.TAG_4);
		// Add more as needed
    }
    
    public void setTagDistance(int tagId, double distance) {
        tagDistances.put(tagId, distance);
    }
    
    public void toggleTracking() {
        trackingEnabled = !trackingEnabled;
    }
    
    public boolean isTrackingEnabled() {
        return trackingEnabled;
    }
    
    // Methods to get data from specific Limelight
    public boolean hasValidTarget(int limelightIndex) {
        if (limelightIndex >= 0 && limelightIndex < limelights.length) {
            return tv_subs.get(limelightIndex).get() == 1;
        }
        return false;
    }
    
    public Optional<AprilTagTarget> getBestTarget() {
        AprilTagTarget bestTarget = null;
        double bestArea = 0;
        
        for (int i = 0; i < limelights.length; i++) {
            if (hasValidTarget(i)) {
                double currentArea = ta_subs.get(i).get();
                if (currentArea > bestArea) {
                    bestTarget = new AprilTagTarget(
                        (int)tid_subs.get(i).get(),
                        tx_subs.get(i).get(),
                        poseX_subs.get(i).get(),
                        poseY_subs.get(i).get(),
                        i
                    );
                    bestArea = currentArea;
                }
            }
        }
        
        return Optional.ofNullable(bestTarget);
    }
    
    public static class AprilTagTarget {
        public final int id;
        public final double tx;
        public final double poseX;
        public final double poseY;
        public final int limelightIndex;
        
        public AprilTagTarget(int id, double tx, double poseX, double poseY, int limelightIndex) {
            this.id = id;
            this.tx = tx;
            this.poseX = poseX;
            this.poseY = poseY;
            this.limelightIndex = limelightIndex;
        }
    }
    
    @Override
    public void periodic() {
        // Update tracking values in NetworkTables when we have a valid target
        getBestTarget().ifPresent(target -> {
            double currentDistance = Math.sqrt(
                Math.pow(poseX_subs.get(target.limelightIndex).get(), 2) +
                Math.pow(poseY_subs.get(target.limelightIndex).get(), 2)
            );
            
            currentDistancePub.set(currentDistance);
            angleToTargetPub.set(target.tx);
            targetDistancePub.set(tagDistances.getOrDefault(target.id, 0.0));
        });
        
        // Update Shuffleboard
        updateShuffleboard();
    }

	private void updateShuffleboard() {
        // Update tracking layout
        trackingLayout.add("Tracking Enabled", trackingEnabled);
        getBestTarget().ifPresent(target -> {
            trackingLayout.add("Current Tag ID", target.id);
            trackingLayout.add("Target Distance", getTargetDistance());
            trackingLayout.add("Current Distance", getCurrentDistance());
            trackingLayout.add("Angle to Target", getAngleToTarget());
        });
        
        // Update Limelight layouts
        for (int i = 0; i < limelights.length; i++) {
            ShuffleboardLayout layout = limelightLayouts.get(i);
            layout.add("Has Target", hasValidTarget(i));
            layout.add("Tx", tx_subs.get(i).get());
            layout.add("Ty", ty_subs.get(i).get());
            layout.add("Area", ta_subs.get(i).get());
            layout.add("Tag ID", tid_subs.get(i).get());
        }
	}
}