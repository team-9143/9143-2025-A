package frc.robot.subsystems;

import java.util.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
    private final NetworkTable[] limelights;
    private final Map<Integer, Double> tagDistances = new HashMap<>();
    private final Map<Integer, Double> tagHorizontalOffsets = new HashMap<>();
    private boolean trackingEnabled = false;
    
    // Network Tables
    private final NetworkTable trackingTable;
    private final DoubleSubscriber targetDistanceSub;
    private final DoubleSubscriber currentDistanceSub;
    private final DoubleSubscriber angleToTargetSub;
    private final DoubleSubscriber targetHorizontalOffsetSub;
    private final DoubleSubscriber currentHorizontalOffsetSub;
    private final DoublePublisher targetDistancePub;
    private final DoublePublisher currentDistancePub;
    private final DoublePublisher angleToTargetPub;
    private final DoublePublisher targetHorizontalOffsetPub;
    private final DoublePublisher currentHorizontalOffsetPub;
    
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

    // Shuffleboard widgets to persist between updates
    private final GenericEntry trackingEnabledWidget;
    private final GenericEntry currentTagIdWidget;
    private final GenericEntry targetDistanceWidget;
    private final GenericEntry currentDistanceWidget;
    private final GenericEntry angleToTargetWidget;
    private final GenericEntry targetHorizontalOffsetWidget;
    private final GenericEntry currentHorizontalOffsetWidget;
    private final Map<String, List<GenericEntry>> limelightWidgets = new HashMap<>();

    public Vision() {
        // Initialize NetworkTables
        limelights = new NetworkTable[VisionConstants.LIMELIGHT_NAMES.length];

        // Initialize tracking NetworkTable and subscribers
        trackingTable = NetworkTableInstance.getDefault().getTable(VisionConstants.NT_APRILTAG_TABLE);
        targetDistanceSub = trackingTable.getDoubleTopic(VisionConstants.NT_TARGET_DISTANCE).subscribe(0.0);
        currentDistanceSub = trackingTable.getDoubleTopic(VisionConstants.NT_CURRENT_DISTANCE).subscribe(0.0);
        angleToTargetSub = trackingTable.getDoubleTopic(VisionConstants.NT_ANGLE_TO_TARGET).subscribe(0.0);
        targetHorizontalOffsetSub = trackingTable.getDoubleTopic(VisionConstants.NT_TARGET_HORIZONTAL_OFFSET).subscribe(0.0);
        currentHorizontalOffsetSub = trackingTable.getDoubleTopic(VisionConstants.NT_CURRENT_HORIZONTAL_OFFSET).subscribe(0.0);
        
        // Initialize publishers
        targetDistancePub = trackingTable.getDoubleTopic(VisionConstants.NT_TARGET_DISTANCE).publish();
        currentDistancePub = trackingTable.getDoubleTopic(VisionConstants.NT_CURRENT_DISTANCE).publish();
        angleToTargetPub = trackingTable.getDoubleTopic(VisionConstants.NT_ANGLE_TO_TARGET).publish();
        targetHorizontalOffsetPub = trackingTable.getDoubleTopic(VisionConstants.NT_TARGET_HORIZONTAL_OFFSET).publish();
        currentHorizontalOffsetPub = trackingTable.getDoubleTopic(VisionConstants.NT_CURRENT_HORIZONTAL_OFFSET).publish();

        // Create Shuffleboard layouts
        visionTab = Shuffleboard.getTab("Vision");

        trackingLayout = visionTab.getLayout("Tracking Status", BuiltInLayouts.kList)
            .withSize(VisionConstants.ShuffleboardLayout.TRACKING_WIDTH, 
                VisionConstants.ShuffleboardLayout.TRACKING_HEIGHT)
            .withPosition(0, 0)
            .withProperties(Map.of("Label position", "LEFT"));

        // Initialize persistent widgets for tracking data
        trackingEnabledWidget = trackingLayout.add("Tracking Enabled", false).getEntry();
        currentTagIdWidget = trackingLayout.add("Current Tag ID", 0).getEntry();
        
        // Create a nested layout for distance tracking
        ShuffleboardLayout distanceLayout = trackingLayout.getLayout("Distance", BuiltInLayouts.kList)
            .withProperties(Map.of("Label position", "LEFT"));
        targetDistanceWidget = distanceLayout.add("Target", 0.0).getEntry();
        currentDistanceWidget = distanceLayout.add("Current", 0.0).getEntry();
        
        // Create a nested layout for horizontal offset tracking
        ShuffleboardLayout horizontalLayout = trackingLayout.getLayout("Horizontal Offset", BuiltInLayouts.kList)
            .withProperties(Map.of("Label position", "LEFT"));
        targetHorizontalOffsetWidget = horizontalLayout.add("Target", 0.0).getEntry();
        currentHorizontalOffsetWidget = horizontalLayout.add("Current", 0.0).getEntry();
        
        // Add angle tracking
        angleToTargetWidget = trackingLayout.add("Angle to Target", 0.0).getEntry();
        
        // Initialize each Limelight
        for (int i = 0; i < VisionConstants.LIMELIGHT_NAMES.length; i++) {
            String name = VisionConstants.LIMELIGHT_NAMES[i];

            ShuffleboardLayout layout = visionTab.getLayout(name, BuiltInLayouts.kList)
                .withSize(VisionConstants.ShuffleboardLayout.VISION_TAB_WIDTH, 
                    VisionConstants.ShuffleboardLayout.VISION_TAB_HEIGHT)
                .withPosition(2 + (i * 2), 0)
                .withProperties(Map.of("Label position", "LEFT"));
            
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

            List<GenericEntry> widgets = new ArrayList<>();
            widgets.add(layout.add("Has Target", false).getEntry());
            widgets.add(layout.add("Tx", 0.0).getEntry());
            widgets.add(layout.add("Ty", 0.0).getEntry());
            widgets.add(layout.add("Area", 0.0).getEntry());
            widgets.add(layout.add("Tag ID", 0.0).getEntry());

            limelightWidgets.put(name, widgets);
            limelightLayouts.add(layout);
        }

        initializeTagDistances();
        initializeTagOffsets();
    }

    private void initializeTagDistances() {
        tagDistances.put(1, VisionConstants.TagDistances.TAG_1_DISTANCE);
        tagDistances.put(2, VisionConstants.TagDistances.TAG_2_DISTANCE);
        tagDistances.put(3, VisionConstants.TagDistances.TAG_3_DISTANCE);
        tagDistances.put(4, VisionConstants.TagDistances.TAG_4_DISTANCE);
    }

    private void initializeTagOffsets() {
        tagHorizontalOffsets.put(1, VisionConstants.TagDistances.TAG_1_HORIZONTAL);
        tagHorizontalOffsets.put(2, VisionConstants.TagDistances.TAG_2_HORIZONTAL);
        tagHorizontalOffsets.put(3, VisionConstants.TagDistances.TAG_3_HORIZONTAL);
        tagHorizontalOffsets.put(4, VisionConstants.TagDistances.TAG_4_HORIZONTAL);
    }

    public double getTargetDistance() {
        return targetDistanceSub.get();
    }

    public double getTargetHorizontalOffset() {
        return targetHorizontalOffsetSub.get();
    }
    
    public double getCurrentDistance() {
        return currentDistanceSub.get();
    }

    public double getCurrentHorizontalOffset() {
        return currentHorizontalOffsetSub.get();
    }
    
    public double getAngleToTarget() {
        return angleToTargetSub.get();
    }
    
    public void setTagDistance(int tagId, double distance) {
        tagDistances.put(tagId, distance);
    }

    public void setTagHorizontalOffset(int tagId, double offset) {
        tagHorizontalOffsets.put(tagId, offset);
    }
    
    public void toggleTracking() {
        trackingEnabled = !trackingEnabled;
    }
    
    public boolean isTrackingEnabled() {
        return trackingEnabled;
    }
    
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

            // Calculate horizontal offset based on pose
            double currentHorizontalOffset = poseY_subs.get(target.limelightIndex).get();
            
            currentDistancePub.set(currentDistance);
            angleToTargetPub.set(target.tx);
            targetDistancePub.set(tagDistances.getOrDefault(target.id, 0.0));
            currentHorizontalOffsetPub.set(currentHorizontalOffset);
            targetHorizontalOffsetPub.set(tagHorizontalOffsets.getOrDefault(target.id, 0.0));
        });
        
        updateShuffleboard();
    }

	private void updateShuffleboard() {
        // Update tracking status widgets
        trackingEnabledWidget.setBoolean(trackingEnabled);

        getBestTarget().ifPresent(target -> {
            currentTagIdWidget.setDouble(target.id);
            targetDistanceWidget.setDouble(getTargetDistance());
            currentDistanceWidget.setDouble(getCurrentDistance());
            angleToTargetWidget.setDouble(getAngleToTarget());
            targetHorizontalOffsetWidget.setDouble(getTargetHorizontalOffset());
            currentHorizontalOffsetWidget.setDouble(getCurrentHorizontalOffset());
        });
        
        // Update Limelight widgets
        for (int i = 0; i < limelights.length; i++) {
            String name = VisionConstants.LIMELIGHT_NAMES[i];
            List<GenericEntry> widgets = limelightWidgets.get(name);
            
            widgets.get(0).setBoolean(hasValidTarget(i));
            widgets.get(1).setDouble(tx_subs.get(i).get());
            widgets.get(2).setDouble(ty_subs.get(i).get());
            widgets.get(3).setDouble(ta_subs.get(i).get());
            widgets.get(4).setDouble(tid_subs.get(i).get());
        }
	}
}