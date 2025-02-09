package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final NetworkTable limelight;
    private final ShuffleboardTab visionTab;
    private final ShuffleboardLayout limelightLayout;
    private final ShuffleboardLayout aprilTagLayout;

    // Network Table subscribers
    private final DoubleSubscriber tv_sub, tx_sub, ty_sub, ta_sub, tid_sub, pipeline_sub;
    private final DoubleSubscriber poseX_sub, poseY_sub, poseZ_sub;
    private final DoubleSubscriber poseYaw_sub, posePitch_sub, poseRoll_sub;

    // Publishers for simulation
    private final DoublePublisher tv_pub, tx_pub, ty_pub, ta_pub;
    private final IntegerPublisher pipeline_pub, cam_pub;
    private final DoublePublisher poseX_pub, poseY_pub, poseZ_pub;
    private final DoublePublisher poseYaw_pub, posePitch_pub, poseRoll_pub;

    public Vision() {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        
        // Create main Vision tab
        visionTab = Shuffleboard.getTab("Vision");
        
        // Create layouts for organizing data
        limelightLayout = visionTab.getLayout("Limelight", "List")
            .withSize(2, 4)
            .withPosition(0, 0);
            
        aprilTagLayout = visionTab.getLayout("AprilTag", "List")
            .withSize(2, 4)
            .withPosition(2, 0);

        // Initialize subscribers
        tv_sub = limelight.getDoubleTopic("tv").subscribe(0);
        tx_sub = limelight.getDoubleTopic("tx").subscribe(0);
        ty_sub = limelight.getDoubleTopic("ty").subscribe(0);
        ta_sub = limelight.getDoubleTopic("ta").subscribe(0);
        tid_sub = limelight.getDoubleTopic("tid").subscribe(0);
        pipeline_sub = limelight.getDoubleTopic("getpipe").subscribe(0);

        // AprilTag pose subscribers
        poseX_sub = limelight.getDoubleTopic("poseX").subscribe(0);
        poseY_sub = limelight.getDoubleTopic("poseY").subscribe(0);
        poseZ_sub = limelight.getDoubleTopic("poseZ").subscribe(0);
        poseYaw_sub = limelight.getDoubleTopic("poseYaw").subscribe(0);
        posePitch_sub = limelight.getDoubleTopic("posePitch").subscribe(0);
        poseRoll_sub = limelight.getDoubleTopic("poseRoll").subscribe(0);

        // Initialize publishers for simulation
        tv_pub = limelight.getDoubleTopic("tv").publish();
        tx_pub = limelight.getDoubleTopic("tx").publish();
        ty_pub = limelight.getDoubleTopic("ty").publish();
        ta_pub = limelight.getDoubleTopic("ta").publish();
        
        poseX_pub = limelight.getDoubleTopic("poseX").publish();
        poseY_pub = limelight.getDoubleTopic("poseY").publish();
        poseZ_pub = limelight.getDoubleTopic("poseZ").publish();
        poseYaw_pub = limelight.getDoubleTopic("poseYaw").publish();
        posePitch_pub = limelight.getDoubleTopic("posePitch").publish();
        poseRoll_pub = limelight.getDoubleTopic("poseRoll").publish();
        pipeline_pub = limelight.getIntegerTopic("pipeline").publish();
        cam_pub = limelight.getIntegerTopic("camMode").publish();
    }

    // Basic target methods
    public boolean hasValidTarget() {
        return tv_sub.getAsDouble() == 1;
    }

    public double getTx() {
        return tx_sub.getAsDouble();
    }

    public double getTy() {
        return ty_sub.getAsDouble();
    }

    public double getArea() {
        return ta_sub.getAsDouble();
    }

    // Pipeline and camera control methods
    public void setPipeline(int pipeline) {
        pipeline_pub.accept(pipeline);
    }

    public int getPipeline() {
        return (int) pipeline_sub.getAsDouble();
    }

    public void setDriverCam(boolean isDriverCam) {
        cam_pub.accept(isDriverCam ? 1 : 0);
    }

    public void toggleDriverCam() {
        boolean currentMode = getCurrentCameraMode();
        setDriverCam(!currentMode);
    }

    public boolean getCurrentCameraMode() {
        return ((IntegerSubscriber) cam_pub).get() == 1;
    }

    // AprilTag methods
    public int getAprilTagID() {
        return (int) tid_sub.getAsDouble();
    }

    public double getAprilTagPoseX() {
        return poseX_sub.getAsDouble();
    }

    public double getAprilTagPoseY() {
        return poseY_sub.getAsDouble();
    }

    public double getAprilTagPoseZ() {
        return poseZ_sub.getAsDouble();
    }

    public double getAprilTagYaw() {
        return poseYaw_sub.getAsDouble();
    }

    public double getAprilTagPitch() {
        return posePitch_sub.getAsDouble();
    }

    public double getAprilTagRoll() {
        return poseRoll_sub.getAsDouble();
    }

    @Override
    public void periodic() {
        if (RobotBase.isSimulation()) {
            simulateLimelight();
        }

        // Update Limelight values in Shuffleboard
        limelightLayout.add("Has Target", hasValidTarget());
        limelightLayout.add("Tx", getTx());
        limelightLayout.add("Ty", getTy());
        limelightLayout.add("Area", getArea());
        limelightLayout.add("Pipeline", getPipeline());
        limelightLayout.add("Driver Mode", getCurrentCameraMode());

        // Update AprilTag values in Shuffleboard
        aprilTagLayout.add("ID", getAprilTagID());
        aprilTagLayout.add("X", getAprilTagPoseX());
        aprilTagLayout.add("Y", getAprilTagPoseY());
        aprilTagLayout.add("Z", getAprilTagPoseZ());
        aprilTagLayout.add("Yaw", getAprilTagYaw());
        aprilTagLayout.add("Pitch", getAprilTagPitch());
        aprilTagLayout.add("Roll", getAprilTagRoll());
    }

    private void simulateLimelight() {
        // Simulate basic target data
        tv_pub.set(1);
        tx_pub.set(Math.random() * 20 - 10);
        ty_pub.set(Math.random() * 20 - 10);
        ta_pub.set(Math.random() * 100);

        // Simulate AprilTag pose data
        poseX_pub.set(Math.random() * 10);
        poseY_pub.set(Math.random() * 10);
        poseZ_pub.set(Math.random() * 10);
        poseYaw_pub.set(Math.random() * 360 - 180);
        posePitch_pub.set(Math.random() * 180 - 90);
        poseRoll_pub.set(Math.random() * 180 - 90);
    }
    
    public void logTargetInfo() {
        if (hasValidTarget()) {
            System.out.println("Target Info:");
            System.out.println("Horizontal Angle (Tx): " + getTx());
            System.out.println("Vertical Angle (Ty): " + getTy());
            System.out.println("Target Area: " + getArea());
            System.out.println("AprilTag ID: " + getAprilTagID());
            System.out.println("AprilTag Pose X: " + getAprilTagPoseX());
            System.out.println("AprilTag Pose Y: " + getAprilTagPoseY());
            System.out.println("AprilTag Pose Z: " + getAprilTagPoseZ());
            System.out.println("AprilTag Yaw: " + getAprilTagYaw());
            System.out.println("AprilTag Pitch: " + getAprilTagPitch());
            System.out.println("AprilTag Roll: " + getAprilTagRoll());
        } else {
            System.out.println("No valid target found.");
        }
    }
}