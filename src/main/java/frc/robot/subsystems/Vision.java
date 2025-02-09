package frc.robot.subsystems;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Vision extends SubsystemBase {
	private final NetworkTable m_limelight;
	private final DoubleSubscriber tv_sub, tx_sub, ty_sub, ta_sub, tid_sub, pipeline_sub;
	private final DoublePublisher tv_pub, tx_pub, ty_pub, ta_pub;
	private final IntegerPublisher pipeline_pub, cam_pub;
	// AprilTag pose and ID fields (Subscribers)
	private final DoubleSubscriber poseX_sub,
	poseY_sub,
	poseZ_sub,
	poseYaw_sub,
	posePitch_sub,
	poseRoll_sub;
	// AprilTag pose Publishers for simulation
	private final DoublePublisher poseX_pub,
	poseY_pub,
	poseZ_pub,
	poseYaw_pub,
	posePitch_pub,
	poseRoll_pub;
	public Vision() {
		m_limelight = NetworkTableInstance.getDefault().getTable("limelight");
		// Subscribers for reading data
		tv_sub = m_limelight.getDoubleTopic("tv").subscribe(0);
		tx_sub = m_limelight.getDoubleTopic("tx").subscribe(0);
		ty_sub = m_limelight.getDoubleTopic("ty").subscribe(0);
		ta_sub = m_limelight.getDoubleTopic("ta").subscribe(0);
		tid_sub = m_limelight.getDoubleTopic("tid").subscribe(0); // AprilTag ID
		pipeline_sub = m_limelight.getDoubleTopic("getpipe").subscribe(0);
		// AprilTag pose data subscribers
		poseX_sub = m_limelight.getDoubleTopic("poseX").subscribe(0);
		poseY_sub = m_limelight.getDoubleTopic("poseY").subscribe(0);
		poseZ_sub = m_limelight.getDoubleTopic("poseZ").subscribe(0);
		poseYaw_sub = m_limelight.getDoubleTopic("poseYaw").subscribe(0);
		posePitch_sub = m_limelight.getDoubleTopic("posePitch").subscribe(0);
		poseRoll_sub = m_limelight.getDoubleTopic("poseRoll").subscribe(0);
		// Publishers for writing simulated data
		tv_pub = m_limelight.getDoubleTopic("tv").publish();
		tx_pub = m_limelight.getDoubleTopic("tx").publish();
		ty_pub = m_limelight.getDoubleTopic("ty").publish();
		ta_pub = m_limelight.getDoubleTopic("ta").publish();
		// AprilTag pose data publishers for simulation
		poseX_pub = m_limelight.getDoubleTopic("poseX").publish();
		poseY_pub = m_limelight.getDoubleTopic("poseY").publish();
		poseZ_pub = m_limelight.getDoubleTopic("poseZ").publish();
		poseYaw_pub = m_limelight.getDoubleTopic("poseYaw").publish();
		posePitch_pub = m_limelight.getDoubleTopic("posePitch").publish();
		poseRoll_pub = m_limelight.getDoubleTopic("poseRoll").publish();
		// Integer Publishers for controlling pipeline and camera mode
		pipeline_pub = m_limelight.getIntegerTopic("pipeline").publish();
		cam_pub = m_limelight.getIntegerTopic("camMode").publish();
	}
	// Returns horizontal angle to target in degrees.
	public double getTx() {
		return tx_sub.getAsDouble();
	}
	// Returns vertical angle to target in degrees.
	public double getTy() {
		return ty_sub.getAsDouble();
	}
	// Returns percent area of target relative to camera.
	public double getArea() {
		return ta_sub.getAsDouble();
	}
	// Returns true if a valid target exists
	public boolean hasValidTarget() {
		return tv_sub.getAsDouble() == 1;
	}
	// Sets the active pipeline
	public void setPipeline(int pipeline) {
		pipeline_pub.accept(pipeline);
	}
	// Returns the current active pipeline index
	public int getPipeline() {
		return (int) pipeline_sub.getAsDouble();
	}
	// Sets the Limelight camera mode: true for Driver Cam, false for Vision Processing
	public void setDriverCam(boolean isDriverCam) {
		cam_pub.accept(isDriverCam ? 1 : 0); // 1 for DriverCam, 0 for VisionProcessing
	}
	// Toggles between Driver Cam and Vision Processing modes
	public void toggleDriverCam() {
		boolean currentMode = getCurrentCameraMode();
		setDriverCam(!currentMode);
	}
	// Returns the current camera mode (true for Driver Cam, false for Vision Processing)
	public boolean getCurrentCameraMode() {
		return ((IntegerSubscriber) cam_pub).get() == 1; // 1 means DriverCam mode is active
	}
	// New methods for AprilTag data
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
		// Update SmartDashboard values
		SmartDashboard.putBoolean("Limelight/Has Target", hasValidTarget());
		SmartDashboard.putNumber("Limelight/Tx", getTx());
		SmartDashboard.putNumber("Limelight/Ty", getTy());
		SmartDashboard.putNumber("Limelight/Area", getArea());
		SmartDashboard.putNumber("Limelight/Pipeline", getPipeline());
		SmartDashboard.putBoolean("Limelight/Driver Cam Mode", getCurrentCameraMode());
		// Update SmartDashboard with AprilTag info
		SmartDashboard.putNumber("AprilTag/ID", getAprilTagID());
		SmartDashboard.putNumber("AprilTag/Pose X", getAprilTagPoseX());
		SmartDashboard.putNumber("AprilTag/Pose Y", getAprilTagPoseY());
		SmartDashboard.putNumber("AprilTag/Pose Z", getAprilTagPoseZ());
		SmartDashboard.putNumber("AprilTag/Yaw", getAprilTagYaw());
		SmartDashboard.putNumber("AprilTag/Pitch", getAprilTagPitch());
		SmartDashboard.putNumber("AprilTag/Roll", getAprilTagRoll());
	}
	// Simulates Limelight values for testing in simulation mode
	private void simulateLimelight() {
		tv_pub.set(1); // Simulate target is visible
		tx_pub.set(Math.random() * 20 - 10); // Random horizontal angle (-10 to 10 degrees)
		ty_pub.set(Math.random() * 20 - 10); // Random vertical angle
		ta_pub.set(Math.random() * 100); // Random area (0-100%)
		// Simulate AprilTag pose values
		poseX_pub.set(Math.random() * 10); // Random pose data
		poseY_pub.set(Math.random() * 10);
		poseZ_pub.set(Math.random() * 10);
		poseYaw_pub.set(Math.random() * 360 - 180);
		posePitch_pub.set(Math.random() * 180 - 90);
		poseRoll_pub.set(Math.random() * 180 - 90);
	}
	public void logTargetInfo() {
		// Log or print relevant target info
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