package frc.robot.devices;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Class for Limelight interfacing.
public class Limelight {
	NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
	NetworkTableEntry tx = table.getEntry("tx");
	NetworkTableEntry ty = table.getEntry("ty");
	NetworkTableEntry ta = table.getEntry("ta");
	// Read values periodically
	double x = tx.getDouble(0.0);
	double y = ty.getDouble(0.0);
	double area = ta.getDouble(0.0);
	// Post to smart dashboard periodically
	public void robotPeriodic() {
		SmartDashboard.putNumber("LimelightX", x);
		SmartDashboard.putNumber("LimelightY", y);
		SmartDashboard.putNumber("LimelightArea", area);
	}
	// Limelight datatable.
	private final NetworkTable m_limelight;
	private final DoubleSubscriber tv_sub, tx_sub, ty_sub, ta_sub, tid_sub, pipeline_sub;
	private final IntegerPublisher pipeline_pub, cam_pub;
	// Initialize a new limelight.
	// @param limelight {@link NetworkTable} that points to the limelight data stream.
	protected Limelight(NetworkTable limelight) {
		m_limelight = limelight;
		tv_sub = m_limelight.getDoubleTopic("tv").subscribe(0);
		tx_sub = m_limelight.getDoubleTopic("tx").subscribe(0);
		ty_sub = m_limelight.getDoubleTopic("ty").subscribe(0);
		ta_sub = m_limelight.getDoubleTopic("ta").subscribe(0);
		tid_sub = m_limelight.getDoubleTopic("tid").subscribe(0);
		pipeline_sub = m_limelight.getDoubleTopic("getpipe").subscribe(0);
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
	// Returns {@code true} if a valid target exists
	public boolean getValid() {
		return (tv_sub.getAsDouble() == 1) ? true : false;
	}
	// Returns ID of the targeted AprilTag.
	public int getTid() {
		return (int) tid_sub.getAsDouble();
	}
	// Returns active pipeline index [0..9].
	public int getPipeline() {
		return (int) pipeline_sub.getAsDouble();
	}
	// Set the limelight pipeline. Recommended to wait for {@link Limelight#getPipeline()} to
	// corroborate before using functional code that depends on it.
	// @param pipeline pipeline index to set [0..9]
	public void setPipeline(int pipeline) {
		pipeline_pub.accept(pipeline);
	}
	// Set the limelight operation mode.
	// @param isDriverCam {@code true} for driver camera, {@code false} for vision processing
	public void setDriverCam(boolean isDriverCam) {
		cam_pub.accept(isDriverCam ? 1 : 0);
	}
}