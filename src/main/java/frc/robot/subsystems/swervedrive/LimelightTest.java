import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightTest {
    private NetworkTable limelightTable;

    @Override
    public void robotInit() {
        // Get the NetworkTable for the Limelight
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

        // Set the stream mode to "Standard" (side-by-side)
        limelightTable.getEntry("stream").setNumber(0);

        // Set the camera mode to "Vision Processing" (enabled)
        limelightTable.getEntry("camMode").setNumber(0);

        // Set the LED mode to "On"
        limelightTable.getEntry("ledMode").setNumber(1);

        // Add the Limelight stream to Shuffleboard
        ShuffleboardTab tab = Shuffleboard.getTab("Limelight");
        tab.add("Limelight Stream", limelightTable.getEntry("stream"));
    }

    @Override
    public void teleopPeriodic() {
        // Update Shuffleboard with the current stream URL
        String streamUrl = "http://172.29.1.1/";
        SmartDashboard.putString("Limelight Stream URL", streamUrl);
    }
}