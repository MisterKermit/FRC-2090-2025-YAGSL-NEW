package frc.robot.subsystems.swervedrive;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightTest extends SubsystemBase{
    private NetworkTable limelightTable;

    public LimelightTest() {
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
    public void periodic() {
        // Update Shuffleboard with the current stream URL
        String streamUrl = "http://172.29.1.1/";
        SmartDashboard.putString("Limelight Stream URL", streamUrl);
    }
}