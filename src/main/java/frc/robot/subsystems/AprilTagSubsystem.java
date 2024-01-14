package frc.robot.subsystems;

import java.util.Arrays;
import java.util.EnumSet;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagSubsystem extends SubsystemBase {
	private static AprilTagSubsystem s_subsystem;
	private TimestampedDoubleArray botpose;
	private NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("vision");

	public AprilTagSubsystem() {
		// Singleton
		if (s_subsystem != null) {
			try {
				throw new Exception("AprilTag subsystem already initialized!");
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		s_subsystem = this;

		// Subscriber
		NetworkTableInstance ins = NetworkTableInstance.getDefault(); // gets the NetworkTable
		NetworkTable table = ins.getTable("limelight"); // gets the Limelight table
		var v = table.getDoubleArrayTopic("botpose"); // gets the topic named botpose
		var s = v.subscribe(new double[6]); // subscribe to the topic named botpose
		ins.addListener(s, EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> changedBotPose(event, s));
	}

	private void changedBotPose(NetworkTableEvent event, DoubleArraySubscriber s) {
		TimestampedDoubleArray a = s.getAtomic();
		if (a.timestamp != 0)
			botpose = a;
		double[] pose2d = { a.value[0] + 8.27/* 6.351 */, a.value[1] + 4.1, a.value[5] };
		SmartDashboard.putNumberArray("Pose2D", pose2d);
	}

	private void changedJson(NetworkTableEvent event, StringSubscriber s) {
		// TODO: add it
	}

	private void changedBotPose(NetworkTableEvent event) {
		{
			var v = event.valueData.value;
			botpose = new TimestampedDoubleArray(v.getTime(), v.getServerTime(), v.getDoubleArray());
			System.out.println(botpose.toString());
			double[] pose2d = { botpose.value[0] + 8.27, botpose.value[1] + 4.1, botpose.value[5] };
			// copy[0] *= -1;

			visionTable.getDoubleArrayTopic("Pose2D").publish().set(pose2d);
		}
		;
	}

	public static AprilTagSubsystem get() { // returns the first instance called, guarantees that there's not
		return s_subsystem;
	}

	/**
	 * A method run periodically (every 20 ms).
	 */
	@Override
	public void periodic() {
	}

	public TimestampedDoubleArray getBotpose() {
		return botpose;
	}
}
