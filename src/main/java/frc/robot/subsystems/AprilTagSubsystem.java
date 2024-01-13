package frc.robot.subsystems;

import java.util.Arrays;
import java.util.EnumSet;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagSubsystem extends SubsystemBase {
	private static AprilTagSubsystem s_subsystem;
	private TimestampedDoubleArray botpose;

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
		try {
			NetworkTableInstance ins = NetworkTableInstance.getDefault(); // gets the NetworkTable
			NetworkTable table = ins.getTable("limelight"); // gets the Limelight table
			var v = table.getDoubleArrayTopic("botpose"); // gets the topic named botpose
			var s = v.subscribe(new double[6]); // subscribe to the topic named botpose
			ins.addListener(s, EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
				TimestampedDoubleArray a = s.getAtomic();
				if (a.timestamp != 0)
					botpose = a;
				// double[] copy = Arrays.copyOf(a.value, a.value.length);
				// double[] copy = new double[6];
				double[] pose2d = { a.value[0] + 8/* 6.351 */, a.value[1], a.value[5] };
				// copy[0] *= -1;
				SmartDashboard.putNumberArray("Pose2D", pose2d);
			});
		} catch (Exception e) {
			e.printStackTrace();
			;
		}
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
