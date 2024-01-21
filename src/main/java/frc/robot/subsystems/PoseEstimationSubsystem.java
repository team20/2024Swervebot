package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import hlib.drive.Pose;
import hlib.drive.PoseEstimator;
import hlib.drive.PoseEstimatorWeighted;

public class PoseEstimationSubsystem extends AprilTagSubsystem {

	PoseEstimator m_poseEstimator = new PoseEstimatorWeighted(1.0, 10, 0.1);

	public PoseEstimationSubsystem() {
		super();
	}

	public static PoseEstimationSubsystem get() { // returns the first instance called, guarantees that there's not
		return (PoseEstimationSubsystem) s_subsystem;
	}

	/**
	 * A method run periodically (every 20 ms).
	 */
	@Override
	public void periodic() {
	}

	TimestampedDoubleArray changedBotPose(NetworkTableEvent event) {
		var botpose = super.changedBotPose(event);
		if (botpose != null) {
			m_poseEstimator.update(new Pose(botpose.value[0], botpose.value[1], botpose.value[5] * Math.PI / 180));
			var poseEstimated = m_poseEstimator.poseEstimated();
			visionTable.getEntry("Pose Estimated").setString("" + poseEstimated());
			if (poseEstimated != null)
				visionTable.getEntry("Pose2D'").setDoubleArray(toPose2DAdvantageScope(poseEstimated.x(),
						poseEstimated.y(), poseEstimated.directionalAngleInDegrees()));
		}
		return botpose;
	}

	public Pose poseEstimated() {
		return m_poseEstimator.poseEstimated();
	}
}
