package hlib.drive;

/**
 * A {@code PoseEstimatorWeighted} can estimate the {@code Pose} of a {@code Robot}.
 * 
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 */
public class PoseEstimatorWeighted extends PoseEstimatorInconsistencyTolerant {

	/**
	 * The weight applied to each {@code Pose} detected by the LimeLight.
	 */
	double weight;

	/**
	 * Constructs a {@code PoseEstimatorWeighted}.
	 * 
	 * @param distanceThreshold
	 *            the distance threshold for outlier detection (i.e., the difference in x- or y-coordinates of the
	 *            {@code Pose} from LimeLight compared to the {@code Pose} that has been estimated in order for the
	 *            {@code Pose} from LimeLight to be considered an outlier)
	 * @param rejectionLimit
	 *            the number of rejections before resetting the {@code PoseEstimatorWeighted}
	 * @param weight
	 *            the weight of each new {@code Pose} from the LimeLight.
	 */
	public PoseEstimatorWeighted(double distanceThreshold, int rejectionLimit, double weight) {
		super(distanceThreshold, rejectionLimit);
		this.weight = weight;
	}

	/**
	 * Updates the {@code Pose} that has been updated by this {@code PoseEstimatorWeighted} based on the specified
	 * {@code Pose} from the LimeLight.
	 * 
	 * @param poseDetected
	 *            the {@code Pose} from the LimeLight
	 */
	@Override
	public void setPoseEstimated(Pose poseDetected) {
		if (poseDetected != null) {
			errorTracker.update(poseDetected, this.poseEstimated);
			this.poseEstimated = weightedSum(poseDetected, weight, this.poseEstimated, 1 - weight);
			if (this.poseEstimated.isInvalid())
				reset();
		}
	}

	/**
	 * Calculates the specified weighted sum.
	 * 
	 * @param p1
	 *            the first {@code Pose}
	 * @param w1
	 *            the weight of the first {@code Pose}
	 * @param p2
	 *            the second {@code Pose}
	 * @param w2
	 *            the weight of the second {@code Pose}
	 * @return the weighted sum of the specified {@code Pose}s
	 */
	public static Pose weightedSum(Pose p1, double w1, Pose p2, double w2) {
		if (p1 == null)
			return p2;
		if (p2 == null)
			return p1;
		double a1 = p1.directionalAngle();
		double a2 = p2.directionalAngle();
		if (a1 > a2 + Math.PI)
			a2 += 2 * Math.PI;
		else if (a2 > a1 + Math.PI)
			a1 += 2 * Math.PI;
		return new Pose(p1.x() * w1 + p2.x() * w2, p1.y() * w1 + p2.y() * w2, a1 * w1 + a2 * w2);
	}

}
