package hlib.drive;

/**
 * A {@code PoseEstimatorInconsistencyTolerant} can estimate the {@code Pose} of a {@code Robot}.
 * 
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 */
public class PoseEstimatorInconsistencyTolerant extends PoseEstimator {

	/**
	 * The number of rejections before resetting this {@code PoseEstimatorInconsistencyTolerant}.
	 */
	int relectionLimit;

	/**
	 * The number of rejections of the {@code Pose}s from the LimeLight.
	 */
	int rejections = 0;

	/**
	 * Constructs a {@code PoseEstimatorInconsistencyTolerant}.
	 * 
	 * @param distanceThreshold
	 *            the distance threshold for outlier detection (i.e., the difference in x- or y-coordinates of the
	 *            {@code Pose} from LimeLight compared to the {@code Pose} that has been estimated in order for the
	 *            {@code Pose} from LimeLight to be considered an outlier)
	 * @param rejectionLimit
	 *            the number of rejections before resetting the {@code RobotPoseEstimatorInconsistencyTolerant}
	 */

	public PoseEstimatorInconsistencyTolerant(double distanceThreshold, int rejectionLimit) {
		super(distanceThreshold);
		this.relectionLimit = rejectionLimit;
	}

	/**
	 * Determines whether or not the specified {@code Pose} is an outlier.
	 * 
	 * @param poseDetected
	 *            the {@code Pose} from the LimeLight
	 * @return {@code true} if either the x- or the y-coordinate value of the {@code Pose} is different by more than the
	 *         threshold compared to the {@code Pose} that has been estimated; {@code false} otherwise (i.e., the
	 *         specified {@code Pose} is not an outlier)
	 */
	protected boolean isOutlier(Pose poseDetected) {
		if (super.isOutlier(poseDetected)) {
			if (++rejections > relectionLimit)
				reset();
			return true;
		}
		if (poseDetected != null && this.poseEstimated != null)
			rejections = 0;
		return false;
	}

	/**
	 * Resets this {@code PoseEstimatorInconsistencyTolerant}.
	 */
	protected void reset() {
		System.out.println("resetting the pose estimator...");
		poseEstimated = null;
		rejections = 0;
	}

}
