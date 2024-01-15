package hlib.drive;

/**
 * A {@code PoseEstimator} can estimate the {@code Pose} of a {@code Robot}.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class PoseEstimator {

	/**
	 * The distance threshold for outlier detection (i.e., difference in x- or y-coordinates of the {@code Pose} from
	 * LimeLight compared to the {@code Pose} that has been estimated in order for the {@code Pose} from LimeLight to be
	 * considered an outlier).
	 */
	double distanceThreshold;

	/**
	 * The {@code Pose} of the {@code Robot} that has been estimated by this {@code PoseEstimator}.
	 */
	Pose poseEstimated = null;

	/**
	 * The {@code PoseErrorTracker} used by this {@code PoseEstimator}.
	 */
	PoseErrorTracker errorTracker = new PoseErrorTracker();

	/**
	 * The number of occasions where the {@code Pose} of the robot was detected.
	 */
	int posesDetected = 0;

	/**
	 * The number of occasions where the {@code Pose} of the robot was not detected.
	 */
	int poseDetectionFailures = 0;

	/**
	 * The number of {@code Pose} outliers that have been detected by this {@code PoseEstimator}.
	 */
	int outliers = 0;

	/**
	 * The start time (in milliseconds) of this {@code PoseEstimator}.
	 */
	long startTime = System.currentTimeMillis();

	/**
	 * Constructs a {@code PoseEstimator}.
	 * 
	 * @param distanceThreshold
	 *            the distance threshold for outlier detection (i.e., the difference in x- or y-coordinates of the
	 *            {@code Pose} from LimeLight compared to the {@code Pose} that has been estimated in order for the
	 *            {@code Pose} from LimeLight to be considered an outlier)
	 */
	public PoseEstimator(double distanceThreshold) {
		this.distanceThreshold = distanceThreshold;
	}

	/**
	 * Returns the {@code Pose} of a {@code Robot} estimated by this {@code PoseEstimator}.
	 * 
	 * @return the {@code Pose} of a {@code Robot} estimated by this {@code PoseEstimator}
	 */
	public Pose poseEstimated() {
		return poseEstimated;
	}

	/**
	 * Updates this {@code PoseEstimator} based on the specified {@code Pose}.
	 * 
	 * @param poseDetected
	 *            the {@code Pose} from the LimeLight
	 * @return {@code false} if the specified {@code Pose} seems like an outlier (because either the x- or the
	 *         y-coordinate value of the {@code Pose} is different by more than the threshold compared to the
	 *         {@code Pose} that has been estimated) and thus rejected; {@code true} if this {@code RobotPoseEstimator}
	 *         is updated based on the specified {@code Pose}
	 */
	public final synchronized boolean update(Pose poseDetected) {
		if (poseDetected != null)
			posesDetected++;
		else
			poseDetectionFailures++;
		if (isOutlier(poseDetected))
			return false;
		setPoseEstimated(poseDetected);
		return true;
	}

	/**
	 * Returns the rate (the number of poses per second) at which the {@code Pose} of the robot has been detected.
	 * 
	 * @return the rate (the number of poses per second) at which the {@code Pose} of the robot has been detected
	 */
	public double poseDetectionRate() {
		double time = 0.001 * (System.currentTimeMillis() - startTime);
		if (time > 0)
			return posesDetected / time;
		else
			return 0;
	}

	/**
	 * Returns the rate (the number of failures per second) at which the {@code Pose} of the robot has not been
	 * detected.
	 * 
	 * @return the rate (the number of failures per second) at which the {@code Pose} of the robot has not been detected
	 */
	public double poseDetectionFailureRate() {
		double time = 0.001 * (System.currentTimeMillis() - startTime);
		if (time > 0)
			return poseDetectionFailures / time;
		else
			return 0;
	}

	/**
	 * Returns the number of {@code Pose} outliers that have been detected by this {@code PoseEstimator}.
	 * 
	 * @return the number of {@code Pose} outliers that have been detected by this {@code PoseEstimator}
	 */
	public int outliers() {
		return outliers;
	}

	/**
	 * Returns the largest differences (in x- and y-coordinate values as well as directional angle values) between the
	 * {@code Pose} estimated by this {@code RobotPoseEstimator} and the {@code Pose}s from the LimeLight.
	 * 
	 * @return the largest differences (in x- and y-coordinate values as well as directional angle values) between the
	 *         {@code Pose} estimated by this {@code RobotPoseEstimator} and the {@code Pose}s from the LimeLight
	 */
	public Pose largestPoseInconsistency() {
		return errorTracker.largestPoseError();
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
		if (poseDetected == null || this.poseEstimated == null)
			return false;
		if (poseDetected.isInvalid())
			return true;
		Pose error = PoseErrorTracker.error(poseDetected, this.poseEstimated);
		if (Math.abs(error.x()) > distanceThreshold || Math.abs(error.y()) > distanceThreshold) {
			outliers++;
			return true;
		} else
			return false;
	}

	/**
	 * Updates the {@code Pose} that has been updated by this {@code PoseEstimator} based on the specified
	 * {@code Pose} from the LimeLight
	 * 
	 * @param poseDetected
	 *            the {@code Pose} from the LimeLight
	 */
	protected void setPoseEstimated(Pose poseDetected) {
		if (poseDetected != null) {
			errorTracker.update(poseDetected, this.poseEstimated);
			this.poseEstimated = poseDetected;
		}
	}

	/**
	 * Returns the number of detected {@code Pose}s that have been given to this {@code PoseEstimator}.
	 * 
	 * @return the number of detected {@code Pose}s that have been given to this {@code PoseEstimator}
	 */
	public int posesDetected() {
		return this.posesDetected;
	}

}
