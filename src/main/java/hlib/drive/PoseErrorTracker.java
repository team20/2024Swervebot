package hlib.drive;

/**
 * A {@code PoseErrorTracker} can keep track of errors in {@code Pose}s.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class PoseErrorTracker {

	/**
	 * The {@code Pose} containing the largest error values in x- and y-coordinates and the directional angle.
	 */
	Pose largestPoseError = new Pose(0, 0, 0);

	/**
	 * Constructs a {@code PoseErrorTracker}.
	 */
	public PoseErrorTracker() {
	}

	/**
	 * Calculates the error between the specified {@code Pose}s.
	 * 
	 * @param pose
	 *            a {@code Pose} estimated
	 * @param reference
	 *            a {@code Pose} used as a reference
	 * @return a {@code Pose} containing the calculated values
	 */
	public static Pose error(Pose pose, Pose reference) {
		if (pose == null || reference == null)
			return null;
		return new Pose(pose.x() - reference.x(), pose.y() - reference.y(),
				pose.directionalAngle() - reference.directionalAngle());
	}

	/**
	 * Updates this {@code PoseErrorTracker} using the specified {@code Pose}s.
	 * 
	 * @param pose
	 *            a {@code Pose} estimated
	 * @param reference
	 *            a {@code Pose} used as a reference
	 */
	public void update(Pose pose, Pose reference) {
		update(error(pose, reference));
	}

	/**
	 * Returns the {@code Pose} containing the largest error values in x- and y-coordinates and the directional angle.
	 * 
	 * @return the {@code Pose} containing the largest error values in x- and y-coordinates and the directional angle
	 */
	public Pose largestPoseError() {
		return largestPoseError;
	}

	/**
	 * Updates this {@code PoseErrorTracker} using the specified {@code Pose} containing error values.
	 * 
	 * @param error
	 *            a {@code Pose} containing error values
	 */
	protected void update(Pose error) {
		if (error != null)
			this.largestPoseError = new Pose(maxError(error.x(), largestPoseError.x()),
					maxError(error.y(), largestPoseError.y()),
					maxError(error.directionalAngle(), largestPoseError.directionalAngle()));
	}

	/**
	 * Returns the maximum error value given the specified error values.
	 * 
	 * @param e1
	 *            an error value
	 * @param e2
	 *            an error value
	 * @return the maximum error value given the specified error values
	 */
	protected double maxError(double e1, double e2) {
		if (Math.abs(e1) > Math.abs(e2))
			return e1;
		else
			return e2;
	}

}
