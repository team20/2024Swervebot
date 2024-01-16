package hlib.drive;

/**
 * A {@code Pose} represents a pose (i.e., a position with a direction) in a
 * 2-dimensional space.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class Pose extends Position {

	/**
	 * The directional angle (in radians) of this {@code Pose}.
	 */
	protected double directionalAngle;

	/**
	 * Constructs a {@code Pose}.
	 * 
	 * @param x
	 *                         the x-coordinate value of the {@code Pose}
	 * @param y
	 *                         the y-coordinate value of the {@code Pose}
	 * @param directionalAngle
	 *                         the directional angle (in radians) of the
	 *                         {@code Pose}
	 */
	public Pose(double x, double y, double directionalAngle) {
		super(x, y);
		this.directionalAngle = normalize(directionalAngle);
	}

	/**
	 * Constructs a {@code Pose}.
	 * 
	 * @param p
	 *                         the {@code Position} of the {@code Pose}
	 * @param directionalAngle
	 *                         the directional angle (in radians) of the
	 *                         {@code Pose}
	 */
	public Pose(Position p, double directionalAngle) {
		this(p.x, p.y, directionalAngle);
	}

	/**
	 * Returns the directional angle (in degreees) of this {@code Pose}.
	 * 
	 * @return the directional angle (in degrees) of this {@code Pose}
	 */
	public double directionalAngleInDegrees() {
		return directionalAngle * 180 / Math.PI;
	}

	/**
	 * Returns the directional angle (in radians) of this {@code Pose}.
	 * 
	 * @return the directional angle (in radians) of this {@code Pose}
	 */
	public double directionalAngle() {
		return directionalAngle;
	}

	/**
	 * Returns a {@code String} representation of this {@code Pose}.
	 * 
	 * @return a {@code String} representation of this {@code Pose}
	 */
	@Override
	public String toString() {
		return String.format("(%.3f, %.3f, %.1f degrees)", x, y, directionalAngle * 180 / Math.PI);
	}

	/**
	 * Determines whether or not the given {@code Object} is equal to this
	 * {@code Pose}.
	 * 
	 * @return {@code true} if the given {@code Object} is equal to this
	 *         {@code Pose}; {@code false} otherwise
	 */
	@Override
	public boolean equals(Object o) {
		if (o instanceof Pose) {
			Pose p = (Pose) o;
			return super.equals(p) && this.directionalAngle == p.directionalAngle;
		} else
			return false;
	}

	/**
	 * Adds this {@code Pose} and the specified {@code Pose}.
	 * 
	 * @param other
	 *              a {@code Pose}
	 * @return the {@code Pose} after adding this {@code Pose} and the specified
	 *         {@code Pose}
	 */
	public Pose add(Pose other) {
		Position o = other.rotate(directionalAngle + other.directionalAngle);
		return new Pose(x + o.x, y + o.y, directionalAngle + other.directionalAngle);
	}

	/**
	 * Returns the {@code Pose} after moving this {@code Pose} along the direction
	 * of this {@code Pose} by the specified
	 * magnitude.
	 * 
	 * @param magnitude
	 *                  the magnitude of movement
	 * @return the {@code Pose} after moving this {@code Pose} along the direction
	 *         of this {@code Pose} by the specified
	 *         magnitude
	 */
	public Pose move(double magnitude) {
		return new Pose(x + magnitude * Math.cos(directionalAngle), y + magnitude * Math.sin(directionalAngle),
				directionalAngle);
	}

	/**
	 * Converts the specified angle (in radians) to the corresponding angle (in
	 * radians) between -Math.PI (exclusive)
	 * and Math.PI (inclusive).
	 * 
	 * @param angle
	 *              an angle (in radians)
	 * @return the corresponding angle (in radians) between -Math.PI (exclusive) and
	 *         Math.PI (inclusive)
	 */
	public static double normalize(double angle) {
		if (angle <= -Math.PI) {
			angle -= Math.floor(angle / (2 * Math.PI)) * (2 * Math.PI);
			return angle > Math.PI ? angle - 2 * Math.PI : angle;
		} else if (angle > Math.PI) {
			angle -= Math.floor(angle / (2 * Math.PI)) * (2 * Math.PI);
			return angle > Math.PI ? angle - 2 * Math.PI : angle;
		} else
			return angle;
	}

	/**
	 * Determines whether or not this {@code Pose} has an invalid value (e.g., NaN).
	 * 
	 * @return {@code true} if this {@code Pose} has an invalid value (e.g., NaN);
	 *         {@code false} otherwise
	 */
	public boolean isInvalid() {
		return x == Double.NaN || y == Double.NaN || directionalAngle == Double.NaN;
	}

}
