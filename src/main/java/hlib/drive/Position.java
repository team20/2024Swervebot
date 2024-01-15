package hlib.drive;

/**
 * A {@code Position} represents a position in a 2-dimensional space.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class Position {

	/**
	 * The x-coordinate value of this {@code Position}.
	 */
	protected double x;

	/**
	 * The y-coordinate value of this {@code Position}.
	 */
	protected double y;

	/**
	 * Constructs a {@code Position}.
	 * 
	 * @param x
	 *            the x-coordinate value of the {@code Position}
	 * @param y
	 *            the y-coordinate value of the {@code Position}
	 */
	public Position(double x, double y) {
		this.x = x;
		this.y = y;
	}

	/**
	 * Returns a {@code String} representation of this {@code Position}.
	 * 
	 * @return a {@code String} representation of this {@code Position}
	 */
	@Override
	public String toString() {
		return String.format("(%.3f, %.3f)", x, y);
	}

	/**
	 * Determines whether or not the given {@code Object} is equal to this {@code Position}.
	 * 
	 * @return {@code true} if the given {@code Object} is equal to this {@code Position}; {@code false} otherwise
	 */
	@Override
	public boolean equals(Object o) {
		if (o instanceof Position) {
			Position p = (Position) o;
			return this.x == p.x && this.y == p.y;
		} else
			return false;
	}

	/**
	 * Returns the x-coordinate value of this {@code Position}.
	 * 
	 * @return the x-coordinate value of this {@code Position}
	 */
	public double x() {
		return x;
	}

	/**
	 * Returns the y-coordinate value of this {@code Position}.
	 * 
	 * @return the y-coordinate value of this {@code Position}
	 */
	public double y() {
		return y;
	}

	/**
	 * Returns the angle (in radians) between the x-axis and the line from the current {@code Position} to the specified
	 * {@code Position}.
	 * 
	 * @param p
	 *            a {@code Position}
	 * @return the angle (in radians) between the x-axis and the line from the current {@code Position} to the specified
	 *         {@code Position}
	 */
	public double angleTo(Position p) {
		return Math.atan2(p.y - y, p.x - x);
	}

	/**
	 * Returns the distance between this {@code Position} and the specified {@code Position}.
	 * 
	 * @param p
	 *            another {@code Position}
	 * @return the distance between this {@code Position} and the specified {@code Position}
	 */
	public double distance(Position p) {
		double x = this.x - p.x;
		double y = this.y - p.y;
		return Math.sqrt(x * x + y * y);
	}

	/**
	 * Returns the {@code Position} after rotating this {@code Position} about the origin by the specified angle.
	 * 
	 * @param angle
	 *            an angle
	 * @return the {@code Position} after rotating this {@code Position} about the origin by the specified angle
	 */
	public Position rotate(double angle) {
		return new Position(Math.cos(angle) * x - Math.sin(angle) * y, Math.sin(angle) * x + Math.cos(angle) * y);
	}

	/**
	 * Returns the {@code Position} after translating this {@code Position} by the specified {@code Position}.
	 * 
	 * @param p
	 *            another {@code Position}
	 * @return the {@code Position} after translating this {@code Position} by the specified {@code Position}
	 */
	public Position translate(Position p) {
		return new Position(x + p.x, y + p.y);
	}

}
