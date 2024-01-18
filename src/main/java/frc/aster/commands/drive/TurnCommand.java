package frc.aster.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.aster.Constants.DriveConstants;
import frc.aster.subsystems.DriveSubsystem;

/**
 * The {@code TurnCommand} rotates the robot counter-clockwise to a specific
 * angle relative to its current heading. It
 * utilizes a {@code PIDController} to maintain precision in reaching the target
 * angle.
 * 
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 */
public class TurnCommand extends Command {

	/**
	 * {@code Supplier<Double>} that calculates the target angle in degrees by which
	 * the robot should rotate
	 * counter-clockwise (executed when the {@code TurnCommand} is submitted to the
	 * scheduler and initialized for
	 * execution)
	 */
	private Supplier<Double> m_targetAngleCalculator;

	/**
	 * The {@code PIDController} for controlling the rotation.
	 */
	private PIDController m_turnController;

	/**
	 * Constructs a {@code TurnCommand}.
	 * 
	 * @param targetAngle
	 *                    the target angle in degrees by which the robot should
	 *                    rotate counter-clockwise
	 */
	public TurnCommand(double targetAngle) {
		this(() -> targetAngle);
	}

	/**
	 * Constructs a {@code TurnCommand}.
	 * 
	 * @param targetAngleCalculator
	 *                              a {@code Supplier<Double>} that calculates the
	 *                              target angle in degrees by which the robot
	 *                              should
	 *                              rotate counter-clockwise (executed when the
	 *                              {@code TurnCommand} is submitted to the
	 *                              scheduler and
	 *                              initialized for execution)
	 */
	public TurnCommand(Supplier<Double> targetAngleCalculator) {
		m_targetAngleCalculator = targetAngleCalculator;
		m_turnController = new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD);
		m_turnController.setTolerance(DriveConstants.kTurnTolerance);
		m_turnController.enableContinuousInput(-180, 180);
		addRequirements(DriveSubsystem.get());
	}

	/**
	 * Is invoked whenever this {@code TurnCommand} is initially scheduled for
	 * execution.
	 */
	@Override
	public void initialize() {
		// DriveSubsystem.get().zeroHeading();
		// m_turnController.setIntegratorRange(-0.05, 0.05);
		double setPoint = DriveSubsystem.get().getHeading();
		try {
			setPoint += m_targetAngleCalculator.get();
		} catch (Exception e) {
		}
		m_turnController.setSetpoint(setPoint);
		SmartDashboard.putString("turn:initialize", String.format("set point: %.1f", setPoint));
	}

	/**
	 * Is invoked periodically while this {@code TurnCommand} is scheduled.
	 */
	@Override
	public void execute() {
		double heading = DriveSubsystem.get().getHeading();
		double turnSpeed = m_turnController.calculate(heading);
		// force the turn speed to be either <= -0.15 or >= 0.15
		turnSpeed = Math.abs(turnSpeed) < 0.15 ? Math.signum(turnSpeed) * 0.15 : turnSpeed;
		DriveSubsystem.get().tankDrive(-turnSpeed, turnSpeed);
		SmartDashboard.putString("turn:execute", String.format("heading: %.1f, turn speed: %.1f", heading, turnSpeed));
	}

	/**
	 * Is invoked once this {@code TurnCommand} is ended or interrupted.
	 * 
	 * @param interrupted
	 *                    indicates if this {@code TurnCommand} was interrupted
	 */
	@Override
	public void end(boolean interrupted) {
		DriveSubsystem.get().tankDrive(0, 0);
	}

	/**
	 * Determines whether or not this {@code TurnCommand} needs to end.
	 * 
	 * @return {@code true} if this {@code TurnCommand} needs to end; {@code false}
	 *         otherwise
	 */
	@Override
	public boolean isFinished() {
		return m_turnController.atSetpoint();
	}
}