package frc.aster.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.aster.Constants.DriveConstants;
import frc.aster.subsystems.DriveSubsystem;

public class TurnCommand extends Command {
	/** Creates a new TurnCommand. */
	private double m_targetAngle;
	private PIDController m_turnController = new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI,
			DriveConstants.kTurnD);

	/** Creates a new TurnCommand. */
	public TurnCommand(double targetAngle) {
		m_targetAngle = targetAngle;
		addRequirements(DriveSubsystem.get());
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		DriveSubsystem.get().zeroHeading();
		m_turnController.setSetpoint(m_targetAngle);
		m_turnController.enableContinuousInput(-180, 180);
		m_turnController.setTolerance(DriveConstants.kTurnTolerance);
		// m_turnController.setIntegratorRange(-0.05, 0.05);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double currAngle = DriveSubsystem.get().getHeading();
		double turnSpeed = m_turnController.calculate(currAngle);

		// if turn speed is less than 0.1 make it 0.1 in the right direction
		turnSpeed = Math.abs(turnSpeed) < 0.15 ? Math.signum(turnSpeed) * 0.15 : turnSpeed;

		SmartDashboard.putNumber("Heading", currAngle);
		DriveSubsystem.get().tankDrive(-turnSpeed, turnSpeed);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		DriveSubsystem.get().tankDrive(0, 0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return m_turnController.atSetpoint();
	}
}