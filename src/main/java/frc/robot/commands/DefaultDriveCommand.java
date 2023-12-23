// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Swerve drive joystick command
 */
public class DefaultDriveCommand extends Command {
	private final DriveSubsystem m_driveSubsystem;
	private Supplier<Double> m_yAxisDrive;
	private Supplier<Double> m_xAxisDrive;
	private Supplier<Double> m_rotationAxis;

	public DefaultDriveCommand(DriveSubsystem driveSubsystem, Supplier<Double> xAxisDrive, Supplier<Double> yAxisDrive,
			Supplier<Double> rotationAxis) {
		m_driveSubsystem = driveSubsystem;
		m_yAxisDrive = yAxisDrive;
		m_xAxisDrive = xAxisDrive;
		m_rotationAxis = rotationAxis;
		addRequirements(m_driveSubsystem);
	}

	/**
	 * The main body of a command. Called repeatedly while the command is scheduled.
	 * Takes joystick inputs, calculates the wheel angles and speeds, moves the
	 * robots with those values, and logs the calculated numbers
	 */
	@Override
	public void execute() {
		// Get the forward, strafe, and rotation speed, using a deadband on the joystick
		// input so slight movements don't move the robot
		double fwdSpeed = -MathUtil.applyDeadband(m_yAxisDrive.get(), ControllerConstants.kDeadzone);
		double strSpeed = -MathUtil.applyDeadband(m_xAxisDrive.get(), ControllerConstants.kDeadzone);
		double rotSpeed = MathUtil.applyDeadband(m_rotationAxis.get(), ControllerConstants.kDeadzone);

		ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
				fwdSpeed, strSpeed, rotSpeed, Rotation2d.fromDegrees(DriveSubsystem.get().getHeading()));

		m_driveSubsystem.setSwerveStates(m_driveSubsystem.calculateModuleStates(speeds));
	}
}
