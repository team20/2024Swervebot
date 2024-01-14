// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.aster;

import edu.wpi.first.wpilibj2.command.Command;
import frc.aster.subsystems.DriveSubsystem;
import frc.aster.commands.drive.DefaultDriveCommand;

import frc.robot.subsystems.AprilTagSubsystem;

public class RobotContainer implements frc.robot.util.RobotContainer {
	private DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private AprilTagSubsystem m_aprilTagSubsystem = new AprilTagSubsystem();

	public RobotContainer() {

	}

	private void configureButtonBindings() {

	}

	// TODO get auto command from auto chooser
	public Command getAutonomousCommand() {
		return null;
	}
}
