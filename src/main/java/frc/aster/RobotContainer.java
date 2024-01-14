// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.aster;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.aster.commands.drive.TurnCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.aster.subsystems.DriveSubsystem;
import frc.aster.Constants.ControllerConstants;
import frc.aster.commands.drive.DefaultDriveCommand;

import frc.robot.subsystems.AprilTagSubsystem;

public class RobotContainer implements frc.robot.util.RobotContainer {
	DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	AprilTagSubsystem m_aprilTagSubsystem = new AprilTagSubsystem();
	Joystick m_operatorController = new Joystick(ControllerConstants.kOperatorControllerPort);

	public RobotContainer() {
		configureButtonBindings();
	}

	private void configureButtonBindings() {
		new JoystickButton(m_operatorController, ControllerConstants.Button.kTriangle)
				.whileTrue(new TurnCommand(30));
	}

	public Command getAutonomousCommand() {
		return null;
	}
}
