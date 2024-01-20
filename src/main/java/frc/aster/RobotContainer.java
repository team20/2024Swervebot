// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.aster;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.aster.commands.drive.TurnCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.aster.subsystems.DriveSubsystem;
import frc.aster.Constants.ControllerConstants;
import frc.aster.commands.drive.DefaultDriveCommand;

import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import hlib.drive.Pose;
import hlib.drive.Position;

public class RobotContainer implements frc.robot.util.RobotContainer {
	DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	// AprilTagSubsystem m_aprilTagSubsystem = new AprilTagSubsystem();
	PoseEstimationSubsystem m_poseEstimationSubsystem = new PoseEstimationSubsystem();
	Joystick m_operatorController = new Joystick(ControllerConstants.kOperatorControllerPort);

	public RobotContainer() {
		configureButtonBindings();
	}

	private void configureButtonBindings() {
		// new JoystickButton(m_operatorController,
		// ControllerConstants.Button.kTriangle)
		// .whileTrue(new TurnCommand(30));
		var target = new Position(6.809, -3.859);
		Supplier<Double> c = () -> {
			Pose pose = m_poseEstimationSubsystem.poseEstimated();
			if (pose != null) {
				System.out.println(pose.angleInDegrees(target) + ", " + pose.directionalAngleInDegrees());
				return pose.angleInDegrees(target) - pose.directionalAngleInDegrees();
			}
			return null;
		};
		new JoystickButton(m_operatorController, ControllerConstants.Button.kTriangle)
				.whileTrue(new TurnCommand(c));
	}

	public Command getAutonomousCommand() {
		return null;
	}
}
