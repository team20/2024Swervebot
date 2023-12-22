// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.SwerveConstants.*;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;

public class DriveSubsystem extends SubsystemBase {
	private SwerveModule m_frontLeftSwerveModule;
	private SwerveModule m_frontRightSwerveModule;
	private SwerveModule m_backLeftSwerveModule;
	private SwerveModule m_backRightSwerveModule;

	// Creating my kinematics object using the module locations
	private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
			kFrontLeftLocation, kFrontRightLocation, kBackLeftLocation, kBackRightLocation);
	private static DriveSubsystem s_subsystem;
	private AHRS m_gyro = new AHRS(SPI.Port.kMXP);
	private MedianFilter filter = new MedianFilter(5);

	/** Creates a new DriveSubsystem. */
	public DriveSubsystem() {
		// Singleton
		if (s_subsystem != null) {
			try {
				throw new Exception("Motor subsystem already initalized!");
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		s_subsystem = this;

		// Initialize modules
		{
			m_frontLeftSwerveModule = new SwerveModule(
					kFrontLeftCANCoderPort,
					kFrontLeftDrivePort,
					kFrontLeftSteerPort,
					FrontLeftZero,
					kFrontLeftDriveInverted);

			m_frontRightSwerveModule = new SwerveModule(
					kFrontRightCANCoderPort,
					kFrontRightDrivePort,
					kFrontRightSteerPort,
					FrontRightZero,
					kFrontRightDriveInverted);

			m_backLeftSwerveModule = new SwerveModule(
					kBackLeftCANCoderPort,
					kBackLeftDrivePort,
					kBackLeftSteerPort,
					BackLeftZero,
					kBackLeftDriveInverted);

			m_backRightSwerveModule = new SwerveModule(
					kBackRightCANCoderPort,
					kBackRightDrivePort,
					kBackRightSteerPort,
					BackRightZero,
					kBackRightDriveInverted);
		}
		new Thread(() -> {
			try {
				Thread.sleep(1000);
				m_gyro.reset();
			} catch (Exception e) {
			}
		});
		resetEncoders();
	}

	public static DriveSubsystem get() {
		return s_subsystem;
	}

	public double getHeading() {
		return -m_gyro.getYaw();
	}

	public void resetHeading() {
		m_gyro.reset();
	}

	public void resetEncoders() {
		// Zero drive encoders
		m_frontLeftSwerveModule.getDriveEncoder().setPosition(0);
		m_frontRightSwerveModule.getDriveEncoder().setPosition(0);
		m_backLeftSwerveModule.getDriveEncoder().setPosition(0);
		m_backRightSwerveModule.getDriveEncoder().setPosition(0);
	}

	public void setWheelRotationToZeroDegrees() {
		setSteerMotors(0, 0, 0, 0);
	}

	public SwerveModuleState[] drive(ChassisSpeeds speeds) {
		return m_kinematics.toSwerveModuleStates(speeds);
	}

	/**
	 * Makes our drive motors spin at the specified speeds
	 * 
	 * @param frontLeftSpeed
	 *                        Speed of the front left wheel in duty cycles [-1, 1]
	 * @param frontRightSpeed
	 *                        Speed of the front right wheel in duty cycles [-1, 1]
	 * @param backLeftSpeed
	 *                        Speed of the back left wheel in duty cycles [-1, 1]
	 * @param backRightSpeed
	 *                        Speed of the back right wheel in duty cycles [-1, 1]
	 */
	public void setDriveMotors(double frontLeftSpeed, double frontRightSpeed, double backLeftSpeed,
			double backRightSpeed) {
		m_frontLeftSwerveModule.getDriveMotor().set(frontLeftSpeed * kDriveScale);
		m_frontRightSwerveModule.getDriveMotor().set(frontRightSpeed * kDriveScale);
		m_backLeftSwerveModule.getDriveMotor().set(backLeftSpeed * kDriveScale);
		m_backRightSwerveModule.getDriveMotor().set(backRightSpeed * kDriveScale);
	}

	/**
	 * Sets the target angles in degrees for each wheel on the robot
	 * 
	 * @param frontLeftAngle  The target angle of the front left wheel in degrees
	 * @param frontRightAngle The target angle of the front right wheel in degrees
	 * @param backLeftAngle   The target angle of the back left wheel in degrees
	 * @param backRightAngle  The target angle of the back right wheel in degrees
	 */
	public void setSteerMotors(double frontLeftAngle, double frontRightAngle, double backLeftAngle,
			double backRightAngle) {
		m_frontLeftSwerveModule.getPIDController().setSetpoint(frontLeftAngle);
		m_frontRightSwerveModule.getPIDController().setSetpoint(frontRightAngle);
		m_backLeftSwerveModule.getPIDController().setSetpoint(backLeftAngle);
		m_backRightSwerveModule.getPIDController().setSetpoint(backRightAngle);
	}

	/**
	 * Sets module states for each swerve module.
	 * 
	 * @param moduleStates The module states, in order of FL, FR, BL, BR
	 */
	public void setSwerveStates(SwerveModuleState[] moduleStates) {
		m_frontLeftSwerveModule.setModuleState(moduleStates[0]);
		m_frontRightSwerveModule.setModuleState(moduleStates[1]);
		m_backLeftSwerveModule.setModuleState(moduleStates[2]);
		m_backRightSwerveModule.setModuleState(moduleStates[3]);
	}

	/**
	 * Recalculates the PID output, and uses it to drive our steer motors. Also logs
	 * wheel rotations, and PID setpoints
	 */
	@Override
	public void periodic() {
		// For each of our steer motors, feed the current angle of the wheel into its
		// PID controller, and use it to calculate the duty cycle for its motor, and
		// spin the motor

		m_frontLeftSwerveModule.getSteerMotor().set(m_frontLeftSwerveModule.getPIDController()
				.calculate(360 * m_frontLeftSwerveModule.getCANCoder().getAbsolutePosition().getValueAsDouble()));
		m_frontRightSwerveModule.getSteerMotor().set(m_frontRightSwerveModule.getPIDController()
				.calculate(360 * m_frontRightSwerveModule.getCANCoder().getAbsolutePosition().getValueAsDouble()));
		m_backLeftSwerveModule.getSteerMotor().set(m_backLeftSwerveModule.getPIDController()
				.calculate(360 * m_backLeftSwerveModule.getCANCoder().getAbsolutePosition().getValueAsDouble()));
		m_backRightSwerveModule.getSteerMotor().set(m_backRightSwerveModule.getPIDController()
				.calculate(360 * m_backRightSwerveModule.getCANCoder().getAbsolutePosition().getValueAsDouble()));
	}
}
