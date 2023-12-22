import static org.junit.jupiter.api.Assertions.*;

import java.util.Arrays;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveTest {
	@Test
	void testModuleStates() {
		var driveSubsystem = new DriveSubsystem();
		var expectedStates = new SwerveModuleState[4];

		// Go Northwest
		var nwStates = driveSubsystem.drive(new ChassisSpeeds(1, 1, 0), false);
		Arrays.fill(expectedStates, new SwerveModuleState(1, Rotation2d.fromDegrees(45)));
		assertArrayEquals(expectedStates, nwStates);

		// Go Northeast
		var neStates = driveSubsystem.drive(new ChassisSpeeds(1, -1, 0), false);
		Arrays.fill(expectedStates, new SwerveModuleState(1, Rotation2d.fromDegrees(-45)));
		assertArrayEquals(expectedStates, neStates);

		// Go Southwest
		var swStates = driveSubsystem.drive(new ChassisSpeeds(-1, 1, 0), false);
		Arrays.fill(expectedStates, new SwerveModuleState(1, Rotation2d.fromDegrees(135)));
		assertArrayEquals(expectedStates, swStates);

		// Go Southwest
		var seStates = driveSubsystem.drive(new ChassisSpeeds(-1, -1, 0), false);
		Arrays.fill(expectedStates, new SwerveModuleState(1, Rotation2d.fromDegrees(225)));
		assertArrayEquals(expectedStates, seStates);
	}

}
