package frc.robot;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.LoggedTunableNumber;

public final class Constants {
	/**
	 * Controller joystick range: -1 to 1 NEO max: 5676 RPM Vortex max: 6326 RPM
	 * Swerve top speed = 14.5 ft/s = 4.5 m/s
	 *
	 * Gyro: - Forward = ? - Left = ? - Counterclockwise = ? Odometry - Forward = x+
	 * - Left = y+ - Counterclockwise = z+
	 */

	public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;
	public static final RobotType currentRobot = (currentMode == Mode.REAL) ? RobotType.REALBOT : RobotType.SIMBOT;
	public static final boolean tuningMode = false;

	public static final ModuleType powerDistributionType = ModuleType.kRev;
	public static final boolean fieldOriented = true;
	public static final boolean lightsExist = true;

	public static final class OIConstants {
		public static final ControllerType controllerTypeDriver = ControllerType.XBOX;
		public static final ControllerType controllerTypeOperator = ControllerType.XBOX;

		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;

		public static final double kDeadband = 0.1;
	}

	public static final class CAN {
		public static final int kIntakePort = 4;

		public static final int kLeftClimberPort = 5;
		public static final int kRightClimberPort = 6;

		public static final int kFlywheelPort = 7;

		public static final int kArm1Port = 9;
		public static final int kArm2Port = 10;

		public static final int kFrontLeftDriveMotorPort = 11;
		public static final int kFrontRightDriveMotorPort = 12;
		public static final int kBackLeftDriveMotorPort = 13;
		public static final int kBackRightDriveMotorPort = 14;

		public static final int kFrontLeftTurningMotorPort = 21;
		public static final int kFrontRightTurningMotorPort = 22;
		public static final int kBackLeftTurningMotorPort = 23;
		public static final int kBackRightTurningMotorPort = 24;
	}

	public static final class CameraConstants {
		public static final Transform3d kRobotToCameraStraight = new Transform3d(
				new Translation3d(Units.inchesToMeters(-10.472), Units.inchesToMeters(-4.102),
						Units.inchesToMeters(7.591)),
				new Rotation3d(0, Math.PI / 9, Math.PI));
		public static final Transform3d kRobotToCameraRight = new Transform3d(
				new Translation3d(Units.inchesToMeters(-9.302), Units.inchesToMeters(-10.747),
						Units.inchesToMeters(7.014)),
				new Rotation3d(0, Math.PI / 6, 7 * Math.PI / 6));
		public static final Transform3d kRobotToCameraLeft = new Transform3d(
				new Translation3d(Units.inchesToMeters(-9.302), Units.inchesToMeters(10.747),
						Units.inchesToMeters(7.014)),
				new Rotation3d(0, Math.PI / 6, 5 * Math.PI / 6));
	}

	public static final class ModuleConstants {
		public static final double kWheelDiameterMeters = Units.inchesToMeters(3.75);
		public static final double kDriveMotorGearRatio = 1 / 6.75;
		public static final double kTurningMotorGearRatio = 1 / (150 / 7.0);

		public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
		public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
		public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
		public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

		public static final double kPTurning = 0.5;
	}

	public static final class OdometryConstants {
		public static final double kOdometerDriftCorrection = 0.25;

		public static final Translation3d kBlueStagePosition = new Translation3d(0, 5.551894, 2.457194);
		public static final Translation3d kRedStagePosition = new Translation3d(16.518826, 5.551894, 2.457194);

		public static final double kRobotElevation = 8.259413;
	}

	public static final class DriveConstants {
		// Distance between right and left wheels:
		public static final double kTrackWidth = Units.inchesToMeters(20.75);
		// Distance between front and back wheels:
		public static final double kWheelBase = Units.inchesToMeters(20.75);
		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2, kTrackWidth / 2), // Front left (+/+)
				new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // Front right (+/-)
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // Back left (-/+)
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // Back right (-/-)

		// Edit depending on specs of MK4i
		public static final boolean kFrontLeftTurningEncoderReversed = true;
		public static final boolean kFrontRightTurningEncoderReversed = true;
		public static final boolean kBackLeftTurningEncoderReversed = true;
		public static final boolean kBackRightTurningEncoderReversed = true;

		public static final boolean kFrontLeftDriveEncoderReversed = false;
		public static final boolean kFrontRightDriveEncoderReversed = true;
		public static final boolean kBackLeftDriveEncoderReversed = false;
		public static final boolean kBackRightDriveEncoderReversed = true;

		public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
		public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
		public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
		public static final int kBackRightDriveAbsoluteEncoderPort = 3;

		public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
		public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
		public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
		public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

		public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -1.677 - 0.005 - 0.004 + 0.011;
		public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -0.804 + 0.069 + 0.071 - 0.121 - 0.008;
		public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 1.452 - 0.01 + 0.046 - 0.016 + 0.014;
		public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 2.132 - 0.064 + 0.017 - 0.012 - 0.017;

		public static final double kPhysicalMaxSpeedMetersPerSecond = 4.5;
		public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 3 * Math.PI;

		public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
		public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxSpeedMetersPerSecond * 0.4;

		public static final double swerveDemoScaleFactor = 0.25;
		public static final double kDemoTeleDriveMaxSpeedMetersPerSecond = kTeleDriveMaxSpeedMetersPerSecond
				* swerveDemoScaleFactor;
		public static final double kDemoTeleDriveMaxAngularSpeedRadiansPerSecond = kTeleDriveMaxAngularSpeedRadiansPerSecond
				* (swerveDemoScaleFactor * 2);

		public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
		public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

		public static final HolonomicPathFollowerConfig kHolonomicConfig = new HolonomicPathFollowerConfig(
				new PIDConstants(0.25, 0.0, 0.0), // Translation PID constants
				new PIDConstants(0.5, 0.0, 0.0), // Rotation PID constants
				kTeleDriveMaxSpeedMetersPerSecond, // Max module speed, in m/s
				// Drive base radius in meters. Distance from robot center to furthest module.
				Math.sqrt(Math.pow(kTrackWidth, 2) + Math.pow(kWheelBase, 2)) / 2, new ReplanningConfig());
	}

	public static final class ClimberConstants {
		public static final LoggedTunableNumber kClimbVoltage = new LoggedTunableNumber("Climber Voltage", 5);

		public static final double kClimberPositionReduction = 1.0;
		public static final double kClimberVelocityReduction = 1.0;

		public static final int kLeftClimberAbsoluteEncoderPort = 0;
		public static final int kRightClimberAbsoluteEncoderPort = 1;

		public static final double kLeftClimberEncoderOffset = 0.0;
		public static final double kRightClimberEncoderOffset = 0.0;
	}

	public static final class ArmConstants {
		public static final int kArmEncoderPort = 5;
		public static final double kArmEncoderOffset = -2.65016;
		public static final boolean kArmEncoderReversed = true;

		// Physics
		public static final double kArmLength = 0.58;
		public static final double kArmReduction = 144.0;
		public static final double kArmJKgMetersSquared = 0.515;

		// Constraints
		public static final double kMaxArmAngle = -0.084;
		public static final double kMinArmAngle = -1.76625;

		public static final LoggedTunableNumber kMaxSpeed = new LoggedTunableNumber("Arm/Max Speed", 4.5);
		public static final LoggedTunableNumber kMaxAcceleration = new LoggedTunableNumber("Arm/Max Acceleration", 1);
		public static final LoggedTunableNumber kManualVoltage = new LoggedTunableNumber("Arm/ManualVoltage", 8);

		// Arm presets
		public static final double kTaxiAngle = -1.175;
		public static final double kRoombaAngle = -0.13;
		public static final double kSpeakerFrontAngle = -0.31;
		public static final double kSpeakerAngleAngle = -0.31;
		public static final double kSpeakerStageAngle = -0.75;
		public static final double kAmpAngle = -1.620;
		public static final double kTrapAngle = -0.13816;
		public static final double kSourceAngle = -1.37;

		// Control
		public static final LoggedTunableNumber kArmPIDTolerance = new LoggedTunableNumber("Arm/PID Tolerance", 0.0001);
		public static final double kSetpointTolerance = 0.2;

		// Arm characterization
		public static final SysIdRoutine.Config characterizationConfig = new SysIdRoutine.Config(
				Volts.of(2).per(Seconds.of(1)), Volts.of(5), Seconds.of(5));
	}

	public static final class ShooterConstants {

		public static final double shooterDemoScaleFactor = 0.25;

		public static final double kShooterReduction = 1.0;

		public static final double kLowVelocity = 800;
		public static final double kVelocityThreshold = 0.8;
		public static final double kVelocityThresholdLow = 0.6;

		public static final LoggedTunableNumber kShooterPIDTolerance = new LoggedTunableNumber("Shooter/PID Tolerance",
				0.5);

		public static final LoggedTunableNumber kAmpRPM = new LoggedTunableNumber("Shooter/Amp RPM", 750);
		public static final LoggedTunableNumber kSpeakerFrontRPM = new LoggedTunableNumber("Shooter/SpeakerFront RPM",
				4500);
		public static final LoggedTunableNumber kSpeakerAngleRPM = new LoggedTunableNumber("Shooter/SpeakerAngle RPM",
				4500);
		public static final LoggedTunableNumber kSpeakerStageRPM = new LoggedTunableNumber("Shooter/SpeakerStage RPM",
				4000);
		public static final LoggedTunableNumber kTrapRPM = new LoggedTunableNumber("Shooter/Manual RPM", 1000);
		public static final LoggedTunableNumber kIdleRPM = new LoggedTunableNumber("Shooter/Idle RPM", 4500);

		// TODO: Change to computed value
		public static final LoggedTunableNumber kAutoShooterExitVel = new LoggedTunableNumber(
				"Auto Shooter Exit Velocity", 10);

		// Constants for auto-aiming
		public static final boolean ampUpperEntry = false;
		public static final boolean speakerUpperEntry = true;

		public static final double ampHoriEntryRange = Math.PI / 6;
		public static final double speakerHoriEntryRange = Math.PI / 2;
	}

	public static final class IntakeConstants {
		public static final double kIntakeReduction = 4.0 * 3.0;
		public static final int kSensorPort = 8;

		public static final LoggedTunableNumber kIntakeVoltage = new LoggedTunableNumber("Intake/Voltage", 10);
	}

	public static final class AutoConstants {
		public static final double kAutoShootTime = 0.65;
		public static final double kAutoArmTime = 1.0;
		public static final double kAutoIntakeTime = 0.75;
		public static final double kAutoRetractTime = 0.1;
	}

	public static final class GainsConstants {
		public static final Gains flywheelGains = switch (currentRobot) {
			case REALBOT -> new Gains(0.00000065361, 0.0, 0.0, 0.0091151, 0.0018015, 0.0, 0.0);
			case SIMBOT -> new Gains(1.0, 0.0, 0.0, 0.009078, 0.00103, 0.0, 0.0);
		};

		public static final Gains armGains = switch (currentRobot) {
			case REALBOT -> new Gains(15, 0.0, 0, 0.016186, 0.02131, 0.087119, 1.4338);
			case SIMBOT -> new Gains(1.0, 0.0, 0.0, 0.009078, 2.77, 0.06, 1.07);
		};
	}

	public record Gains(double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {
	}

	public static enum RobotType {
		/** Physical robot */
		REALBOT,
		/** Simulated robot */
		SIMBOT
	}

	public static enum Mode {
		/** Running on a real robot */
		REAL,
		/** Running a simulator */
		SIM,
		/** Replaying from a log file */
		REPLAY
	}

	public static enum ControllerType {
		XBOX, LOGITECH, JOYSTICK
	}
}
