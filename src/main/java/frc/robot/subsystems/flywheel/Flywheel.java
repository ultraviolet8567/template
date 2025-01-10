package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Lights;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
	private final FlywheelIO io;
	private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

	private FlywheelMode mode;

	/*
	 * Initialize all components and one-time logic to be completed on boot-up here
	 */
	public Flywheel(FlywheelIO io) {
		this.io = io;
		this.mode = FlywheelMode.OFF;
	}

	/** Runs periodically (about once every 20 ms) */
	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Flywheel", inputs);
	}

	/** Set the speed mode of the flywheel */
	public void setSpeed(FlywheelMode mode) {
		this.mode = mode;
	}

	/** Runs the flywheel at the target velocity */
	public void run() {
		double targetVel = getTargetVelocity();
		io.setVelocity(targetVel);
	}

	/** Runs the flywheel at the given scale factor of the target velocity */
	public void run(double scaleDown) {
		double targetVel = scaleDown * getTargetVelocity();
		io.setVelocity(targetVel);
	}

	/**
	 * Checks whether the flywheel is at the intended velocity within tolerance. At
	 * lower velocities, the threshold is greater since the flywheels are less
	 * stable
	 */
	public boolean atVelocity() {
		double threshold = (getTargetVelocity() < ShooterConstants.kLowVelocity)
				? ShooterConstants.kVelocityThresholdLow
				: ShooterConstants.kVelocityThreshold;

		return inputs.velocityRPM >= threshold * inputs.targetVelocityRPM;
	}

	/** Return the intended flywheel velocity */
	public double getTargetVelocity() {
		double vel = switch (mode) {
			case OFF -> 0;
			case LOW -> 1000;
			case MEDIUM -> 2000;
			case HIGH -> 3000;
			default -> 0;
		};

		if (Lights.getInstance().isDemo && vel >= 800) {
			return ShooterConstants.shooterDemoScaleFactor * vel;
		} else {
			return vel;
		}
	}

	/** Stops the flywheel */
	public void stop() {
		mode = FlywheelMode.OFF;
		io.stop();
	}

	public static enum FlywheelMode {
		/** Off */
		OFF,
		/** Low speed */
		LOW,
		/** Medium speed */
		MEDIUM,
		/** High speed */
		HIGH
	}
}
