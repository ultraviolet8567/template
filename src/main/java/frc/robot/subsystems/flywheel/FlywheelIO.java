package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
	@AutoLog
	class FlywheelIOInputs {
		public double velocityRPM = 0.0;
		public double targetVelocityRPM = 0.0;
		public double appliedVoltage = 0.0;
		public double[] currentAmps = new double[]{0.0};
		public double[] tempCelsius = new double[]{0.0};

		public double rotations = 0.0;
	}

	/** Updates the values in the FlywheelIOInputs class */
	default void updateInputs(FlywheelIOInputs inputs) {
	}

	/** Sets axle motor voltage */
	default void setInputVoltage(double volts) {
	}

	/** Stops the motor */
	default void stop() {
	}

	/** Sets the PID and feed-forward parameters */
	default void setGains(double kP, double kI, double kD, double ffkS, double ffkV) {
	}

	/** Sets the idle mode of the motor */
	default void setBrakeMode(boolean brake) {
	}

	/** Run flywheels at exact velocity */
	default void setVelocity(double targetVel) {
	}
}
