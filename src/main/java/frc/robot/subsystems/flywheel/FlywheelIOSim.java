package frc.robot.subsystems.flywheel;

import static frc.robot.Constants.GainsConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.ShooterConstants;

public class FlywheelIOSim implements FlywheelIO {
	private final FlywheelSim flywheelSim;

	private final PIDController flywheelPID;
	private SimpleMotorFeedforward flywheelFF;

	private double appliedVoltage;

	public FlywheelIOSim() {
		System.out.println("[Init] Creating FlywheelIOSim");

		flywheelSim = new FlywheelSim(DCMotor.getNEO(1), ShooterConstants.kShooterReduction, 0.1);
		flywheelPID = new PIDController(flywheelGains.kP(), flywheelGains.kI(), flywheelGains.kD());
		flywheelFF = new SimpleMotorFeedforward(flywheelGains.ffkS(), flywheelGains.ffkV());

		appliedVoltage = 0.0;
	}

	@Override
	public void updateInputs(FlywheelIOInputs inputs) {
		flywheelSim.update(0.02);

		inputs.velocityRPM = flywheelSim.getAngularVelocityRPM();
		inputs.appliedVoltage = appliedVoltage;
		inputs.currentAmps = new double[]{flywheelSim.getCurrentDrawAmps()};
	}

	@Override
	public void setInputVoltage(double volts) {
		appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
		flywheelSim.setInputVoltage(volts);
	}

	@Override
	public void stop() {
		setInputVoltage(0.0);
	}

	@Override
	public void setGains(double kP, double kI, double kD, double ffkS, double ffkV) {
		flywheelPID.setPID(kP, kI, kD);
		flywheelFF = new SimpleMotorFeedforward(ffkS, ffkV);
	}

	@Override
	public void setVelocity(double targetVel) {
		double volts = flywheelPID.calculate(flywheelSim.getAngularVelocityRPM(), targetVel)
				+ flywheelFF.calculate(targetVel);

		setInputVoltage(volts);
	}
}
