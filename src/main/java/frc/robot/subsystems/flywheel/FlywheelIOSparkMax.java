package frc.robot.subsystems.flywheel;

import static frc.robot.Constants.GainsConstants.*;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.SparkConfig;
import frc.robot.util.SparkConfig.SparkType;

public class FlywheelIOSparkMax implements FlywheelIO {
	private final CANSparkFlex flywheelMotor;
	private final RelativeEncoder flywheelEncoder;
	private final SparkPIDController flywheelPID;
	private SimpleMotorFeedforward flywheelFF;

	private double target;

	public FlywheelIOSparkMax() {
		System.out.println("[Init] Creating FlywheelIOSparkMax");

		flywheelMotor = new CANSparkFlex(CAN.kFlywheelPort, MotorType.kBrushless);
		SparkConfig.config(flywheelMotor, SparkType.kSparkFlex);
		flywheelMotor.setInverted(false);

		flywheelEncoder = flywheelMotor.getEncoder();
		flywheelEncoder.setVelocityConversionFactor(1.0 / ShooterConstants.kShooterReduction);

		flywheelPID = flywheelMotor.getPIDController();

		setGains(flywheelGains.kP(), flywheelGains.kI(), flywheelGains.kD(), flywheelGains.ffkS(),
				flywheelGains.ffkV());

		target = 0;
	}

	@Override
	public void updateInputs(FlywheelIOInputs inputs) {
		inputs.velocityRPM = flywheelEncoder.getVelocity();
		inputs.targetVelocityRPM = target;
		inputs.appliedVoltage = flywheelMotor.getAppliedOutput() * flywheelMotor.getBusVoltage();
		inputs.currentAmps = new double[]{flywheelMotor.getOutputCurrent()};
		inputs.tempCelsius = new double[]{flywheelMotor.getMotorTemperature()};
		inputs.rotations = flywheelEncoder.getPosition();
	}

	@Override
	public void setInputVoltage(double volts) {
		double appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
		flywheelMotor.setVoltage(appliedVolts);
	}

	@Override
	public void stop() {
		setInputVoltage(0.0);
	}

	@Override
	public void setGains(double kP, double kI, double kD, double ffkS, double ffkV) {
		flywheelPID.setP(kP);
		flywheelPID.setI(kI);
		flywheelPID.setD(kI);
		flywheelFF = new SimpleMotorFeedforward(ffkS, ffkV);
	}

	@Override
	public void setBrakeMode(boolean brake) {
		flywheelMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
	}

	@Override
	public void setVelocity(double targetVel) {
		flywheelPID.setReference(targetVel, CANSparkBase.ControlType.kVelocity, 0, flywheelFF.calculate(targetVel));
		target = targetVel;
	}
}
