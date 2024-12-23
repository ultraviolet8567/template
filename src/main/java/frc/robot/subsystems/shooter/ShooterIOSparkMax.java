package frc.robot.subsystems.shooter;

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

public class ShooterIOSparkMax implements ShooterIO {
	private final CANSparkFlex shooterTopMotor, shooterBottomMotor;
	private final RelativeEncoder shooterTopEncoder, shooterBottomEncoder;
	private final SparkPIDController shooterTopPID, shooterBottomPID;
	private SimpleMotorFeedforward shooterTopFF, shooterBottomFF;

	private double topTarget, bottomTarget;

	public ShooterIOSparkMax() {
		System.out.println("[Init] Creating ShooterIOSparkMax");

		shooterTopMotor = new CANSparkFlex(CAN.kShooterTopPort, MotorType.kBrushless);
		SparkConfig.config(shooterTopMotor, SparkType.kSparkFlex);
		shooterTopMotor.setInverted(false);

		shooterBottomMotor = new CANSparkFlex(CAN.kShooterBottomPort, MotorType.kBrushless);
		SparkConfig.config(shooterTopMotor, SparkType.kSparkFlex);
		shooterBottomMotor.setInverted(false);

		shooterTopEncoder = shooterTopMotor.getEncoder();
		shooterTopEncoder.setVelocityConversionFactor(1.0 / ShooterConstants.kShooterReduction);

		shooterBottomEncoder = shooterBottomMotor.getEncoder();
		shooterBottomEncoder.setVelocityConversionFactor(1.0 / ShooterConstants.kShooterReduction);

		shooterTopPID = shooterTopMotor.getPIDController();
		shooterBottomPID = shooterBottomMotor.getPIDController();

		setGains(shooterTopGains.kP(), shooterTopGains.kI(), shooterTopGains.kD(), shooterTopGains.ffkS(),
				shooterTopGains.ffkV(), shooterBottomGains.kP(), shooterBottomGains.kI(), shooterBottomGains.kD(),
				shooterBottomGains.ffkS(), shooterBottomGains.ffkV());

		topTarget = 0;
		bottomTarget = 0;
	}

	@Override
	public void updateInputs(ShooterIOInputs inputs) {
		inputs.topVelocityRPM = shooterTopEncoder.getVelocity();
		inputs.topTargetVelocityRPM = topTarget;
		inputs.topAppliedVoltage = shooterTopMotor.getAppliedOutput() * shooterTopMotor.getBusVoltage();
		inputs.topCurrentAmps = new double[]{shooterTopMotor.getOutputCurrent()};
		inputs.topTempCelsius = new double[]{shooterBottomMotor.getMotorTemperature()};

		inputs.bottomVelocityRPM = shooterBottomEncoder.getVelocity();
		inputs.bottomTargetVelocityRPM = bottomTarget;
		inputs.bottomAppliedVoltage = shooterBottomMotor.getAppliedOutput() * shooterBottomMotor.getBusVoltage();
		inputs.bottomCurrentAmps = new double[]{shooterBottomMotor.getOutputCurrent()};
		inputs.bottomTempCelsius = new double[]{shooterBottomMotor.getMotorTemperature()};

		inputs.topRotations = shooterTopEncoder.getPosition();
		inputs.bottomRotations = shooterBottomEncoder.getPosition();
	}

	@Override
	public void setTopInputVoltage(double volts) {
		double appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
		shooterTopMotor.setVoltage(appliedVolts);
	}

	@Override
	public void setBottomInputVoltage(double volts) {
		double appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
		shooterBottomMotor.setVoltage(appliedVolts);
	}

	@Override
	public void setInputVoltage(double topVolts, double bottomVolts) {
		setTopInputVoltage(topVolts);
		setBottomInputVoltage(bottomVolts);
	}

	@Override
	public void stop() {
		setInputVoltage(0.0, 0.0);
	}

	@Override
	public void setGains(double tkP, double tkI, double tkD, double tffkS, double tffkV, double bkP, double bkI,
			double bkD, double bffkS, double bffkV) {
		shooterTopPID.setP(tkP);
		shooterTopPID.setI(tkI);
		shooterTopPID.setD(tkI);
		shooterTopFF = new SimpleMotorFeedforward(tffkS, tffkV);

		shooterBottomPID.setP(bkP);
		shooterBottomPID.setI(bkI);
		shooterBottomPID.setD(bkD);
		shooterBottomFF = new SimpleMotorFeedforward(bffkS, bffkV);
	}

	@Override
	public void setBrakeMode(boolean brake) {
		shooterTopMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
		shooterBottomMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
	}

	@Override
	public void setVelocity(double topTargetVel, double bottomTargetVel) {
		shooterTopPID.setReference(topTargetVel, CANSparkBase.ControlType.kVelocity, 0,
				shooterTopFF.calculate(topTargetVel));
		shooterBottomPID.setReference(bottomTargetVel, CANSparkBase.ControlType.kVelocity, 0,
				shooterBottomFF.calculate(bottomTargetVel));

		topTarget = topTargetVel;
		bottomTarget = bottomTargetVel;
	}
}
