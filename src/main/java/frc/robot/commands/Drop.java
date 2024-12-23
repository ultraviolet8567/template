package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class Drop extends Command {
	private Intake intake;

	public Drop(Intake intake) {
		this.intake = intake;

		addRequirements(intake);
	}

	@Override
	public void execute() {
		intake.drop();
	}

	@Override
	public void end(boolean interrupted) {
		intake.stop();
	}
}
