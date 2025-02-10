package frc.robot.commands.generic;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.CilmberSUB;
// import frc.robot.superstructure.SuperstructureState;
import frc.robot.subsystems.Climber.ClimberRealIO;
import frc.robot.subsystems.Climber.ClimberBaseIO.ClimberInputs;

public class ClimbCMD extends Command{
    

	private final CilmberSUB Climb;

	public ClimbCMD(CilmberSUB Climb) {
		this.Climb = Climb;

		addRequirements(Climb);
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		
	}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void end(boolean interrupted) {}
}