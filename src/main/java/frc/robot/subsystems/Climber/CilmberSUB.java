package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.Climber.ClimberBaseIO;

public class CilmberSUB extends SubsystemBase{
    
    public enum State {
        Stowed(0, 0),
        Climb(0, 0);
        
        private final int Pos;
		private final double speed;

        State(int Pos, double speed) {
			this.Pos = Pos;
			this.speed = speed;
		}

		public int getPos() {
			return Pos;
		}

		public double getSpeed() {
			return speed;
		}
}

	private final ClimberBaseIO io;

	private final ClimberBaseIO.ClimberInputs inputs = new ClimberBaseIO.ClimberInputs();

    	public CilmberSUB(ClimberBaseIO io) {
		this.io = io;
	}

    @Override
	public void periodic() {

		// Update inputs
		io.updateInputs(inputs);

		

		// Process inputs
		Logger.processInputs("Climber", inputs);
		// Gets the distance traveled
		
	}

	public Command setState(State state) {
		return new InstantCommand(
				() -> {
					io.setClimberPos(state.getPos());
				});

}
}