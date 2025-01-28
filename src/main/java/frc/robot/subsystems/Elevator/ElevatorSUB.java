package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ElevatorSUB extends SubsystemBase {
//TODO - Need to add the height in meters.
	public enum State {
		STOWED(0),
		CORAL_STATION(0),
		// ALGAE_GROUND(0),
		ALGAE_PROCESSOR(0),
		// ALGAE_BARGE(0),
		L1_SCORING(0),
		L2_SCORING(0),
		L3_SCORING(0),
		L4_SCORING(0);

		private final double heightM;

		State(double heightM) {
			this.heightM = heightM;
		}

		public double getHeightM() {
			return heightM;
		}
	}

	private final ElevatorBaseIO io;

	private final ElevatorBaseIO.ElevatorInputs inputs = new ElevatorBaseIO.ElevatorInputs();

	public ElevatorSUB(ElevatorBaseIO io) {
		this.io = io;
	}

	@Override
	public void periodic() {

		// Update inputs
		io.updateInputs(inputs);

		

		// Process inputs
		Logger.processInputs("Elevator", inputs);
		// Gets the distance traveled
		
	}

	public Command setState(State state) {
		return new InstantCommand(
				() -> {
					io.setPositionM(state.getHeightM());
				});
	}
}
