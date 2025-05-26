package frc.robot.commands;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class ElevatorCommand extends Command{
    
    
    /** An example command that uses an example subsystem. */

      private final ElevatorSubsystem m_Elevator;
      
      public ElevatorCommand(ElevatorSubsystem Elevator) {
        m_Elevator = Elevator;
        addRequirements(Elevator);
      }
    
      // Called when the command is initially scheduled.
      @Override
      public void initialize() {}
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
        
      }
    
      // Called once the command ends or is interrupted. Here we ensure the Elevator is not
      // running once we let go of the button
      @Override
      public void end(boolean interrupted) {
        
      }
    
      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
        return false;
      }
    }