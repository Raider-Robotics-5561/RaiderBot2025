package frc.robot.commands.generic;

import frc.robot.subsystems.Climber.TestClimberSub;
import edu.wpi.first.wpilibj2.command.Command;

/** An ClimberUpCommand that uses a climb subsystem. */
public class ClimberUpCommand extends Command {
  private final TestClimberSub m_climber;
  public static final class ClimberConstants {
    public static final int CLIMBER_MOTOR_ID = 14;
    public static final int CLIMBER_MOTOR_CURRENT_LIMIT = 30;
    public static final double CLIMBER_MOTOR_VOLTAGE_COMP = 12;
    public static final double CLIMBER_SPEED_DOWN = -0.2;
    public static final double CLIMBER_SPEED_UP = 0.2;
  }
  /**
   * Runs the climber up, note that this can change 
   * based on how the winch is wound.
   *
   * @param climber The subsystem used by this command.
   */
  public ClimberUpCommand(TestClimberSub climber) {
    m_climber = climber;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.runClimber(ClimberConstants.CLIMBER_SPEED_UP);
  }

  // Called once the command ends or is interrupted.. Here we ensure the climber is not
  // running once we let go of the button
  @Override
  public void end(boolean interrupted) {
    m_climber.runClimber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}