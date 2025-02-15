package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Elevator.Elevator;

public class ElevatorGoTo extends Command {
  private final Elevator mElevator;
  double mSetPoint;

  public ElevatorGoTo(double setPoint, Elevator pElevatorSubsystem) {
    this.mElevator = pElevatorSubsystem;
    this.mSetPoint =
        MathUtil.clamp(setPoint, ElevatorConstants.kMinHeight, ElevatorConstants.kMaxHeight);

    addRequirements(mElevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    mElevator.doPID(mSetPoint);
  }

  @Override
  public void end(boolean interrupted) {
    mElevator.stop();
  }

  @Override
  public boolean isFinished() {
    return mElevator.PIDFinished();
  }
}