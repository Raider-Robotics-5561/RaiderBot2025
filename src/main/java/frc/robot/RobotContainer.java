package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.io.File;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.commands.ClimberUpCommand;
import frc.robot.commands.ClimberDownCommand;
import frc.robot.subsystems.Climber.ClimbSubsystem;
import frc.robot.miscConstants;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorFFCommand;
import frc.robot.subsystems.Elevator.ElevatorPIDCommand;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  final         CommandXboxController DriveController = new CommandXboxController(0);
  final         CommandXboxController OporatorController = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));
  public final ClimbSubsystem m_climber = new ClimbSubsystem();

  private final Elevator elevator;


  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                             () -> DriveController.getLeftY() * -1,
                                                             () -> DriveController.getLeftX() * -1)
                                                            .withControllerRotationAxis(DriveController::getRightX)
                                                            .deadband(miscConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(DriveController::getLeftX,
                                                                                             DriveController::getLeftY)
                                                           .headingWhile(true);
                                                           

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                      () -> -DriveController.getLeftY(),
                                                                      () -> -DriveController.getLeftX())
                                                                    .withControllerRotationAxis(() -> DriveController.getRawAxis(
                                                                        // () -> -driverJoystick.getRawAxis(1),
                                                                        // () -> -driverJoystick.getRawAxis(0))
                                                                    // .withControllerRotationAxis(() -> driverJoystick.getRawAxis(
                                                                        2))
                                                                    .deadband(miscConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                DriveController.getRawAxis(
                                                                                                                  // driverJoystick.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                DriveController.getRawAxis(
                                                                                                                  // driverJoystick.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    elevator = new Elevator();

    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {

      OporatorController.povUp().whileTrue(new ClimberUpCommand(m_climber));
      OporatorController.povDown().whileTrue(new ClimberDownCommand(m_climber));

          OporatorController
        .y()
        .onTrue(
            new ParallelCommandGroup(
                new ElevatorPIDCommand(ElevatorConstants.Positions.L4, elevator)))
        .onFalse(
            new ParallelCommandGroup(new ElevatorFFCommand(elevator)));

    OporatorController
        .b()
        .onTrue(
            new ParallelCommandGroup(
                new ElevatorPIDCommand(ElevatorConstants.Positions.L1, elevator)))
        .onFalse(
            new ParallelCommandGroup(new ElevatorFFCommand(elevator)));

            OporatorController
        .x()
        .onTrue(
            new ParallelCommandGroup(
                new ElevatorPIDCommand(ElevatorConstants.Positions.POSTINTAKE, elevator)))
        .onFalse(
            new ParallelCommandGroup(
                new ElevatorFFCommand(elevator)));


      DriveController.button(8).onTrue((Commands.runOnce(drivebase::zeroGyro)));
      DriveController.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      DriveController.y().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
      DriveController.start().whileTrue(Commands.none());
      DriveController.back().whileTrue(Commands.none());
      DriveController.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      DriveController.rightBumper().onTrue(Commands.none());
    
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
