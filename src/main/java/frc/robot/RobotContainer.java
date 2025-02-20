package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.io.File;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import swervelib.SwerveInputStream;
import swervelib.parser.json.modules.DriveConversionFactorsJson;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.commands.ClimberUpCommand;
import frc.robot.commands.ClimberDownCommand;
import frc.robot.subsystems.Claw.Claw;
import frc.robot.subsystems.Climber.ClimbSubsystem;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorFFCommand;
import frc.robot.subsystems.Elevator.ElevatorPIDCommand;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.util.Constants.ElevatorConstants;
import frc.robot.util.Constants.SwerveConstants;
import frc.robot.util.Constants.miscConstants;
import frc.robot.commands.swervedrive.drivebase.*;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  final         CommandXboxController DriveController = new CommandXboxController(0);
  final         CommandXboxController OPController = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));
  public final ClimbSubsystem 
  m_climber = new ClimbSubsystem();


  // private final Claw claw;
  // private final Elevator elevator;
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                             () -> DriveController.getLeftY() * -1,
                                                             () -> DriveController.getLeftX() * -1)
                                                            .withControllerRotationAxis(DriveController::getRightX)
                                                            .deadband(miscConstants.DEADBAND)
                                                            .scaleTranslation(0.25)
                                                            .scaleRotation(0.15)
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
    // elevator = new Elevator();
    // claw     = new Claw();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(false);
   

  }
  private void configureBindings()
  {

    OPController.povUp().whileTrue(new ClimberUpCommand(m_climber));
    OPController.povDown().whileTrue(new ClimberDownCommand(m_climber));

    // OPController
    //     .y()
    //     .onTrue(
    //         new ParallelCommandGroup(
    //             new ElevatorPIDCommand(ElevatorConstants.Positions.L4, elevator)))
    //     .onFalse(
    //         new ParallelCommandGroup(new ElevatorFFCommand(elevator)));

    //         OPController
    //     .b()
    //     .onTrue(
    //         new ParallelCommandGroup(
    //             new ElevatorPIDCommand(ElevatorConstants.Positions.L1, elevator)))
    //     .onFalse(
    //         new ParallelCommandGroup(new ElevatorFFCommand(elevator)));

            // OPController
        // .x()
        // .onTrue(
        //     new ParallelCommandGroup(
        //         new ElevatorPIDCommand(ElevatorConstants.Positions.POSTINTAKE, elevator)))
        // .onFalse(
        //     new ParallelCommandGroup(
        //         new ElevatorFFCommand(elevator)));
 // Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
 Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
 // Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
 // Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
     // driveDirectAngle);
 Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
 // Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
 // Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
     // driveDirectAngleKeyboard);

   if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      // driverJoystick.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      // driverJoystick.button(8).onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      DriveController.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      DriveController.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      DriveController.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      DriveController.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      DriveController.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      DriveController.back().whileTrue(drivebase.centerModulesCommand());
      DriveController.leftBumper().onTrue(Commands.none());
      DriveController.rightBumper().onTrue(Commands.none());
    } else
    {
      DriveController.button(8).onTrue((Commands.runOnce(drivebase::zeroGyro)));

      DriveController.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));

      DriveController.y().whileTrue(
      drivebase.driveToPose(new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));

      DriveController.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

      DriveController.axisGreaterThan(3, 0.01).onChange(Commands.runOnce(() -> {
        driveAngularVelocity.scaleTranslation(DriveController.getRightTriggerAxis() + 0.25);
        driveAngularVelocity.scaleRotation((DriveController.getRightTriggerAxis() * miscConstants.RotationSpeedScale) + 0.25);
      }).repeatedly()).whileFalse(Commands.runOnce(() -> { 
        driveAngularVelocity.scaleTranslation(0.25);
        driveAngularVelocity.scaleRotation(0.15);
      }).repeatedly());


      // DriveController
      // .rightTrigger(miscConstants.DEADBAND)
      // .whileTrue(Commands.runOnce(() -> {driveAngularVelocity.scaleTranslation(SwerveConstants.kMaxSpeedScalar);
      // }))
      // .whileFalse(Commands.runOnce(() -> {driveAngularVelocity.scaleTranslation(SwerveConstants.kUnboostScalar);
      //  }));
       }

    
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
