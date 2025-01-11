package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Const.DriveConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import frc.robot.Const;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;



/**
 * Basic simulation of a swerve subsystem with the methods needed by PathPlanner
 */
public class SwerveSubsystem extends SubsystemBase {
    // private SwerveModule[] modules = { frontLeft, frontRight, backLeft, backRight };
    private SwerveDriveKinematics kinematics;
    private SwerveDriveOdometry odometry;
    
    private Field2d field = new Field2d();
    
    //This is used to publish swerve state data to adantage scope
    StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
    
    public SwerveSubsystem() {
      kinematics = new SwerveDriveKinematics(
        Const.Swerve.flModuleOffset, 
        Const.Swerve.frModuleOffset, 
        Const.Swerve.blModuleOffset, 
        Const.Swerve.brModuleOffset
      );
      odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getPositions());
  
      // Configure AutoBuilder
      // AutoBuilder.configure(
      //   this::getPose, 
      //   this::resetPose, 
      //   this::getSpeeds, 
      //   this::driveRobotRelative, // TODO: Check our order on this too
      //   Const.PathPlannerConfig.pathFollowerConfig,
      //   Const.PathPlannerConfig.robotconfig,
      //   () -> {
      //       // Boolean supplier that controls when the path will be mirrored for the red alliance
      //       // This will flip the path being followed to the red side of the field.
      //       // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
  
      //       var alliance = DriverStation.getAlliance();
      //       if (alliance.isPresent()) {
      //           return alliance.get() == DriverStation.Alliance.Red;
      //       }
      //       return false;
      //   },
      //   this
      // );
  
      // Set up custom logging to add the current path to a field 2d widget
      PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
  
      SmartDashboard.putData("Field", field);

      new Thread(() -> {
        try {
            Thread.sleep(1000);
            zeroHeading();
        } catch (Exception e) {
        }
    }).start();

    }


    public final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private SwerveModule[] modules = { frontLeft, frontRight, backLeft, backRight };

    private SwerveModulePosition[] swerveModulePos = new SwerveModulePosition[] {
                                                    frontLeft.getPosition(),
                                                    frontRight.getPosition(),
                                                    backLeft.getPosition(),
                                                    backRight.getPosition()
                                                    };

    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
                                                                        new Rotation2d(0), 
                                                                        swerveModulePos);



    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), swerveModulePos, pose);
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), swerveModulePos);
        //SmartDashboard.putNumber("Robot Heading", getHeading());
        //SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

        odometry.update(gyro.getRotation2d(), getPositions());
    
        field.setRobotPose(getPose());

    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
      }
      
    public void resetPose(Pose2d pose) {
        odometry.resetPosition(gyro.getRotation2d(), getPositions(), pose);
    }

    public ChassisSpeeds getSpeeds() {
      return kinematics.toChassisSpeeds(getModuleStates());
    }
    
    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
      driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
    }
    
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
      ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
  
      SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
      setStates(targetStates);
    }

    public void setStates(SwerveModuleState[] targetStates) {
      SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Const.Swerve.maxModuleSpeed);

      frontLeft.setDesiredState(targetStates[0]);
      frontRight.setDesiredState(targetStates[1]);
      backLeft.setDesiredState(targetStates[2]);
      backRight.setDesiredState(targetStates[3]);

      publisher.set(targetStates);
    }

    public SwerveModuleState[] getModuleStates() {
      SwerveModuleState[] states = new SwerveModuleState[modules.length];
      for (int i = 0; i < modules.length; i++) {
        states[i] = modules[i].getState();
      }
      return states;
    }    

    public SwerveModulePosition[] getPositions() {
      SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
      for (int i = 0; i < modules.length; i++) {
        positions[i] = modules[i].getPosition();
      }
      return positions;
    }
}
