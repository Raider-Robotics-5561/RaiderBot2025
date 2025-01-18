package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Const {
    // TODO Lets Double Check everything in here
    public static CANBus CANivore = new CANBus("Drive");
    //  NOTE I dont think we need this but I will leave it here for now.
    // //Front Left SensortoWheelRotation = 6.53;
    //    public static double FLSensorToWheelInch = 0.519;
    //    public static double FLSensorInchToMeter = 0.013;

    // //Front RightSensorToWheelRotation= 6.572
    //   public static double FRSensorToWheelInch = 0.524;
    //   public static double FRSensorInchToMeter = 0.013;

    // //Back Right SensorToRotation = 6.630
    //   public static double BRSensorToWheelInch = 0.529;
    //   public static double BRSensorInchToMeter = 0.013;

    // // Back Left SensorToRotation = 6.619
    // public static double BLSensorToWheelInch = 0.528;
    // public static double BLSensorInchToMeter = 0.013;
    // ENDNOTE

  // TODO Add all CAN IDs here.
    public static final class CANIDs {
        public static final int UnderbellyIntakeID = 13;
        public static final int UnderbellyIntakeAssistID = 24;
    
  }
    public static final class Swerve {
    
        //Our Serve Module Offsets
        public static final Translation2d flModuleOffset = new Translation2d(0.4, 0.4);
        public static final Translation2d frModuleOffset = new Translation2d(0.4, -0.4);
        public static final Translation2d blModuleOffset = new Translation2d(-0.4, 0.4);
        public static final Translation2d brModuleOffset = new Translation2d(-0.4, -0.4);

        //Max Speed
        public static final double maxModuleSpeed = 4.5; // M/S

  }

  public static final class PathPlannerConfig {
    // NOTE: Check what order are we feeding our last param 
    // TODO: Update params
    // public static final RobotConfig robotconfig = new RobotConfig(0, 0, null, null);
    
    //Our PIDS to use for path Following
    // NOTE: Will need to be changed
    public static final PPHolonomicDriveController pathFollowerConfig = new PPHolonomicDriveController(
      new PIDConstants(0.001, 0, 0), 
      new PIDConstants(0.001, 0, 0)
    );

  }

  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 6.12;
    public static final double kTurningMotorGearRatio = 1 / 21.43;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

        public static final double kPTurning = 0.1;

  }
    

  public static final class DriveConstants {
    // Distance between right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(20.75);
    
    // Distance between front and back wheels
    public static final double kWheelBase = Units.inchesToMeters(20.75);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //FL [0]
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), //FR  [1]   
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), //BL  [2]
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2));//BR [3]


    public static final int kFrontLeftDriveMotorPort = 3;
    public static final int kBackLeftDriveMotorPort = 8;
    public static final int kFrontRightDriveMotorPort = 2;
    public static final int kBackRightDriveMotorPort = 6;

    public static final int kFrontLeftTurningMotorPort = 4;
    public static final int kBackLeftTurningMotorPort = 7;
    public static final int kFrontRightTurningMotorPort = 1;
    public static final int kBackRightTurningMotorPort = 5;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftDriveEncoderReversed = true;
    public static final boolean kBackLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = true;
    public static final boolean kBackRightDriveEncoderReversed = true;

    public static final int kFrontLeftDriveAbsoluteEncoderPort = 17;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 19;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 18;
    public static final int kBackRightDriveAbsoluteEncoderPort = 20;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;  
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.7669;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -1.966;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 3.0219;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 1.4051;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 0.3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 0.3;

    public static final PIDConstants DrivePID = new PIDConstants(0.01,0,0.01,0);
    public static final PIDConstants SteerPID = new PIDConstants(0.01, 0, 0.01, 0);

  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
    public static final double kPXController = 1.5;
    public static final double kPYController = 1.5;
    public static final double kPThetaController = 3;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
            new TrapezoidProfile.Constraints(
                    kMaxAngularSpeedRadiansPerSecond,
                    kMaxAngularAccelerationRadiansPerSecondSquared);
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;
        public static final int kzeroGyroButtonIdx = 8; //REVIEW - Update this value

    public static final double kDeadband = 0.05;
  }
}