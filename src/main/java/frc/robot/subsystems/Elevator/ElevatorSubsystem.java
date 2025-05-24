package frc.robot.subsystems.Elevator;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.LocalUtil.TunableNumber;

public class ElevatorSubsystem extends SubsystemBase {
    

    private final SparkMax mElevatorSparkMax;
    private final SparkMax mElevatorSparkMaxFollower;
    private final RelativeEncoder elevatorRelEncoder;

    // private double motorVoltage = 0;

    private TunableNumber elevatorFF, elevatorP, elevatorI, elevatorD;
    private TunableNumber elevatorTunableSetpoint;

    public ElevatorSubsystem () {

    // Set up the climb motor as a brushless motor
    mElevatorSparkMax = new SparkMax(ElevatorConstants.kMotorID, MotorType.kBrushless);
    mElevatorSparkMaxFollower = new SparkMax(ElevatorConstants.kFollowerMotorID, MotorType.kBrushless);

    elevatorRelEncoder = mElevatorSparkMax.getEncoder();
   
    SparkMaxConfig ElevatorConfig = new SparkMaxConfig();
    // ElevatorConfig.voltageCompensation(ElevatorConstants.CLIMBER_MOTOR_VOLTAGE_COMP);
    // ElevatorConfig.smartCurrentLimit(ElevatorConstants.CLIMBER_MOTOR_CURRENT_LIMIT);
    ElevatorConfig.idleMode(IdleMode.kBrake);
    
    ElevatorConstants.ElevatorConfigs.kElevatorFollowerConfig
    .follow(ElevatorConstants.kMotorID, true);
    
    mElevatorSparkMaxFollower.configure(
      ElevatorConstants.ElevatorConfigs.kElevatorFollowerConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kNoPersistParameters);


    mElevatorSparkMax.configure(
        ElevatorConstants.ElevatorConfigs.kElevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
        SmartDashboard.putNumber("Elevator Target Position", 0);
    SmartDashboard.putNumber("Elevator Target Velocity", 0);

    elevatorFF = new TunableNumber("Elevator/Elevator FF");
    elevatorP = new TunableNumber("Elevator/Elevator P");
    elevatorI = new TunableNumber("Elevator/Elevator I");
    elevatorD = new TunableNumber("Elevator/Elevator D");
    elevatorTunableSetpoint = new TunableNumber("Elevator/Setpoint");

    elevatorFF.setDefault(ElevatorConstants.kG);
    elevatorP.setDefault(ElevatorConstants.kP);
    elevatorI.setDefault(0);
    elevatorD.setDefault(ElevatorConstants.kD);
    elevatorTunableSetpoint.setDefault(getEncoderMeasurement());
    }

    public double getEncoderMeasurement() {
        return elevatorRelEncoder.getPosition();
      }




    @Override
    public void periodic() {
        // stopIfLimit();
        // subClaw.setElevatorHight(getEncoderMeasurement());
        // SmartDashboard.putNumber("Elevator ABS Position", elevatorAbsEncoder.getPosition());
        // SmartDashboard.putNumber("Elevator ABS Velocity", elevatorAbsEncoder.getVelocity());
        SmartDashboard.putNumber("Elevator Rel Position", elevatorRelEncoder.getPosition());
        SmartDashboard.putNumber("Elevator Rel Velocity", elevatorRelEncoder.getVelocity());
        // SmartDashboard.putNumber("Elevator Output", getMotorOutput());
        SmartDashboard.putNumber("Elevator Voltage", mElevatorSparkMax.getBusVoltage());
        // SmartDashboard.putNumber("Stepped Elevator Voltage", motorVoltage);
    }

}