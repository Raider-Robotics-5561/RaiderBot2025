package frc.robot.subsystems.Climber;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
// import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.miscConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;


public class ClimberRealIO{ //implements ClimberBaseIO{

  private SparkMax ClimberMotor;
  private SparkMaxConfig ClimberConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder relativeEncoder;

  
public ClimberRealIO(){

    ClimberMotor = new SparkMax(14, MotorType.kBrushless);
    relativeEncoder = ClimberMotor.getEncoder();

    SparkMaxConfig globalConfig = new  SparkMaxConfig();
    SparkMaxConfig ClimberConfig = new SparkMaxConfig();
    closedLoopController = ClimberMotor.getClosedLoopController();

    ClimberConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    globalConfig
    .smartCurrentLimit(30)
    .idleMode(IdleMode.kBrake);

     ClimberConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    ClimberMotor.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.setDefaultNumber("Target Position", 0);
    SmartDashboard.setDefaultNumber("Target Velocity", 0);
    SmartDashboard.setDefaultBoolean("Control Mode", false);
    SmartDashboard.setDefaultBoolean("Reset Encoder", false);

}

public void RunRPM(){

     if (SmartDashboard.getBoolean("Control Mode", false)) {
      /*
       * Get the target velocity from SmartDashboard and set it as the setpoint
       * for the closed loop controller.
       */
      double targetVelocity = 500; //SmartDashboard.getNumber("Target Velocity", 0);
      closedLoopController.setReference(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    } else {
      /*
       * Get the target position from SmartDashboard and set it as the setpoint
       * for the closed loop controller.
       */
      double targetPosition = SmartDashboard.getNumber("Target Position", 0);
      closedLoopController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

     // Display encoder position and velocity
     SmartDashboard.putNumber("Actual Position", relativeEncoder.getPosition());
     SmartDashboard.putNumber("Actual Velocity", relativeEncoder.getVelocity());
 
     if (SmartDashboard.getBoolean("Reset Encoder", false)) {
       SmartDashboard.putBoolean("Reset Encoder", false);
       // Reset the encoder position to 0
       relativeEncoder.setPosition(0);
     }
}
}