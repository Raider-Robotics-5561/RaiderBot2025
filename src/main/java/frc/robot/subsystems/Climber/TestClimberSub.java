package frc.robot.subsystems.Climber;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestClimberSub extends SubsystemBase {

    private final SparkMax climbMotor;

    public static final class ClimberConstants {
        public static final int CLIMBER_MOTOR_ID = 14;
        public static final int CLIMBER_MOTOR_CURRENT_LIMIT = 30;
        public static final double CLIMBER_MOTOR_VOLTAGE_COMP = 12;
        public static final double CLIMBER_SPEED_DOWN = -0.5;
        public static final double CLIMBER_SPEED_UP = 0.5;
      }
    /**
     * This subsytem that controls the climber.
     */
    public TestClimberSub () {

    // Set up the climb motor as a brushless motor
    climbMotor = new SparkMax(ClimberConstants.CLIMBER_MOTOR_ID, MotorType.kBrushless);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    climbMotor.setCANTimeout(250);

    // Create and apply configuration for climb motor. Voltage compensation helps
    // the climb behave the same as the battery
    // voltage dips. The current limit helps prevent breaker trips or burning out
    // the motor in the event the climb stalls.
    SparkMaxConfig climbConfig = new SparkMaxConfig();
    climbConfig.voltageCompensation(ClimberConstants.CLIMBER_MOTOR_VOLTAGE_COMP);
    climbConfig.smartCurrentLimit(ClimberConstants.CLIMBER_MOTOR_CURRENT_LIMIT);
    climbConfig.idleMode(IdleMode.kBrake);
    climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
    }

    /**
     * Use to run the climber, can be set to run from 100% to -100%.
     * Keep in mind that the direction changes based on which way the winch is wound.
     * 
     * @param speed motor speed from -1.0 to 1, with 0 stopping it
     */
    public void runClimber(double speed){
        climbMotor.set(speed);
    }

}