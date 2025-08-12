package frc.robot.subsystems.Elevator;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;


/**
 * Elevator subsystem using SparkMAX with NEO motor
 */
@Logged(name = "ElevatorSubsystem")
public class ElevatorSubsystem extends SubsystemBase {
  // Constants
  private final int canID = 10;
  private final double gearRatio = 12;
  private final double kP = 1;
  private final double kI = 0;
  private final double kD = 0;
  private final double maxVelocity = 1; // meters per second
  private final double maxAcceleration = 1; // meters per second squared
  private final boolean brakeMode = true;
  private final boolean enableStatorLimit = true;
  private final int statorCurrentLimit = 40;
  private final boolean enableSupplyLimit = false;
  private final double supplyCurrentLimit = 40;
  private final double drumRadius = 0.0254; // meters
  private final double minheight = 0;
  private final double maxheight = 1; 
  
  // Feedforward
  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(
    0, // kS
    0.34, // kG
    9.21, // kV
    0.03  // kA
  );
  
  // Motor controller
  private final SparkMax motor;
private final RelativeEncoder encoder;
  
  // Control mode
  public enum ControlMode {
    OPEN_LOOP,
    POSITION,
    VELOCITY
  }
  private ControlMode currentControlMode = ControlMode.OPEN_LOOP;
  private double targetPosition = 0.0;
  private double targetVelocity = 0.0;
  
  // Profiled PID Controller
  private ProfiledPIDController profiledPIDController;
  private TrapezoidProfile.Constraints constraints;
  
  // Simulation
  private final ElevatorSim elevatorSim;
  
  /**
   * Creates a new Elevator Subsystem.
   */
  public ElevatorSubsystem() {
    // Initialize motor controller
    SparkMaxConfig motorConfig = new SparkMaxConfig();
motor = new SparkMax(canID, MotorType.kBrushless);
motorConfig.idleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);

// Configure encoder
encoder = motor.getEncoder();
encoder.setPosition(0);

// Set ramp rates
  motorConfig.openLoopRampRate(3);
  motorConfig.closedLoopRampRate(3);

// Set current limits
 motorConfig.smartCurrentLimit(statorCurrentLimit);


// Save configuration
motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    // Initialize simulation
    elevatorSim = new ElevatorSim(
      DCMotor.getNEO(1), // Motor type
      gearRatio,
      5, // Carriage mass (kg)
      drumRadius, // Drum radius (m)
      minheight, // Min height (m)
      maxheight, // Max height (m)
      true, // Simulate gravity
      0 // Starting height (m)
    );
    
    // Initialize ProfiledPIDController
    // Convert from meters to rotations for constraints
    double maxVelocityRotations = maxVelocity / (2.0 * Math.PI * drumRadius);
    double maxAccelerationRotations = maxAcceleration / (2.0 * Math.PI * drumRadius);
    
    constraints = new TrapezoidProfile.Constraints(maxVelocityRotations, maxAccelerationRotations);
    profiledPIDController = new ProfiledPIDController(kP, kI, kD, constraints);
  }
  
  /**
   * Control loop function that runs at a fixed frequency.
   * This is used for SparkMAX and SparkFlex controllers to implement
   * closed-loop control outside of the main robot loop.
   */
  private void controlLoopFn() {
    switch (currentControlMode) {
      case POSITION:
        double currentPos = getPosition();
        double output = profiledPIDController.calculate(currentPos, targetPosition);
        double velocity = profiledPIDController.getSetpoint().velocity;
        double feedforwardOutput = feedforward.calculate(velocity);
        setVoltage(output + feedforwardOutput);
        break;
        
      case VELOCITY:
        double currentVel = getVelocity();
        double velOutput = profiledPIDController.calculate(currentVel, targetVelocity);
        double accel = profiledPIDController.getSetpoint().velocity - currentVel;
        double velFeedforwardOutput = feedforward.calculate(targetVelocity, accel);
        
        // Apply the combined PID output and feedforward to the motor
        double velocityVoltage = velOutput + velFeedforwardOutput;
        motor.setVoltage(velocityVoltage);
        break;
        
      case OPEN_LOOP:
      default:
        // Do nothing, voltage is set directly
        break;
    }
  }
  
  /**
   * Clean up resources when the subsystem is destroyed.
   */
  public void close() {
     motor.close();
   }

  
  /**
   * Update simulation and telemetry.
   */
  @Override
  public void periodic() {
    
    controlLoopFn();
  }
  
  /**
   * Update simulation.
   */
  @Override
  public void simulationPeriodic() {
    // Set input voltage from motor controller to simulation
    elevatorSim.setInput(getVoltage());
    
    // Update simulation by 20ms
    elevatorSim.update(0.020);

    // Convert meters to motor rotations
    double positionToRotations = (1 / (2.0 * Math.PI * drumRadius)) * gearRatio;
    double motorPosition = elevatorSim.getPositionMeters() * positionToRotations;
    double motorVelocity = elevatorSim.getVelocityMetersPerSecond() * positionToRotations;

    
  }
  
 
  
  /**
   * Get the current position in the Rotations.
   * @return Position in Rotations
   */
   @Logged(name = "Position/Rotations")
  public double getPosition() {
    // Rotations
      return encoder.getPosition() / gearRatio;
  }

  
  /**
   * Get the current velocity in rotations per second.
   * @return Velocity in rotations per second
   */
   @Logged(name = "Velocity")
  public double getVelocity() {
    return encoder.getVelocity() / gearRatio / 60.0; // Convert from RPM to RPS
  }
  
  /**
   * Get the current applied voltage.
   * @return Applied voltage
   */
   @Logged(name = "Voltage")
  public double getVoltage() {
    return motor.getAppliedOutput() * motor.getBusVoltage();
  }
  
  /**
   * Get the current motor current.
   * @return Motor current in amps
   */
   @Logged(name = "Current")
  public double getCurrent() {
    return motor.getOutputCurrent();
  }
  
  /**
   * Get the current motor temperature.
   * @return Motor temperature in Celsius
   */
   @Logged(name = "Temperature")
  public double getTemperature() {
    return motor.getMotorTemperature();
  }
  
  /**
   * Set elevator position.
   * @param position The target position in meters
   */
  public void setPosition(double position) {
    setPosition(position, 0);
  }
  
  /**
   * Set elevator position with acceleration.
   * @param position The target position in meters
   * @param acceleration The acceleration in meters per second squared
   */
  public void setPosition(double position, double acceleration) {
    // Convert meters to rotations
    double positionRotations = position / (2.0 * Math.PI * drumRadius);
    
    // Use the ProfiledPIDController
    targetPosition = positionRotations;
    currentControlMode = ControlMode.POSITION;
    
    // If acceleration is specified, update constraints
    if (acceleration > 0) {
      double maxAccelRotations = acceleration / (2.0 * Math.PI * drumRadius);
      constraints = new TrapezoidProfile.Constraints(constraints.maxVelocity, maxAccelRotations);
      profiledPIDController.setConstraints(constraints);
    }
  }
  
  /**
   * Set elevator velocity.
   * @param velocity The target velocity in meters per second
   */
  public void setVelocity(double velocity) {
    setVelocity(velocity, 0);
  }
  
  /**
   * Set elevator velocity with acceleration.
   * @param velocity The target velocity in meters per second
   * @param acceleration The acceleration in meters per second squared
   */
  public void setVelocity(double velocity, double acceleration) {
    // Convert meters/sec to rotations/sec
    double velocityRotations = velocity / (2.0 * Math.PI * drumRadius);
    
    // Use the ProfiledPIDController
    targetVelocity = velocityRotations;
    currentControlMode = ControlMode.VELOCITY;
    
    // If acceleration is specified, update constraints
    if (acceleration > 0) {
      double maxAccelRotations = acceleration / (2.0 * Math.PI * drumRadius);
      constraints = new TrapezoidProfile.Constraints(constraints.maxVelocity, maxAccelRotations);
      profiledPIDController.setConstraints(constraints);
    }
  }
  
  /**
   * Set motor voltage directly.
   * @param voltage The voltage to apply
   */
  public void setVoltage(double voltage) {
    currentControlMode = ControlMode.OPEN_LOOP;
    motor.setVoltage(voltage);
  }
  
  /**
   * Get the elevator simulation for testing.
   * @return The elevator simulation model
   */
  public ElevatorSim getSimulation() {
    return elevatorSim;
  }

  public double getMinHeightMeters() {
    return minheight;
  }

  public double getMaxHeightMeters() {
    return maxheight;
  }

  /**
   * Creates a command to set the elevator to a specific height.
   * @param heightMeters The target height in meters
   * @return A command that sets the elevator to the specified height
   */
  public Command setHeightCommand(double heightMeters) {
    return runOnce(() -> setPosition(heightMeters));
  }
  
  /**
   * Creates a command to move the elevator to a specific height with a profile.
   * @param heightMeters The target height in meters
   * @return A command that moves the elevator to the specified height
   */
  public Command moveToHeightCommand(double heightMeters) {
    return run(() -> {
      // Just set the position and let the profiled controller handle it
      setPosition(heightMeters);
    }).until(() -> {
      double currentHeight = getPosition() * (2.0 * Math.PI * drumRadius);
      return Math.abs(heightMeters - currentHeight) < 0.02; // 2cm tolerance
    });
  }
  
  /**
   * Creates a command to stop the elevator.
   * @return A command that stops the elevator
   */
  public Command stopCommand() {
    return runOnce(() -> setVelocity(0));
  }
  
  /**
   * Creates a command to move the elevator at a specific velocity.
   * @param velocityMetersPerSecond The target velocity in meters per second
   * @return A command that moves the elevator at the specified velocity
   */
  public Command moveAtVelocityCommand(double velocityMetersPerSecond) {
    return run(() -> setVelocity(velocityMetersPerSecond));
  }
}