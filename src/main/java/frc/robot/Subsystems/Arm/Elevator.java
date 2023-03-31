package frc.robot.Subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  public static final double MIN_HEIGHT = 0.1; //Min height in meters from the ground !FILLER VALUE!
  public static final double MAX_HEIGHT = 1.3; //Max height in meters from the ground !FILLER VALUE!
  private static final double encoderUnitsPerRot = 1; //Number of encoder units per rotation !FILLER VALUE!
  private static final double rotationsPerMeter = 5; //Motor rotations per meter of travel !FILLER VALUE!
  private static final double kP = 0.1; //Proportional control for height movement !FILLER VALUE!

  //Target
  private double targetPos = 0.5; //Target height in meters from the ground !FILLER VALUE!
  private double targetVel = 0; //Target velocity in mps with away from the ground being positive !FILLER VALUE!
  private boolean m_targetHoming = false; //Controls whether the mechanisms automatically seeks out targets

  //Components
  private CANSparkMax m_elevatorLeft = new CANSparkMax(57, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax m_elevatorRight = new CANSparkMax(59, CANSparkMaxLowLevel.MotorType.kBrushless);
  private RelativeEncoder m_encoder = m_elevatorLeft.getEncoder();

  public Elevator() {
    //Basic Setup
    this.setName("Elevator");
    this.register();

    //Motor Setup
    this.m_elevatorLeft.restoreFactoryDefaults();
    this.m_elevatorRight.restoreFactoryDefaults();
    this.m_elevatorLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
    this.m_elevatorRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
    //Check if one motor needs to be inverted
  }

  /** Sets the target height in meters relative to the ground */
  public void setTargetPos(double target) {
    this.targetPos = Math.max(MIN_HEIGHT, Math.min(MAX_HEIGHT, target));
    if(Math.abs(this.targetPos-target) > 0.0001) throw new IllegalArgumentException("Target is beyond Elevator range.");
  }
  /** Sets mps velocity of elevator with up positive */
  public void setTargetVel(double target) {
    this.targetVel = target;
  }
  /** Meter position of elevator off of ground */
  public double getPos() {
    //https://www.chiefdelphi.com/t/help-using-neo-motor-encoders/405181/8
    return MIN_HEIGHT + this.m_encoder.getPosition()/encoderUnitsPerRot/rotationsPerMeter;
  }

  //Target Homing
  public void enableHoming() {
    this.m_targetHoming = true;
  }
  public void disableHoming() {
    this.m_targetHoming = false;
  }

  @Override
  public void periodic() {
    if(this.m_targetHoming) {
      try {
        setTargetPos(targetPos + (this.targetVel*0.02)); //0.02 mps for CommandScheduler update cycle
      } catch(IllegalArgumentException e) {
        this.targetVel = 0; //Stops movement if range end met.
        System.out.println("Elevator range met. Setting Velocity to 0.");
      }
      
      double motorPower = kP * Math.cbrt((this.targetPos - getPos()) / (MAX_HEIGHT-MIN_HEIGHT));
        //Dividing error scales to -1 to 1
      this.m_elevatorLeft.set(motorPower);
      this.m_elevatorRight.set(motorPower);
    }
  }

  @Deprecated //Why is this deprecated
  public void setSpeed(double targetSpeed)
  {
    this.m_elevatorLeft.set(targetSpeed);
    this.m_elevatorRight.set(targetSpeed);
  }
}