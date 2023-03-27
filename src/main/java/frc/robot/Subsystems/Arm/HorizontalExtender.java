package frc.robot.Subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.PowerDistributionModule;

public class HorizontalExtender extends SubsystemBase {
  public static final double MIN_EXTENSION = 0.1; //Min height in meters from the robot center !FILLER VALUE!
  public static final double MAX_EXTENSION = 1.3; //Max height in meters from the robot center !FILLER VALUE!
  private static final double encoderUnitsPerRot = 1; //Number of encoder units per rotation !FILLER VALUE!
  private static final double rotationsPerMeter = 5; //Motor rotations per meter of travel !FILLER VALUE!
  private static final double kP = 0.1; //Proportional control for extension movement !FILLER VALUE!

  private double m_targetPos = 0.5; //Target extension in meters from the robot center !FILLER VALUE!
  private double m_targetVel = 0; //Target velocity in mps with away from the ground being positive !FILLER VALUE!
  private boolean m_targetHoming = false; //Controls whether the mechanisms automatically seeks out targets

  private CANSparkMax m_motor = new CANSparkMax(49, CANSparkMaxLowLevel.MotorType.kBrushed); //Confirm Brushed!-!
  private Encoder m_encoder = new Encoder(0, 1); //!FILLER VALUE!

  public HorizontalExtender() {
    //Basic Setup
    this.setName("Horizontal Extender");
    this.register();

    //Motor Setup
    this.m_motor.restoreFactoryDefaults();
    this.m_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    //Initial Command
    CommandScheduler.getInstance().schedule(new FindExtenderEnd());
  }

  /** Sets the target height in meters relative to the ground */
  public void setTargetPos(double target) throws IllegalArgumentException {
    this.m_targetPos = Math.max(MIN_EXTENSION, Math.min(MAX_EXTENSION, target));
    if(Math.abs(this.m_targetPos-target) > 0.0001) throw new IllegalArgumentException("Target is beyond Extender range.");
  }
  /** Sets mps velocity of elevator with up positive */
  public void setTargetVel(double target) {
    this.m_targetVel = target;
  }
  /** Meter position of elevator off of ground */
  public double getPos() {
    //https://www.chiefdelphi.com/t/help-using-neo-motor-encoders/405181/8
    return MIN_EXTENSION + this.m_encoder.getDistance()/encoderUnitsPerRot/rotationsPerMeter;
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
        setTargetPos(m_targetPos + (this.m_targetVel*0.02)); //0.02 mps for CommandScheduler update cycle
      } catch(IllegalArgumentException e) {
        this.m_targetVel = 0; //Stops movement if range end met.
        System.out.println("Extender range met. Setting Velocity to 0.");
      }

      double motorPower = kP * Math.cbrt((this.m_targetPos - getPos()) / (MAX_EXTENSION-MIN_EXTENSION));
        //Dividing error scales to -1 to 1
      this.m_motor.set(motorPower);
    }
  }

  @Deprecated
  public void setSpeed(double targetSpeed)
  {
    this.m_motor.set(targetSpeed);
  }

  private class FindExtenderEnd extends CommandBase {
    private static final int channel = 1; //!FILLER VALUE!
    private static final double thresholdCurrent = 1; //!FILLER VALUE!

    public FindExtenderEnd() {
      super.addRequirements(HorizontalExtender.this);
    }

    @Override
    public void initialize() {
      HorizontalExtender.this.disableHoming();
      HorizontalExtender.this.m_motor.set(-0.1);
    }

    @Override
    public boolean isFinished() {
      return PowerDistributionModule.getInstance().getCurrent(channel) > thresholdCurrent;
    }

    @Override
    public void end(boolean interrupted) {
      HorizontalExtender.this.m_motor.set(0);
      HorizontalExtender.this.m_encoder.reset();
      HorizontalExtender.this.enableHoming();
    }
  }
}
