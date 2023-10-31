package frc.robot.Subsystems.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.HashMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

public class EndEffector extends SubsystemBase {
  //Components
  private final Solenoid m_coneFlipSolenoid = new Solenoid(33, PneumaticsModuleType.CTREPCM, 1);
  private final CANSparkMax m_intakemotor = new CANSparkMax(37, CANSparkMaxLowLevel.MotorType.kBrushless); 
  private final CANSparkMax m_wristmotor = new CANSparkMax(26, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_wristmotor2 = new CANSparkMax(27, CANSparkMaxLowLevel.MotorType.kBrushless); //FILLER VALUE
  private RelativeEncoder m_encoder = m_wristmotor.getEncoder();//.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
  private PIDController m_pid = new PIDController(0.15, 0, 0);

  //Target
  private double m_targetAngle = 90;
  private GamePieceMode m_mode = GamePieceMode.CUBE;
  private boolean m_targetHoming = false; //Controls whether the mechanisms automatically seeks out targets

  //Constants-
  public static final double ARM_LENGTH = 0.25; //!FILLER VALUE!
  public static final double MIN_ANGLE = -7.1279; //!FILLER VALUE!
  public static final double MAX_ANGLE = -1.366;//-0.2857; //!FILLER VALUE!
  public static final double CENTER_ANGLE = -5.2708; //!FILLER VALUE!
  private static final double kP = 4.75; //!FILLER VALUE!
  private static final HashMap<GamePieceMode,Translation2d> targetOffsets = new HashMap<>();

  private double velocityDelta = 0;

  public EndEffector(){
    //Basic Setup
    this.setName("End Effector");
    this.register();

    //Motor setup
    this.m_intakemotor.restoreFactoryDefaults();
    this.m_intakemotor.setInverted(false);
    this.m_intakemotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    this.m_wristmotor.restoreFactoryDefaults();
    this.m_wristmotor.setInverted(false);
    this.m_wristmotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    setPosToStorred();

    //Target offsets
    targetOffsets.put(GamePieceMode.CUBE, new Translation2d()); //!FILLER VALUE!
    targetOffsets.put(GamePieceMode.UPRIGHT_CONE, new Translation2d()); //!FILLER VALUE!
    targetOffsets.put(GamePieceMode.TIPPED_CONE, new Translation2d()); //!FILLER VALUE!
  }

  public void setTargetAngle(double angle) throws IllegalArgumentException {
    this.m_targetAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, -angle/110*(MAX_ANGLE-CENTER_ANGLE)));
    //if(Math.abs(this.m_targetAngle-angle) > 0.0001) throw new IllegalArgumentException("Angle is beyond EndEffector range.");
  }
  public void setMode(GamePieceMode mode) {
    this.m_mode = mode;
  }
  public double getCurrentAngle() {
    return 110*(m_encoder.getPosition()-CENTER_ANGLE)/(MAX_ANGLE-CENTER_ANGLE);
  }
  public double getTargetAngle() {
    return this.m_targetAngle;
  }
  public Translation2d getMeasuredPiecePos() { 
    return getOffsetFor(m_mode, Rotation2d.fromDegrees(getCurrentAngle())); 
  }
  public Translation2d getTargetPiecePos() { 
    return getOffsetFor(m_mode, Rotation2d.fromDegrees(this.m_targetAngle)); 
  }
  /** Gets the offset from the EndEffector wrist to its game piece */
  public static Translation2d getOffsetFor(GamePieceMode mode, Rotation2d angle) {
    return targetOffsets.get(mode)
      .plus(new Translation2d(ARM_LENGTH, 0))
      .rotateBy(angle);
  }

  public void intakeControl(boolean state) {
    switch(this.m_mode) {
      case CUBE:
        this.setIntakeSpeed(state ? 0.2 : 0);
        break;
      case UPRIGHT_CONE:
        this.setIntakeSpeed(state ? -0.4 : 0);
        break;
      case TIPPED_CONE:
        this.setSolenoid(!state);
        this.setIntakeSpeed(state ? -0.6 : 0);
    }
  }

  //Can we make all of these self calls so its easier to change? Ie instead of m_intakemotor.set, we use setIntakeSpeed?
  public void expelControl(boolean state) {
    switch(this.m_mode) {
      case CUBE:
        this.setIntakeSpeed(state ? -0.2 : 0);;
        break;
      case UPRIGHT_CONE:
        this.setIntakeSpeed(state ? 0.4 : 0);
        break;
      case TIPPED_CONE:
        this.setSolenoid(state);
        this.setIntakeSpeed(state ? 0.6 : 0);
    }
  }

  //Target Homing
  public void enableHoming() {
    this.m_targetHoming = true;
  }
  public void disableHoming() {
    this.m_targetHoming = false;
  }
  public void setPosToStorred() {
    this.m_encoder.setPosition(MAX_ANGLE);
  }

  @Override
  public void periodic(){
    //SmartDashboard.putNumber("Raw Encoder", this.m_encoder.getPosition());
    //System.out.println("1: " + this.m_encoder.getPosition() + " 2: " + this.m_wristmotor2.getEncoder().getPosition());

    if(this.m_targetHoming) {
      double angleDifference = this.getCurrentAngle() - m_targetAngle;
      double motorpower = (0.17*Math.cos(2*Math.PI*getCurrentAngle()/360));//(0.75*Math.pow(Math.cos(2*Math.PI*getCurrentAngle()/360),2) + 0.25) * m_pid.calculate(getCurrentAngle(), this.m_targetAngle);//-Math.pow(Math.cos(2*Math.PI*getCurrentAngle()/360),2) * kP * (angleDifference/360 - m_encoder.getVelocity());
      
      this.setArmSpeed(motorpower + velocityDelta);
    }
  }
  public void setVelocityDelta(double delta) {
    this.velocityDelta = (0.35)*delta;
  } 
  public void registerPositionDown() {
    this.m_encoder.setPosition(-7.8898);
  }

  @Deprecated
  public void setArmSpeed(double speed) {
    this.m_wristmotor.set(speed);
    this.m_wristmotor2.set(speed);
  }
  @Deprecated
  public void setIntakeSpeed(double speed) {
    this.m_intakemotor.set(speed);
  }
  @Deprecated
  public void setSolenoid(boolean state) {
    this.m_coneFlipSolenoid.set(state);
  }
  @Deprecated
  public void m1test() {
    this.m_wristmotor.set(-0.5);
  }
  @Deprecated
  public void m2test() {
    this.m_wristmotor2.set(-0.5);
  }
}
