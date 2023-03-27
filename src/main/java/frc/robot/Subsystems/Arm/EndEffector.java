package frc.robot.Subsystems.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.HashMap;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class EndEffector extends SubsystemBase {
  //Components
  private final Solenoid m_coneFlipSolenoid = new Solenoid(33, PneumaticsModuleType.CTREPCM, 1);
  private final CANSparkMax m_intakemotor = new CANSparkMax(61, CANSparkMaxLowLevel.MotorType.kBrushless); //!FILLER VALUE!
  private final CANSparkMax m_wristmotor = new CANSparkMax(62, CANSparkMaxLowLevel.MotorType.kBrushless); //!FILLER VALUE!
  private final CANCoder m_cCoder = new CANCoder(0);

  //Target
  private double m_targetAngle = 0;
  private GamePieceMode m_mode = GamePieceMode.CUBE;
  private boolean m_targetHoming = false; //Controls whether the mechanisms automatically seeks out targets

  //Constants
  public static final double ARM_LENGTH = 0.25; //!FILLER VALUE!
  public static final double MIN_ANGLE = -50; //!FILLER VALUE!
  public static final double MAX_ANGLE = 50; //!FILLER VALUE!
  public static final double CENTER_ANGLE = 0; //!FILLER VALUE!
  private static final double kP = 0.01; //!FILLER VALUE!
  private static final HashMap<GamePieceMode,Translation2d> targetOffsets = new HashMap<>();

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

    //Target offsets
    targetOffsets.put(GamePieceMode.CUBE, new Translation2d()); //!FILLER VALUE!
    targetOffsets.put(GamePieceMode.UPRIGHT_CONE, new Translation2d()); //!FILLER VALUE!
    targetOffsets.put(GamePieceMode.TIPPED_CONE, new Translation2d()); //!FILLER VALUE!
  }

  public void setTargetAngle(double angle) throws IllegalArgumentException {
    this.m_targetAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angle));
    if(Math.abs(this.m_targetAngle-angle) > 0.0001) throw new IllegalArgumentException("Angle is beyond EndEffector range.");
  }
  public void setMode(GamePieceMode mode) {
    this.m_mode = mode;
  }
  public double getCurrentAngle() {
    return m_cCoder.getAbsolutePosition();
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
        this.m_intakemotor.set(state ? 0.2 : 0);
        break;
      case UPRIGHT_CONE:
        this.m_intakemotor.set(state ? -0.4 : 0);
        break;
      case TIPPED_CONE:
        this.m_coneFlipSolenoid.set(!state);
        this.m_intakemotor.set(state ? -0.6 : 0);
    }
  }
  public void expelControl(boolean state) {
    switch(this.m_mode) {
      case CUBE:
        this.m_intakemotor.set(state ? -0.2 : 0);
        break;
      case UPRIGHT_CONE:
        this.m_intakemotor.set(state ? 0.4 : 0);
        break;
      case TIPPED_CONE:
        this.m_coneFlipSolenoid.set(state);
        this.m_intakemotor.set(state ? 0.6 : 0);
    }
  }

  //Target Homing
  public void enableHoming() {
    this.m_targetHoming = true;
  }
  public void disableHoming() {
    this.m_targetHoming = false;
  }

  @Override
  public void periodic(){
    if(this.m_targetHoming) {
      double angleDifference = this.getCurrentAngle() - m_targetAngle;
      double motorpower = angleDifference * kP / (MAX_ANGLE-MIN_ANGLE);
      
      this.m_wristmotor.set(motorpower);
    }
  }

  @Deprecated
  public void setArmSpeed(double speed) {
    this.m_wristmotor.set(speed);
  }
  @Deprecated
  public void setIntakeSpeed(double speed) {
    this.m_intakemotor.set(speed);
  }
  @Deprecated
  public void setSolenoid(boolean state) {
    this.m_coneFlipSolenoid.set(state);
  }
}
