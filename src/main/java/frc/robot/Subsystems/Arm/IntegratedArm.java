package frc.robot.Subsystems.Arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntegratedArm extends SubsystemBase {
  //Instance
  private IntegratedArm instance;

  //Sub-Mechanisms
  private final ArmActuator m_arm = new ArmActuator();
  private final EndEffector m_endEffector = new EndEffector();

  //Target
  private Pose2d m_target = new Pose2d(
    new Translation2d(HorizontalExtender.MIN_EXTENSION, Elevator.MIN_HEIGHT), 
    new Rotation2d(EndEffector.MAX_ANGLE)
  );
  private GamePieceMode m_mode = GamePieceMode.CUBE;

  //Singleton Control
  private IntegratedArm(){
    //Basic Setup
    this.setName("Integrated Arm");
    this.register();
  }
  public IntegratedArm getInstance() {
    if(instance==null) instance = new IntegratedArm();
    return instance;
  }

  //Measured State
  public Pose2d getTargetPose() {
    return this.m_target;
  }
  public Pose2d getMeasuredPos() { 
    return new Pose2d(
      m_arm.getMeasuredPos()
        .plus(m_endEffector.getMeasuredPiecePos()), 
      Rotation2d.fromDegrees(m_endEffector.getCurrentAngle())
    ); 
  }
  public boolean hasAchievedState() {
    return 
      Math.abs(this.m_target.getX()-getMeasuredPos().getX()) < 0.02 &&
      Math.abs(this.m_target.getY()-getMeasuredPos().getY()) < 0.02 &&
      Math.abs(this.m_target.getRotation().getDegrees()-getMeasuredPos().getRotation().getDegrees()) < 3;
  } 

  //Setters
  public void setTargetPos(Pose2d pose) {
      this.m_target = pose;
      try {
        this.m_endEffector.setTargetAngle(pose.getRotation().getDegrees());
        this.m_arm.setTargetPos(
          pose
            .getTranslation()
            .minus(this.m_endEffector.getTargetPiecePos())
        );
      } catch(IllegalArgumentException e) {
        System.out.println(e.getMessage());
      }
  }
  public void setMode(GamePieceMode mode) {
    this.m_mode = mode;
    this.m_endEffector.setMode(this.m_mode);
  }

  //Control Intake
  public void intakeControl(boolean state) {
    this.m_endEffector.intakeControl(state);
  }
  public void expelControl(boolean state) {
    this.m_endEffector.expelControl(state);
  }

  //Target Homing
  public void enableHoming() {
    this.m_arm.enableHoming();
    this.m_endEffector.enableHoming();
  }
  public void disableHoming() {
    this.m_arm.disableHoming();
    this.m_endEffector.disableHoming();
  }
}
