package frc.robot.Subsystems.Arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**TODO
 * Make dynamic motor control <- What is this?
 */
public class ArmActuator extends SubsystemBase {
  //Sub-Mechanisms
  private Elevator m_elevator = new Elevator();
  private HorizontalExtender m_extender = new HorizontalExtender();

  //Target
  private Translation2d targetPos;

  //Constants
  public final double X_MIN = HorizontalExtender.MIN_EXTENSION;
  public final double X_MAX = HorizontalExtender.MAX_EXTENSION;
  public final double Y_MIN = Elevator.MIN_HEIGHT;
  public final double Y_MAX = Elevator.MAX_HEIGHT;
  
  public ArmActuator(){
    //Basic Setup
    this.setName("Arm Actuator");
    this.register();
  }

  /** Sets the target position for arm relative to the gound and robot center */
  public void setTargetPos(Translation2d pos) throws IllegalArgumentException {
    this.m_extender.setTargetPos(this.targetPos.getX());
    this.m_elevator.setTargetPos(this.targetPos.getY());
  }
  /** Gets position for the end of the arm relative to the ground and robot center */
  public Translation2d getMeasuredPos() { 
    return new Translation2d(
      this.m_extender.getPos(), 
      this.m_elevator.getPos()
    ); 
  }

  //Target Homing
  public void enableHoming() {
    this.m_elevator.enableHoming();
    this.m_elevator.enableHoming();
  }
  public void disableHoming() {
    this.m_extender.disableHoming();
    this.m_extender.disableHoming();
  }
}
