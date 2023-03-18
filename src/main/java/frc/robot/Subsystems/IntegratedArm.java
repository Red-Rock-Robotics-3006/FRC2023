package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntegratedArm extends SubsystemBase {
    public Translation2d getMeasuredPos() { return new Translation2d(); }
    public double getMeasuredEndEffectorAngle() { return 0d; }
    public boolean hasAchievedState() { return true; } 
    public void setTargetPos(Translation2d pos) {}
    public void setTargetPosSafeTravel(Translation2d pos, double arcRadius) {}
    public void setTargetEndEffectorAngle(double angle) {}
    public void intake(EndEffector.Mode mode) {}
    public void expel(EndEffector.Mode mode) {}
}
