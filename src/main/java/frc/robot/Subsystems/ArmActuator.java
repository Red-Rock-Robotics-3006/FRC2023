package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmActuator extends SubsystemBase {
    public void setTargetPos(Translation2d pos) {}
    public void setTargetPosSafeTravel(Translation2d pos, double arcRadius) {}
    public Translation2d getMeasuredPos() { return new Translation2d(); }
}
