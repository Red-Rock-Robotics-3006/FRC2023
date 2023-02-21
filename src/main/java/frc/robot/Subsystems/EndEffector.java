package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {
    public void setTargetAngle(double angle) {}
    public void intake(Mode mode) {}
    public void expel(Mode mode) {}
    public Translation2d getTargetCenterOffset() { return new Translation2d(); }
    public double getMeasuredAngle() { return 0; }

    public enum Mode {
        CUBE,
        UPRIGHT_CONE,
        TIPPED_CONE
    }
}
