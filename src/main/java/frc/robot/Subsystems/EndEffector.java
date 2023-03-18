package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class EndEffector extends SubsystemBase {

    private DoubleSolenoid handSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

    private boolean pcmMode = false; //True is forward, false is back

    public void setTargetAngle(double angle) {}
    public void intake(Mode mode) {}
    public void expel(Mode mode) {}
    public Translation2d getTargetCenterOffset() { return new Translation2d(); }
    public double getMeasuredAngle() { return 0; }

    public void togglePneumatics()
    {
        this.pcmMode = !this.pcmMode;
        this.handSolenoid.set(this.pcmMode ? kForward : kReverse);
    }

    public enum Mode {
        CUBE,
        UPRIGHT_CONE,
        TIPPED_CONE
    }
}
