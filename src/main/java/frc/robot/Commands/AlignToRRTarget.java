package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Exceptions.InvalidStateException;
import frc.robot.Exceptions.NoTargetsFoundException;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Limelight;

public class AlignToRRTarget extends CommandBase {
    private static final double kP = 0.01;
    private static final double maxError = 1;
    private final double offset;
    private boolean noTargetsFlag = false;
    private double error = 0;

    public AlignToRRTarget(double offset) {
        this.offset = offset;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Limelight.getInstance().setCamMode(Limelight.CAM_MODE.PROCESSING);
        Limelight.getInstance().setPipeline(Limelight.PIPELINE.RETROREFLECTIVE);
        Limelight.getInstance().setLEDMode(Limelight.LED_MODE.CURRENT_PIPELINE);

        try {
            this.error = Limelight.getInstance().getNodeCamPos().getFirst().getCenter().getX() - this.offset;
            Drivetrain.getInstance().drive(0, 0, kP*this.error, false);
        } catch(NoTargetsFoundException e) {
            this.noTargetsFlag = true;
        } catch(InvalidStateException e) {
            System.out.println("INCORRECTLY CONFIGURED RETROREFLECTIVE PIPELINE.");
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override 
    public boolean isFinished() {
        return this.noTargetsFlag || Math.abs(this.error) < maxError;
    }
}
