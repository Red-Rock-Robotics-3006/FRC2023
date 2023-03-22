package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.GyroSubsystem;

public class CSBalance extends CommandBase {
    private static final int gyroRelevantAxis = 1;
    private static final double movementThreshold = 5; //Degrees

    private double maxAngle = 0;
    private boolean angleCorrection;

    public CSBalance(boolean angleCorrection) {
        setName("Charge Station Balance");
        setSubsystem("Drivetrain");
        addRequirements(Drivetrain.getInstance());

        this.angleCorrection = angleCorrection;
    }

    @Override
    public void initialize() {
        Drivetrain.getInstance().drive(0, 0.1, 0, true);
    }

    @Override
    public void execute() {
        //Measure angle progress
        double[] ypr_deg = new double[3];
        GyroSubsystem.getPigeonInstance().getYawPitchRoll(ypr_deg);

        this.maxAngle = Math.max(this.maxAngle, ypr_deg[gyroRelevantAxis]);

        //PID Angle Correction
        if(this.angleCorrection) {
            double linearControl = Math.pow(GyroSubsystem.getPigeonInstance().getYaw()/360, 3)*0.1;
            System.out.println(linearControl);
            //Drivetrain.getInstance().drive(0, 0.1, linearControl, true);
        }
    }

    @Override
    public boolean isFinished() {
        double[] ypr_deg = new double[3];
        GyroSubsystem.getPigeonInstance().getYawPitchRoll(ypr_deg);

        return (this.maxAngle - ypr_deg[gyroRelevantAxis]) > movementThreshold;
    }

    @Override
    public void end(boolean interrupted) {
        Drivetrain.getInstance().drive(0, 0, 0, true);
    }
}
