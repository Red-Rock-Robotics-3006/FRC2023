package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Subsystems.Drivetrain;

@SuppressWarnings("unused")
public class RobotContainer {
    private final Joystick m_joystick1 = new Joystick(0);
    private final Joystick m_joystick2 = new Joystick(1);
    private final Drivetrain m_swerve = new Drivetrain(new Pose2d());

    //Add in SlewRateLimiters

    private Command driveCommand = null;

    public RobotContainer() {}

    public void enableControllers() {
        if(m_swerve.getDefaultCommand() != null) m_swerve.getDefaultCommand().cancel();

        RunCommand dc = new RunCommand(
            () -> m_swerve.drive(
                m_joystick1.getRawAxis(0), 
                m_joystick1.getRawAxis(1), 
                m_joystick1.getRawAxis(2)*100, 
                true
            ),
            m_swerve
        );
        dc.setName("Joystick Control");

        this.driveCommand = dc;
        m_swerve.setDefaultCommand(dc);
    }
    public void disableControllers() {
        if(this.driveCommand == null && this.driveCommand.getName().equals("Joystick Control")) {
            this.driveCommand.cancel();
            this.driveCommand = null;
        }
    }

    public void zeroAllOutputs() {
        if(m_swerve.getDefaultCommand() != null) m_swerve.getDefaultCommand().cancel();

        RunCommand dc = new RunCommand(
            () -> m_swerve.zeroWheels(),
            m_swerve
        );
        dc.setName("Zeroing Control");

        this.driveCommand = dc;
        m_swerve.setDefaultCommand(dc);
    }

    public Command getAutonomousCommand() {
        return new RunCommand(
            () -> {}, 
            this.m_swerve
        );
    }
}
