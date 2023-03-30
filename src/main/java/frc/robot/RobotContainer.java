package frc.robot;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import frc.robot.Subsystems.Drivetrain.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Subsystems.Drivetrain;

import java.util.function.Consumer;

@SuppressWarnings("unused")
public class RobotContainer {
    private final Joystick m_joystick1 = new Joystick(0);
    private final Joystick m_joystick2 = new Joystick(1);
    private final Drivetrain m_swerve = new Drivetrain(new Pose2d());

    private SlewRateLimiter filterForLeftRightUpDown = new SlewRateLimiter(0.5); 
    private SlewRateLimiter filterForRotation = new SlewRateLimiter(0.5);
    //mess with parameter a bit to get desired output-flow??


    // Load paths to trajectories
    PathPlannerTrajectory BottomOut = PathPlanner.loadPath("Bottom-Out", new PathConstraints(1.5, 3.3));
    PathPlannerTrajectory BottomStation = PathPlanner.loadPath("Bottom-Station", new PathConstraints(1.5, 3.3));
    PathPlannerTrajectory BottomOutWait = PathPlanner.loadPath("Bottom-Out-Wait", new PathConstraints(1.5, 3.3));
    PathPlannerTrajectory MiddleBottom = PathPlanner.loadPath("Middle-Bottom", new PathConstraints(1.5, 3.3));
    PathPlannerTrajectory MiddleBottomReturn = PathPlanner.loadPath("Middle-Bottom-Return", new PathConstraints(1.5, 3.3));
    PathPlannerTrajectory MiddleTopReturn = PathPlanner.loadPath("Middle-Top-Return", new PathConstraints(1.5, 3.3));
    PathPlannerTrajectory MiddleTop = PathPlanner.loadPath("Middle-Top", new PathConstraints(1.5, 3.3));
    PathPlannerTrajectory TopOut = PathPlanner.loadPath("Top-Out", new PathConstraints(1.5, 3.3));
    PathPlannerTrajectory TopStation = PathPlanner.loadPath("Top-Station", new PathConstraints(1.5, 3.3));
    PathPlannerTrajectory TopOutWait = PathPlanner.loadPath("Top-Out-Wait", new PathConstraints(1.5, 3.3));

    ArrayList<PathPlannerTrajectory> pathOne = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("New Path", new PathConstraints(1.5, 3.3));
    ArrayList<PathPlannerTrajectory> pathTwo = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("New New Path", new PathConstraints(1.5, 3.3));
    ArrayList<PathPlannerTrajectory> TopAuto = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("Top-Out-Event", new PathConstraints(1.5, 3.3));
    ArrayList<PathPlannerTrajectory> BottomAuto = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("Bottom-Out-Event", new PathConstraints(1.5, 3.3));


    //Auto selection
    private Command[] autoCommands = new Command[]{};
    private final SendableChooser<Command> m_chooser = new SendableChooser<>();
    
    private Command driveCommand = null;

    public RobotContainer() {
        

        Map<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        eventMap.put("intakeDown", new PrintCommand("Intake Down"));
        eventMap.put("marker2", new PrintCommand("Passed marker 2"));
        eventMap.put("marker3", new PrintCommand("Passed marker 4"));
        eventMap.put("marker4", new PrintCommand("Passed marker 3"));

        // Replace PrintCommand with scoring command
        eventMap.put("Score", new PrintCommand("REPLACE THIS COMMAND WITH THE COMMAND TO SCORE"));

        // Replace with wait
        // Hopefully this won't be necessary, so ignore for now
        eventMap.put("Wait5s", new PrintCommand("REPLACE THIS WITH WHATEVER WILL MAKE THE ROBOT WAIT FOR 5 SECONDS"));

        //Consumer<SwerveModuleState[]> stateConsumer = (SwerveModuleState[] states) -> {m_swerve.setModuleStates(states);};
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            m_swerve.getOdometry()::getPoseMeters,
            m_swerve::resetPose,
            m_swerve.getKinematics(),
            new PIDConstants(0.0, 0.0, 0.0),
            new PIDConstants(0.0, 0.0, 0.0),
            /*new Consumer<SwerveModuleState[]>() {
                @Override
                public void accept(SwerveModuleState[] states) {
                    m_swerve.setModuleStates(states);
                }
            }*///m_swerve::setModuleStates,
            //stateConsumer,
            m_swerve::setModuleStates,
            eventMap,
            true,
            m_swerve
        );

        //Command autoOne = autoBuilder.fullAuto(pathOne);
        //Command autoTwo = autoBuilder.fullAuto(pathTwo);
        Command TopAutoC = autoBuilder.fullAuto(TopAuto);
        Command BottomAutoC = autoBuilder.fullAuto(BottomAuto);

        // ADD COMMAND IMPLEMENTATION OF SIMPLE AUTOS
        // Sample command I think
        //m_swerve.followTrajectoryCommand(BottomOut, true);

        autoCommands[0] = TopAutoC;
        autoCommands[1] = BottomAutoC;

        //Auto selection setup
        m_chooser.setDefaultOption(this.autoCommands[0].getName(), this.autoCommands[0]);
        for(int i = 1; i < this.autoCommands.length; i++) {
            m_chooser.addOption(this.autoCommands[i].getName(), this.autoCommands[i]);
        }
        SmartDashboard.putData("Auto choices", m_chooser);
    }

    public void enableControllers() {
        if(m_swerve.getDefaultCommand() != null) m_swerve.getDefaultCommand().cancel();

        RunCommand dc = new RunCommand(
            () -> m_swerve.drive(
                filterForLeftRightUpDown.calculate(m_joystick1.getRawAxis(0)), 
                filterForLeftRightUpDown.calculate(m_joystick1.getRawAxis(1)), 
                filterForRotation.calculate(m_joystick1.getRawAxis(2)*100),
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
        return this.m_chooser.getSelected();
    }
}
