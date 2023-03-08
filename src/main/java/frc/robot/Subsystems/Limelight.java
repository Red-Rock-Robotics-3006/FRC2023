package frc.robot.Subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Exceptions.InvalidStateException;
import frc.robot.Exceptions.NoTargetsFoundException;
import frc.robot.Other.BoundingBox;
import frc.robot.Other.DetectedObject;

public class Limelight {
    private static Limelight instance;

    private PIPELINE pipelineSelection;
    private LED_MODE ledSelection;
    private CAM_MODE camModeSelection;

    private Limelight() {}

    public Limelight getInstance() {
        if(instance==null) instance = new Limelight();
        return instance;
    }

    // Get Outputs
    public Pair<Pose3d,Double> getBotFieldPos() throws InvalidStateException, NoTargetsFoundException {
        if(this.pipelineSelection!=PIPELINE.APRILTAGS || this.camModeSelection!=CAM_MODE.PROCESSING) 
            throw new InvalidStateException("Invalid Limelight state.");

        double tagID = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getNumber(0).intValue();
        if(tagID==-1)
            throw new NoTargetsFoundException("No April Tags visible.");
        
        double[] output = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[7]);
        Translation3d t3d = new Translation3d(output[0], output[1], output[2]);
        Rotation3d r3d = new Rotation3d(output[3], output[4], output[5]);
        Pose3d p3d = new Pose3d(t3d, r3d);

        return new Pair<>(p3d, output[6]); //Returns Pos and Latency
    }
    public Pair<BoundingBox,Double> getNodeCamPos() throws InvalidStateException, NoTargetsFoundException {
        if(this.pipelineSelection!=PIPELINE.RETROREFLECTIVE || this.camModeSelection!=CAM_MODE.PROCESSING) 
            throw new InvalidStateException("Invalid Limelight state.");

        double tagID = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getNumber(0).intValue();
        if(tagID==-1)
            throw new NoTargetsFoundException("No targets visible.");
        
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getNumber(0).doubleValue();
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getNumber(0).doubleValue();
        double processingLatency = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getNumber(0).doubleValue();
        double captureLatency = NetworkTableInstance.getDefault().getTable("limelight").getEntry("cl").getNumber(0).doubleValue();

        return new Pair<>(new BoundingBox(new Translation2d(tx, ty)), processingLatency+captureLatency); //Returns Translation from robot forwards to target and Latency
    }
    public Pair<BoundingBox,Double> getGameElementCamPos() throws InvalidStateException, NoTargetsFoundException {
        if(this.pipelineSelection!=PIPELINE.GAMEPIECE_BOXES || this.camModeSelection!=CAM_MODE.PROCESSING) 
            throw new InvalidStateException("Invalid Limelight state.");

        double tagID = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getNumber(0).intValue();
        if(tagID==-1)
            throw new NoTargetsFoundException("No targets visible.");
        
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getNumber(0).doubleValue();
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getNumber(0).doubleValue();
        int tHorizontal = NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor").getNumber(0).intValue();
        int tVertical = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tvert").getNumber(0).intValue();
        int tClass = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tclass").getNumber(0).intValue();

        double processingLatency = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getNumber(0).doubleValue();
        double captureLatency = NetworkTableInstance.getDefault().getTable("limelight").getEntry("cl").getNumber(0).doubleValue();

        return new Pair<>(
            new DetectedObject(
                new Translation2d(tx, ty), 
                tHorizontal, 
                tVertical,
                tClass), 
            processingLatency+captureLatency
        ); //Returns BoundingBox on gamepiece and Latency
    }
    public Pair<BoundingBox,Double> getConeRotCamPos() throws InvalidStateException, NoTargetsFoundException {
        if(this.pipelineSelection!=PIPELINE.GAMEPIECE_BOXES || this.camModeSelection!=CAM_MODE.PROCESSING) 
            throw new InvalidStateException("Invalid Limelight state.");

        double tagID = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getNumber(0).intValue();
        if(tagID==-1)
            throw new NoTargetsFoundException("No targets visible.");
        
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getNumber(0).doubleValue();
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getNumber(0).doubleValue();
        int tHorizontal = NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor").getNumber(0).intValue();
        int tVertical = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tvert").getNumber(0).intValue();
        int tClass = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tclass").getNumber(0).intValue();

        double processingLatency = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getNumber(0).doubleValue();
        double captureLatency = NetworkTableInstance.getDefault().getTable("limelight").getEntry("cl").getNumber(0).doubleValue();

        return new Pair<>(
            new DetectedObject(
                new Translation2d(tx, ty), 
                tHorizontal, 
                tVertical,
                tClass), 
            processingLatency+captureLatency
        ); //Returns BoundingBox on cone-end and Latency
    }

    // Control
    //Limelight Control Vals shoulds never be set elsewhere
    public void setPipeline(PIPELINE p) {
        this.pipelineSelection = p;
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(p.getVal());
    }
    public void setLEDMode(LED_MODE m) {
        this.ledSelection = m;
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(m.getVal());
    }
    public void setCamMode(CAM_MODE m) {
        this.camModeSelection = m;
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(m.getVal());
    }

    // Read State
    public PIPELINE getPipeline() { return this.pipelineSelection; }
    public LED_MODE getLEDMode() { return this.ledSelection; }
    public CAM_MODE getCamMode() { return this.camModeSelection; }

    // State Enums
    public enum PIPELINE {
        APRILTAGS(0),
        RETROREFLECTIVE(1),
        GAMEPIECE_BOXES(2),
        TIPPED_CONE_BOXES(3);

        private final int val;
        private PIPELINE(int i) { this.val = i; }

        public int getVal() { return this.val; }
    }
    public enum LED_MODE {
        CURRENT_PIPELINE(0),
        FORCE_OFF(1),
        FORCE_BLINK(2),
        FORCE_ON(3);

        private final int val;
        private LED_MODE(int i) { this.val = i; }

        public int getVal() { return this.val; }
    }
    public enum CAM_MODE {
        PROCESSING(0),
        RAW(1);

        private final int val;
        private CAM_MODE(int i) { this.val = i; }

        public int getVal() { return this.val; }
    }
}
