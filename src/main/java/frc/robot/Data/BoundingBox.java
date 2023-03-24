package frc.robot.Data;

import edu.wpi.first.math.geometry.Translation2d;

public class BoundingBox {
    private final static double hPix = 640; //Needs confirmation (may conflict with docs which claim 0-320 range on box side sizes)
    private final static double vPix = 480; //Needs confirmation (may conflict with docs which claim 0-320 range on box side sizes)
    private final static double hFOV = 59.6;
    private final static double vFOV = 45.7;

    private final Translation2d camPos; //Angle offsets from center to target (in accordance with LL outputs)
    private final int hLength; //Box h side length in pixels (in accordance with LL outputs)
    private final int vLength; //Box v side length in pixels (in accordance with LL outputs)

    public BoundingBox(Translation2d pos, int hLength, int vLength) {
        this.camPos = new Translation2d(pos.getX(), pos.getY());
        this.hLength = hLength;
        this.vLength = vLength;
    }
    public BoundingBox(Translation2d pos) {
        this(pos, 0, 0);
    }

    public Translation2d getCenter() {
        return new Translation2d(camPos.getX(), camPos.getY());
    }

    public Translation2d getBoxSize() {
        return new Translation2d(hLength*(hFOV/hPix), vLength*(vFOV/vPix));
    }
    private Translation2d getBoxSizeFlipped() {
        return new Translation2d(hLength*(hFOV/hPix), -vLength*(vFOV/vPix));
    }

    public Translation2d getUpperLeft() {
        return getCenter().minus(getBoxSizeFlipped().div(2));
    }
    public Translation2d getUpperRight() {
        return getCenter().plus(getBoxSize().div(2));
    }
    public Translation2d getLowerLeft() {
        return getCenter().minus(getBoxSize().div(2));
    }
    public Translation2d getLowerRight() {
        return getCenter().plus(getBoxSizeFlipped().div(2));
    }
}

