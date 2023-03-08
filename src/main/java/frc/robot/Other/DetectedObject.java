package frc.robot.Other;

import edu.wpi.first.math.geometry.Translation2d;

public class DetectedObject extends BoundingBox {
    private int category;

    public DetectedObject(Translation2d pos, int hLength, int vLength, int cat) {
        super(pos, hLength, vLength);
        this.category = cat;
    }

    public int getCategory() {
        return this.category;
    }
}
