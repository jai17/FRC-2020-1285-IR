/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team1285.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Add your docs here.
 */
public class Limelight {
    public final static int PIPELINE_DEFAULT = 0;
    public final static int OFF = 1;
    public final static int BLINK = 2;
    public final static int ON = 3;

    private static Limelight mInstance;

    private NetworkTable table;
    private boolean landscape = true;

    public static Limelight getInstance() {
        if (mInstance == null) {
            mInstance = new Limelight("limelight", false);
        }
        return mInstance;
    }

    // Change if using two limelights
    private Limelight(String name, boolean landscape) {
        table = NetworkTableInstance.getDefault().getTable(name);
        this.landscape = landscape;
    }

    public void setLEDMode(int ledMode) {// 0 is pipeline default, 1 is off, 2 is blink, 3 is on
        this.table.getEntry("ledMode").setNumber(ledMode);
    }

    public double getTargetX() {
        if (landscape)
            return this.table.getEntry("tx").getDouble(0);
        else
            return this.table.getEntry("ty").getDouble(0) + 2.6; // because portrait
    }

    public double getTargetY() {
        if (landscape)
            return this.table.getEntry("ty").getDouble(0);
        else
            return this.table.getEntry("tx").getDouble(0); // because portrait
    }

    public boolean getTargetExists() {
        return this.table.getEntry("tv").getDouble(0) == 1;
    }

    public double getTargetArea() {
        return this.table.getEntry("ta").getDouble(0);
    }

    public void setPipeline(int pipeline) {
        this.table.getEntry("pipeline").setNumber(pipeline);
    }

    public double pixelToDegree() {
        double pixel = getTargetX();
        // System.out.println(pixel);
        // System.out.println((-Math.toDegrees(Math.atan(((pixel - 120) *
        // Math.tan(Math.toRadians(45.7/2.0))) / 120.0))));
        return (-Math.toDegrees(Math.atan((2.0 * pixel * Math.tan(Math.toRadians(59.7 / 2.0)) / 240.0))));
    }
}