/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.vitruvianlib.driverstation.Shuffleboard;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Vision {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    static UsbCamera usbCamera;

    public Vision() {
    }

    public int getPipeline() {
        return (int) limelightTable.getEntry("pipeline").getDouble(0);
    }

    public void setPipeline(int pipeline) {
      /*  Pipeline Definitions:
          0: Default Driver Vision
          1: Focus Target Center
          2: Focus Target Left
          3: Focus Target Right
       */
        limelightTable.getEntry("pipeline").setDouble(pipeline);
    }

    public int getCameraMode() {
        return (int) limelightTable.getEntry("camMode").getDouble(0);
    }

    public void setCameraMode(int camMode) {
        limelightTable.getEntry("camMode").setDouble(camMode);
    }

    public boolean isValidTarget() {
        return (limelightTable.getEntry("tv").getDouble(0) == 1) ? true : false;
    }

    public double getTargetX() {
        return limelightTable.getEntry("tx").getDouble(0);
    }

    public double getTargetY() {
        return limelightTable.getEntry("ty").getDouble(0);
    }

    public double getTargetArea() {
        return limelightTable.getEntry("ta").getDouble(0);
    }

    public double getTargetSkew() {
        return limelightTable.getEntry("ts").getDouble(0);
    }

    public double getLatency() {
        return limelightTable.getEntry("tl").getDouble(0);
    }

    public double getTShort() {
        return limelightTable.getEntry("tshort").getDouble(0);
    }

    public double getTLong() {
        return limelightTable.getEntry("tlong").getDouble(0);
    }

    public double getTHorz() {
        return limelightTable.getEntry("thor").getDouble(0);
    }

    public double getTVert() {
        return limelightTable.getEntry("tvert").getDouble(0);
    }

    public double getRawTx(int index) {
        return limelightTable.getEntry("tx" + Integer.toString(index)).getDouble(0);
    }

    public double getRawTy(int index) {
            return limelightTable.getEntry("ty" + Integer.toString(index)).getDouble(0);
    }

    public double getRawTa(int index) {
        return limelightTable.getEntry("ta" + Integer.toString(index)).getDouble(0);
    }

    public double getRawTs(int index) {
        return limelightTable.getEntry("ts" + Integer.toString(index)).getDouble(0);
    }




    public boolean IsTargetGood() {
        // TODO: Update with valid values
        boolean isAlignedX = Math.abs(getTargetX()) < 1;

        return isAlignedX; //&& isAlignedY & isAreaValid;
    }

    public void initUSBCamera() {
        try {
            usbCamera = CameraServer.getInstance().startAutomaticCapture();
            usbCamera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
            usbCamera.setResolution(160, 160);
            usbCamera.setFPS(15);
            usbCamera.setExposureManual(3);
        } catch(Exception e) {

        }
    }

    public void updateShuffleboard() {
        Shuffleboard.putBoolean("Vision","IsValidTarget", isValidTarget());
//        Shuffleboard.putBoolean("Vision", "IsTargetGood", IsTargetGood());
        Shuffleboard.putNumber("Vision", "Pipeline", getPipeline());
        Shuffleboard.putNumber("Vision", "tx", getTargetX());
        Shuffleboard.putNumber("Vision", "ty", getTargetY());
        Shuffleboard.putNumber("Vision", "ta", getTargetArea());
        Shuffleboard.putNumber("Vision", "ts", getTargetSkew());
//        Shuffleboard.putNumber("Vision", "tl", getLatency());
        Shuffleboard.putNumber("Vision", "tshort", getTShort());
        Shuffleboard.putNumber("Vision", "tlong", getTLong());
        Shuffleboard.putNumber("Vision", "thorz", getTHorz());
        Shuffleboard.putNumber("Vision", "tvert", getTVert());
    }

    public void updateSmartDashboard() {
//        Shuffleboard.putBoolean("Vision","IsValidTarget", isValidTarget());
////        Shuffleboard.putBoolean("Vision", "IsTargetGood", IsTargetGood());
//        Shuffleboard.putNumber("Vision", "Pipeline", getPipeline());
//        Shuffleboard.putNumber("Vision", "tx", getTargetX());
//        Shuffleboard.putNumber("Vision", "ty", getTargetY());
//        Shuffleboard.putNumber("Vision", "ta", getTargetArea());
//        Shuffleboard.putNumber("Vision", "ts", getTargetSkew());
////        Shuffleboard.putNumber("Vision", "tl", getLatency());
//        Shuffleboard.putNumber("Vision", "tshort", getTShort());
//        Shuffleboard.putNumber("Vision", "tlong", getTLong());
//        Shuffleboard.putNumber("Vision", "thorz", getTHorz());
//        Shuffleboard.putNumber("Vision", "tvert", getTVert());

//        for(int i = 0; i < 3; i++) {
//            Shuffleboard.putNumber("Vision", "Raw tx" + Integer.toString(i), getRawTx(i));
//            Shuffleboard.putNumber("Vision", "Raw ty" + Integer.toString(i), getRawTy(i));
//            Shuffleboard.putNumber("Vision", "Raw ta" + Integer.toString(i), getRawTa(i));
//            Shuffleboard.putNumber("Vision", "Raw ts" + Integer.toString(i), getRawTs(i));
//        }
    }
}
