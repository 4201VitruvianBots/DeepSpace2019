/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.vitruvianlib.drivers.FactoryTalonSRX;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Vision extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    static UsbCamera usbCamera;

    public Vision() {
        super("Vision");
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

    public double getTargetX() {
        return limelightTable.getEntry("tx").getDouble(0);
    }

    public boolean isValidTarget() {
        return (limelightTable.getEntry("tv").getDouble(0) == 1) ? true : false;
    }

    public void initUSBCamera() {
        usbCamera = CameraServer.getInstance().startAutomaticCapture();
        //usbCamera.setc
    }


    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
