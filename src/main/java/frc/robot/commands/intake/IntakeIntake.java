/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Intake;

/**
 * An example command.  You can replace me with your own command.
 */
public class IntakeIntake extends Command {
    Timer stopwatch = new Timer();
    private static boolean isTripped = false;

    public IntakeIntake() {
        // Use requires() here to declare subsystem dependencies
//        requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        switch (Intake.intakeState) {
            case 2:
                isTripped = false;
            case 1:
            case 0:
            default:
                //Robot.intake.setHarpoonExtend(false);
                break;
        }

        Intake.overridePassive = true;
    }
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        switch (Intake.intakeState) {
            case 2:
                Robot.intake.setCargoIntakeOutput(RobotMap.CARGO_INTAKE_SPEED);
                break;
            case 1:
//                Robot.intake.setHatchGroundIntakeOutput(RobotMap.HATCH_GROUND_INTAKE_SPEED);
                break;
            case 0:
            default:
                Robot.intake.setHatchIntakeOutput(RobotMap.HATCH_INTAKE_SPEED);
                break;
        }
    }

    @Override
    protected boolean isFinished() {
        if(Intake.intakeState == 2) {
            if (Robot.intake.bannerIR.get() && Intake.enableBannerSensor && !isTripped) {
                isTripped = true;
                stopwatch.reset();
                stopwatch.start();
            } else if(isTripped && !Robot.intake.bannerIR.get()) {
                isTripped = false;
                stopwatch.stop();
            }

            if (stopwatch.get() > 0.5)
                return true;
        }
        return false;
    }
    // Called once after isFinished returns true
    @Override
    protected void end() {
        switch (Intake.intakeState) {
            case 2:
                /*if (Robot.intake.bannerIR.get() && isTripped) {
                    Robot.wrist.setAbsoluteAngle(RobotMap.WRIST_RETRACTED_CARGO_ANGLE);
//                    Timer.delay(0.1);
                    Robot.intake.setCargoIntakeOutput(RobotMap.CARGO_HOLD_SPEED);
                    isTripped = false;
                } else*/
                    Robot.intake.setCargoIntakeOutput(0);
                break;
            case 1:
//                Robot.intake.setHatchIntakeOutput(RobotMap.HATCH_GROUND_HOLD_SPEED);
                break;
            case 0:
            default:
                stopwatch.reset();
                stopwatch.start();
                while (stopwatch.get() < 0.25) {

                }
                stopwatch.stop();
                Robot.intake.setHatchIntakeOutput(0);
                break;
        }

        Intake.overridePassive = false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
