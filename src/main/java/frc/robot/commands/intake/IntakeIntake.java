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

import static frc.robot.subsystems.Intake.enableBannerSensor;

/**
 * An example command.  You can replace me with your own command.
 */
public class IntakeIntake extends Command {
    Timer stopwatch = new Timer();

    public IntakeIntake() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        switch (Intake.intakeState) {
            case 2:
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
        if(Intake.intakeState == 2)
            return Robot.intake.bannerIR.get() && Intake.enableBannerSensor;
        else
            return false;
    }
    // Called once after isFinished returns true
    @Override
    protected void end() {
        switch (Intake.intakeState) {
            case 2:
                if (Robot.intake.bannerIR.get()) {
                    stopwatch.reset();
                    stopwatch.start();
                    while (stopwatch.get() < 0.5) {

                    }
                    stopwatch.stop();
                    Robot.wrist.setAbsolutePosition(RobotMap.WRIST_RETRACTED_CARGO_ANGLE);
//                    Timer.delay(0.1);
                    Robot.intake.setCargoIntakeOutput(RobotMap.CARGO_HOLD_SPEED);
                } else {
                    Robot.intake.setCargoIntakeOutput(0);
                }
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
