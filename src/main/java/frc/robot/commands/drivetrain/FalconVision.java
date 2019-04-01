package frc.robot.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.robot.Robot;
import frc.vitruvianlib.util.MovingAverage;

public class FalconVision extends PIDCommand {
    static double kP = 0.14; //Proportion for turning
    static double kI = 0; //Proportion for turning
    static double kD = 1.5;
    //Proportion for turning
    static double kF = 0; //Proportion for turning

    double lastTx = 0;
    boolean targetLocked = false;

    private MovingAverage averageTx = new MovingAverage(4);
    private double error = 0;

    public FalconVision() {
        super(kP, kI, kD);
        this.getPIDController().setF(kF);
        requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
//        lastTx = 0;
//        targetLocked = false;
        //Robot.driveTrain.setDriveMotorsState(false);
        this.getPIDController().setInputRange(-27, 27);
        this.getPIDController().setAbsoluteTolerance(0.1);
//        this.getPIDController().setOutputRange(-1, 1);
        this.getPIDController().setOutputRange(-0.85, 0.85);

        Robot.vision.setPipeline(0);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {

        if(Robot.vision.isValidTarget()) {
            this.getPIDController().enable();
        } else
            this.getPIDController().disable();
    }

    @Override
    protected double returnPIDInput() {
        averageTx.increment(Robot.vision.getTargetX());
        error = averageTx.getAverage() / 27.0;
//        double currentTx = -Robot.vision.getTargetX();

//        if(Robot.vision.isValidTarget())
//            lastTx = currentTx;

//        if(Math.abs(currentTx) < 3)
//            targetLocked = true;
//
//        if(targetLocked)
//            getPIDController().setD(0);

//        if(targetRatio < 0.45 && targetRatio > 0.15)
//            lastTx = currentTx;
//            lastRatio = targetRatio;
//        }
//        lastTx = Robot.vision.getTargetSkew() > 10 ? lastTx : currentTx;
//        lastTx = (targetRatio > .45 || targetRatio < 0.15) ? lastTx : currentTx;
//        lastTx = (Math.abs(currentTx - lastTx) > 10) ? lastTx : currentTx;
//        lastTx = (Math.abs(currentTx - lastTx) < 10) && Math.abs(lastTx) < Math.abs(currentTx) ? lastTx : currentTx;

        return averageTx.getAverage();
    }

    @Override
    protected void usePIDOutput(double output) {
//        double leftOutput = (Robot.m_oi.getLeftJoystickY() + Robot.m_oi.getRightJoystickX()) + output;
//        double rightOutput = (Robot.m_oi.getLeftJoystickY() - Robot.m_oi.getRightJoystickX()) - output;

//        leftOutput = lastTx > 15 ? leftOutput * 0.8 : leftOutput;
//        rightOutput = lastTx < -15 ? rightOutput * 0.8 : rightOutput;
        double leftOutput = 0;
        double rightOutput = 0;
        double PIDOutput = Math.abs(output) < 0.1 ? 0.1 : output;
        if(Robot.vision.isValidTarget()) {
            leftOutput = Robot.m_oi.getLeftJoystickY() + PIDOutput;
            rightOutput = Robot.m_oi.getLeftJoystickY() - PIDOutput;
        } else {
            leftOutput = Robot.m_oi.getLeftJoystickY() + Robot.m_oi.getRightJoystickX();
            rightOutput = Robot.m_oi.getLeftJoystickY() - Robot.m_oi.getRightJoystickX();
        }

        if (Robot.driveTrain.getTalonControlMode() == ControlMode.Velocity)
            Robot.driveTrain.setMotorVelocityOutput(leftOutput, rightOutput);
        else
            Robot.driveTrain.setMotorPercentOutput(leftOutput, rightOutput);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        getPIDController().disable();
        //Robot.driveTrain.setDriveMotorsState(true);
        //Robot.driveTrain.setDriveOutput(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
