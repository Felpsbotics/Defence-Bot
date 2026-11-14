package frc.robot.commands.Drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class DriveCommand extends Command{

    private final DriveSubsystem drive;
    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;
    private final DoubleSupplier turnInput;

    public DriveCommand(DriveSubsystem drive, DoubleSupplier forwardInput, DoubleSupplier leftInput, DoubleSupplier turnInput){
        
        this.drive = drive;
        this.forwardInput = forwardInput;
        this.leftInput = leftInput;
        this.turnInput = turnInput;

        addRequirements(drive);
    }

    @Override
    public void initialize() { 
    }

    @Override
    public void execute(){
        //if is not enabled, cancel scheduling
        if (!RobotState.isEnabled()){
            drive.stop();
            return;
        }

        //apply controller deadzone to inputs
        double fwd = MathUtil.applyDeadband(forwardInput.getAsDouble(), Constants.OperatorConstants.controllerDeadZone);
        double left = MathUtil.applyDeadband(leftInput.getAsDouble(), Constants.OperatorConstants.controllerDeadZone);
        double turn = MathUtil.applyDeadband(turnInput.getAsDouble(), Constants.OperatorConstants.controllerDeadZone);

        drive.drive(fwd, left, turn);
    }


    @Override
    public void end(boolean interrupted) {drive.stop();}

    @Override
    public boolean isFinished(){
        return false;
    }


}