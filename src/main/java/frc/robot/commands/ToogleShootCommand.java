package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.vision.LimelightDistance;

public class ToogleShootCommand extends Command{

    private final ShooterSubsystem shooter;

    public ToogleShootCommand(ShooterSubsystem shooter){
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute(){
        double distance = LimelightDistance.getDistance();
        shooter.setRPMFromDistance(distance);
    }

    @Override
    public void end(boolean interrupted){
        shooter.setRPM(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}
