package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class VisionShootCommand extends Command {
    private final ShooterSubsystem shooter;

    public VisionShootCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        double distance = shooter.getVisionDistanceMeters();
        if (distance > 0) {
            shooter.setRPMFromDistance(distance);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setRPM(0);
    }

    @Override
    public boolean isFinished() {
        return false; // Roda enquanto o B estiver pressionado
    }
}
