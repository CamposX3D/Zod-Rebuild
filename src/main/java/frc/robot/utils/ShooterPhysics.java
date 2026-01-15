package frc.robot.utils;

public class ShooterPhysics {

    private static final double g = 9.81;

    public static double calculateRPM(
        double distance,
        double heightDiff,
        double shooterAngle,
        double wheelRadius
    ) {
        double term = distance * Math.tan(shooterAngle) - heightDiff;

        if(term <= 0){
            return 0;
        }

        double velocity =
            (distance / Math.cos(shooterAngle)) *
                Math.sqrt(g / (2 * term));

        return (velocity / (2 * Math.PI * wheelRadius)) * 60.0;
    }   
}
