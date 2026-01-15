package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class ShooterConstants {

    //Alturas
    public static final double TARGET_HEIGHT = 1.8288;
    public static final double SHOOTER_EXIT_HEIGHT = 1.0;

    //Geometria
    public static final double SHOOTER_ANGLE_RAD = 
        Math.toRadians(40);

    //Shooter Wheel
    public static final double WHEEL_RADIUS =
        Units.inchesToMeters(2);  //Roda de 4"

    public static final double MIN_RPM = 2500;
    public static final double MAX_RPM = 6000;

    public static final int SHOOTER_ID = 0;
    
}
