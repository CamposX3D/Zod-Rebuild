package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.math.SwerveMath;

public class SwerveConstants {

     public static final class Dimensoes {
    public static final double LOOP_TIME = 0.13; // Tempo de loop (sparkMax + normal = 130ms)
    public static final double ROBOT_MASS = 49; // Massa do robô

    // Posições do centro de massa
    private static final double X_MASS = 0;
    private static final double Y_MASS = 0;
    private static final double Z_MASS = .08;

    // Centro de massa do chassi
    public static final Matter CHASSIS = new Matter(
        new Translation3d(X_MASS, Y_MASS, (Z_MASS)), ROBOT_MASS);

    // Máxima aceleração e velocidade
    public static final double MAX_ACCE_AUTO = 4;
    public static final double MAX_VEL_AUTO = 4;

    // Diâmetro da roda do módulo
    public static final double wheelDiameterInMeters = Units.inchesToMeters(3.75);

    // Redução para motor de acionamento e ângulo
    public static final double driveGearRatio = 6.75;
    public static final double angleGearRatio = 21.42;

    // PPR do encoder interno NEO;
    public static final double pulsePerRotation = 1;

    // Fatores de conversão para motores de acionamento e ângulo
    public static final double driveConversion = SwerveMath.calculateMetersPerRotation(wheelDiameterInMeters,
        driveGearRatio, pulsePerRotation);
    public static final double angleConversion = SwerveMath.calculateDegreesPerSteeringRotation(angleGearRatio,
        pulsePerRotation);
  }

  /**
   * Configurações relacionadas ao controle de tração.
   */
  public static final class Tracao {
    public static final boolean fieldRelative = true; // Orientação ao campo
    public static final boolean isOpenLoop = false; // Malha fechada
    public static final boolean accelCorrection = false; // Correção de aceleração

    // Multiplicadores para suavizar entradas do joystick
    public static final double multiplicadorRotacional = 0.8;
    public static final double multiplicadorTranslacionalY = 0.7;
    public static final double multiplicadorTranslacionalX = 0.7;

    public static final double TURN_CONSTANT = 0.75; // Constante de rotação
    public static final double MAX_SPEED = 4.8; // Velocidade máxima (m/s)
    public static final double DT = 0.02; // Intervalo de tempo (s)
    public static final double constantRotation = 4.0; // Rotação constante
  }

  /**
   * Configurações das trajetórias autônomas.
   */
  public static final class Trajetoria {
    public static final String NOME_TRAJETORIA = "New Auto";
    public static final String NOME_TRAJETORIA2 = "New Path2";
  }

  /**
   * Estados usados em strings.
   */
  public static final class StateStrings {

    public static final String OFF = "OFF";
    public static final String ON = "ON";
  }

    
}
