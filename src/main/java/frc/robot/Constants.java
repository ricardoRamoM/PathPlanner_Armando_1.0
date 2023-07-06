
package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

  public static  class constantesChasis {
    public static double velocidadMaxima = 0.7;
    public static double diametroLlanta = 0.1524;
    public static double relacionBaja = 1.4;
    public static int botonArcade = 1;
    public static int botonTank = 2;
    public static boolean arcaadee = true;
    
    
    public static double radioDeLlantaPulgadas = 3;
    public static double relacionDeTransmision = 12.84;
    public static double factorDeConversionAMetros = Units.inchesToMeters((1/(relacionDeTransmision*2*Math.PI*0.0762)*10));
  }

  
  public static class constantesRamsetController {
    public static final double ksVolts = 0.213649;
    public static final double kvVoltsPerMeter = 3.228;
    public static final double kaVoltsSquaredPerMeter = 4.3441;
    public static final double kpDriveVelocity = 3.0162;

    public static double kCanchaWidth = Units.inchesToMeters(23);
    public static final DifferentialDriveKinematics kinematicasDrive = new DifferentialDriveKinematics(kCanchaWidth);

    public static final double kVelMaxMPS = 3;
    public static final double kAcelMaxMPS2 = 3;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;



  }
}
