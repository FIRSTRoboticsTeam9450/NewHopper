package frc.robot.util.model;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;

public class TurretModel {

    private static Translation3d P;
    private static Translation3d V;
    private static double vI;

    //theta, phi
    //TODO: remove delta_t after testing
    public static double[] calculate(Translation3d P, ChassisSpeeds V, double vI) {
        TurretModel.P = P;
        TurretModel.V = new Translation3d(-V.vxMetersPerSecond, -V.vyMetersPerSecond, 0);
        TurretModel.vI = vI;
        double t = newtonMethod();
        Logger.recordOutput("TurretModel/t", t);
        Logger.recordOutput("TurretModel/P", P);
        return new double[]{Math.atan2(Y(t), X(t)), Math.atan2(Math.sqrt(Math.pow(X(t), 2) + Math.pow(Y(t), 2)), Z(t))};
    }

    private static double X(double t) {
        return P.getX() + V.getX()*t;
    }

    private static double Y(double t) {
        return P.getY() + V.getY()*t;
    }

    private static double Z(double t) {
        return P.getZ() + V.getZ()*t + TurretModelConstants.TERMINAL_VELOCITY*(t - D(t));
    }

    private static double f(double t) {
        return Math.pow(X(t), 2) + Math.pow(Y(t), 2) + Math.pow(Z(t), 2) - Math.pow(vI*D(t),2);
    }

    private static double fPrime(double t) {
        return 2*(V.getX()*X(t)+V.getY()*Y(t)+Z(t)*(V.getZ()+TurretModelConstants.G*D(t)))-2*Math.pow(vI,2)*D(t)*Math.exp(-(t*TurretModelConstants.G)/TurretModelConstants.TERMINAL_VELOCITY);
    }

    private static double D(double t) {
        return (TurretModelConstants.TERMINAL_VELOCITY/TurretModelConstants.G)*(1-Math.exp(-(t*TurretModelConstants.G)/TurretModelConstants.TERMINAL_VELOCITY));
    }

    public static double newtonMethod() {
        double guess = TurretModelConstants.NEWTON_GUESS;
        for(int i = 0; i < TurretModelConstants.NEWTON_ITERATIONS; i++) {
            guess -= f(guess) / fPrime(guess);
        }
        return guess;
    }

}
