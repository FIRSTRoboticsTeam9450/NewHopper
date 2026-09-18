package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class TurretModel {

    public static final double G = 9.81;

    //theta,phi (theta = rotation,phi = hood angle)
    public static double[] getRotation(Translation3d target, Translation2d robotPos, double vx, double vy, double v) {
        target = target.minus(new Translation3d(robotPos));
        double p = 4*(Math.pow(vx,2) + Math.pow(vy,2) + G * target.getZ() - Math.pow(v,2))/Math.pow(G,2);
        double q = 8*(vx*target.getX()+vy*target.getY())/Math.pow(G,2);
        double t_0 = getT0(target, p, q);
        return new double[]{Math.atan2(target.getY() + vy*t_0,target.getX() + vx*t_0),Math.atan2(Math.sqrt(Math.pow((target.getX()+vx*t_0),2) + Math.pow(target.getY()+vy*t_0,2)),target.getZ()+G*Math.pow(t_0,2)/2)};
    }

    private static double getT0(Translation3d target, double p, double q) {
        double r = 4*(Math.pow(target.getX(),2)+Math.pow(target.getY(),2)+Math.pow(target.getZ(),2))/Math.pow(G,2);
        double q_c = (2*Math.pow(-4* p,3)-72*(-4* p)*(-8*r)+27*64*(4* p *r-Math.pow(q,2)))/13824;
        double p_c = (-192*r-Math.pow(-4* p,2))/576;
        double sub_uv = (Math.pow(q_c, 2) / 4) + (Math.pow(p_c, 3) / 27);
        double u_0 = Math.pow((-Math.pow(q_c,2)/2)+Math.sqrt(sub_uv),1.0/3);
        double v_0 = Math.pow((-Math.pow(q_c,2)/2)-Math.sqrt(sub_uv),1.0/3);
        double m_0 = u_0 + v_0;
        double S = Math.sqrt(2*m_0- p);
        return (S+Math.sqrt(Math.pow(S,2)-4*m_0-2* q /S))/2;
    }

}
