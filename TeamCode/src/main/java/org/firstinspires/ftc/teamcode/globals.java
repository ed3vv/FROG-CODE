package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;

@Config
public class globals {
    @Config
    public static class kalman {

        // Starting uncertainty (covariance) in odometry estimate, in inches^2
        public static double pX0 = 0.01;
        public static double pY0 = 0.01;

        // How much uncertainty (covariance) in odometry you add each update loop, in inches^2 per loop
        public static double qX = 1;
        public static double qY = 1;
        public static double qH = 1;
        // Camera measurement noise (variance), in inches^2
        public static double rX = 4;
        public static double rY = 4;
        public static double rH = 4;
    }

}