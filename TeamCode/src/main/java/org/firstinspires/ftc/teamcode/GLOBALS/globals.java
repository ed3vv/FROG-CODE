package org.firstinspires.ftc.teamcode.GLOBALS;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;


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

    @Config
    public static class launcher {


        public static float p =0.001F; //0.001
        public static float i = 0.1F;
        public static float d = 0F;
        public static float ks = 0.067F; //0.0000216
        public static float kv = 0.00017F; //0.000000120871
        public static float velTime = 0.4F;
        public static float ang = 0F;
        public static float accelAlpha = 0.05F;
        public static float targetRPM = 3000F;

    }

    @Config
    public static class gate {
        public static float close = 0.25F; //
        public static float open = 0.8F;
    }

    @Config
    public static class turret {
        public static float camP = 0.05F;
        public static float turretOffset = -1F;
        public static float goalY = 129.5F;
        public static float goalX = 15;
        public static float pFarTele = 0.00006F;
        public static float i = 0F;
        public static float d = 0.000000F; // 0.00000065



        public static float turretangle = 302F;

    }

    @Config
    public static class states{
        public static Pose autoEndPose = new Pose(60, 84, Math.PI/2);
    }


    @Config
    public static class offsets{
        public static double xoff = 85;
        public static double yoff = -131.25;
    }
    public static double pHg = 1.0;
    public static double qH = 0.29;
    public static double rH = 0.009;

}

