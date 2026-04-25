package org.firstinspires.ftc.teamcode.TELEOP;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "reloc")
public class Reloc extends OpMode {
    private int[] validIDs = {20, 24};
    private Limelight3A limelight;
    private Follower follower;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        follower.setStartingPose(new Pose(72, 72, 3 * Math.PI / 4));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(85);
    }

    @Override
    public void loop() {
        follower.update();
        Pose odoPose = follower.getPose();

        // Feed odometry heading to Limelight for MT2 (positive degrees, CCW)
        if (odoPose != null) {
            limelight.updateRobotOrientation(-135);
        }

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D mt1 = result.getBotpose();
            Pose3D mt2 = result.getBotpose_MT2();

            if (mt1 != null) {
                Pose robotMT1 = convertPose3DtoPedroPose(mt1);
                telemetry.addData("x mt1", robotMT1.getX());
                telemetry.addData("y mt1", robotMT1.getY());
                telemetry.addData("h mt1 (deg)", robotMT1.getHeading());
            }
            telemetry.addLine(" ");
            if (mt2 != null) {
                Pose robotMT2 = convertPose3DtoPedroPose(mt2);
                telemetry.addData("x mt2", robotMT2.getX());
                telemetry.addData("y mt2", robotMT2.getY());
                telemetry.addData("h mt2 (deg)", Math.toDegrees(robotMT2.getHeading()));

                // Raw meters - all three axes so we can identify which is which
                telemetry.addLine(" ");
                telemetry.addData("MT2 raw x (m)", mt2.getPosition().x);
                telemetry.addData("MT2 raw y (m)", mt2.getPosition().y);
                telemetry.addData("MT2 raw z (m)", mt2.getPosition().z);
                telemetry.addData("MT2 raw yaw (deg)", mt2.getOrientation().getYaw());
                telemetry.addData("MT2 raw pitch (deg)", mt2.getOrientation().getPitch());
                telemetry.addData("MT2 raw roll (deg)", mt2.getOrientation().getRoll());
            }
        } else {
            telemetry.addLine("No valid LL result");
        }

        telemetry.addLine(" ");
        if (odoPose != null) {
            telemetry.addData("ODO X", odoPose.getX());
            telemetry.addData("ODO Y", odoPose.getY());
            telemetry.addData("ODO H (deg)", Math.toDegrees(odoPose.getHeading()));
        }

        telemetry.update();
    }

    public static Pose convertPose3DtoPedroPose(Pose3D pose3d) {
        final double M_TO_IN = 39.37007874015748;
//        double x = 72 + pose3d.getPosition().x * M_TO_IN;
//        double y = 72 + pose3d.getPosition().y * M_TO_IN;
//
//        double yaw = Math.toRadians(pose3d.getOrientation().getYaw());
//
//        while (yaw >= Math.PI) yaw -= 2 * Math.PI;
//        while (yaw < -Math.PI) yaw += 2 * Math.PI;
        double x = pose3d.getPosition().x;
        double y = pose3d.getPosition().y;
        double yaw = pose3d.getOrientation().getYaw();


        return new Pose(x, y, yaw);
    }
}