package org.firstinspires.ftc.teamcode.TELEOP;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Reloc extends OpMode {
    private int[] validIDs = {20, 24};
    private Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(85);


    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {

            Pose robot = convertPose3DtoPedroPose(result.getBotpose());
            limelight.updateRobotOrientation(robot.getHeading() + 90);
            Pose robotMT2 = convertPose3DtoPedroPose(result.getBotpose_MT2());
            telemetry.addData("x mt1", robot.getX());
            telemetry.addData("y mt1", robot.getY());
            telemetry.addData("h mt1", robot.getHeading());
            telemetry.addLine(" ");
            telemetry.addData("x mt2", robotMT2.getX());
            telemetry.addData("y mt2", robotMT2.getY());
            telemetry.addData("h mt2", robotMT2.getHeading());
        }
    }
    public static Pose convertPose3DtoPedroPose(Pose3D pose3d) {

        double x = pose3d.getPosition().y * 39.3701;
        double y = -pose3d.getPosition().y * 39.3701;
        double yaw = Math.toRadians(pose3d.getOrientation().getYaw() - 90);

        return new Pose(x, y, yaw);
    }
}
