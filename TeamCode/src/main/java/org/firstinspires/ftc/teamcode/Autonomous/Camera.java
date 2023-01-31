package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Camera {
    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;

    private SleeveDetection.ParkingPosition position;

    public boolean ready = false;

    public Camera(String hardwarename){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, hardwarename), cameraMonitorViewId);
        
        sleeveDetection = new SleeveDetection();

        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override public void onOpened(){
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
                ready = true;
            }

            @Override public void onError(int errorCode) {}
        });
    }

    void wait_till_ready(){
        while(!ready){};
    }

    void capture(){
        wait_till_ready();
        sleeveDetection.getPosition();
    }

    SleeveDetection.ParkingPosition read(){
        return position;
    }
}