/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode.Vuforia;

import android.graphics.Bitmap;
import android.media.ImageWriter;
import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.CameraCalibration;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Arrays;
import java.util.Locale;

@Autonomous(name = "Vuforia Grab OpenCV Picture", group = "Vuforia")
//@Disabled

public class ExampleVuforiaGrabPicture extends OpMode {
    private VuforiaLocalizer vuforiaLocalizer;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables velocityVortexImages;
    private VuforiaTrackable legosTarget;
    private VuforiaTrackableDefaultListener legosListener;

    Bitmap bitmap = null;
    Image RGBImage = null;
    Mat img = null;
    VuforiaLocalizer.CloseableFrame vuforiaFrame = null;

    private OpenGLMatrix lastLocation;
    private OpenGLMatrix phoneLocation;
    private OpenGLMatrix robotLocationTransform;

    float robotX, robotY, robotZ, robotAngle;
    Boolean buttonPressed = false;

//    public final static Scalar blueLow = new Scalar(108, 0, 220);
//    public final static Scalar blueHigh = new Scalar(178, 225, 255);

    beacon beaconState;

    public enum beacon {
        BEACON_NOT_VISIBLE,
        BEACON_RED_BLUE,
        BEACON_BLUE_RED,
        BEACON_ALL_RED,
        BEACON_ALL_BLUE
    }


    @Override
    public void init() {
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId); // To remove the camera view from the screen, remove the R.id.cameraMonitorViewId
        parameters.vuforiaLicenseKey = "AcZlc3n/////AAAAGWPeDCNLuk38gPuwF9cpyK2BYbGciGSeJy9AkSXPprQUEtg/VxgqB6j9WJuQvGo4pq+h4gwPSd134WD707FXnbuJjqdqkh5/92mATPs96WQ2RVoaU8QLbsJonufIl2T6qqqT83aOJHbz34mGJszad+Mw7VAWM11av5ltOoq8/rSKbmSFxAVi3d7oiT3saE0XBx4svhpGLwauy6Y0L7X0fC7FwHKCnw/RPL4V+Q8v2rtCTOwvjfnjxmRMind01HSWcxd9ppBwzvHVCPhePccnyWVv5jNiYXia9r4FlrJpAPgZ1GsCfdbt6AoT6Oh2Hnx267J+MHUnLi/C+0brvnQfcDregLBfnZApfd2c1WDiXJp/";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        vuforiaLocalizer = ClassFactory.getInstance().createVuforia(parameters);
//        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
//        vuforiaLocalizer.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time
        vuforiaLocalizer.enableConvertFrameToBitmap();
        // These are the vision targets that we want to use
        // The string needs to be the name of the appropriate .xml file in the assets folder
        velocityVortexImages = vuforiaLocalizer.loadTrackablesFromAsset("FTC_2016-17");
        //Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4); //Will track all 4 images

        // Setup the targets to be tracked
        legosTarget = velocityVortexImages.get(2);
        legosTarget.setName("Legos");
        legosTarget.setLocation(createMatrix(-150, 0, 0, 90, 0, 90));

        // Set phone location on robot
        phoneLocation = createMatrix(0, 0, 0, 90, 0, 180);

        /**
         * Let the trackable listeners we care about know where the phone is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */
        legosListener = (VuforiaTrackableDefaultListener) legosTarget.getListener();
        legosListener.setPhoneInformation(phoneLocation, parameters.cameraDirection);


        lastLocation = createMatrix(0, 0, 0, 0, 0, 0);
        beaconState = beacon.BEACON_NOT_VISIBLE;
    }


    @Override
    public void init_loop() {

    }


    @Override
    public void start() {
//        velocityVortexImages.activate();//Start tracking the data sets we care about.
//        robotLocationTransform = legosListener.getUpdatedRobotLocation();
    }


    @Override
    public void loop() {
        vuforiaLocalizer.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>()
        {
            @Override public void accept(Frame frame)
            {
                bitmap = vuforiaLocalizer.convertFrameToBitmap(frame);
            }
        }));


        beaconState = getBeaconState(bitmap);
    }


    @Override
    public void stop() {

    }

    // Creates a matrix for determining the locations and orientations of objects
    // Units are millimeters for x, y, and z, and degrees for u, v, and w
    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w) {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    String formatMatrix(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

    public beacon getBeaconState(Bitmap bm) {
        if (bm != null) {

            Mat crop = new Mat(bm.getHeight(), bm.getWidth(), CvType.CV_8UC3);
            Utils.bitmapToMat(bm, crop);

            Bitmap bmOut = Bitmap.createBitmap(crop.width(), crop.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(crop, bmOut);

            if (gamepad1.x && !buttonPressed) {
                buttonPressed = true;
                saveImage(bmOut);
            } else if (!gamepad1.x && buttonPressed) {
                buttonPressed = false;
            }
        }

        return beacon.BEACON_NOT_VISIBLE;
    }


    public void saveImage(Bitmap bitmap) {
        if (bitmap != null) {
            String nameOfFile = "imageCaptured";

            String state = Environment.getExternalStorageState();
            if (Environment.MEDIA_MOUNTED.equals(state)) {
                Log.i("Mike", "Able to write to storage");
            } else {
                Log.i("Mike", "Cannot write to storage");
            }


            File dir = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES);

            Log.i("Mike", "saveImage: " + dir.getPath());

            if (!dir.exists()) {
                Log.i("Mike", "Dir does not exist");
            } else {
                Log.i("Mike", "Dir Exists");

                File file = new File(dir, nameOfFile + ".jpg");

                try {
                    FileOutputStream fOut = new FileOutputStream(file);
                    bitmap.compress(Bitmap.CompressFormat.JPEG, 100, fOut);

                    fOut.flush();
                    fOut.close();

                } catch (FileNotFoundException e) {
                } catch (IOException e) {
                    Log.i("Mike", e.toString());
                }
            }
        }
    }

}