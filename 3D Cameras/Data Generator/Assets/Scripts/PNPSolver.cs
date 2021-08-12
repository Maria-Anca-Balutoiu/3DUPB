using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Runtime.InteropServices;

public class PNPSolver : MonoBehaviour {
    public Camera unityCamera;
    public Camera opencvCamera;

    /* Structures needed for OpenCV compatibilty */
    struct Vec2
    {
        public float x;
        public float y;
    };

    struct Vec3
    {
        public float x;
        public float y;
        public float z;
    };

    struct Vec4
    {
        public float x;
        public float y;
        public float z;
        public float w;
    };

    struct Mat
    {
        public float fx;
        public float fy;
        public float px;
        public float py;
        public float width;
        public float height;
    };

    /*OpenCV PNP function */
    [DllImport("Computer Vision Plugin", EntryPoint = "PNP")]
    private static extern void PNP(Vec3 position1, Vec4 rotation1, Mat cameraIntrinsics, Vec3[] points3DUnity, Vec2[] points2DUnity, Vec3[] position2, Vec4[] rotation2);
    void Start () {
        opencvCamera.enabled = false;
        float f;
        float ax, ay, sizeX, sizeY;
        float x0, y0, shiftX, shiftY;
        int width, height;

        ax = 488.7f;
        ay = 492.0f;
        x0 = 238.5f;
        y0 = 322.5f;
        width = 480;
        height = 640;
        f = 30.0f;

        sizeX = f * width / ax;
        sizeY = f * height / ay;

        shiftX = -(x0 - width / 2.0f) / width;
        shiftY = (y0 - height / 2.0f) / height;

        /*Camera intrinsic parameters */
        Camera.main.usePhysicalProperties = true;
        Camera.main.sensorSize = new Vector2(sizeX, sizeY);
        Camera.main.focalLength = f;
        Camera.main.lensShift = new Vector2(shiftX, shiftY);

        unityCamera.usePhysicalProperties = true;
        unityCamera.sensorSize = new Vector2(sizeX, sizeY);
        unityCamera.focalLength = f;
        unityCamera.lensShift = new Vector2(shiftX, shiftY);

        opencvCamera.usePhysicalProperties = true;
        opencvCamera.sensorSize = new Vector2(sizeX, sizeY);
        opencvCamera.focalLength = f;
        opencvCamera.lensShift = new Vector2(shiftX, shiftY);
    }

	void Update () {
        /* Switch between Unity and OpenCV cameras */
        if (Input.GetKeyDown(KeyCode.C))
        {
            if (unityCamera.enabled == true)
            {
                unityCamera.enabled = false;
                opencvCamera.enabled = true;
            }
            else
            {
                unityCamera.enabled = true;
                opencvCamera.enabled = false;
            }
        }

        /* Run PNP */
        if (Input.GetKeyDown(KeyCode.Space))
        {
            GameObject[] featurePoints;
            Vec3[] coordinates3D = new Vec3[8];
            Vec2[] coordinates2D = new Vec2[8];
            int i = 0;

            /* Log feature points of the 8 cubes */
            featurePoints = GameObject.FindGameObjectsWithTag("FeaturePoint");

            foreach (GameObject fp in featurePoints)
            {
                /* The 3D coordinates of each feature point */
                Vec3 pointPosition = new Vec3();
                pointPosition.x = fp.transform.position.x;
                pointPosition.y = fp.transform.position.y;
                pointPosition.z = fp.transform.position.z;
                coordinates3D[i] = pointPosition;

                /* The 2D coordinates on camera screen of each feature point */
                Vec2 position2D = new Vec2();
                position2D.x = unityCamera.WorldToScreenPoint(fp.transform.position).x;
                position2D.y = unityCamera.WorldToScreenPoint(fp.transform.position).y;
                coordinates2D[i] = position2D;

                i += 1;
            }

            /* Position of the known camera */
            Vec3 position = new Vec3();
            position.x = Camera.main.transform.position.x;
            position.y = Camera.main.transform.position.y;
            position.z = Camera.main.transform.position.z;
            
            /* Rotation of the known camera */
            Vec4 rotation = new Vec4();
            rotation.x = Camera.main.transform.rotation.x;
            rotation.y = Camera.main.transform.rotation.y;
            rotation.z = Camera.main.transform.rotation.z;
            rotation.w = Camera.main.transform.rotation.w;

            /* Camera intrinsic parameters */
            Mat cameraIntrinsics = new Mat();
            cameraIntrinsics.fx = 488.7f;
            cameraIntrinsics.fy = 492.0f;
            cameraIntrinsics.px = 238.5f;
            cameraIntrinsics.py = 322.5f;
            cameraIntrinsics.width = 480.0f;
            cameraIntrinsics.height = 640.0f;

            /* PNP OpenCV plugin call */
            Vec3[] resultPosition = new Vec3[1];
            Vec4[] resultRotation = new Vec4[1];
            PNP(position, rotation, cameraIntrinsics, coordinates3D, coordinates2D, resultPosition, resultRotation);

            /* Unity camera values vs PNP results */
            Debug.Log(string.Format("Position Local Unity {0} {1} {2} - Position OpenCV {3} {4} {5}",
                                    unityCamera.transform.localPosition.x, unityCamera.transform.localPosition.y, unityCamera.transform.localPosition.z,
                                    resultPosition[0].x, resultPosition[0].y, resultPosition[0].z));
            Debug.Log(string.Format("Rotation Local Unity {0} {1} {2} {3} - Rotation OpenCV {4} {5} {6} {7}",
                                    unityCamera.transform.localRotation.x, unityCamera.transform.localRotation.y, unityCamera.transform.localRotation.z, unityCamera.transform.localRotation.w,
                                    resultRotation[0].x, resultRotation[0].y, resultRotation[0].z, resultRotation[0].w));

            /* Assign the results to the OpenCV camera */
            opencvCamera.transform.localPosition = new Vector3(resultPosition[0].x, resultPosition[0].y, resultPosition[0].z);
            opencvCamera.transform.localRotation = new Quaternion(resultRotation[0].x, resultRotation[0].y, resultRotation[0].z, resultRotation[0].w);
        }
    }
}