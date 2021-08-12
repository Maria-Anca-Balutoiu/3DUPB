using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MoveCamera1 : MonoBehaviour
{
    float speed;
    public GameObject child;

	void Start ()
    {
        speed = 5.0f;
	}
	
	void Update ()
    {
        /* Save first image */
        if (Input.GetKeyDown(KeyCode.Space))
        {
            ScreenCapture.CaptureScreenshot("image0.jpg");
        }

        /* Move camera */
        if (Input.GetMouseButton(1))
        {
            Vector3 position = child.transform.position;
            Quaternion rotation = child.transform.rotation;

            if (Input.GetKey(KeyCode.LeftControl))
            {
                float y = Input.GetAxis("Mouse X");
                float x = Input.GetAxis("Mouse Y");
                Vector3 rotateValue = new Vector3(x, y * -1, 0);
                transform.eulerAngles = transform.eulerAngles - rotateValue;
            }

            if (Input.GetKey(KeyCode.D))
            {
                transform.Translate(speed * Time.deltaTime * Vector3.right);
            }
            if (Input.GetKey(KeyCode.A))
            {
                transform.Translate(-speed * Time.deltaTime * Vector3.right);
            }
            if (Input.GetKey(KeyCode.S))
            {
                transform.Translate(-speed * Time.deltaTime * Vector3.forward);
            }
            if (Input.GetKey(KeyCode.W))
            {
                transform.Translate(speed * Time.deltaTime * Vector3.forward);
            }

            child.transform.position = position;
            child.transform.rotation = rotation;
        }
    }
}
