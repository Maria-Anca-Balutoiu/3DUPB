using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MoveCamera2 : MonoBehaviour
{
    float speed;

	void Start ()
    {
        speed = 5.0f;
    }
	
	void Update ()
    {
        /* Save first image */
        if (Input.GetKeyDown(KeyCode.Space))
        {
            ScreenCapture.CaptureScreenshot("image1.jpg");
        }

        /* Move camera */
        if (Input.GetMouseButton(1))
        {
            float y = Input.GetAxis("Mouse X");
            float x = Input.GetAxis("Mouse Y");
            Vector3 rotateValue = new Vector3(x, y * -1, 0);
            transform.eulerAngles = transform.eulerAngles - rotateValue;

            if (Input.GetKey(KeyCode.RightArrow))
            {
                transform.Translate(speed * Time.deltaTime * Vector3.right);
            }
            if (Input.GetKey(KeyCode.LeftArrow))
            {
                transform.Translate(-speed * Time.deltaTime * Vector3.right);
            }
            if (Input.GetKey(KeyCode.DownArrow))
            {
                transform.Translate(-speed * Time.deltaTime * Vector3.forward);
            }
            if (Input.GetKey(KeyCode.UpArrow))
            {
                transform.Translate(speed * Time.deltaTime * Vector3.forward);
            }
        }
    }
}
