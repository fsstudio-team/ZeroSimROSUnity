using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ZOROSTransformUnitTest : MonoBehaviour {
    public Transform _singleMoveInCircle;
    public Transform _singleRotateAroundAxis;
    public float _circleRadius = 2.0f;
    public float _circleSpeed = 5.0f;
    private float _circleAngle = 0.0f;

    // Start is called before the first frame update
    void Start() {

    }

    // Update is called once per frame
    void Update() {
        _circleAngle += _circleSpeed * Time.deltaTime;

        // move object in circle
        var offset = new Vector3(Mathf.Sin(_circleAngle), 0, Mathf.Cos(_circleAngle)) * _circleRadius;
        _singleMoveInCircle.position = offset;

        // rotate object around it's own up pivot
        _singleRotateAroundAxis.rotation = Quaternion.Euler(0,_circleAngle, 0);
    }
}
