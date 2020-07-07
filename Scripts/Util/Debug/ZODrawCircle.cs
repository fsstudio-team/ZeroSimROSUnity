using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ZO.Util {

    [ExecuteInEditMode]
    public class ZODrawCircle : MonoBehaviour {
        public float _radius = 1.0f;
        public float _lineWidth = 0.02f;
        public int _segments = 360;
        public Color _color = Color.green;

        private LineRenderer _lineRenderer = null;

        public Vector3 Position {
            get { return transform.position; }
            set { transform.position = value; }
        }

        public float Radius {
            get { return _radius; }
            set { _radius = value; }
        }

        // Start is called before the first frame update
        void Awake() {

            BuildCircle();
        }

        private void BuildCircle() {
            _lineRenderer = GetComponent<LineRenderer>();
            if (_lineRenderer == null) {
                _lineRenderer = gameObject.AddComponent<LineRenderer>();
                _lineRenderer.sharedMaterial = new Material(Shader.Find("Unlit/Color"));
                _lineRenderer.sharedMaterial.color = new Color(1,1,1,1);
            }

            _lineRenderer.startColor = _color;
            _lineRenderer.endColor = _color;
            
            _lineRenderer.useWorldSpace = false;
            _lineRenderer.positionCount = _segments + 1;

            _lineRenderer.startWidth = _lineWidth;
            _lineRenderer.endWidth = _lineWidth;

            for (int i = 0; i < _lineRenderer.positionCount; i++) {
                float rad = Mathf.Deg2Rad * (i * 360.0f / _segments);
                _lineRenderer.SetPosition(i, new Vector3(Mathf.Sin(rad) * _radius, 0, Mathf.Cos(rad) * _radius));
            }

        }

        private void OnValidate() {
            BuildCircle();
        }

    }
}