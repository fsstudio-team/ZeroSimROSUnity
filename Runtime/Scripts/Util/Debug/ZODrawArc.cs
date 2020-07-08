using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ZO.Util {

    [ExecuteInEditMode]
    public class ZODrawArc : MonoBehaviour {
        public float _startAngle = 0.0f;
        public float _endAngle = 45.0f;
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

        public float StartAngle {
            get { return _startAngle; }
            set { _startAngle = value; }
        }

        public float EndAngle {
            get { return _endAngle; }
            set { _endAngle = value; }
        }

        public Color Color {
            get { return _color; }
            set { _color = value; }
        }

        public float LineWidth {
            get { return _lineWidth; }
            set { _lineWidth = value; }
        }

        // Start is called before the first frame update
        void Awake() {

            BuildArc();
        }

        public void BuildArc() {
            _lineRenderer = GetComponent<LineRenderer>();
            if (_lineRenderer == null) {
                _lineRenderer = gameObject.AddComponent<LineRenderer>();
                _lineRenderer.sharedMaterial = new Material(Shader.Find("Unlit/Color"));
            }

            _lineRenderer.startColor = _color;
            _lineRenderer.endColor = _color;
            _lineRenderer.material.color = _color;
            _lineRenderer.useWorldSpace = false;
            _lineRenderer.positionCount = _segments + 1;

            _lineRenderer.startWidth = _lineWidth;
            _lineRenderer.endWidth = _lineWidth;

            float angle = _startAngle;
            float arcLength = _endAngle - _startAngle;

            for (int i = 0; i < _lineRenderer.positionCount; i++) {
                float rad = Mathf.Deg2Rad * angle;
                _lineRenderer.SetPosition(i, new Vector3( (Mathf.Sin(rad) * _radius), transform.position.y, (Mathf.Cos(rad) * _radius)));

                angle += (arcLength / _segments);
            }

        }

        private void OnValidate() {
            BuildArc();
        }

    }
}