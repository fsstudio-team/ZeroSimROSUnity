using UnityEngine;

namespace ZO.Util
{
    public class ZODragRigidBody : MonoBehaviour {
    public float _appliedForce = 500;

    Rigidbody _selectedRigidbody;
    Camera _targetCamera;
    Vector3 _originalScreenTargetPosition;
    Vector3 _originalRigidbodyPos;
    float _selectionDistance;

    // Start is called before the first frame update
    void Start() {
        _targetCamera = GetComponent<Camera>();
    }

    void Update() {
        if (!_targetCamera)
            return;

        if (Input.GetMouseButtonDown(0) && (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift))) {
            //Check if we are hovering over Rigidbody, if so, select it
            _selectedRigidbody = GetRigidbodyFromMouseClick();
        }
        if (Input.GetMouseButtonUp(0) && _selectedRigidbody) {
            //Release selected Rigidbody if there any
            _selectedRigidbody = null;
        }
    }

    void FixedUpdate() {
        if (_selectedRigidbody) {
            Vector3 mousePositionOffset = _targetCamera.ScreenToWorldPoint(new Vector3(Input.mousePosition.x, Input.mousePosition.y, _selectionDistance)) - _originalScreenTargetPosition;
            _selectedRigidbody.velocity = (_originalRigidbodyPos + mousePositionOffset - _selectedRigidbody.transform.position) * _appliedForce * Time.deltaTime;
        }
    }

    Rigidbody GetRigidbodyFromMouseClick() {
        RaycastHit hitInfo = new RaycastHit();
        Ray ray = _targetCamera.ScreenPointToRay(Input.mousePosition);
        bool hit = UnityEngine.Physics.Raycast(ray, out hitInfo);
        if (hit) {
            if (hitInfo.collider.attachedRigidbody) {
                _selectionDistance = Vector3.Distance(ray.origin, hitInfo.point);
                _originalScreenTargetPosition = _targetCamera.ScreenToWorldPoint(new Vector3(Input.mousePosition.x, Input.mousePosition.y, _selectionDistance));
                _originalRigidbodyPos = hitInfo.collider.transform.position;
                return hitInfo.collider.attachedRigidbody;
            }
        }

        return null;
    }
}
}
