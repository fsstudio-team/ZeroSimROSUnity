using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ZO.Util {

    /// <summary>
    /// This class uses a trigger and a layermask to detect collision with other objects.
    /// Can be polled to check if it's currently colliding with something.
    /// Not intended to be used independently, a sensor class should be
    /// composed with ZOContactDetector components in order to implement their functionality. 
    /// </summary>
    public class ZOContactDetector : MonoBehaviour {

        [SerializeField]
        [Tooltip("The trigger collider that will be used for the contact detector. If none is set, it will attempt to get a collider from this GameObject")]
        Collider _contactTrigger;

        LayerMask _collisionLayers; // meant to be set externally using property

        public LayerMask CollisionLayers { set { _collisionLayers = value; } }

        /// <summary>
        /// Boolean indicating if this contact switch is currently detecting a collision.
        /// </summary>
        private bool _detectingContact;

        /// <summary>
        /// Set of colliders currently colliding with the trigger, with no repeated elements.
        /// We need this to avoid inconsistent states for '_detectingContact'. 
        /// Note that we store Colliders and not GameObjects, this is in case a GameObject has multiple colliders in it.
        /// We assume a list will be ok for performance since we don't expect more than a couple colliders at a time.
        /// </summary>
        private List<Collider> _objectsColliding;

        /// <summary>
        /// Use this property to poll the current state of this contact switch.
        /// </summary>
        public bool IsDetectingContact { get { return _detectingContact; } }

        void Awake(){

            // if the user hasn't set a collider, attempt to get one from this GameObject
            if(_contactTrigger == null) {
                _contactTrigger = GetComponent<Collider>();

                if(_contactTrigger == null) throw new System.Exception($"ZOContactSwitch '{name}' is missing a trigger collider!");
            }
            
            if(!_contactTrigger.isTrigger) throw new System.Exception($"ZOContactSwitch '{name}' has a collider but it's not configured as a trigger!");
        }

        // Start is called before the first frame update
        void Start() {
            _objectsColliding = new List<Collider>();
        }


        private void OnTriggerEnter(Collider other) {
            
            // check if the 'other' collider belongs to the collision layermask
            if((_collisionLayers & (1 << other.gameObject.layer)) > 0){
                _detectingContact = true;

                // add this collider to the list in case it's not there already
                // it should never happen that we attempt to add a repeated object, but making sure does no harm
                if(!_objectsColliding.Contains(other))
                    _objectsColliding.Add(other);
            }

        }

        private void OnTriggerExit(Collider other) {
            
            // check if the 'other' collider belongs to the collision layermask
            if((_collisionLayers & (1 << other.gameObject.layer)) > 0){
                _objectsColliding.Remove(other);

                // if all the colliders that have entered, have already exited, then this sensor is no longer detecting a contact
                if(_objectsColliding.Count == 0){
                    _detectingContact = false;
                }

            }

        }

    }

    

}