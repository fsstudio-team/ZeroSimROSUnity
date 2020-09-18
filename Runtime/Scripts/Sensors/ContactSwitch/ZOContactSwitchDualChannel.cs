using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading.Tasks;
using System;
using ZO.Util;


namespace ZO.Sensors {

    /// <summary>
    /// This class implements a dual channel contact switch sensor.
    /// </summary>
    public class ZOContactSwitchDualChannel : ZOGameObjectBase {
        public string _contactSwitchId = "Not Set";

        [SerializeField]
        ZOContactDetector channel1;

        [SerializeField]
        ZOContactDetector channel2;

        [SerializeField]
        LayerMask _collisionLayers;

        public Func<ZOContactSwitchDualChannel, string, bool, bool, Task> OnPublishDelegate { private get; set; }

        // Start is called before the first frame update
        void Start() {
            channel1.CollisionLayers = _collisionLayers;
            channel2.CollisionLayers = _collisionLayers;
        }

        // ZOUpdateHZ is synchronized with the HZ set for this sensor. Runs inside Monobehavior's Update() method.
        protected override void ZOFixedUpdateHzSynchronized() {

            bool channel1HasContact = channel1.IsDetectingContact;
            bool channel2HasContact = channel2.IsDetectingContact;

            if (OnPublishDelegate != null) {
                Task publishTask = OnPublishDelegate(this, _contactSwitchId, channel1HasContact, channel2HasContact);
            }

        }

        protected override void ZOUpdateHzSynchronized() {
        }

        protected override void ZOUpdate() {
        }

        protected override void ZOFixedUpdate() {
        }
    }

}