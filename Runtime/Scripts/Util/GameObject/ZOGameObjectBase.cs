using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using ZO.Util;

namespace ZO.Util {

    /// <summary>
    /// Abstract class that defines a lot of Zero Sim core game object behavior.
    ///  
    /// Update Rates:  Can setup update rate using the <c>ZOUpdateHzSynchronized()</c> and
    /// Startup: Use <c>ZOStart()</c> instead standard Unity <c>Start()</c> method.
    /// <c>ZOFixedUpdateHzSynchronized</c> methods.
    /// </summary>
    public abstract class ZOGameObjectBase : MonoBehaviour {

        public float _updateRateHz;
        public float UpdateRateHz {
            get => _updateRateHz;
            set => _updateRateHz = value;
        }
        public bool _debug;
        public bool IsDebug {
            get => _debug;
            set => _debug = value;
        }

        private float _nextUpdateTime = 0.0f;
        public float NextUpdateTime {
            get { return _nextUpdateTime; }
            set { _nextUpdateTime = value; }
        }
        private float _nextFixedUpdateTime = 0.0f;
        public float NextFixedUpdateTime {
            get { return _nextFixedUpdateTime; }
            set { _nextFixedUpdateTime = value; }
        }

        private static float _nextUpdateTimeOffset = 0.0f;

#if UNITY_EDITOR

        private ZOFrequencyCounter _updateHzCounter = new ZOFrequencyCounter(ZOFrequencyCounter.TimeSourceType.RENDER_UPDATE);
        private ZOFrequencyCounter _fixedUpdateHzCounter = new ZOFrequencyCounter(ZOFrequencyCounter.TimeSourceType.FIXED_UPDATE);

        [ZOReadOnly]
        [SerializeField]
        private float _currentUpdateHz;
        [ZOReadOnly]
        [SerializeField]
        private float _currentFixedUpdateHz;

        private bool _isFirstUpdate = true;

#endif

        // -------------------------------------------------------------------------------------------------------------------------
        // --------------- VIRTUAL METHODS FOR SENSOR IMPLEMENTATION ---------------------------------------------------------------
        // -------------------------------------------------------------------------------------------------------------------------

        /// <summary>
        /// Start function for Zero Sim Objects
        /// </summary>
        protected virtual void ZOStart() { }

        /// <summary>
        /// Awake function for Zero Sim Objects
        /// </summary>
        protected virtual void ZOAwake() {}

        /// <summary>
        /// Destruction function for Zero Sim Objects
        /// </summary>
        protected virtual void ZOOnDestroy() { }

        /// <summary>Update function for Zero Sim Objects. Will run inside this Monobehavior's Update function.</summary>
        protected virtual void ZOUpdate() { }
        
        /// <summary>FixedUpdate function for Zero Sim Objects. Will run inside this Monobehavior's Update function</summary>
        protected virtual void ZOFixedUpdate() { }

        /// <summary>HZ synched function that runs inside Monobehavior's Update function. It's guaranteed to use the set frequency HZ for this sensor.</summary>
        protected virtual void ZOUpdateHzSynchronized() { }
        
        /// <summary>HZ synched function that runs inside Monobehavior's FixedUpdate function. It's guaranteed to use the set frequency HZ for this sensor.</summary>
        protected virtual void ZOFixedUpdateHzSynchronized() { }

        // -------------------------------------------------------------------------------------------------------------------------
        /// <summary>
        /// Do not redefine this method in child classes, use ZOStart() instead.
        /// </summary>
        void Start() {
            ZOStart();

            /// Offset update time so we don't get big CPU spikes
            NextUpdateTime = _nextUpdateTimeOffset;
            NextFixedUpdateTime = _nextUpdateTimeOffset;
            _nextUpdateTimeOffset += 0.13f;
        }

        private void Awake() {
            ZOAwake();    
        }

        private void OnDestroy() {
            ZOOnDestroy();
        }

        /// <summary>
        /// Update is called once per frame
        /// IMPORTANT: do not redefine this method in child classes, use ZOUpdate() instead.
        /// </summary>
        void Update() {
            ZOUpdate();

            if (Time.time >= _nextUpdateTime) {
                _nextUpdateTime = Time.time + (1.0f / _updateRateHz);

                ZOUpdateHzSynchronized();

#if UNITY_EDITOR
                _currentUpdateHz = _updateHzCounter.Tick();
#endif
            }

        }

        /// <summary>
        /// IMPORTANT: do not redefine this method in child classes, use ZOFixedUpdate() instead.
        /// </summary>
        void FixedUpdate() {
            ZOFixedUpdate();

            if (Time.fixedTime >= _nextFixedUpdateTime) {
                _nextFixedUpdateTime = Time.fixedTime + (1.0f / _updateRateHz);

                ZOFixedUpdateHzSynchronized();

#if UNITY_EDITOR
                _currentFixedUpdateHz = _fixedUpdateHzCounter.Tick();
#endif
            }
        }

        /// <summary>Starts a coroutine that runs 'singleLoopLogic' using the HZ update rate.</summary>
        protected Coroutine StartHZEnforcedCoroutine(Action singleLoopLogic) {
            return StartCoroutine(HZEnforcedCoroutine(singleLoopLogic));
        }

        private IEnumerator HZEnforcedCoroutine(Action singleLoopLogic) {
            float nextUpdateTime = 0f;

            while (true) {
                // run single loop logic if it's time to do it
                if (Time.time >= nextUpdateTime) {
                    nextUpdateTime = Time.time + (1.0f / _updateRateHz);

                    singleLoopLogic();

                }

                yield return new WaitForEndOfFrame();
            }

        }
    }
}