using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace ZO.Util {

    /// <summary>
    /// Abstract class that defines a lot of Zero Sim core game object behavior.
    ///  
    /// Update Rates:  Can setup update rate using the <c>ZOUpdateHzSynchronized()</c> and
    /// Startup: Use <c>ZOStart()</c> instead standard Unity <c>Start()</c> method.
    /// <c>ZOFixedUpdateHzSynchronized</c> methods.
    /// </summary>
    public abstract class ZOGameObjectBase : MonoBehaviour {


        [Header("ZeroSim GameObject Base")]
        public float _updateRateHz;

        /// <summary>
        /// The update rate in Hz
        /// </summary>
        /// <value></value>
        public float UpdateRateHz {
            get => _updateRateHz;
            set => _updateRateHz = value;
        }


        /// <summary>
        /// The delta update time in seconds.
        /// </summary>
        /// <value></value>
        public float UpdateTimeSeconds {
            get {
                if (UpdateRateHz == 0.0f) {
                    return 0.0f; // avoid divide by zero
                }
                
                return 1.0f / UpdateRateHz;
            }
        }


        public bool _debug;

        /// <summary>
        /// Debug flag dependent on implementation. May not do anything.
        /// </summary>
        /// <value></value>
        public bool IsDebug {
            get => _debug;
            set => _debug = value;
        }

        private float _nextUpdateTime = 0.0f;
        public float NextUpdateTime {
            get { return _nextUpdateTime; }
            set { _nextUpdateTime = value; }
        }
        private double _nextFixedUpdateTime = 0.0f;
        public double NextFixedUpdateTime {
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
        protected virtual void ZOAwake() { }

        /// <summary>
        /// OnEnable function for Zero Sim Objects
        /// </summary>
        protected virtual void ZOOnEnable() { }


        /// <summary>
        /// Reset function for Zero Sim Objects
        /// </summary>
        protected virtual void ZOReset() { }

        /// <summary>
        /// OnGUI is called for rendering and handling GUI events.
        /// OnGUI is the only function that can implement the "Immediate Mode" GUI (IMGUI) system for rendering and 
        /// handling GUI events. Your OnGUI implementation might be called several times per frame (one call per 
        /// event). For more information on GUI events see the Event reference. If the MonoBehaviour's enabled 
        /// property is set to false, OnGUI() will not be called.
        /// 
        /// If Debug property is set to false `ZOOnGUI` will not be called.
        /// </summary>
        protected virtual void ZOOnGUI() { }


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
        private void Start() {
            ZOStart();

            /// Offset update time so we don't get big CPU spikes
            NextUpdateTime = _nextUpdateTimeOffset;
            NextFixedUpdateTime = _nextUpdateTimeOffset;
            _nextUpdateTimeOffset += 0.13f;
        }

        private void OnEnable() {
            ZOOnEnable();
        }

        private void Reset() {
            ZOReset();
        }

        private void Awake() {
            ZOAwake();
        }

        private void OnDestroy() {
            ZOOnDestroy();
        }


        /// <summary>
        /// OnGUI is called for rendering and handling GUI events.
        /// OnGUI is the only function that can implement the "Immediate Mode" GUI (IMGUI) system for rendering and 
        /// handling GUI events. Your OnGUI implementation might be called several times per frame (one call per 
        /// event). For more information on GUI events see the Event reference. If the MonoBehaviour's enabled 
        /// property is set to false, OnGUI() will not be called.
        /// </summary>
        private void OnGUI() {
            if (IsDebug == true) {
                ZOOnGUI();
            }

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

            if (Time.fixedUnscaledTimeAsDouble >= _nextFixedUpdateTime) {
                _nextFixedUpdateTime = Time.fixedUnscaledTimeAsDouble + (1.0 / _updateRateHz);

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