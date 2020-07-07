using UnityEngine;

namespace ZO.Util {

    /// <summary>
    /// Use this class to calculate frequency of some process by creating an instance and calling Tick() for counting.
    /// </summary>
    public class ZOFrequencyCounter{

        private int _tickCount;
        private float _startLapseTimeStamp = 0.0f;
        private float _timeLapse = 1.0f; // default to 1 second for calculating Hz
        private float _currentHz;
        private TimeSourceType _timeSourceType;

        /// <summary>
        /// used to specify if we need to use Time.time or Time.fixedTime
        /// </summary>
        public enum TimeSourceType { RENDER_UPDATE, FIXED_UPDATE}

        public ZOFrequencyCounter(TimeSourceType timeSourceType = TimeSourceType.FIXED_UPDATE){
            this._timeSourceType = timeSourceType;
        }

        public float Tick(){
            UpdateHZValue();
            _tickCount++;

            return _currentHz;
        }

        public float GetHZ(){
            UpdateHZValue();

            return _currentHz;
        }

        private void UpdateHZValue(){

            float currentTimeStamp = _timeSourceType == TimeSourceType.RENDER_UPDATE ? Time.time : Time.fixedTime;
            float elapsedTime = currentTimeStamp - _startLapseTimeStamp;
            
            if(elapsedTime >= _timeLapse){
                // Current time lapse has ended. Calculate Hz and reset
                _currentHz = _tickCount / elapsedTime;

                _startLapseTimeStamp = currentTimeStamp;

                _tickCount = 0;

            }
        }
    }

}