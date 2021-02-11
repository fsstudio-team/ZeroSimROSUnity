using System;
using System.Threading;
using System.Diagnostics;

namespace ZO.Util {
    public sealed class ZOClock : IDisposable {
        private readonly long _maxIdleTime = TimeSpan.FromSeconds(10).Ticks;
        private const long TicksMultiplier = 1000 * TimeSpan.TicksPerMillisecond;

        private readonly ThreadLocal<DateTime> _startTime =
            new ThreadLocal<DateTime>(() => DateTime.UtcNow, false);

        private readonly ThreadLocal<double> _startTimestamp =
            new ThreadLocal<double>(() => Stopwatch.GetTimestamp(), false);

        public DateTime UtcNow {
            get {
                double endTimestamp = Stopwatch.GetTimestamp();

                var durationInTicks = (endTimestamp - _startTimestamp.Value) / Stopwatch.Frequency * TicksMultiplier;
                if (durationInTicks >= _maxIdleTime) {
                    _startTimestamp.Value = Stopwatch.GetTimestamp();
                    _startTime.Value = DateTime.UtcNow;
                    return _startTime.Value;
                }

                return _startTime.Value.AddTicks((long)durationInTicks);
            }
        }

        public void Dispose() {
            _startTime.Dispose();
            _startTimestamp.Dispose();
        }
    }

}