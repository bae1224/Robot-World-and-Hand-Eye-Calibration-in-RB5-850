"""Freshness of driver responses, independent of the controller's timer field.

Linux CLOCK_MONOTONIC is shared by the driver and recorder on the same boot.
This proves recent response receipt, not controller-internal sensor sample age.
"""
import math

MESSAGE_TYPE = 'rbpodo_msgs/msg/MeasuredFeedback'
CLOCK_SOURCE = 'driver_receive_monotonic'


def validate_feedback(reading, previous, now_ns, max_age_s=.2, boot_id=None):
    sequence = reading.get('feedback_sequence')
    stamp = reading.get('feedback_received_monotonic_ns')
    stream = reading.get('feedback_stream_id')
    if (type(sequence) is not int or sequence <= 0 or type(stamp) is not int or stamp <= 0
            or not isinstance(stream, str) or ':' not in stream):
        raise ValueError('Missing or invalid driver feedback freshness metadata')
    if boot_id is not None and stream.split(':', 1)[0] != boot_id:
        raise ValueError('Feedback driver and recorder must run on the same Linux host/boot')
    if (not math.isfinite(max_age_s) or max_age_s <= 0 or not math.isfinite(now_ns)
            or not 0 <= now_ns - stamp <= max_age_s * 1e9):
        raise ValueError('Stale or future driver feedback receive timestamp')
    if previous is not None:
        if stream != previous['feedback_stream_id']:
            raise ValueError('Driver stream restarted; reconnect and start a new recording')
        if (sequence <= previous['feedback_sequence']
                or stamp <= previous['feedback_received_monotonic_ns']):
            raise ValueError('Stale or reordered driver feedback')
