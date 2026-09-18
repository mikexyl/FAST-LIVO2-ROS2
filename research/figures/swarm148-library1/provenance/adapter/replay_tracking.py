"""Exact scan accounting for replay; a later ACK never acknowledges a gap."""
class ScanTracker:
    def __init__(self):
        self.sent = set()
        self.acked = set()
        self.last_sent = None

    def publish(self, stamp):
        if self.last_sent is not None and stamp <= self.last_sent:
            raise ValueError('Scan timestamps must increase strictly per robot')
        self.sent.add(stamp)
        self.last_sent = stamp

    def acknowledge(self, stamp):
        if stamp not in self.sent:
            raise ValueError(f'ACK for unpublished scan {stamp}')
        if stamp in self.acked:
            raise ValueError(f'Duplicate scan ACK {stamp}')
        self.acked.add(stamp)

    @property
    def pending(self):
        return self.sent - self.acked
