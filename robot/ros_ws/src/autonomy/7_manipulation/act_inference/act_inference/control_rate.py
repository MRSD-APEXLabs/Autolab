"""Deadline-based control pacing without bursts of catch-up servo commands."""


class ControlRate:
    def __init__(self, period, clock, wait):
        self.period, self.clock, self.wait_fn = period, clock, wait
        self.deadline = clock() + period

    def wait(self):
        now = self.clock()
        if now > self.deadline:
            # Skip missed ticks; never make a large motion step or send a burst to catch up.
            self.deadline = now + self.period
        stopped = self.wait_fn(max(0., self.deadline - now))
        self.deadline += self.period
        return stopped
