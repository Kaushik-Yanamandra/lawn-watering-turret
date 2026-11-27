"""Protocol tests for TurretLink against a fake Turret Node (rev B protocol, no hardware).

    python -m pytest tests          or      python tests/test_link.py
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from turret.hardware import NodeError, TurretLink, checksum, frame, parse  # noqa: E402


class FakeNode:
    """Mimics firmware rev B closely enough to exercise the driver. The node
    only replies; it never sends unsolicited lines."""

    def __init__(self, corrupt_replies=0, garble_commands=0, dose_polls=3, lose_dose_reply=False):
        self.out = bytearray()
        self.inbuf = b""
        self.valve = False
        self.frames = []
        self.corrupt_replies = corrupt_replies
        self.garble_commands = garble_commands
        self.dose_polls = dose_polls
        self.lose_dose_reply = lose_dose_reply
        self.dose = {"id": -1, "running": False, "polls": 0, "ml": 0.0, "starts": 0}
        self.uptime = 3600
        self.faults = 0

    # serial API used by TurretLink
    def write(self, data):
        self.inbuf += data
        while b"\n" in self.inbuf:
            line, self.inbuf = self.inbuf.split(b"\n", 1)
            if self.garble_commands:
                self.garble_commands -= 1
                self._send("ERR CHECKSUM")
                continue
            body = parse(line)
            if body is None:
                self._send("ERR CHECKSUM")
                continue
            self.frames.append(body)
            self._handle(body)

    def read(self, n):
        b = bytes(self.out[:n])
        del self.out[:n]
        return b

    def close(self):
        pass

    def _send(self, text):
        f = frame(text)
        if self.corrupt_replies:
            f = f.replace(b"*", b"#", 1)
            self.corrupt_replies -= 1
        self.out += f

    def _handle(self, t):
        tok = t.split()
        c = tok[0]
        d = self.dose
        if c == "PING":
            self._send("OK PONG 2.0.0")
        elif c == "MOVE":
            self._send("OK MOVE 120")
        elif c == "VALVE":
            self.valve = tok[1] == "1"
            self._send(f"OK VALVE {tok[1]}")
        elif c == "DOSE":
            did = int(tok[1])
            if did != d["id"]:                     # a repeated id never restarts the dose
                d.update(id=did, running=True, polls=0, ml=float(tok[2]))
                d["starts"] += 1
                self.valve = True
            if self.lose_dose_reply:
                self.lose_dose_reply = False
                return
            self._send(f"OK DOSE {did}")
        elif c == "DOSE?":
            if d["running"]:
                d["polls"] += 1
                if d["polls"] >= self.dose_polls:
                    d["running"] = False
                    self.valve = False
            state = "RUN" if d["running"] else "END"
            reason = "-" if d["running"] else "DONE"
            self._send(f"OK DOSEST {d['id']} {state} {d['ml']:.0f} 17100 {reason}")
        elif c == "STAT":
            self._send(f"OK STAT 12.5 -20.0 0 0.00 15230 24.6 23.8 2510 1800 3300 {self.faults} {self.uptime}")
        elif c == "CLR":
            self.faults = 0 if (len(tok) > 1 and tok[1] == "STUCK") else self.faults & 8
            self._send("OK CLR")
        elif c == "PARK":
            self._send("OK PARK")
        else:
            self._send("ERR CMD")


def test_checksum_roundtrip():
    assert checksum("PING") == ord("P") ^ ord("I") ^ ord("N") ^ ord("G")
    assert parse(frame("MOVE 1.0 2.0")) == "MOVE 1.0 2.0"
    assert parse(b"MOVE 1.0 2.0*00") is None
    assert parse("garbage") is None


def test_commands():
    node = FakeNode()
    link = TurretLink(ser=node)
    assert link.ping() == "OK PONG 2.0.0"
    link.move(30, -10, settle=0.0)
    assert "MOVE 30.0 -10.0" in node.frames
    link.valve(True, max_s=5)
    assert node.valve and "VALVE 1 5000" in node.frames
    link.valve(False)
    st = link.status()
    assert st.pan == 12.5 and st.psi == 24.6 and st.probe_mv == (2510, 1800, 3300) and st.faults == 0


def test_dose_completes_by_polling():
    node = FakeNode()
    link = TurretLink(ser=node)
    ml, ms, reason = link.dose(2.0, max_s=20)
    assert (ml, reason) == (2000.0, "DONE") and not node.valve


def test_lost_dose_reply_does_not_restart_dose():
    node = FakeNode(lose_dose_reply=True)
    link = TurretLink(ser=node)
    ml, ms, reason = link.dose(1.5, max_s=20)
    assert reason == "DONE" and node.dose["starts"] == 1
    assert sum(1 for f in node.frames if f.startswith("DOSE ")) == 2      # resent once, started once


def test_retry_after_corrupted_reply():
    node = FakeNode(corrupt_replies=1)
    link = TurretLink(ser=node)
    assert link.ping() == "OK PONG 2.0.0"
    assert node.frames.count("PING") == 2


def test_retry_after_node_reports_bad_checksum():
    node = FakeNode(garble_commands=1)
    link = TurretLink(ser=node)
    assert link.ping() == "OK PONG 2.0.0"


def test_reboot_detected():
    node = FakeNode()
    link = TurretLink(ser=node)
    link.status()
    node.uptime = 5
    try:
        link.status()
    except NodeError as e:
        assert "rebooted" in str(e)
    else:
        raise AssertionError("expected NodeError")


def test_clear_keeps_stuck():
    node = FakeNode()
    node.faults = 8 | 1
    link = TurretLink(ser=node)
    link.clear_faults()
    assert node.faults == 8
    link.clear_faults(stuck=True)
    assert node.faults == 0


def test_err_raises():
    link = TurretLink(ser=FakeNode())
    try:
        link.cmd("BOGUS")
    except NodeError as e:
        assert "ERR CMD" in str(e)
    else:
        raise AssertionError("expected NodeError")


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("pass", name)
