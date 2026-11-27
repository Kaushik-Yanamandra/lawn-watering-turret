"""Sunrise and sunset from latitude/longitude (NOAA solar-position equations).

Used to start the dawn session at first usable light: the check-shot
camera needs daylight to see the splash, and in Houston 05:30 is dark on
every day of the year (sunrise runs from about 06:20 CDT in June to about
07:45 CDT in early March).

Accuracy is about one minute, which is plenty here. No network needed.
"""
from __future__ import annotations

import math
from datetime import date, datetime, timedelta, timezone


def _sun_event(d: date, lat: float, lon: float, rising: bool, zenith_deg: float = 90.833) -> datetime | None:
    """UTC time of sunrise (rising=True) or sunset on date d.

    zenith 90.833 deg = official sunrise (refraction + solar disc);
    96 deg = civil twilight. Returns None in polar day/night.
    """
    n = d.timetuple().tm_yday
    gamma = 2 * math.pi / 365 * (n - 1 + 0.5)          # fractional year at local noon, close enough
    eqtime = 229.18 * (0.000075 + 0.001868 * math.cos(gamma) - 0.032077 * math.sin(gamma)
                       - 0.014615 * math.cos(2 * gamma) - 0.040849 * math.sin(2 * gamma))
    decl = (0.006918 - 0.399912 * math.cos(gamma) + 0.070257 * math.sin(gamma)
            - 0.006758 * math.cos(2 * gamma) + 0.000907 * math.sin(2 * gamma)
            - 0.002697 * math.cos(3 * gamma) + 0.00148 * math.sin(3 * gamma))
    phi = math.radians(lat)
    cos_ha = (math.cos(math.radians(zenith_deg)) / (math.cos(phi) * math.cos(decl))
              - math.tan(phi) * math.tan(decl))
    if not -1.0 <= cos_ha <= 1.0:
        return None
    ha = math.degrees(math.acos(cos_ha))
    if not rising:
        ha = -ha
    minutes = 720 - 4 * (lon + ha) - eqtime              # minutes after 00:00 UTC
    return datetime(d.year, d.month, d.day, tzinfo=timezone.utc) + timedelta(minutes=minutes)


def sunrise_utc(d: date, lat: float, lon: float) -> datetime | None:
    return _sun_event(d, lat, lon, rising=True)


def sunset_utc(d: date, lat: float, lon: float) -> datetime | None:
    return _sun_event(d, lat, lon, rising=False)


def watering_start_utc(d: date, lat: float, lon: float, after_sunrise_min: float = 10.0) -> datetime:
    """When the dawn session may start: a few minutes after sunrise, once the
    camera has enough light to see splashes and before the day's wind rises."""
    sr = sunrise_utc(d, lat, lon)
    if sr is None:
        raise ValueError("no sunrise at this latitude on this date")
    return sr + timedelta(minutes=after_sunrise_min)
