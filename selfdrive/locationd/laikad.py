#!/usr/bin/env python3
from cereal import log, messaging
from laika import AstroDog
from laika.helpers import ConstellationId
from laika.ephemeris import convert_ublox_ephem
from laika.raw_gnss import GNSSMeasurement, calc_pos_fix, correct_measurements, process_measurements, read_raw_ublox


def process_ublox_msg(ublox_msg, dog: AstroDog, ublox_mono_time: int, correct=False):
  if ublox_msg.which == 'measurementReport':
    report = ublox_msg.measurementReport
    if len(report.measurements) == 0:
      return None
    new_meas = read_raw_ublox(report)
    measurements = process_measurements(new_meas, dog)
    if len(measurements) == 0:
      return None

    if correct:
      # pos fix needs more than 5 processed_measurements
      pos_fix = calc_pos_fix(measurements)
      if len(pos_fix) == 0:
        return None
      measurements = correct_measurements(measurements, pos_fix[0][:3], dog)
      if len(measurements) == 0:
        return None
    # pos or vel fixes can be an empty list if not enough correct measurements are available
    meas_msgs = [create_measurement_msg(m) for m in measurements]

    dat = messaging.new_message('gnssMeasurements')
    dat.gnssMeasurements = {
      "ubloxMonoTime": ublox_mono_time,
      "correctedMeasurements": meas_msgs
    }
    return dat
  elif ublox_msg.which == 'ephemeris':
    ephem = convert_ublox_ephem(ublox_msg.ephemeris)
    dog.add_ephem(ephem, dog.orbits)
  # elif ublox_msg.which == 'ionoData':
  # todo add this. Needed to correct messages offline. First fix ublox_msg.cc to sent them.
  #  This is needed to correct the measurements


def create_measurement_msg(meas: GNSSMeasurement):
  c = log.GnssMeasurements.CorrectedMeasurement.new_message()
  c.constellationId = meas.constellation_id.value
  c.svId = int(meas.prn[1:])
  if len(meas.observables_final) > 0:
    observables = meas.observables_final
  else:
    observables = meas.observables

  c.glonassFrequency = meas.glonass_freq if meas.constellation_id == ConstellationId.GLONASS else 0
  c.pseudorange = float(observables['C1C'])
  c.pseudorangeStd = float(meas.observables_std['C1C'])
  c.pseudorangeRate = float(observables['D1C'])
  c.pseudorangeRateStd = float(meas.observables_std['D1C'])
  c.satPos = meas.sat_pos_final.tolist()
  c.satVel = meas.sat_vel.tolist()
  return c


def main():
  dog = AstroDog(use_internet=False)
  sm = messaging.SubMaster(['ubloxGnss'])
  pm = messaging.PubMaster(['gnssMeasurements'])

  while True:
    sm.update()

    if sm.updated['ubloxGnss']:
      ublox_msg = sm['ubloxGnss']
      msg = process_ublox_msg(ublox_msg, dog, sm.logMonoTime['ubloxGnss'])
      if msg is None:
        msg = messaging.new_message('gnssMeasurements')
      pm.send('gnssMeasurements', msg)


if __name__ == "__main__":
  main()
