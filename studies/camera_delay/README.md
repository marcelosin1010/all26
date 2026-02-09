# Camera Delay

This project supports direct measurement of end-to-end camera delay, using a test rig.

## Test Rig

The test rig comprises a rotating disc, with a small Apriltag on it,
driven at a low constant speed by a motor.

There are four measurements of the disc angle:

* The real world.  This defines the "zero" of rotation, relative to the test board (e.g. "level").
* The motor.  The motor has an estimate of position, which we can largely ignore.
* One of the absolute encoders we commonly use, connected to the Rio.
* A camera running the usual Apriltag detection code.

The real-world standard is used to set all the measurements the same,
for zero speed and zero position.

The reason we're ignoring the motor measurement is that its timing isn't well documented.

## Position Sensor Delay

The delay of the position sensor itself is quoted as 0.1 ms:

https://look.ams-osram.com/m/287d7ad97d1ca22e/original/AS5048-DS000298.pdf

The delay due to the protocol is uniformly distributed between zero and one ms,
because the sensor output is repeats every ms, and the Rio provides the
most-recently-read value at reading time, with very little delay.

So the total position sensor delay is uniformly distributed between 0.1 and 1.1 ms.

I'm not sure how to *measure* this delay. I think we should just assume a
fixed 0.6 ms delay for the position sensor.

## Camera Delay

The camera pipeline works as follows:

1. The sensor is made active, usually for a very short time, currently 1.5 ms.
2. The sensor records a blurred average of the real world during that time.
3. The sensor is made inactive, keeping the captured pixels.
4. The pixels are streamed out of the sensor, row by row.
5. When the first pixel arrives at the Raspberry Pi's image signal processor, it records the system time.
6. Quite a bit of time passes, maybe 30 ms.
7. Finally the last pixel arrives, making a complete frame.
8. The frame is used by our analysis code, which takes awhile, maybe another 30 ms.
9. The total delay is computed (see below).
10. The NetworkTables timestamp is computed (see below).
11. The results are sent over NetworkTables.
12. The NetworkTables receiver adjusts the timestamp for clock offset.
13. Our code polls for NetworkTables input.
14. A small amount of work happens to decode the input.

The description above has no adjustable fudge factors, and yet we have felt the need
to add them: there is some source of delay not accounted for here.

### Total Delay

The delay internal to the camera/pi system is computed as follows:

1. The image timestamp is rewound by half the shutter open time.
2. The result is subtracted from the current system time, producing the *delay*.

### Network Tables Timestamp

The delay calculated above is subtracted from the Network Tables time (a different timebase
than the system time), to produce the Network Tables timestamp.

## Comparing Measurements

To compare measurements, we'll keep a timestamped history of all the measurements received.
We then log the history at a point further in the past than any of the delays:
one second ago.  The samples should be the same, so we also log the difference, which
should be zero.

## Taking Action

The difference will certainly not be zero, so what should we do?

Previously we added extra delay on the RoboRIO end, but I think that's the wrong
thing to do (among other reasons, it means that the simulator needs to duplicate
the "extra" delay).

It would be better to modify the sender, so that it tells the truth.

If the *variance* of the difference turns out to be large, then we'll need to look more closely.

## Appendix: Camera Delay Background

Sources of camera delay are discussed extensively elsewhere:

* https://docs.google.com/spreadsheets/d/1eJXAVCPSDcnFvezaWv0vLu5IyZi7KkjZglR7nizP6fc
* https://docs.google.com/document/d/1JPJC3cn6eorBrsogOWRN7W_bMo1DgAfIKKjA5wP1av4
* https://docs.google.com/document/d/1Wmumdv1L7AmdqdHs8FIjNInB5be5Tu6RLEa4f_on6Pk

The goal of this work is not to understand the sources of delay,
we just want to measure it empirically.
