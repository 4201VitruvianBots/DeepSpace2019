# Vitruvian Logger

The main purpose for writing this logging system was to make a logger that is smart enough to
organize where to put logs base on what state the robot is in when testing at home, as well as
what event you are in and what match you are playing.

The second purpose was to make this as 'simple' as possible for future reuse, though it does
utilize some harder programming concepts to do this (e.g. lambda expressions).

The main structure of this Logger is as follows:
LOGGER is the main overall controller of the logging system that tells the individual LOGs when
to start/stop recording data and where to put it. LOGGER contains a list of LOGs within it.

LOG is an individual log. A robot will usually have multiple logs for each subsystem to better
organize logged data for later use. Each LOG can be set up with a different update rate for
flexibility (e.g. more important things can be logged more often) The items each LOG records are
LOGFIELDS.

A LOGFIELD is a value you want to log. It uses a name/value pair system, but the value is an
interface to determine what function to call to log data from.


See the example drivetrain log I've put under frc.robot.subsystem.Control for a general example
of how to set up a logger. All of the logs are initialized in Robot.java under disabledInit(),
teleopInit(), and autonomousInit().