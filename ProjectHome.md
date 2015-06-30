**-- The work emulates ten Android sensors in the virtualized Android-x86 remotely in real-time. The exact same sensor-based applications working for a real device can be used on the emulator with this work. Absolutely no change needed. --**

**The work was accomplished by me(Raghavan) as part of my Master's thesis in Computer Science at Columbia University, NYC in Fall 2013. The work also involved following people in the respective sense.**

  * Sole developer and main author: Raghavan Santhanam(Me), Columbia University, NYC
  * Technical point of contact for me to discuss ideas during the project: Songchun Fan, Duke University, NC - http://www.cs.duke.edu/~schfan/
  * My thesis advisor: Prof. Jason Nieh, Columbia University, NYC - http://www.cs.columbia.edu/~nieh/
  * In association with: Prof. Li Erran Li, Columbia University, NYC - http://www.bell-labs.com/user/erranlli/

**The complete details of this work is available in my thesis document as a technical report at my university's website - http://academiccommons.columbia.edu/catalog/ac:171022**

**Demo**

**MS in CS Thesis - Accelerometer Sensor Emulation - Android x86 Virtualization - CU, NYC**
<a href='http://www.youtube.com/watch?feature=player_embedded&v=mSwnKIxPGBs' target='_blank'><img src='http://img.youtube.com/vi/mSwnKIxPGBs/0.jpg' width='425' height=344 /></a>

**MS in CS Thesis - Sensor Emulation - 10 Sensors - Paired real device scenario - Android x86 Virtualization - CU, NYC**
<a href='http://www.youtube.com/watch?feature=player_embedded&v=J09fhk9WU6k' target='_blank'><img src='http://img.youtube.com/vi/J09fhk9WU6k/0.jpg' width='425' height=344 /></a>

**MS in CS Thesis - Sensor Emulation - 10 Sensors - Remote Server Scenario - Android x86 Virtualization - CU, NYC**
<a href='http://www.youtube.com/watch?feature=player_embedded&v=M6AYO2Ne_rc' target='_blank'><img src='http://img.youtube.com/vi/M6AYO2Ne_rc/0.jpg' width='425' height=344 /></a>

## ABSTRACT ##

Enabling the Virtual Phones to remotely sense the Real Phones in real-time
~ A Sensor Emulation initiative for virtualized Android-x86 ~

Smartphones nowadays have the ground-breaking features that were only a figment
of one’s imagination. For the ever-demanding cellphone users, the exhaustive list of
features that a smartphone supports just keeps getting more exhaustive with time.
These features aid one’s personal and professional uses as well. Extrapolating into the
future the features of a present-day smartphone, the lives of us humans using
smartphones are going to be unimaginably agile.

With the above said emphasis on the current and future potential of a smartphone,
the ability to virtualize smartphones with all their real-world features into a virtual
platform, is a boon for those who want to rigorously experiment and customize the
virtualized smartphone hardware without spending an extra penny. Once
virtualizable independently on a larger scale, the idea of virtualized smartphones with
all the virtualized pieces of hardware takes an interesting turn with the sensors being
virtualized in a way that’s closer to the real-world behavior.

When accessible remotely with the real-time responsiveness, the above mentioned
real-world behavior will be a real dealmaker in many real-world systems, namely, the
life-saving systems like the ones that instantaneously get alerts about harmful
magnetic radiations in the deep mining areas, etc. And these life-saving systems would
be installed on a large scale on the desktops or large servers as virtualized
smartphones having the added support of virtualized sensors which remotely fetch
the real hardware sensor readings from a real smartphone in real-time. Based on
these readings the lives working in the affected areas can be alerted and thus saved by
the people who are operating the at the desktops or large servers hosting the
virtualized smartphones.

In addition, the direct and one of the biggest advantages of such a real hardware
sensor driven Sensor Emulation in an emulated Android(-x86) environment is that the
Android applications that use sensors can now run on the emulator and act under the
influence of real hardware sensors’ due to the emulated sensors.

The current work of Sensor Emulation is quite unique when compared to the existing
and past sensor-related works. The uniqueness comes from the full-fledged sensor
emulation in a virtualized smartphone environment as opposed to building some
sophisticated physical systems that usually aggregate the sensor readings from the real
hardware sensors, might be in a remote manner and in real-time. For example,
wireless sensor networks based remote-sensing systems that install real hardware
sensors in remote places and have the sensor readings from all those sensors at a
centralized server or something similar, for the necessary real-time or offline analysis.
In these systems, apart from collecting mere real hardware sensor readings into a
centralized entity, nothing more is being achieved unlike in the current work of
Sensor Emulation wherein the emulated sensors behave exactly like the remote real
hardware sensors. The emulated sensors can be calibrated, speeded up or slowed
down(in terms of their sampling frequency), influence the sensor-based application
running inside the virtualized smartphone environment exactly as the real hardware
sensors of a real phone would do to the sensor-based application running in that real
phone. In essence, the current work is more about generalizing the sensors with all its
real-world characteristics as far as possible in a virtualized platform than just a
framework to send and receive sensor readings over the network between the real
and virtual phones.

Realizing the useful advantages of Sensor Emulation which is about adding
virtualized sensors support to emulated environments, the current work emulates a
total of ten sensors present in the real smartphone, Samsung Nexus S, an Android
device. Virtual phones run Android-x86 while real phones run Android. The real
reason behind choosing Android-x86 for virtual phone is that x86-based Android
devices are feature-rich over ARM based ones, for example a full-fledged x86 desktop
or a tablet has more features than a relatively small smartphone. Out of the ten, five
are real sensors and the rest are virtual or synthetic ones. The real ones are
Accelerometer, Magnetometer, Light, Proximity, and Gyroscope whereas the virtual
ones are Corrected Gyroscope, Linear Acceleration, Gravity, Orientation, and Rotation
Vector. The emulated Android-x86 is of Android release version Jelly Bean 4.3.1 which
differs only very slightly in terms of bug fixes from Android Jelly Bean 4.3 running on
the real smartphone.

One of the noteworthy aspects of the Sensor Emulation accomplished is being
demand-less - exactly the same sensor-based Android applications will be able to use
the sensors on the real and virtual phones, with absolutely no difference in terms of
their sensor-based behavior.

The emulation’s core idea is the socket-communication between two modules of
Hardware Abstraction Layer(HAL) which is driver-agnostic, remotely over a wireless
network in real-time. Apart from a Paired real-device scenario from which the real
hardware sensor readings are fetched, the Sensor Emulation also is compatible with a
Remote Server Scenario wherein the artificially generated sensor readings are fetched
from a remote server. Due to the Sensor Emulation having been built on mere
end-to-end socket-communication, it’s logical and obvious to see that the real and
virtual phones can run different Android(-x86) releases with no real impact on the
Sensor Emulation being accomplished.

Sensor Emulation once completed was evaluated for each of the emulated sensors
using applications from Android Market as well as Amazon Appstore. The applications
category include both the basic sensor-test applications that show raw sensor readings,
as well as the advanced 3D sensor-driven games which are emulator compatible,
especially in terms of the graphics. The evaluations proved the current work of Sensor
Emulation to be generic, efficient, robust, fast, accurate, and real.

As of this writing i.e., January 2014, the current work of Sensor Emulation is the sole
system-level sensor virtualization work that embraces remoteness in real-time for the
emulated Android-x86 systems. It is important to note that though the current work is
targeted for Android-x86, the code written for the current work makes no assumptions
about underlying platform to be an x86 one. Hence, the work is also logically seen as
compatible with ARM based emulated Android environment though not actually
tested.

## Introduction ##

Being in 2014, the smartphone industry has almost captivated us humans with its astonishing
capabilities. The capabilities that were once seen only as science fiction are now realities.
The capabilities include but not limited to sensing heat of a thermal body, atmospheric
pressure in a submarine, change in ambience for an auto-brightness light, speed a vehicle,
spatial orientation of a stationary object, gravity’s influence on an object in space,
magnetic radiation in a mining area, geographical position in remote parts of our earth,
and a lot more. In recent times, these capabilities have become an inseparable part of our
lives due to their limitless usefulness in our day-to-day activities.

Smartphone Virtualization is an intriguing technology with its capability to host
different platforms on the same piece of hardware with no additional cost - smartphone
customization at its best. The ability to run such virtualized smartphones on machines
such as desktops or large servers leverages the importance of smartphone virtualization.
The leverage is in the form of the opportunity to run hundreds of such virtualized smartphones
independently at the same time on the same machine. And when the sensing capabilities of
a real smartphone can be fully experienced in such a vastly scaled up virtualized
environment, the sensor-based smartphone applications using the virtualized sensors of
the virtualized smartphones, will find the door to exploit the strengths of smartphone
virtualization, wide open. Remote-sensing capability over the environmental changes has
been the epitome of many active researches in the areas of Space exploration, Oceanography,
etc. When the remote-sensing capability is integral to smartphone virtualization, the
resulting opportunities are infinite. In other words, adding Sensor Emulation to the
virtualized smartphones that can be controlled in a remote manner, will be a great
value addition to the smartphone virtualization technology, with those virtualized
smartphones being powered by desktops or large servers as mentioned previously. This
value addition will be a great feat when it’s achievable in real-time. Thus, enabling
virtual(virtualized/emulated) phones(smartphones) to remotely sense physical(real)
phones(smartphones) in real-time which is nothing but Sensor Emulation, is
noteworthy.

Accomplishing the Sensor Emulation in a virtualized smartphone environment
emphasizing the aspects of remoteness and real-timeness in a full-fledged manner is
quite different from building sophisticated physical systems for real-time
remote-sensing. The difference lies in the very notion of emulation of sensor with all
the real-world characteristics. This implies that apart from being able to collect the raw
sensor data remotely in real-time from the real hardware sensors, the emulated sensors
can be operated just like real hardware sensors, in the virtualized smartphone
environment. As a result, the emulated sensors can be calibrated, altered for their
sampling frequency of sensor data, and so on. If need be, the real hardware sensors
driving can be operated via the emulated sensors by having two-way communication
between the real and emulated sensors. All these will be absent in the usual
remote-sensing systems whose aim is to just accumulate data in real-time from the
deployed real hardware sensors in the designated remote places, and analyze them live
or offline. As a matter of fact, building hundreds of sophisticated remote-sensing
physical systems is way expensive than powering hundreds of, even thousands of
virtualized smartphones on a single or a minimal set of resourceful desktops or large
servers, equipped with emulated sensors driven remotely by real hardware sensors in
real-time. Thus, the current work of Sensor Emulation in virtualized smartphone
environment stands out among all the sensor-based works(listed in related works
Chapter) as of January 2014.

The ambitious task of Sensor Emulation, that is enabling virtual phones to remotely
sense physical phones in real-time demands considerable amount of work and is quite
challenging as well. When the time available to accomplish it is relatively less, the
challenge gets even better. Reason being, the software shipped with the smartphones
would not have been be usually designed for the purpose of remote-sensing at all, but
only for localized usage by an ordinary cellphone user. The considerable work will be
in the form of including features into the software of the virtual and physical phones so
that virtual phones powered by desktops or large servers, can do the remote-sensing
using the physical phones. The included features must be robust, fast, accurate,
generic, efficient, and real - a challenge that the current work of Sensor Emulation has
accomplished in a tight schedule of three months from Sep-Nov, 2013.
As far as the current work is concerned, the real phones will be the ARM based Android
smartphones and the virtual phones will be running Android-x86. The reason for
choosing Android-x86 for virtual phones is that once accomplished the Sensor
Emulation can be used in the virtualized Android-x86 based real phones(devices such as
tablets, gadgets, etc) which are feature-rich over the ARM based real Android
smartphones when full-fledged user layout of desktops and tablets are considered.