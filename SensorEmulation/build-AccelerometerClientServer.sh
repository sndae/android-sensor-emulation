 #
 #   Copyright (C) 2013  Raghavan Santhanam, raghavanil4m@gmail.com, rs3294@columbia.edu
 #   This was done as part of my MS thesis research at Columbia University, NYC in Fall 2013.
 #
 #   build-SensorEmulationClientServer.sh is free software: you can redistribute it and/or modify
 #   it under the terms of the GNU General Public License as published by
 #   the Free Software Foundation, either version 3 of the License, or
 #   (at your option) any later version.
 #
 #   build-SensorEmulationClientServer.sh is distributed in the hope that it will be useful,
 #   but WITHOUT ANY WARRANTY; without even the implied warranty of
 #   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 #   GNU General Public License for more details.
 #
 #   You should have received a copy of the GNU General Public License
 #   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 #

set -x
gcc -Wall -O2 -DDEVICE_READINGS AccelerometerClientServer.c -lpthread -o AccelerometerClientServer
