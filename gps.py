from __future__ import print_function
from time import sleep
import sys
import qwiic_titan_gps
from AltAzRange import AltAzimuthRange

# Teste o GPS como módulo individual - ENSAIO GPS

latitude = 0.0
longitude = 0.0
altitude = 0.0

azimtuh = 0.0
distance = 0.0
elavation = 0.0

# 41.450745, -8.294013

AltAzimuthRange.default_observer(41.450745, -8.294013, 200)
trajetoria = AltAzimuthRange()

def run_example():
    print("SparkFun GPS Breakout - XA1110!")
    qwiicGPS = qwiic_titan_gps.QwiicTitanGps()

    if qwiicGPS.connected is False:
        print("Could not connect to to the SparkFun GPS Unit. Double check that\
              it's wired correctly.", file=sys.stderr)
        return

    qwiicGPS.begin()

    while True:
        if qwiicGPS.get_nmea_data() is True:
            for k, v in qwiicGPS.gnss_messages.items():
                print(k, ":", v)
                if (k == "Latitude"):
                    latitude = v
                if (k == "Longitude"):
                    longitude = v
                if (k == "Altitude"):
                    altitude = v

        # sleep(4)
        print("/////////////")
        print(latitude, longitude, altitude)
        trajetoria.target(latitude, longitude, altitude)

        valores = trajetoria.calculate()

        azimuth = valores["azimuth"]
        elevation = valores["elevation"]
        distance = valores["distance"]

        print("////////////////////////")
        print("Azimuth=", azimuth, "Elavation=", elavation, "Distancia=", distance)
        print("////////////////////////")

if __name__ == '__main__':
    try:
        run_example()
    except (KeyboardInterrupt, SystemExit) as exErr:
        print("Ending Basic Example.")
        sys.exit(0)