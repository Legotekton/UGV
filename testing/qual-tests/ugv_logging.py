lat = 30
lon = -87
alt = 27

file = open("ugv_logging.txt", "w")
file.write("lat = " + str(lat) + "\n")
file.write("lon = " + str(lon) + "\n")
file.write("alt = " + str(alt) + "\n")
file.close()
