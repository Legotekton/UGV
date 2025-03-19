import concurrent.futures
import threading
import os

lat = 0
lon = 0
alt = 0
print(lat, lon, alt)

def data_thread(lat, lon, alt):
    lat = 3
    lon = 6
    alt = 2
    print("Changed Values")
    return lat, lon, alt

# Need to use concurrent futures to run the thread to get 
# the returned values back to the main function
with concurrent.futures.ThreadPoolExecutor() as executor:
    future = executor.submit(data_thread, 'lat', 'lon', 'alt')
    lat, lon, alt = future.result()

print(lat, lon, alt)