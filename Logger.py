import numpy as np
import datetime

class Logger():
    def __init__(self, print_events=False):
        self.__timestamp = str(np.datetime64(datetime.datetime.now())).replace(':','')
        self.__start = np.datetime64(datetime.datetime.now()) 
        self.__print_events = print_events

    #logs an event in format timestamp [source]: message
    def log_event(self, source, message):
        with open(f"logs/log-{self.__timestamp}.txt", "a") as f:
            timestamp = str(np.datetime64(datetime.datetime.now()))
            formatted = f"[{timestamp}] [{source}]: {message}\n"
            if self.__print_events:
                print(formatted)
            f.write(formatted)

    def log_auv_location(self, x, y, heading):
        with open(f"logs/data-{self.__timestamp}.out", "a") as f:
            float_time = (np.datetime64(datetime.datetime.now()) - self.__start) / np.timedelta64(1, 'ms')
            arr = np.array([[float_time, x, y, heading]])
            np.savetxt(f, arr, delimiter=',')

