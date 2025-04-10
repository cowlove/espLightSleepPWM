#!/usr/bin/python3
import sys
import re
import fileinput
import io 
import math
import json

from datetime import datetime, timedelta 

keys = sys.argv[1:]
#["Voltage1", "PWM", "Voltage2"]
for line in	io.TextIOWrapper(sys.stdin.buffer, errors='ignore'):
    try:
        m = re.search(r'^\[([^]]*)\](.*)$', line)
        if m:
            d = m.group(1)
            date = datetime.strptime(d, '%Y-%m-%dT%H:%M:%S.%fZ')
            date += timedelta(hours = -7) 
            j = json.loads(m.group(2))
 
            if "LOG" in j:
                for logl in j["LOG"]:
                    if "LTO" in logl:
                        to = logl["LTO"]
                        print("%.3f" % (date.timestamp() - to), end=" ")
                        for k in keys:
                            val = None
                            if k in j:
                                val = j[k]
                            elif k in logl:
                                val = logl[k]
                            elif "." in k:
                                (k1, k2) = k.split(".")
                                if k1 in logl and k2 in logl[k1]:
                                    val = logl[k1][k2]
                                elif k1 in j and k2 in j[k1]:
                                    val = j[k1][k2]
                            print(val, end=" ")
                        print("");
    except Exception as e:
        print("ERROR Exception:",end="")
        print(e)
        print("ERROR ",end="")
        print(line)
        #print ("")
