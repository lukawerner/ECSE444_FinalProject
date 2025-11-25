def parse_message(msg):

    if msg.startswith("T"): # Example message: "T 25", meaning temperature is 25 degrees
        temperature = int(msg.split()[1])
        return {"type": "temperature", "temperature": temperature}

    elif msg.startswith("I"): # Example message: "I 30 10.1", meaning intruder detected at angle 30, distance 10
        parts = msg.split()
        angle = int(parts[1]) + 30
        distance = float(parts[2])
        return {"type": "intruder", "angle": angle, "distance": distance}

    elif msg.startswith("N"): # Example message: "N 30", meaning normal status at angle 30
        angle = int(msg.split()[1]) + 30
        return {"type": "normal", "angle": angle}

    elif msg.startswith("B"):
        parts = msg.split()
        angle = int(parts[1]) + 30
        distance = float(parts[2])
        return {"type": "baseline", "angle": angle, "distance": distance}

    else:
        return {"type": "?", "message": msg}

