import asyncio
import websockets
import json
import statistics
import time
import sys

WS_URL = "ws://127.0.0.1:8000/ws/telemetry"

async def measure(duration=2.0):
    velocities = []
    try:
        async with websockets.connect(WS_URL) as ws:
            start_time = time.time()
            while time.time() - start_time < duration:
                try:
                    data = await asyncio.wait_for(ws.recv(), timeout=0.5)
                    pts = json.loads(data)
                    for pt in pts:
                        if "velocity" in pt:
                            velocities.append(pt["velocity"])
                except asyncio.TimeoutError:
                    continue
    except Exception as e:
        print(f"Error: {e}")
        return
        
    if not velocities:
        print("No data")
        return
        
    mean = statistics.mean(velocities)
    stdev = statistics.stdev(velocities) if len(velocities) > 1 else 0.0
    print(json.dumps({"mean": mean, "stdev": stdev}))

if __name__ == "__main__":
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    asyncio.run(measure(duration))
