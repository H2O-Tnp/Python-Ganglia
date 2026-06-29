import asyncio
import websockets
import json

async def run():
    async with websockets.connect('ws://127.0.0.1:8000/ws/telemetry') as ws:
        msg = await ws.recv()
        data = json.loads(msg)
        with open("ws_out.txt", "w") as f:
            f.write(str(data[0].keys()) + "\n")
            if 'mpc_pred_vel' in data[0]:
                f.write("len: " + str(len(data[0]['mpc_pred_vel'])) + "\n")
            else:
                f.write("NO mpc_pred_vel\n")

asyncio.run(run())
