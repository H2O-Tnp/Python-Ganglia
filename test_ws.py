import asyncio
import websockets
import json

async def run():
    async with websockets.connect('ws://localhost:8000/ws/telemetry') as ws:
        msg = await ws.recv()
        data = json.loads(msg)
        print("Keys:", data[0].keys())
        if 'mpc_pred_vel' in data[0]:
            print("mpc_pred_vel len:", len(data[0]['mpc_pred_vel']))
            print("mpc_pred_vel preview:", data[0]['mpc_pred_vel'][:5])
        else:
            print("mpc_pred_vel NOT IN DATA!")

asyncio.run(run())
