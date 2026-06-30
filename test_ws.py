import asyncio
import websockets
import json

async def run():
    async with websockets.connect('ws://localhost:8000/ws/telemetry') as ws:
        msg = await ws.recv()
        data = json.loads(msg)
        print("Keys:", data[0].keys())
        print("Velocity:", data[0].get("velocity"))
        print("Current:", data[0].get("current"))
        print("Target:", data[0].get("target_velocity"))
        if 'mpc_pred_vel' in data[0]:
            print("mpc_pred_vel len:", len(data[0]['mpc_pred_vel']))

asyncio.run(run())
