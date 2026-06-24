from fastapi import FastAPI
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from pathlib import Path

from routers import ai, connection, control, telemetry

app = FastAPI()
UI_PATH = Path(__file__).parent / "index.html"

# Serve static files for CSS and JS
app.mount("/static", StaticFiles(directory="static"), name="static")

# Include Routers
app.include_router(ai.router)
app.include_router(connection.router)
app.include_router(control.router)
app.include_router(telemetry.router)

@app.get("/")
def get_ui():
    return HTMLResponse(UI_PATH.read_text(encoding="utf-8"))
