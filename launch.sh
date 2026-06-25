#!/bin/bash

echo "🚀 Launching Python-Ganglia Mock Simulation..."

# Free up port 8000 if it's already in use
PIDS=$(lsof -t -i:8000 2>/dev/null)
if [ ! -z "$PIDS" ]; then
    echo "Cleaning up old server processes..."
    kill -9 $PIDS
fi

# Use the virtual environment Python if it exists
PYTHON_CMD="python"
if [ -f ".venv/bin/python" ]; then
    PYTHON_CMD=".venv/bin/python"
elif [ -f "venv/bin/python" ]; then
    PYTHON_CMD="venv/bin/python"
fi

echo "Using python: $PYTHON_CMD"
$PYTHON_CMD mock_sim.py &
SERVER_PID=$!

# Trap Ctrl+C to neatly kill the server when the user exits
trap "echo -e '\n🛑 Stopping simulation server...'; kill -9 $SERVER_PID 2>/dev/null; exit" SIGINT SIGTERM

echo "⏳ Waiting for server to initialize..."
sleep 2

echo ""
echo "✅ Server is running (PID: $SERVER_PID)!"
echo "🌐 Open your web browser to: http://127.0.0.1:8000"
echo "💻 To run the automated tuning test, open a NEW terminal window and run:"
echo "   python scripts/cli_api.py --sequence"
echo ""
echo "Press Ctrl+C here at any time to shut down the server."

# Wait indefinitely for the server process (keeps script alive to catch Ctrl+C)
wait $SERVER_PID
