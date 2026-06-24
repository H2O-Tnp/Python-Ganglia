import os
import json
import google.generativeai as genai
from fastapi import APIRouter
from models import ChatRequest

router = APIRouter()

@router.post("/chat")
def chat_with_ai(req: ChatRequest):
    api_key = os.getenv("GEMINI_API_KEY")
    if not api_key:
        return {"response": "Error: GEMINI_API_KEY environment variable is not set on the server."}
        
    try:
        genai.configure(api_key=api_key)
        model = genai.GenerativeModel('gemini-3.1-flash-lite')
        
        system_prompt = f"""You are an AI Tutor for a DC Motor Control Lab Experiment.
The student is using a web app to tune a DC motor using PID control.
Here is the current state of their UI:
{json.dumps(req.context, indent=2)}

Use this context to answer their questions accurately. Be encouraging, educational, and avoid giving direct answers without explanation."""
        
        prompt = f"{system_prompt}\n\nStudent asks: {req.message}"
        response = model.generate_content(prompt)
        return {"response": response.text}
    except Exception as e:
        return {"response": f"AI Error: {str(e)}"}
