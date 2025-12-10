# Voice Command Processing API Contract

## Service: /process_voice_command

### Request
```json
{
  "audio_data": "string (base64 encoded audio)",
  "language": "string (default: 'en')"
}
```

### Response
```json
{
  "success": "boolean",
  "transcription": "string",
  "intent": "string",
  "parameters": "object",
  "error_message": "string (if success is false)"
}
```

### Example Request
```json
{
  "audio_data": "base64encodedaudiobytes...",
  "language": "en"
}
```

### Example Response (Success)
```json
{
  "success": true,
  "transcription": "Move to the kitchen",
  "intent": "navigation",
  "parameters": {
    "location": "kitchen"
  },
  "error_message": null
}
```

### Example Response (Error)
```json
{
  "success": false,
  "transcription": "",
  "intent": "",
  "parameters": {},
  "error_message": "Audio data format not recognized"
}
```

### Error Codes
- 400: Invalid audio data format
- 401: API authentication failed
- 500: Internal processing error

## Action: /navigate_to_pose

### Goal
```json
{
  "target_pose": {
    "position": {"x": float, "y": float, "z": float},
    "orientation": {"w": float, "x": float, "y": float, "z": float}
  },
  "tolerance": "float (meters)"
}
```

### Result
```json
{
  "status": "int (0=SUCCESS, 1=FAILED, 2=CANCELLED)",
  "message": "string"
}
```

### Feedback
```json
{
  "distance_remaining": "float",
  "current_pose": {
    "position": {"x": float, "y": float, "z": float},
    "orientation": {"w": float, "x": float, "y": float, "z": float}
  }
}
```

### Error Codes
- 400: Invalid target pose
- 404: Navigation map not found
- 500: Navigation system error