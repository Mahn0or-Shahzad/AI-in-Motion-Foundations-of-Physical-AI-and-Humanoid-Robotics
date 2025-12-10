"""
VoiceCommand Data Model

Implements the VoiceCommand data model as defined in the message specification.
"""

from dataclasses import dataclass
from typing import Dict, Any, Optional
from builtin_interfaces.msg import Time


@dataclass
class VoiceCommand:
    """
    VoiceCommand data model representing a processed voice command.

    Fields:
    - audio_data: string (base64 encoded audio)
    - transcription: string (text transcription of audio)
    - intent: string (recognized intent like "move_to", "pick_up")
    - parameters: object (intent-specific parameters)
    - timestamp: Time (time of command received)
    - confidence: float (confidence score of recognition)
    """

    audio_data: str
    transcription: str
    intent: str
    parameters: Dict[str, Any]
    timestamp: Time
    confidence: float

    def __post_init__(self):
        """Validate the VoiceCommand after initialization."""
        # Validate audio data format
        if not self.audio_data or not isinstance(self.audio_data, str):
            raise ValueError("Audio data must be a non-empty string")

        # Validate intent is recognized from predefined list
        valid_intents = [
            "navigation", "manipulation", "perception", "status",
            "stop", "help", "unknown"
        ]
        if self.intent not in valid_intents:
            raise ValueError(f"Intent must be one of {valid_intents}")

        # Validate parameters match expected format for intent
        self._validate_parameters()

        # Validate confidence is between 0.0 and 1.0
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("Confidence must be between 0.0 and 1.0")

    def _validate_parameters(self):
        """Validate that parameters match expected format for the intent."""
        if self.intent == "navigation":
            required_params = ["target_location"]
            for param in required_params:
                if param not in self.parameters:
                    raise ValueError(f"Navigation intent requires parameter: {param}")
        elif self.intent == "manipulation":
            required_params = ["object", "action"]
            for param in required_params:
                if param not in self.parameters:
                    raise ValueError(f"Manipulation intent requires parameter: {param}")
        # Add more intent-specific validations as needed

    def get_state_transition(self, previous_state: Optional[str] = None) -> str:
        """
        Determine the state transition for the voice command processing.

        Returns:
        - 'received' → 'processing' → 'processed' → 'executed' / 'rejected'
        """
        if previous_state is None:
            return "received"

        # Simple logic to determine state transition
        if previous_state == "received":
            return "received → processing"
        elif previous_state == "processing":
            return "processing → processed"
        elif previous_state == "processed":
            # In a real implementation, execution success would determine final state
            return "processed → executed"
        else:
            return previous_state

    def to_dict(self) -> Dict[str, Any]:
        """Convert the VoiceCommand to a dictionary representation."""
        return {
            "audio_data": self.audio_data,
            "transcription": self.transcription,
            "intent": self.intent,
            "parameters": self.parameters,
            "timestamp": self.timestamp,
            "confidence": self.confidence
        }