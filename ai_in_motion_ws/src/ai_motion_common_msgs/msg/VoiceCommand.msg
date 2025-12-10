# VoiceCommand.msg
# Defines a voice command processed by the system

# Base64 encoded audio data
string audio_data

# Text transcription of the audio
string transcription

# Recognized intent (e.g., "navigation", "manipulation", "perception")
string intent

# Intent-specific parameters
string parameters

# Timestamp when command was received
builtin_interfaces/Time timestamp

# Confidence score of the recognition (0.0 to 1.0)
float64 confidence