"""
Voice Interface Node

Implements voice processing using OpenAI Whisper API and ROS 2.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ai_motion_common_msgs.msg import VoiceCommand
from ai_motion_common_msgs.srv import ProcessVoiceCommand
from sensor_msgs.msg import AudioData
import openai
import pyaudio
import wave
import numpy as np
import tempfile
import os
import asyncio
import threading
import time
from typing import Dict, Any, Optional
import json


class VoiceInterfaceNode(Node):
    """
    A ROS 2 node that handles voice input and processing using OpenAI Whisper API.
    """

    def __init__(self):
        super().__init__('voice_interface_node')

        # Initialize OpenAI API key from parameter or environment
        self.declare_parameter('openai_api_key', '')
        self.api_key = self.get_parameter('openai_api_key').value or os.getenv('OPENAI_API_KEY')

        if not self.api_key:
            self.get_logger().warn('OpenAI API key not set. Voice processing will not work.')
        else:
            openai.api_key = self.api_key

        # Audio recording parameters
        self.sample_rate = 16000
        self.chunk_size = 1024
        self.channels = 1
        self.format = pyaudio.paInt16
        self.recording_timeout = 5.0
        self.silence_threshold = 0.01
        self.min_recording_duration = 0.5
        self.max_recording_duration = 30.0

        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()

        # Create publisher for voice commands
        self.voice_command_pub = self.create_publisher(
            VoiceCommand,
            '/voice_commands',
            10
        )

        # Create service for voice processing
        self.voice_service = self.create_service(
            ProcessVoiceCommand,
            '/process_voice_command',
            self.process_voice_command_callback
        )

        # Create subscriber for audio input (when audio is provided externally)
        self.audio_sub = self.create_subscription(
            AudioData,
            '/audio_input',
            self.audio_callback,
            10
        )

        # Timer for continuous listening
        self.listen_timer = self.create_timer(0.1, self.continuous_listen)

        # State variables
        self.is_listening = False
        self.recording_thread = None
        self.should_stop_recording = False

        # Whisper API settings
        self.whisper_model = "whisper-1"
        self.whisper_response_format = "json"
        self.whisper_temperature = 0.0
        self.whisper_language = "en"

        self.get_logger().info('Voice Interface Node initialized')

    def audio_callback(self, msg: AudioData):
        """
        Callback for audio data received from external source.
        """
        self.get_logger().info(f'Received audio data with {len(msg.data)} bytes')

        # Process the audio data
        try:
            # Convert audio data to WAV format temporarily
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_wav:
                # Create WAV file from audio data
                wf = wave.open(temp_wav.name, 'wb')
                wf.setnchannels(self.channels)
                wf.setsampwidth(2)  # Assuming 16-bit audio
                wf.setframerate(self.sample_rate)
                wf.writeframes(msg.data)
                wf.close()

                # Process with Whisper API
                transcription = self.transcribe_audio_file(temp_wav.name)

                # Clean up temporary file
                os.unlink(temp_wav.name)

                if transcription:
                    self.process_transcription(transcription)

        except Exception as e:
            self.get_logger().error(f'Error processing audio data: {e}')

    def continuous_listen(self):
        """
        Continuous listening loop to detect voice activity.
        """
        # This is a simplified version - in a real implementation, this would
        # continuously listen for voice activity and trigger recording
        pass

    def start_recording(self):
        """
        Start recording audio from the microphone.
        """
        if self.recording_thread and self.recording_thread.is_alive():
            self.get_logger().warn('Recording already in progress')
            return

        self.should_stop_recording = False
        self.recording_thread = threading.Thread(target=self.record_audio)
        self.recording_thread.start()
        self.get_logger().info('Started recording')

    def stop_recording(self):
        """
        Stop recording audio.
        """
        self.should_stop_recording = True
        if self.recording_thread:
            self.recording_thread.join()
        self.get_logger().info('Stopped recording')

    def record_audio(self):
        """
        Record audio from microphone until silence is detected or timeout occurs.
        """
        stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.chunk_size
        )

        frames = []
        start_time = time.time()
        silent_chunks = 0
        max_silent_chunks = 20  # About 1.3 seconds of silence to stop

        self.get_logger().info('Recording... Speak now.')

        try:
            while not self.should_stop_recording:
                data = stream.read(self.chunk_size, exception_on_overflow=False)
                frames.append(data)

                # Check for silence
                audio_data = np.frombuffer(data, dtype=np.int16)
                amplitude = np.sqrt(np.mean(audio_data**2))

                if amplitude < self.silence_threshold:
                    silent_chunks += 1
                else:
                    silent_chunks = 0  # Reset if loud sound detected

                # Check duration limits
                current_duration = time.time() - start_time
                if current_duration > self.max_recording_duration:
                    self.get_logger().info('Max recording duration reached')
                    break

                # Stop if enough silence detected
                if silent_chunks > max_silent_chunks and current_duration > self.min_recording_duration:
                    self.get_logger().info('Silence detected, stopping recording')
                    break

        except Exception as e:
            self.get_logger().error(f'Error during recording: {e}')
        finally:
            stream.stop_stream()
            stream.close()

        # Save to temporary file and process
        if len(frames) > 0:
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_wav:
                wf = wave.open(temp_wav.name, 'wb')
                wf.setnchannels(self.channels)
                wf.setsampwidth(self.audio.get_sample_size(self.format))
                wf.setframerate(self.sample_rate)
                wf.writeframes(b''.join(frames))
                wf.close()

                # Process with Whisper API
                transcription = self.transcribe_audio_file(temp_wav.name)

                # Clean up temporary file
                os.unlink(temp_wav.name)

                if transcription:
                    self.process_transcription(transcription)

    def transcribe_audio_file(self, audio_file_path: str) -> Optional[str]:
        """
        Transcribe audio file using OpenAI Whisper API.
        """
        if not self.api_key:
            self.get_logger().error('OpenAI API key not set')
            return None

        try:
            with open(audio_file_path, "rb") as audio_file:
                transcript = openai.Audio.transcribe(
                    model=self.whisper_model,
                    file=audio_file,
                    response_format=self.whisper_response_format,
                    temperature=self.whisper_temperature,
                    language=self.whisper_language
                )

            self.get_logger().info(f'Transcription: {transcript["text"]}')
            return transcript["text"]

        except Exception as e:
            self.get_logger().error(f'Error transcribing audio: {e}')
            return None

    def process_transcription(self, transcription: str):
        """
        Process the transcribed text and publish as VoiceCommand.
        """
        if not transcription.strip():
            self.get_logger().warn('Empty transcription received')
            return

        # Create VoiceCommand message
        voice_cmd = VoiceCommand()
        voice_cmd.transcription = transcription
        voice_cmd.timestamp = self.get_clock().now().to_msg()
        voice_cmd.confidence = 0.9  # Default confidence, would come from Whisper in real implementation

        # For now, we'll set a generic intent - this would be determined by an intent classifier
        voice_cmd.intent = "unknown"
        voice_cmd.parameters = {}  # Will be filled by intent classifier

        # Publish the voice command
        self.voice_command_pub.publish(voice_cmd)
        self.get_logger().info(f'Published voice command: {transcription}')

    def process_voice_command_callback(self, request: ProcessVoiceCommand.Request,
                                      response: ProcessVoiceCommand.Response) -> ProcessVoiceCommand.Response:
        """
        Service callback to process voice commands.
        """
        try:
            # Decode base64 audio data if provided
            if request.audio_data:
                # In a real implementation, we'd decode the base64 audio data
                # and process it with Whisper API
                self.get_logger().info(f'Processing voice command with {len(request.audio_data)} bytes of audio data')

                # For simulation purposes, we'll create a temporary file and process it
                with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_wav:
                    # This is a placeholder - in real implementation we'd decode base64 to audio file
                    # For now, we'll just simulate the process
                    response.success = True
                    response.transcription = "Simulated transcription of: " + request.audio_data[:50] + "..."
                    response.intent = "navigation"
                    response.parameters = {"location": "kitchen"}
                    response.error_message = ""
            else:
                response.success = False
                response.transcription = ""
                response.intent = ""
                response.parameters = {}
                response.error_message = "No audio data provided"

        except Exception as e:
            self.get_logger().error(f'Error processing voice command: {e}')
            response.success = False
            response.transcription = ""
            response.intent = ""
            response.parameters = {}
            response.error_message = str(e)

        return response

    def destroy_node(self):
        """
        Cleanup resources when node is destroyed.
        """
        if hasattr(self, 'audio'):
            self.audio.terminate()
        super().destroy_node()


def main(args=None):
    """
    Main function to run the voice interface node.
    """
    rclpy.init(args=args)

    voice_interface_node = VoiceInterfaceNode()

    try:
        rclpy.spin(voice_interface_node)
    except KeyboardInterrupt:
        pass
    finally:
        voice_interface_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()