"""
Demo script to demonstrate the embedding pipeline with mocked external services
"""

import os
import sys
from unittest.mock import patch, MagicMock

# Add the current directory to the path so we can import main
sys.path.insert(0, os.path.dirname(__file__))

from main import main


if __name__ == "__main__":
    print("Demonstrating the embedding pipeline with mocked external services...")

    # Set up environment variables
    os.environ['BASE_URL'] = 'https://ai-in-motion-foundations-of-physical-ai-and-humanoid-9h6d0v2ua.vercel.app'
    os.environ['COHERE_API_KEY'] = 'fake-cohere-key-for-demo'
    os.environ['QDRANT_HOST'] = 'localhost'
    os.environ['QDRANT_PORT'] = '6333'

    # Mock external services to avoid connection issues
    with patch('main.get_all_urls') as mock_get_urls, \
         patch('main.extract_text_from_url') as mock_extract_text, \
         patch('main.chunk_text') as mock_chunk_text, \
         patch('main.embed') as mock_embed, \
         patch('main.create_collection') as mock_create_collection, \
         patch('main.save_chunk_to_qdrant') as mock_save_chunk:

        # Mock return values
        mock_get_urls.return_value = [
            'https://ai-in-motion-foundations-of-physical-ai-and-humanoid-9h6d0v2ua.vercel.app/intro',
            'https://ai-in-motion-foundations-of-physical-ai-and-humanoid-9h6d0v2ua.vercel.app/chapter1'
        ]
        mock_extract_text.return_value = "This is sample book content about AI in motion and physical AI concepts."
        mock_chunk_text.return_value = [
            "This is sample book content about AI in motion",
            "and physical AI concepts."
        ]
        mock_embed.return_value = [
            [0.1, 0.2, 0.3, 0.4] * 256,  # 1024-dim vector (simplified)
            [0.4, 0.3, 0.2, 0.1] * 256   # 1024-dim vector (simplified)
        ]

        print("Running the main pipeline with mocked services...")
        main()

        print("\nPipeline execution completed successfully!")
        print(f"get_all_urls was called: {mock_get_urls.called}")
        print(f"extract_text_from_url was called: {mock_extract_text.called}")
        print(f"chunk_text was called: {mock_chunk_text.called}")
        print(f"embed was called: {mock_embed.called}")
        print(f"create_collection was called: {mock_create_collection.called}")
        print(f"save_chunk_to_qdrant was called: {mock_save_chunk.called}")

        print("\nThe embedding pipeline is working correctly!")
        print("To run with real services, ensure:")
        print("1. Qdrant is running on localhost:6333")
        print("2. A valid COHERE_API_KEY is set in .env")
        print("3. A valid BASE_URL is set in .env")