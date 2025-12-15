"""
Simple test script to validate core functions that don't require external services
"""

import os
import sys
from unittest.mock import patch, MagicMock
import tempfile

# Add the current directory to the path so we can import main
sys.path.insert(0, os.path.dirname(__file__))

from main import get_all_urls, extract_text_from_url, chunk_text, embed


def test_get_all_urls():
    """Test the get_all_urls function"""
    print("Testing get_all_urls function...")

    # Mock a simple HTML response
    mock_html = """
    <html>
        <body>
            <a href="/page1">Page 1</a>
            <a href="/chapter2">Chapter 2</a>
            <a href="/section3">Section 3</a>
            <a href="/about">About</a>
        </body>
    </html>
    """

    with patch('requests.get') as mock_get:
        mock_response = MagicMock()
        mock_response.content = mock_html
        mock_response.raise_for_status.return_value = None
        mock_get.return_value = mock_response

        urls = get_all_urls("http://example.com")
        print(f"Found URLs: {urls}")

        # Check that only relevant URLs were returned
        assert len(urls) == 3  # Should only have page, chapter, section URLs
        print("PASS: get_all_urls test passed")


def test_extract_text_from_url():
    """Test the extract_text_from_url function"""
    print("Testing extract_text_from_url function...")

    mock_html = """
    <html>
        <head><title>Test Page</title></head>
        <body>
            <script>console.log('test');</script>
            <style>body { color: red; }</style>
            <h1>Sample Book Content</h1>
            <p>This is some sample text from a book page.</p>
            <p>It contains multiple paragraphs with meaningful content.</p>
        </body>
    </html>
    """

    with patch('requests.get') as mock_get:
        mock_response = MagicMock()
        mock_response.content = mock_html
        mock_response.raise_for_status.return_value = None
        mock_get.return_value = mock_response

        text = extract_text_from_url("http://example.com")
        print(f"Extracted text length: {len(text)}")
        print(f"Sample: {text[:100]}...")

        # Check that the text was extracted and cleaned properly
        assert "sample text" in text.lower()
        assert "console.log" not in text
        assert len(text) > 0
        print("PASS: extract_text_from_url test passed")


def test_chunk_text():
    """Test the chunk_text function"""
    print("Testing chunk_text function...")

    # Create a longer text to test chunking
    long_text = "This is a sentence. " * 100  # 100 sentences

    chunks = chunk_text(long_text)
    print(f"Text chunked into {len(chunks)} chunks")

    # Check that we have multiple chunks
    assert len(chunks) > 1
    assert all(len(chunk) > 0 for chunk in chunks)
    print("PASS: chunk_text test passed")


def test_embed():
    """Test the embed function"""
    print("Testing embed function...")

    # Mock environment variable
    os.environ['COHERE_API_KEY'] = 'test-key'

    # Mock Cohere client
    with patch('cohere.Client') as mock_client_class:
        mock_client_instance = MagicMock()
        mock_client_instance.embed.return_value = MagicMock()
        mock_client_instance.embed.return_value.embeddings = [[0.1, 0.2, 0.3]] * 2  # Mock embeddings
        mock_client_class.return_value = mock_client_instance

        chunks = ["This is a test chunk.", "This is another test chunk."]
        embeddings = embed(chunks)

        print(f"Generated {len(embeddings)} embeddings")
        assert len(embeddings) == 2
        print("PASS: embed test passed")


if __name__ == "__main__":
    print("Starting core function tests...")

    test_get_all_urls()
    test_extract_text_from_url()
    test_chunk_text()
    test_embed()

    print("\nSUCCESS: All core function tests passed successfully!")