"""
Embedding Pipeline for Book Content Processing
Extracts content from book pages, generates embeddings, and stores them in Qdrant.
"""

import os
import logging
from typing import List, Dict, Tuple
import requests
from bs4 import BeautifulSoup
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv


# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def get_all_urls(base_url: str) -> List[str]:
    """
    Returns a list of book page URLs from the base URL.

    Args:
        base_url (str): The base URL to extract book page URLs from

    Returns:
        List[str]: List of book page URLs
    """
    try:
        logger.info(f"Fetching URLs from base URL: {base_url}")

        # Add headers to avoid some basic anti-bot measures
        headers = {
            'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
        }

        response = requests.get(base_url, headers=headers)
        response.raise_for_status()

        soup = BeautifulSoup(response.content, 'html.parser')

        # Find all links that point to book pages
        # This is a generic implementation - adjust selectors based on actual site structure
        urls = []
        for link in soup.find_all('a', href=True):
            href = link['href']

            # Handle relative URLs
            if href.startswith('/'):
                full_url = requests.compat.urljoin(base_url, href)
            elif href.startswith('http'):
                full_url = href
            else:
                full_url = requests.compat.urljoin(base_url, href)

            # Filter for likely book page URLs (adjust as needed based on site structure)
            if any(keyword in full_url.lower() for keyword in ['page', 'chapter', 'section', 'content']):
                urls.append(full_url)

        logger.info(f"Found {len(urls)} URLs")
        return urls

    except requests.exceptions.HTTPError as e:
        if response.status_code == 401 or response.status_code == 403:
            logger.error(f"Authentication required for {base_url}. Status code: {response.status_code}")
            logger.error("Please check if the URL requires authentication or has access restrictions.")
            # Return empty list instead of raising exception to allow pipeline to continue
            return []
        else:
            logger.error(f"HTTP Error fetching URLs from {base_url}: {e}")
            raise
    except requests.RequestException as e:
        logger.error(f"Error fetching URLs from {base_url}: {e}")
        # Return empty list instead of raising exception to allow pipeline to continue
        return []
    except Exception as e:
        logger.error(f"Unexpected error in get_all_urls: {e}")
        # Return empty list instead of raising exception to allow pipeline to continue
        return []


def extract_text_from_url(url: str) -> str:
    """
    Returns cleaned page text from the given URL.

    Args:
        url (str): The URL to extract text from

    Returns:
        str: Cleaned page text
    """
    try:
        logger.info(f"Extracting text from URL: {url}")

        # Add headers to avoid some basic anti-bot measures
        headers = {
            'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
        }

        response = requests.get(url, headers=headers)
        response.raise_for_status()

        soup = BeautifulSoup(response.content, 'html.parser')

        # Remove script and style elements
        for script in soup(["script", "style"]):
            script.decompose()

        # Get text content
        text = soup.get_text()

        # Clean up the text
        lines = (line.strip() for line in text.splitlines())
        chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
        text = ' '.join(chunk for chunk in chunks if chunk)

        logger.info(f"Extracted {len(text)} characters from {url}")
        return text

    except requests.exceptions.HTTPError as e:
        if response.status_code == 401 or response.status_code == 403:
            logger.error(f"Authentication required for {url}. Status code: {response.status_code}")
            logger.error("Please check if the URL requires authentication or has access restrictions.")
            return ""  # Return empty string instead of raising exception
        else:
            logger.error(f"HTTP Error extracting text from {url}: {e}")
            raise
    except requests.RequestException as e:
        logger.error(f"Error extracting text from {url}: {e}")
        raise
    except Exception as e:
        logger.error(f"Unexpected error in extract_text_from_url: {e}")
        raise


def chunk_text(text: str) -> List[str]:
    """
    Returns list of semantically meaningful chunks from the text.

    Args:
        text (str): The text to chunk

    Returns:
        List[str]: List of text chunks
    """
    # Simple approach: split text into chunks of approximately 500 characters
    # without breaking sentences
    max_chunk_size = 500
    sentences = text.split('. ')

    chunks = []
    current_chunk = ""

    for sentence in sentences:
        # Add the period back that was removed by split
        sentence_with_period = sentence + ". "

        if len(current_chunk) + len(sentence_with_period) <= max_chunk_size:
            current_chunk += sentence_with_period
        else:
            if current_chunk:
                chunks.append(current_chunk.strip())
            current_chunk = sentence_with_period

    # Add the last chunk if it exists
    if current_chunk.strip():
        chunks.append(current_chunk.strip())

    logger.info(f"Text chunked into {len(chunks)} chunks")
    return chunks


def embed(chunks: List[str]) -> List[List[float]]:
    """
    Generates Cohere embeddings for each chunk.

    Args:
        chunks (List[str]): List of text chunks to embed

    Returns:
        List[List[float]]: List of embeddings (each embedding is a list of floats)
    """
    try:
        cohere_api_key = os.getenv("COHERE_API_KEY")
        if not cohere_api_key:
            raise ValueError("COHERE_API_KEY environment variable not set")

        co = cohere.Client(cohere_api_key)

        logger.info(f"Generating embeddings for {len(chunks)} chunks")

        # Generate embeddings for all chunks at once
        response = co.embed(
            texts=chunks,
            model="embed-english-v3.0",  # Using a common Cohere embedding model
            input_type="search_document"  # Specify the input type
        )

        embeddings = response.embeddings

        logger.info(f"Generated {len(embeddings)} embeddings")
        return embeddings

    except Exception as e:
        logger.error(f"Error generating embeddings: {e}")
        raise


def create_collection(name: str = "rag_embedding"):
    """
    Sets up Qdrant collection.

    Args:
        name (str): Name of the collection to create
    """
    try:
        qdrant_host = os.getenv("QDRANT_HOST")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if not qdrant_host:
            raise ValueError("QDRANT_HOST environment variable not set")

        # Check if the host already contains protocol (http:// or https://)
        if qdrant_host.startswith(('http://', 'https://')):
            # If protocol is already included, use as-is
            qdrant_url = qdrant_host
        else:
            # Add https:// if not present
            qdrant_url = f"https://{qdrant_host}"

        # Initialize Qdrant client for Cloud
        client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key
        )

        # Check if collection already exists
        try:
            collection_info = client.get_collection(collection_name=name)
            logger.info(f"Collection '{name}' already exists")

            # Check if vector size matches expected size
            if collection_info.config.params.vectors.size != 1536:
                logger.warning(f"Collection '{name}' exists but has different vector size: {collection_info.config.params.vectors.size}. Expected: 1536")
                # Note: We won't recreate the collection to avoid data loss, just warn the user
            return
        except Exception as e:
            # Collection doesn't exist, so create it
            logger.info(f"Collection '{name}' does not exist, creating new collection...")

        # Create collection with appropriate vector size for Cohere's embed-english-v3.0 model (1536 dimensions)
        client.create_collection(
            collection_name=name,
            vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE)
        )

        logger.info(f"Collection '{name}' created successfully")

    except Exception as e:
        logger.error(f"Error creating Qdrant collection: {e}")
        raise


def save_chunk_to_qdrant(embedding: List[float], metadata: Dict):
    """
    Upserts embedding and metadata into Qdrant.

    Args:
        embedding (List[float]): The embedding vector
        metadata (Dict): Metadata associated with the chunk
    """
    try:
        qdrant_host = os.getenv("QDRANT_HOST")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if not qdrant_host:
            raise ValueError("QDRANT_HOST environment variable not set")

        # Check if the host already contains protocol (http:// or https://)
        if qdrant_host.startswith(('http://', 'https://')):
            # If protocol is already included, use as-is
            qdrant_url = qdrant_host
        else:
            # Add https:// if not present
            qdrant_url = f"https://{qdrant_host}"

        # Initialize Qdrant client for Cloud
        client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key
        )

        # Create a unique ID for this record (using a simple approach with timestamp)
        import time
        record_id = int(time.time() * 1000000)  # microseconds timestamp as ID

        # Upsert the record into the collection
        client.upsert(
            collection_name="rag_embedding",
            points=[
                models.PointStruct(
                    id=record_id,
                    vector=embedding,
                    payload=metadata
                )
            ]
        )

        logger.info(f"Saved chunk to Qdrant with ID: {record_id}")

    except Exception as e:
        logger.error(f"Error saving chunk to Qdrant: {e}")
        raise


def main():
    """
    Main function to orchestrate the full ingestion pipeline.
    """
    logger.info("Starting embedding pipeline...")

    try:
        # Get base URL from environment
        base_url = os.getenv("BASE_URL")
        if not base_url:
            raise ValueError("BASE_URL environment variable not set")

        # Create Qdrant collection
        create_collection("rag_embedding")

        # Get all URLs
        urls = get_all_urls(base_url)

        if not urls:
            logger.warning(f"No URLs found at {base_url}. Pipeline will complete without processing any content.")
            logger.info("Embedding pipeline completed successfully (no content to process)!")
            return

        # Process each URL
        for i, url in enumerate(urls):
            logger.info(f"Processing URL {i+1}/{len(urls)}: {url}")

            # Extract text from URL
            text = extract_text_from_url(url)

            # Skip if no text was extracted
            if not text.strip():
                logger.warning(f"No text extracted from {url}, skipping...")
                continue

            # Chunk the text
            chunks = chunk_text(text)

            if not chunks:
                logger.warning(f"No chunks created from {url}, skipping...")
                continue

            # Generate embeddings for chunks
            embeddings = embed(chunks)

            # Save each chunk and its embedding to Qdrant
            for j, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
                metadata = {
                    "url": url,
                    "chunk_index": j,
                    "original_text_length": len(chunk),
                    "source": "book_content"
                }
                save_chunk_to_qdrant(embedding, metadata)

        logger.info("Embedding pipeline completed successfully!")

    except Exception as e:
        logger.error(f"Error in main pipeline: {e}")
        raise


if __name__ == "__main__":
    main()