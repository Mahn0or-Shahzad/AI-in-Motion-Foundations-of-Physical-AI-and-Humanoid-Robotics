# AI in Motion - Embedding Pipeline

This project implements an embedding pipeline to extract book content, generate embeddings, and store them in Qdrant for retrieval.

## Features

- Extract content from book pages
- Clean and process text
- Chunk text into semantically meaningful segments
- Generate Cohere embeddings
- Store embeddings in Qdrant vector database
- Support for similarity search

## Requirements

- Python 3.8+
- uv package manager
- Cohere API key
- Qdrant vector database

## Setup

1. **Install uv package manager** (if not already installed):
   ```bash
   pip install uv
   ```

2. **Create virtual environment and install dependencies**:
   ```bash
   cd backend
   uv venv
   source .venv/Scripts/activate  # On Windows
   # source .venv/bin/activate    # On Linux/Mac
   uv pip install -r requirements.txt
   ```

3. **Configure environment variables**:
   Copy the `.env` file and update with your values:
   ```bash
   # Update the .env file with your values
   BASE_URL=your_book_site_url
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_HOST=localhost
   QDRANT_PORT=6333
   QDRANT_API_KEY=your_qdrant_api_key_if_needed
   ```

4. **Run Qdrant** (if running locally):
   ```bash
   # Option 1: Using Docker
   docker run -p 6333:6333 -p 6334:6334 \
     -e QDRANT__SERVICE__API_KEY=your_secret_api_key \
     qdrant/qdrant

   # Option 2: Using the official Qdrant Docker Compose
   # docker-compose up -d
   ```

## Usage

### Run the main pipeline:
```bash
cd backend
source .venv/Scripts/activate
uv run main.py
```

### Run tests:
```bash
# Run core function tests
uv run test_core_functions.py

# Run demo with mocked services
uv run demo_pipeline.py
```

## Project Structure

```
backend/
├── main.py                 # Main pipeline implementation
├── requirements.txt        # Python dependencies
├── .env                   # Environment configuration
├── test_core_functions.py  # Core function tests
├── test_qdrant.py         # Qdrant validation tests
├── test_pipeline.py       # Full pipeline tests
└── demo_pipeline.py       # Demo script with mocked services
```

## Configuration

The pipeline can be configured via environment variables in the `.env` file:

- `BASE_URL`: The base URL to extract book content from
- `COHERE_API_KEY`: Your Cohere API key for generating embeddings
- `QDRANT_HOST`: Qdrant server host (default: localhost)
- `QDRANT_PORT`: Qdrant server port (default: 6333)
- `QDRANT_API_KEY`: Qdrant API key (if required)

## Pipeline Flow

1. **URL Extraction**: Extract all book page URLs from the base URL
2. **Text Extraction**: Extract and clean text from each URL
3. **Text Chunking**: Split text into semantically meaningful chunks
4. **Embedding Generation**: Generate Cohere embeddings for each chunk
5. **Storage**: Store embeddings and metadata in Qdrant

## Testing

The project includes several test scripts:

- `test_core_functions.py`: Tests for core functions without external dependencies
- `test_qdrant.py`: Validates Qdrant connectivity and search functionality
- `test_pipeline.py`: Full pipeline integration tests
- `demo_pipeline.py`: Demonstrates the pipeline with mocked services

## Troubleshooting

- **Qdrant Connection Error**: Ensure Qdrant is running and accessible
- **Cohere API Error**: Verify your API key is valid and has sufficient quota
- **URL Extraction Issues**: Check that the base URL is accessible and contains expected content

## Next Steps

1. Deploy Qdrant in production
2. Add more sophisticated text cleaning and chunking algorithms
3. Implement batch processing for large volumes of content
4. Add monitoring and logging for production use