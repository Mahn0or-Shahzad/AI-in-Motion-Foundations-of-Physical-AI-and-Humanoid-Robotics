# Embedding Pipeline Tasks

## Feature Overview
Implement an embedding pipeline to extract book content, generate embeddings, and store them in Qdrant for retrieval.

## Phases

### Phase 1: Setup
- [x] T001 Create backend folder and initialize Python project using uv package manager
- [x] T002 Create requirements.txt with required dependencies (requests, beautifulsoup4, cohere, qdrant-client, python-dotenv)
- [x] T003 Set up environment configuration with .env file structure

### Phase 2: Foundational
- [x] T004 Create main.py file structure with proper imports and logging setup
- [x] T005 [P] Implement get_all_urls(base_url) function to return list of book page URLs
- [x] T006 [P] Implement extract_text_from_url(url) function to return cleaned page text
- [x] T007 [P] Implement chunk_text(text) function to return list of semantically meaningful chunks
- [x] T008 [P] Implement embed(chunks) function to generate Cohere embeddings for each chunk
- [x] T009 [P] Implement create_collection(name="rag_embedding") function to set up Qdrant collection
- [x] T010 [P] Implement save_chunk_to_qdrant(embedding, metadata) function to upsert embedding and metadata into Qdrant

### Phase 3: Configuration and Environment
- [x] T011 Configure BASE_URL as an environment variable (placeholder during development)
- [x] T012 [P] Set up environment variable loading with python-dotenv
- [x] T013 [P] Add API key configuration for Cohere and Qdrant

### Phase 4: Main Pipeline Integration
- [x] T014 Implement main() function to orchestrate the full ingestion pipeline
- [x] T015 Add proper error handling and logging throughout the pipeline
- [x] T016 [P] Add logging to track success/failure of each step

### Phase 5: Testing and Validation
- [x] T017 Test similarity search in Qdrant to ensure vectors are correctly stored and retrievable
- [x] T018 [P] Create a test script to validate the entire pipeline
- [x] T019 [P] Verify embedding storage and retrieval functionality

## Dependencies
- Python 3.8+
- uv package manager
- Cohere API access
- Qdrant vector database access

## Parallel Execution Opportunities
- T005-T010: Functions can be implemented in parallel as they are independent
- T012-T013: Environment configuration tasks can be done in parallel
- T017-T019: Testing tasks can be done in parallel after main implementation

## Implementation Strategy
1. Start with Phase 1 (Setup) to establish the project structure
2. Move to Phase 2 (Foundational) to implement core functionality
3. Configure environment in Phase 3
4. Integrate everything in Phase 4 (Main Pipeline)
5. Validate in Phase 5 (Testing)