from openai import OpenAI
from typing import Dict, Any, List, Optional
from app.core.config import settings
from app.vector_db import vector_db_client
import logging
import json

logger = logging.getLogger(__name__)

class AgentRAGService:
    def __init__(self):
        self.client = OpenAI(api_key=settings.openai_api_key)
        self.vector_db = vector_db_client
        self.assistant = None
        self._create_assistant()

    def _create_assistant(self):
        """Create an OpenAI Assistant with RAG capabilities"""
        try:
            # Create or retrieve the assistant
            # First, try to get existing assistant by checking if we have its ID stored
            assistant_id = settings.openai_assistant_id if hasattr(settings, 'openai_assistant_id') else None

            if assistant_id:
                try:
                    self.assistant = self.client.beta.assistants.retrieve(assistant_id)
                    logger.info(f"Retrieved existing assistant: {assistant_id}")
                except:
                    # If assistant doesn't exist, create a new one
                    self.assistant = self._create_new_assistant()
            else:
                self.assistant = self._create_new_assistant()

        except Exception as e:
            logger.error(f"Error creating assistant: {e}")
            raise

    def _create_new_assistant(self):
        """Create a new OpenAI Assistant with RAG tools"""
        assistant = self.client.beta.assistants.create(
            name="Physical AI & Humanoid Robotics RAG Assistant",
            description="An expert assistant for the Physical AI & Humanoid Robotics textbook with RAG capabilities",
            model=settings.openai_model,
            instructions="""You are an expert assistant for the Physical AI & Humanoid Robotics textbook.
            Use the retrieve_knowledge tool to search for relevant information from the textbook.
            Always cite the relevant chapters and sections when providing answers.
            If the answer is not in the provided context, clearly state that the information is not available in the textbook.
            For selected text queries, use the provided context string directly with the selected_text_context parameter.""",
            tools=[
                {
                    "type": "function",
                    "function": {
                        "name": "retrieve_knowledge",
                        "description": "Retrieve relevant information from the Physical AI & Humanoid Robotics textbook based on the query",
                        "parameters": {
                            "type": "object",
                            "properties": {
                                "query": {
                                    "type": "string",
                                    "description": "The search query to find relevant information in the textbook"
                                },
                                "top_k": {
                                    "type": "integer",
                                    "description": "Number of results to return (default 6)",
                                    "default": 6
                                },
                                "similarity_threshold": {
                                    "type": "number",
                                    "description": "Minimum similarity threshold for results (default 0.82)",
                                    "default": 0.82
                                }
                            },
                            "required": ["query"]
                        }
                    }
                },
                {
                    "type": "function",
                    "function": {
                        "name": "use_selected_text_context",
                        "description": "Use the provided selected text as the exclusive context for answering questions",
                        "parameters": {
                            "type": "object",
                            "properties": {
                                "selected_text": {
                                    "type": "string",
                                    "description": "The selected text to use as exclusive context"
                                },
                                "question": {
                                    "type": "string",
                                    "description": "The question to answer based on the selected text"
                                }
                            },
                            "required": ["selected_text", "question"]
                        }
                    }
                }
            ]
        )
        logger.info(f"Created new assistant: {assistant.id}")
        return assistant

    def create_thread(self) -> str:
        """Create a new conversation thread"""
        thread = self.client.beta.threads.create()
        return thread.id

    def add_message_to_thread(self, thread_id: str, role: str, content: str):
        """Add a message to the thread"""
        self.client.beta.threads.messages.create(
            thread_id=thread_id,
            role=role,
            content=content
        )

    def run_assistant(self, thread_id: str, instructions: Optional[str] = None) -> str:
        """Run the assistant on the thread and get response"""
        run = self.client.beta.threads.runs.create(
            thread_id=thread_id,
            assistant_id=self.assistant.id,
            instructions=instructions
        )

        # Wait for the run to complete
        import time
        while True:
            run_status = self.client.beta.threads.runs.retrieve(
                thread_id=thread_id,
                run_id=run.id
            )

            if run_status.status == 'completed':
                break
            elif run_status.status == 'failed':
                raise Exception(f"Assistant run failed: {run_status.last_error}")
            elif run_status.status == 'requires_action':
                # Handle tool calls
                tool_outputs = self._handle_tool_calls(run_status)

                # Submit tool outputs
                self.client.beta.threads.runs.submit_tool_outputs(
                    thread_id=thread_id,
                    run_id=run.id,
                    tool_outputs=tool_outputs
                )
                continue

            time.sleep(0.5)  # Poll every 0.5 seconds

        # Get the latest message from the assistant
        messages = self.client.beta.threads.messages.list(
            thread_id=thread_id,
            limit=1,
            order="desc"
        )

        if messages.data:
            message_content = messages.data[0].content[0]
            if message_content.type == "text":
                return message_content.text.value

        return "I couldn't generate a response. Please try again."

    def _handle_tool_calls(self, run_status) -> List[Dict[str, Any]]:
        """Handle tool calls from the assistant"""
        tool_outputs = []

        for tool_call in run_status.required_action.submit_tool_outputs.tool_calls:
            try:
                function_name = tool_call.function.name
                function_args = json.loads(tool_call.function.arguments)

                if function_name == "retrieve_knowledge":
                    result = self._retrieve_knowledge(**function_args)
                elif function_name == "use_selected_text_context":
                    result = self._use_selected_text_context(**function_args)
                else:
                    result = {"error": f"Unknown function: {function_name}"}

                tool_outputs.append({
                    "tool_call_id": tool_call.id,
                    "output": json.dumps(result)
                })
            except Exception as e:
                logger.error(f"Error executing tool call {tool_call.id}: {e}")
                tool_outputs.append({
                    "tool_call_id": tool_call.id,
                    "output": json.dumps({"error": str(e)})
                })

        return tool_outputs

    def _retrieve_knowledge(self, query: str, top_k: int = 6, similarity_threshold: float = 0.82) -> Dict[str, Any]:
        """Retrieve knowledge from the vector database"""
        try:
            # Create embedding for the query
            query_embedding = self.client.embeddings.create(
                model=settings.embedding_model,
                input=query
            ).data[0].embedding

            # Search in vector database
            search_results = self.vector_db.search(
                query_vector=query_embedding,
                top_k=top_k,
                score_threshold=similarity_threshold
            )

            # Format results
            context_text = "\n\n".join([chunk["text"] for chunk in search_results])
            sources = [
                {
                    "chapter": chunk["chapter"],
                    "section": chunk["section"],
                    "url": chunk["url"],
                    "similarity": chunk["similarity"]
                }
                for chunk in search_results
            ]

            return {
                "context": context_text,
                "sources": sources,
                "chunks_retrieved": len(search_results)
            }
        except Exception as e:
            logger.error(f"Error in _retrieve_knowledge: {e}")
            return {"error": f"Failed to retrieve knowledge: {str(e)}"}

    def _use_selected_text_context(self, selected_text: str, question: str) -> Dict[str, Any]:
        """Use selected text as the exclusive context"""
        try:
            # Simply return the selected text as context
            return {
                "context": selected_text,
                "question": question,
                "sources": [{"chapter": "Selected Text", "section": "User Selection", "url": "#selected-text", "similarity": 1.0}]
            }
        except Exception as e:
            logger.error(f"Error in _use_selected_text_context: {e}")
            return {"error": f"Failed to use selected text context: {str(e)}"}

# Global instance
agent_rag_service = AgentRAGService()