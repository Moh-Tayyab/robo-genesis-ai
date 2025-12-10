from typing import List, Dict, Any
import logging

logger = logging.getLogger(__name__)

class PromptService:
    def __init__(self):
        pass

    def build_full_book_rag_prompt(self, question: str, context_chunks: List[Dict[str, Any]]) -> str:
        """
        Build a prompt for full-book RAG
        """
        if not context_chunks:
            return f"""You are a helpful assistant for the Physical AI & Humanoid Robotics textbook.
            The provided context does not contain information to answer the following question: {question}
            Please respond with "Not in selection." and explain why."""

        context_text = "\n\n".join([
            f"Chapter: {chunk['chapter']}\nSection: {chunk['section']}\nContent: {chunk['text']}\n"
            for chunk in context_chunks
        ])

        prompt = f"""You are a helpful assistant for the Physical AI & Humanoid Robotics textbook.
        Answer the question based solely on the provided context.
        If the answer is not in the provided context, respond with "Not in selection." and explain why.

        Context:
        {context_text}

        Question: {question}

        Answer:"""

        return prompt

    def build_selected_text_rag_prompt(self, question: str, selected_text: str) -> str:
        """
        Build a prompt for selected-text RAG with strict context isolation
        """
        prompt = f"""You are an expert assistant. Answer the user's question USING ONLY THE FOLLOWING TEXT. If the answer is not in the text, say 'I don't know based on the selected text.' NEVER use any other knowledge.

SELECTED TEXT:
{selected_text}

Question: {question}

Answer:"""

        return prompt

    def build_system_prompt(self) -> str:
        """
        Build the system prompt for the assistant
        """
        return """You are a helpful assistant for the Physical AI & Humanoid Robotics textbook.
        Provide accurate, concise answers based on the provided context.
        Always cite the relevant chapters and sections when providing answers.
        If the answer is not in the provided context, respond with "Not in selection." and explain why."""

prompt_service = PromptService()