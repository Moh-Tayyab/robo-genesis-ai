"""Prompt Service for building and managing prompts for RAG operations"""
from typing import List, Dict, Any, Optional
from pydantic import BaseModel
import logging
from datetime import datetime

logger = logging.getLogger(__name__)

class PromptTemplate(BaseModel):
    """Model for prompt templates"""
    name: str
    system_prompt: str
    user_prompt_template: str
    context_template: Optional[str] = None


class PromptService:
    """Service for building and managing prompts for RAG operations"""

    def __init__(self):
        self.templates = self._initialize_templates()

    def _initialize_templates(self) -> Dict[str, PromptTemplate]:
        """Initialize default prompt templates"""
        templates = {}

        # Full-book RAG template
        templates["full_book_rag"] = PromptTemplate(
            name="full_book_rag",
            system_prompt=(
                "You are an expert assistant for a Physical AI and Humanoid Robotics textbook. "
                "Answer the user's question based on the provided context from the textbook. "
                "If the context doesn't contain enough information to answer the question, "
                "say 'I cannot answer this question based on the provided textbook content.' "
                "Always cite specific sources from the context when providing answers. "
                "Be accurate, concise, and helpful. Focus on the technical aspects of AI, robotics, "
                "and physical intelligence as described in the textbook."
            ),
            user_prompt_template="Context:\n{context}\n\nQuestion: {query}",
            context_template="Source {index} (Chapter: {chapter}, Section: {section}):\n{content}\n"
        )

        # Selected-text RAG template (will be enhanced for US2)
        templates["selected_text_rag"] = PromptTemplate(
            name="selected_text_rag",
            system_prompt=(
                "You are an expert assistant for a Physical AI and Humanoid Robotics textbook. "
                "Answer the user's question based ONLY on the provided selected text context. "
                "Do NOT use any external knowledge or general textbook information. "
                "If the selected text doesn't contain enough information to answer the question, "
                "say 'I cannot answer this question based only on the provided selected text.' "
                "Always refer to the specific content in the selected text when providing answers. "
                "Be accurate, concise, and strictly constrained to the provided context."
            ),
            user_prompt_template="Selected Text Context:\n{selected_text}\n\nQuestion: {query}",
            context_template="{content}"
        )

        # Hypothetical document generation template (for HyDE)
        templates["hypothetical_document"] = PromptTemplate(
            name="hypothetical_document",
            system_prompt=(
                "You are an expert at creating hypothetical answers to questions. "
                "Generate a detailed, accurate answer to the following question as if it were taken directly from a Physical AI and Humanoid Robotics textbook. "
                "Your response should be in the same style as an academic textbook, using technical terminology appropriately."
            ),
            user_prompt_template="Generate a detailed answer to this question: {query}"
        )

        # Content relevance checking template
        templates["content_relevance"] = PromptTemplate(
            name="content_relevance",
            system_prompt=(
                "You are a content relevance checker. Determine if the provided content is relevant to the given query. "
                "Respond with only 'YES' if relevant, or 'NO' if not relevant."
            ),
            user_prompt_template="Content: {content}\n\nQuery: {query}\n\nIs the content relevant to the query?"
        )

        # Question classification template
        templates["question_classifier"] = PromptTemplate(
            name="question_classifier",
            system_prompt=(
                "You are a question classifier. Classify the user's question as either 'technical', 'conceptual', 'application', or 'other'. "
                "Technical questions ask about specific methods, algorithms, or implementations. "
                "Conceptual questions ask about theories, principles, or definitions. "
                "Application questions ask about how to apply concepts in practice. "
                "Return only the classification label."
            ),
            user_prompt_template="Classify this question: {query}"
        )

        return templates

    def build_full_book_rag_prompt(self, query: str, context_chunks: List[Dict[str, Any]]) -> List[Dict[str, str]]:
        """Build a full-book RAG prompt with context"""
        try:
            template = self.templates["full_book_rag"]

            # Build context from chunks
            context_parts = []
            for i, chunk in enumerate(context_chunks):
                context_part = template.context_template.format(
                    index=i + 1,
                    chapter=chunk.get("chapter", "Unknown"),
                    section=chunk.get("section", "Unknown"),
                    content=chunk.get("content", "")
                )
                context_parts.append(context_part)

            context = "\n".join(context_parts)

            # Format user prompt
            user_prompt = template.user_prompt_template.format(
                context=context,
                query=query
            )

            # Create messages
            messages = [
                {"role": "system", "content": template.system_prompt},
                {"role": "user", "content": user_prompt}
            ]

            logger.debug(f"Built full-book RAG prompt with {len(context_chunks)} context chunks")
            return messages

        except Exception as e:
            logger.error(f"Error building full-book RAG prompt: {e}")
            raise

    def build_selected_text_rag_prompt(self, query: str, selected_text: str) -> List[Dict[str, str]]:
        """Build a selected-text RAG prompt"""
        try:
            template = self.templates["selected_text_rag"]

            # Format user prompt
            user_prompt = template.user_prompt_template.format(
                selected_text=selected_text,
                query=query
            )

            # Create messages
            messages = [
                {"role": "system", "content": template.system_prompt},
                {"role": "user", "content": user_prompt}
            ]

            logger.debug(f"Built selected-text RAG prompt for query: {query[:50]}...")
            return messages

        except Exception as e:
            logger.error(f"Error building selected-text RAG prompt: {e}")
            raise

    def build_hypothetical_document_prompt(self, query: str) -> List[Dict[str, str]]:
        """Build a prompt for generating hypothetical documents (HyDE)"""
        try:
            template = self.templates["hypothetical_document"]

            # Format user prompt
            user_prompt = template.user_prompt_template.format(query=query)

            # Create messages
            messages = [
                {"role": "system", "content": template.system_prompt},
                {"role": "user", "content": user_prompt}
            ]

            logger.debug(f"Built hypothetical document prompt for query: {query[:50]}...")
            return messages

        except Exception as e:
            logger.error(f"Error building hypothetical document prompt: {e}")
            raise

    def build_content_relevance_prompt(self, content: str, query: str) -> List[Dict[str, str]]:
        """Build a prompt for checking content relevance"""
        try:
            template = self.templates["content_relevance"]

            # Format user prompt
            user_prompt = template.user_prompt_template.format(content=content, query=query)

            # Create messages
            messages = [
                {"role": "system", "content": template.system_prompt},
                {"role": "user", "content": user_prompt}
            ]

            logger.debug(f"Built content relevance prompt for query: {query[:30]}...")
            return messages

        except Exception as e:
            logger.error(f"Error building content relevance prompt: {e}")
            raise

    def build_question_classification_prompt(self, query: str) -> List[Dict[str, str]]:
        """Build a prompt for classifying questions"""
        try:
            template = self.templates["question_classifier"]

            # Format user prompt
            user_prompt = template.user_prompt_template.format(query=query)

            # Create messages
            messages = [
                {"role": "system", "content": template.system_prompt},
                {"role": "user", "content": user_prompt}
            ]

            logger.debug(f"Built question classification prompt for query: {query[:30]}...")
            return messages

        except Exception as e:
            logger.error(f"Error building question classification prompt: {e}")
            raise

    def format_citations(self, chunks: List[Dict[str, Any]]) -> str:
        """Format citations from retrieved chunks"""
        try:
            citations = []
            for i, chunk in enumerate(chunks):
                citation = (
                    f"[{i+1}] {chunk.get('chapter', 'Unknown Chapter')}, "
                    f"{chunk.get('section', 'Unknown Section')}"
                )
                if chunk.get('page_number') is not None:
                    citation += f", page {chunk.get('page_number')}"
                if chunk.get('source_file'):
                    citation += f", source: {chunk.get('source_file')}"
                citations.append(citation)

            return "\n".join(citations)

        except Exception as e:
            logger.error(f"Error formatting citations: {e}")
            return ""

    def build_response_with_citations(self, answer: str, chunks: List[Dict[str, Any]]) -> str:
        """Build a response that includes citations to sources"""
        try:
            # Add citations to the answer
            citations = self.format_citations(chunks)
            if citations:
                full_response = f"{answer}\n\nSources:\n{citations}"
            else:
                full_response = answer

            return full_response

        except Exception as e:
            logger.error(f"Error building response with citations: {e}")
            return answer

    def validate_prompt_length(self, messages: List[Dict[str, str]], max_tokens: int = 128000) -> bool:
        """Validate that the prompt doesn't exceed token limits"""
        try:
            # Simple approximation: 1 token ≈ 4 characters
            total_chars = sum(len(msg.get("content", "")) for msg in messages)
            approx_tokens = total_chars // 4

            return approx_tokens < max_tokens

        except Exception as e:
            logger.error(f"Error validating prompt length: {e}")
            return False

    def truncate_context_if_needed(self, context: str, max_length: int = 120000) -> str:
        """Truncate context if it exceeds maximum length"""
        try:
            if len(context) > max_length:
                logger.warning(f"Context truncated from {len(context)} to {max_length} characters")
                return context[:max_length]
            return context

        except Exception as e:
            logger.error(f"Error truncating context: {e}")
            return context


# Global instance
prompt_service = PromptService()

# ✓ SPEC-KIT PLUS VERIFIED