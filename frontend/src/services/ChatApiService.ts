// src/services/ChatApiService.ts

class ChatApiService {
  private baseUrl: string;
  private defaultHeaders: HeadersInit;

  constructor() {
    // Get backend API URL from environment or use default
    this.baseUrl = process.env.REACT_APP_BACKEND_API_URL || 'http://localhost:8000/v1';

    this.defaultHeaders = {
      'Content-Type': 'application/json',
    };
  }

  /**
   * Send a chat message to the backend
   */
  async sendMessage(chatRequest: ChatRequest): Promise<ChatResponse> {
    try {
      const response = await fetch(`${this.baseUrl}/chat`, {
        method: 'POST',
        headers: {
          ...this.defaultHeaders,
          // Add session ID to headers if available
          ...(chatRequest.session_id && { 'session-id': chatRequest.session_id }),
        },
        body: JSON.stringify({
          message: chatRequest.message,
          session_id: chatRequest.session_id,
          user_id: chatRequest.user_id,
          selected_text: chatRequest.selected_text, // For selected-text Q&A
          temperature: chatRequest.temperature,
          max_tokens: chatRequest.max_tokens,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(`Chat API error: ${response.status} - ${errorData.detail || response.statusText}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error sending message:', error);
      throw error;
    }
  }

  /**
   * Create a new chat session
   */
  async createSession(sessionRequest: SessionCreateRequest): Promise<SessionCreateResponse> {
    try {
      const response = await fetch(`${this.baseUrl}/chat/session`, {
        method: 'POST',
        headers: this.defaultHeaders,
        body: JSON.stringify({
          user_id: sessionRequest.user_id,
          initial_message: sessionRequest.initial_message,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(`Create session API error: ${response.status} - ${errorData.detail || response.statusText}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error creating session:', error);
      throw error;
    }
  }

  /**
   * Get chat history for a session
   */
  async getChatHistory(sessionId: string) {
    try {
      const response = await fetch(`${this.baseUrl}/chat/history/${sessionId}`, {
        method: 'GET',
        headers: this.defaultHeaders,
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(`Get chat history API error: ${response.status} - ${errorData.detail || response.statusText}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error getting chat history:', error);
      throw error;
    }
  }

  /**
   * Delete a chat session
   */
  async deleteSession(sessionId: string) {
    try {
      const response = await fetch(`${this.baseUrl}/chat/session/${sessionId}`, {
        method: 'DELETE',
        headers: this.defaultHeaders,
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(`Delete session API error: ${response.status} - ${errorData.detail || response.statusText}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error deleting session:', error);
      throw error;
    }
  }

  /**
   * Get session statistics
   */
  async getSessionStats(sessionId: string) {
    try {
      const response = await fetch(`${this.baseUrl}/chat/session/${sessionId}/stats`, {
        method: 'GET',
        headers: this.defaultHeaders,
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(`Get session stats API error: ${response.status} - ${errorData.detail || response.statusText}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error getting session stats:', error);
      throw error;
    }
  }

  /**
   * Get health status of the backend
   */
  async getHealthStatus() {
    try {
      const response = await fetch(`${this.baseUrl}/health`, {
        method: 'GET',
        headers: this.defaultHeaders,
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(`Health check API error: ${response.status} - ${errorData.detail || response.statusText}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error getting health status:', error);
      throw error;
    }
  }
}

// Create a singleton instance
export const chatApiService = new ChatApiService();

// Types for the API
export interface ChatRequest {
  message: string;
  session_id?: string;
  user_id?: string;
  selected_text?: string; // For selected-text Q&A
  temperature?: number;
  max_tokens?: number;
}

export interface ChatResponse {
  response: string;
  session_id: string;
  message_id: string;
  sources: Array<{
    id: string;
    chapter: string;
    section: string;
    page_number?: number;
    source_file?: string;
  }>;
  retrieved_chunks: number;
  timestamp: string;
}

export interface SessionCreateRequest {
  user_id?: string;
  initial_message?: string;
}

export interface SessionCreateResponse {
  session_id: string;
  created_at: string;
}

export interface ChatMessage {
  role: 'user' | 'assistant';
  content: string;
  timestamp: string;
}

export interface ChatHistoryResponse {
  messages: ChatMessage[];
  session_id: string;
  total_messages: number;
}

export interface SessionStats {
  session_id: string;
  user_id?: string;
  title: string;
  created_at: string;
  updated_at: string;
  total_messages: number;
  user_messages: number;
  assistant_messages: number;
  total_retrievals: number;
  first_message_at?: string;
  last_message_at?: string;
}