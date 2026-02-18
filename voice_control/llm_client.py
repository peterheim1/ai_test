"""Ollama LLM client for general questions."""

import ollama


class LLMClient:
    """Routes general questions to a local Mistral model via Ollama."""

    def __init__(self, model: str = "mistral",
                 host: str = "http://localhost:11434",
                 system_prompt: str = "",
                 max_tokens: int = 100):
        self.model = model
        self.system_prompt = system_prompt or (
            "You are Robbie, a helpful robot assistant. "
            "Give brief, conversational answers (1-2 sentences max). "
            "If you don't know, say so."
        )
        self.max_tokens = max_tokens
        self._client = ollama.Client(host=host)

    def health_check(self) -> bool:
        """Check if Ollama is running and the model is available."""
        try:
            models = self._client.list()
            available = [m.model for m in models.models]
            return any(self.model in name for name in available)
        except Exception:
            return False

    def ask(self, question: str) -> str:
        """Ask a general question and get a text response.

        Args:
            question: The user's question.

        Returns:
            LLM response text.
        """
        try:
            response = self._client.chat(
                model=self.model,
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": question},
                ],
                options={"num_predict": self.max_tokens},
            )
            return response.message.content.strip()
        except Exception as e:
            return f"I can't answer questions right now: {e}"
