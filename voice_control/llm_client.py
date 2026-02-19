"""Ollama LLM client for general questions with daily short-term memory."""

from datetime import datetime, date

import ollama


class LLMClient:
    """Routes general questions to a local Mistral model via Ollama.

    Maintains a short-term memory of today's conversation in time order.
    Previous days' history is automatically pruned.
    """

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
        # Short-term memory: list of {"ts": datetime, "role": str, "content": str}
        self._history: list[dict] = []

    def health_check(self) -> bool:
        """Check if Ollama is running and the model is available."""
        try:
            models = self._client.list()
            available = [m.model for m in models.models]
            return any(self.model in name for name in available)
        except Exception:
            return False

    def _get_today_history(self) -> list[dict]:
        """Return today's messages in time order, pruning older entries."""
        today = date.today()
        self._history = [e for e in self._history if e["ts"].date() == today]
        return [{"role": e["role"], "content": e["content"]} for e in self._history]

    def get_history_summary(self) -> list[dict]:
        """Return today's history as a list of {time, role, content} dicts for display."""
        today = date.today()
        return [
            {
                "time": e["ts"].strftime("%H:%M:%S"),
                "role": e["role"],
                "content": e["content"],
            }
            for e in self._history
            if e["ts"].date() == today
        ]

    def ask(self, question: str) -> str:
        """Ask a general question and get a text response.

        Today's conversation history is included as context so the LLM
        remembers what was said earlier in the day.

        Args:
            question: The user's question.

        Returns:
            LLM response text.
        """
        today_history = self._get_today_history()
        messages = [{"role": "system", "content": self.system_prompt}]
        messages.extend(today_history)
        messages.append({"role": "user", "content": question})

        try:
            response = self._client.chat(
                model=self.model,
                messages=messages,
                options={"num_predict": self.max_tokens},
            )
            answer = response.message.content.strip()

            # Store exchange in daily memory
            now = datetime.now()
            self._history.append({"ts": now, "role": "user", "content": question})
            self._history.append({"ts": now, "role": "assistant", "content": answer})

            return answer
        except Exception as e:
            return f"I can't answer questions right now: {e}"
