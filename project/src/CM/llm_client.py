"""Provider-agnostic LLM client for the grounding dialog.

Every client exposes  complete(system, messages, schema) -> dict  where messages is a list of
{"role": "user"|"assistant", "content": str} and the return value is the parsed JSON object the
model produced. Providers: openai (default, SDK already installed), anthropic (pip install
anthropic), fake (scripted replies for tests / offline UI runs).
"""
import json
import logging
import os

import config

logger = logging.getLogger(__name__)
ERR_PREVIEW_CHARS = 200          # how much of a bad reply to quote in the error
OPENAI_SCHEMA_NAME = "grounding"


class LLMError(RuntimeError):
    """Raised when the provider fails or returns something that is not JSON."""


def _parse_json(text):
    try:
        return json.loads(text)
    except ValueError as exc:
        raise LLMError(f"model reply is not JSON: {exc}: {text[:ERR_PREVIEW_CHARS]!r}") from exc


# ------------------------------------------------------------------ OpenAI
class OpenAIClient:
    def __init__(self, model=config.OPENAI_MODEL, api_key=None, base_url=None, extra_body=None,
                 max_tokens=config.LLM_MAX_TOKENS):
        from openai import OpenAI          # imported lazily so other providers need no SDK
        self.client = OpenAI(timeout=config.LLM_TIMEOUT_S, api_key=api_key, base_url=base_url)
        self.model = model
        self.extra_body = extra_body or {}
        self.max_tokens = max_tokens

    def complete(self, system, messages, schema):
        from openai import APIError
        try:
            resp = self.client.chat.completions.create(
                model=self.model,
                max_tokens=self.max_tokens,
                messages=[{"role": "system", "content": system}] + messages,
                response_format={"type": "json_schema",
                                 "json_schema": {"name": OPENAI_SCHEMA_NAME, "strict": False, "schema": schema}},
                extra_body=self.extra_body,
            )
        except APIError as exc:
            raise LLMError(f"{self.model}: {exc}") from exc
        choice = resp.choices[0]
        if choice.finish_reason == "length":
            raise LLMError(f"{self.model}: reply truncated at max_tokens={self.max_tokens} "
                           "(thinking tokens count too; raise LLM_MAX_TOKENS or lower thinking)")
        return _parse_json(choice.message.content or "")


# ------------------------------------------------------------------ Gemini
class GeminiClient(OpenAIClient):
    """Gemini through its OpenAI-compatible endpoint (same request shape, GEMINI_API_KEY)."""

    def __init__(self, model=config.GEMINI_MODEL):
        key = os.environ.get("GEMINI_API_KEY")
        if not key:
            raise LLMError("GEMINI_API_KEY is not set (put it in project/src/CM/.env)")
        super().__init__(model=model, api_key=key, base_url=config.GEMINI_BASE_URL,
                         extra_body=config.GEMINI_EXTRA_BODY, max_tokens=config.GEMINI_MAX_TOKENS)


# ------------------------------------------------------------------ Anthropic
class AnthropicClient:
    def __init__(self, model=config.ANTHROPIC_MODEL):
        import anthropic                   # pip install anthropic
        self.client = anthropic.Anthropic(timeout=config.LLM_TIMEOUT_S)
        self.model = model

    def complete(self, system, messages, schema):
        import anthropic
        try:
            resp = self.client.beta.messages.create(
                model=self.model,
                max_tokens=config.LLM_MAX_TOKENS,
                system=system,
                messages=messages,
                output_config={"format": {"type": "json_schema", "schema": schema}},
                betas=["server-side-fallback-2026-07-01"],
                fallbacks="default",
            )
        except anthropic.RateLimitError as exc:
            raise LLMError(f"anthropic rate limit: {exc}") from exc
        except anthropic.APIStatusError as exc:
            raise LLMError(f"anthropic {exc.status_code}: {exc.message}") from exc
        except anthropic.APIConnectionError as exc:
            raise LLMError(f"anthropic connection: {exc}") from exc
        if resp.stop_reason == "refusal":
            detail = resp.stop_details.category if resp.stop_details else "unknown"
            raise LLMError(f"anthropic refused the request ({detail})")
        text = next((b.text for b in resp.content if b.type == "text"), "")
        return _parse_json(text)


# ------------------------------------------------------------------ Fake
class FakeClient:
    """Returns scripted replies in order; used by tests and by CM_PROVIDER=fake."""

    def __init__(self, replies=None):
        self.replies = list(replies or [])
        self.calls = []                    # (system, messages) of every call, for assertions

    def complete(self, system, messages, schema):
        self.calls.append((system, messages))
        if not self.replies:
            raise LLMError("fake client: no scripted reply left")
        reply = self.replies.pop(0)
        return reply if isinstance(reply, dict) else _parse_json(reply)


def make_client(provider=config.PROVIDER, **kwargs):
    provider = (provider or "").lower()
    if provider == "openai":
        return OpenAIClient(**kwargs)
    if provider == "gemini":
        return GeminiClient(**kwargs)
    if provider == "anthropic":
        return AnthropicClient(**kwargs)
    if provider == "fake":
        return FakeClient(**kwargs)
    raise ValueError(f"unknown provider {provider!r} (openai | gemini | anthropic | fake)")
